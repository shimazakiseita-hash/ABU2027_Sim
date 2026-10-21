"""
ABU Robocon 2027 Phase 1 2Dシム: 手動操縦コンソール(br_teleop_node)。

意思決定ノード(br_decision_tr_node/br_decision_br_node)を起動しない状態
(`enable_decision:=false`)で、TR/BRを手動で操作するためのデバッグ用ノード。
br_visualizer_nodeと同じく独立したpygameウィンドウを持つ(別プロセスとして
起動されるため自然に別ウィンドウになる)。ソフトに詳しくないメンバーでも
使えるよう、画面は日本語のボタンUIとし、「次にすること」の案内・操作結果の
通知・今押しても意味のないボタンの無効化(理由表示付き)を行う。

トピック契約(docs/topic_contract.md)に定義済みの/tr_cmd_vel, /br_cmd_vel,
/tr_gripper_cmd, /br_gripper_cmd, /br_build_actionへpublishするだけで、
sim_bridge_node側の変更は不要(「ロボットへの指令は単一トピックで受ける」
設計のおかげで、意思決定ノードの代わりにこのノードが同じトピックへ
publishすれば手動操縦に差し替えられる)。状態表示のための購読は、意思決定
ノードと同じ規約(/true_state/*は購読禁止、観測トピックのみ)に従い
/tr_pose_estimated, /br_pose_estimated, /detected_blocks, /detected_mustika,
/detected_towersを使う(/score/*, /match_endedは審判の結果発行トピックなので対象外)。

操作方法(全てキーボードでも画面上のボタンのクリックでも操作できる):
    Tab    : 操作するロボットをTR<->BRで切替
    W/A/S/D: 移動(押している間だけ。方向ボタンはクリックしている間だけ)
    Shift  : 押している間だけゆっくり移動(ボタンのクリックで固定ON/OFF)
    Q / E  : つかむ対象(近い順)を前/次に切替(候補のクリックで直接選択)
    Space  : つかむ / はなす
    Z / X  : 建築スポットを前/次に切替
    1      : BRが持っているアース/スカイを選択中のスポットに置く
    2      : 選択中のスポットの一番上のスカイを裏返す
    3      : BRが持っているムスティカを中央の柱に置く
    Esc / ウィンドウを閉じる: 終了
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import pygame
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from br_msgs.msg import BuildAction, DetectedBlockArray, DetectedMustika, GripperCmd, TowerArray

from . import field_constants as fc
from . import field_drawing as fd
from .robot_spec import declare_robot_specs

CONTROL_HZ = 20.0
WINDOW_WIDTH = 680
WINDOW_HEIGHT = 700
MAX_CANDIDATE_CHIPS = 5
# ゆっくり(精密操作)モードの速度倍率
PRECISION_SPEED_SCALE = 0.4
# 単発操作のボタンを、発火後に光らせておく時間
FLASH_SEC = 0.15
# 操作結果の通知を表示しておく時間
MESSAGE_SEC = 3.0
# つかむ/はなす/置くを押してから、機構の動作時間(機体仕様)に加えて結果を待つ猶予
ACTION_CONFIRM_MARGIN_SEC = 1.5

_ROBOT_IDS = ('tr', 'br')
_ROBOT_NAMES = {'tr': '運搬ロボ TR', 'br': '建築ロボ BR'}

_COLOR_NAMES = {fc.TeamColor.RED: '赤', fc.TeamColor.BLUE: '青'}
_SPOT_TEAM_NAMES = {**_COLOR_NAMES, '': '共用'}
_TYPE_NAMES = {'earth': 'アース', 'sky': 'スカイ', 'mustika': 'ムスティカ'}

# 移動ボタン名 -> (キー, u方向, v方向, 表示)。field_drawing.mm_to_pxはvが増えるほど
# 画面上で上に描画するため、W(上)=+v, S(下)=-v, A(左)=-u, D(右)=+uとする
_MOVE_KEYS = {
    'up': (pygame.K_w, 0.0, 1.0, '↑'),
    'left': (pygame.K_a, -1.0, 0.0, '←'),
    'down': (pygame.K_s, 0.0, -1.0, '↓'),
    'right': (pygame.K_d, 1.0, 0.0, '→'),
}

# 単発操作のキー -> ボタン名
_KEY_TO_BUTTON = {
    pygame.K_SPACE: 'grip',
    pygame.K_q: 'target_prev',
    pygame.K_e: 'target_next',
    pygame.K_z: 'spot_prev',
    pygame.K_x: 'spot_next',
    pygame.K_1: 'place',
    pygame.K_2: 'flip',
    pygame.K_3: 'mustika',
}

_BG = (24, 26, 32)
_PANEL = (36, 39, 48)
_BTN = (56, 60, 72)
_BTN_DISABLED = (42, 44, 52)
_BTN_BORDER = (86, 92, 108)
_BTN_LIT = (70, 140, 230)
_BTN_HOLDING = (140, 100, 35)
_TEXT = (232, 232, 236)
_DIM = (140, 145, 158)
_FAINT = (95, 99, 110)
_GREEN = (80, 200, 120)
_MSG_INFO = (32, 62, 104)
_MSG_OK = (30, 92, 56)
_MSG_WARN = (120, 74, 22)
_TEAM_COLOR = {fc.TeamColor.RED: (225, 85, 85), fc.TeamColor.BLUE: (85, 135, 235)}


def _block_name(block_id: str, block_type: str, top_color: str = '', short: bool = False) -> str:
    if block_type == 'mustika':
        return 'ムスティカ'
    number = block_id.rsplit('_', 1)[-1].lstrip('0') or '0'
    type_name = _TYPE_NAMES.get(block_type, block_type)
    if block_type == 'earth':
        # idは'earth_<チーム色>_<番号>'
        color = _COLOR_NAMES.get(block_id.split('_')[1], '') if block_id.count('_') >= 2 else ''
        return f'{type_name}{color}{number}' if short else f'{type_name}（{color}）{number}番'
    if short:
        return f'{type_name}{number}'
    top = _COLOR_NAMES.get(top_color, '')
    return f'{type_name} {number}番（上面 {top}）' if top else f'{type_name} {number}番'


def _spot_name(spot_id: str, level: int, team: str) -> str:
    return f'L{level}・{_SPOT_TEAM_NAMES.get(team, team)}・{spot_id.rsplit("_", 1)[-1]}番'


@dataclass
class _Candidate:
    id: str
    block_type: str
    top_color: str
    dist: float
    reach: float

    @property
    def in_range(self) -> bool:
        return self.dist <= self.reach


class _Button:
    def __init__(self, rect: tuple[int, int, int, int], label: str, sub: str = '', key: str = '',
                 on_click: Callable[[], None] | None = None):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.sub = sub
        self.key = key
        self.on_click = on_click
        self.flash_until = 0.0


class BrTeleopNode(Node):
    def __init__(self):
        super().__init__('br_teleop_node')

        # pygame.init()は音声/ジョイスティック等の全サブシステムを初期化しようと
        # するが、このコンソールは表示とキー入力しか使わない。音声デバイスの
        # 無いサンドボックス環境ではオーディオ初期化が数十秒〜90秒近くブロック
        # することがあり(rclpy構築後だと発生しやすい)、必要なサブシステムのみ
        # 個別に初期化してこれを回避する。
        pygame.display.init()
        pygame.font.init()
        font_path = fd.find_japanese_font()
        if font_path is None:
            self.get_logger().warn(
                '日本語フォントが見つからないため文字化けします。'
                '`sudo apt install fonts-noto-cjk` でインストールしてください。')
        self._font_tiny = pygame.font.Font(font_path, 13)
        self._font_small = pygame.font.Font(font_path, 15)
        self._font = pygame.font.Font(font_path, 19)
        self._font_large = pygame.font.Font(font_path, 26)
        self._screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('ABU Robocon 2027 操縦コンソール')

        self._active_robot = 'tr'
        # ロボットごとに、最後に送ったグリッパー指令(開/閉)を保持する
        self._gripper_open = {'tr': True, 'br': True}
        self._quit_requested = False

        # 状態表示用(観測トピックから受け取る。/true_state/*は購読しない)
        self._pose: dict[str, PoseWithCovarianceStamped | None] = {'tr': None, 'br': None}
        self._detected_blocks = DetectedBlockArray()
        self._detected_mustika: DetectedMustika | None = None
        self._towers: dict[str, object] = {}
        self._score = {'red': 0, 'blue': 0}
        self._match_ended = False

        # つかむ対象の候補(近い順)。毎tick、操作中ロボットの位置基準で作り直す
        self._grasp_candidates: list[_Candidate] = []
        self._grasp_selection_index = 0

        # 手動建築: field_constants.BUILD_SPOTSへのインデックス
        self._build_spot_index = 0

        # 移動・ゆっくりモードの押下状態(ボタンの点灯表示にも使う)
        self._move_held = {name: False for name in _MOVE_KEYS}
        self._precise_lock = False
        self._precise_active = False

        # 操作結果の通知(文言, 背景色, 表示終了時刻)
        self._message: tuple[str, tuple[int, int, int], float] | None = None
        # 「つかむ」の結果待ち(ロボット, 対象id, 表示名, 判定期限)
        # 機構の動作の結果待ち(完了判定, 完了時の処理, 成功時の文言, 失敗時の文言, 期限, 失敗時の処理)
        self._pending: tuple | None = None
        self._specs = declare_robot_specs(self)

        self._buttons = self._create_buttons()
        # 候補チップは候補数に応じて毎回描き直すため、最後に描いた位置を覚えておく
        self._chip_rects: list[tuple[pygame.Rect, int]] = []

        self.pub_cmd_vel = {
            'tr': self.create_publisher(Twist, '/tr_cmd_vel', 10),
            'br': self.create_publisher(Twist, '/br_cmd_vel', 10),
        }
        self.pub_gripper_cmd = {
            'tr': self.create_publisher(GripperCmd, '/tr_gripper_cmd', 10),
            'br': self.create_publisher(GripperCmd, '/br_gripper_cmd', 10),
        }
        self.pub_build_action = self.create_publisher(BuildAction, '/br_build_action', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/tr_pose_estimated', self._on_tr_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/br_pose_estimated', self._on_br_pose, 10)
        self.create_subscription(DetectedBlockArray, '/detected_blocks', self._on_blocks, 10)
        self.create_subscription(DetectedMustika, '/detected_mustika', self._on_mustika, 10)
        self.create_subscription(TowerArray, '/detected_towers', self._on_towers, 10)
        self.create_subscription(Int32, '/score/red', self._on_score_red, 10)
        self.create_subscription(Int32, '/score/blue', self._on_score_blue, 10)
        self.create_subscription(Bool, '/match_ended', self._on_match_ended, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    def _create_buttons(self) -> dict[str, _Button]:
        buttons = {
            'robot_tr': _Button((16, 10, 186, 46), _ROBOT_NAMES['tr'], key='Tab',
                                on_click=lambda: self._set_active_robot('tr')),
            'robot_br': _Button((210, 10, 186, 46), _ROBOT_NAMES['br'], key='Tab',
                                on_click=lambda: self._set_active_robot('br')),
            'grip': _Button((364, 176, 284, 76), 'つかむ', key='Space', on_click=self._toggle_gripper),
            'precise': _Button((364, 260, 284, 56), 'ゆっくり移動', 'Shiftを押す間 / クリックで固定',
                               'Shift', self._toggle_precise_lock),
            'target_prev': _Button((32, 368, 56, 56), '◀', key='Q',
                                   on_click=lambda: self._cycle_grasp_selection(-1)),
            'target_next': _Button((592, 368, 56, 56), '▶', key='E',
                                   on_click=lambda: self._cycle_grasp_selection(1)),
            'spot_prev': _Button((32, 520, 56, 50), '◀', key='Z', on_click=lambda: self._cycle_build_spot(-1)),
            'spot_next': _Button((592, 520, 56, 50), '▶', key='X', on_click=lambda: self._cycle_build_spot(1)),
            'place': _Button((32, 588, 200, 62), 'ここに置く', key='1', on_click=self._place_block),
            'flip': _Button((240, 588, 200, 62), 'スカイを裏返す', key='2', on_click=self._flip_sky),
            'mustika': _Button((448, 588, 200, 62), 'ムスティカを置く', key='3', on_click=self._place_mustika),
        }
        for name, rect in (('up', (142, 176, 60, 60)), ('left', (74, 242, 60, 60)),
                           ('down', (142, 242, 60, 60)), ('right', (210, 242, 60, 60))):
            key, _, _, arrow = _MOVE_KEYS[name]
            buttons[name] = _Button(rect, arrow, key=pygame.key.name(key).upper())
        return buttons

    # --- 観測購読 ---

    def _on_tr_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose['tr'] = msg

    def _on_br_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose['br'] = msg

    def _on_blocks(self, msg: DetectedBlockArray) -> None:
        self._detected_blocks = msg

    def _on_mustika(self, msg: DetectedMustika) -> None:
        self._detected_mustika = msg

    def _on_towers(self, msg: TowerArray) -> None:
        self._towers = {t.build_spot_id: t for t in msg.towers}

    def _on_score_red(self, msg: Int32) -> None:
        self._score['red'] = msg.data

    def _on_score_blue(self, msg: Int32) -> None:
        self._score['blue'] = msg.data

    def _on_match_ended(self, msg: Bool) -> None:
        self._match_ended = self._match_ended or msg.data

    # --- 状態の読み取り ---

    def _current_uv(self, robot: str) -> tuple[float, float] | None:
        pose = self._pose[robot]
        if pose is None:
            return None
        p = pose.pose.pose.position
        return (p.x, p.y)

    def _held(self, robot: str) -> tuple[str, str, str] | None:
        """robotが実際に保持している物の(id, block_type, top_color)。"""
        for block in self._detected_blocks.blocks:
            if block.held_by == robot:
                return (block.id, block.block_type, block.top_color)
        mustika = self._detected_mustika
        if mustika is not None and mustika.held_by == robot:
            return ('mustika', 'mustika', '')
        return None

    def _held_name(self, robot: str) -> str:
        held = self._held(robot)
        return _block_name(*held) if held is not None else 'なし'

    def _update_grasp_candidates(self) -> None:
        cur = self._current_uv(self._active_robot)
        candidates: list[_Candidate] = []
        if cur is not None:
            for block in self._detected_blocks.blocks:
                # 保持中のものと、既に塔に積まれたもの(level>0)は対象外
                if block.held_by != 'none' or block.level != 0:
                    continue
                dist = math.hypot(block.position.x - cur[0], block.position.y - cur[1])
                reach = self._specs[self._active_robot].grasp_range_mm(block.block_type)
                candidates.append(_Candidate(block.id, block.block_type, block.top_color, dist, reach))
            mustika = self._detected_mustika
            if mustika is not None and mustika.held_by == 'none':
                dist = math.hypot(mustika.position.x - cur[0], mustika.position.y - cur[1])
                reach = self._specs[self._active_robot].grasp_range_mm('mustika')
                candidates.append(_Candidate('mustika', 'mustika', '', dist, reach))
        candidates.sort(key=lambda c: c.dist)
        self._grasp_candidates = candidates
        if self._grasp_candidates:
            self._grasp_selection_index %= len(self._grasp_candidates)
        else:
            self._grasp_selection_index = 0

    def _selected_candidate(self) -> _Candidate | None:
        if not self._grasp_candidates:
            return None
        return self._grasp_candidates[self._grasp_selection_index]

    def _selected_spot(self) -> tuple[str, int, str]:
        spot_id, level, team, _ = fc.BUILD_SPOTS[self._build_spot_index]
        return spot_id, level, team

    def _grip_view(self) -> tuple[str, str, bool]:
        """つかむ/はなすボタンの(表示, 補足/無効理由, 押せるか)。"""
        held = self._held(self._active_robot)
        if held is not None:
            return 'はなす', f'{_block_name(*held)}をはなす', True
        target = self._selected_candidate()
        if target is None:
            return 'つかむ', '近くに物が見えません', False
        name = _block_name(target.id, target.block_type, target.top_color)
        if target.in_range:
            return 'つかむ', name, True
        return 'つかむ', f'遠すぎます（あと約{(target.dist - target.reach) / 10:.0f}cm）', False

    def _br_too_far(self, point) -> str | None:
        """BRが置く/裏返すには遠すぎるなら理由の文言(シムの受付条件と同じ距離)。"""
        cur = self._current_uv('br')
        spec = self._specs['br']
        near_mm = spec.half_width_mm + spec.grasp_reach_mm + fc.BUILD_SPOT_SIZE[0] / 2
        if cur is None:
            return 'BRの位置が分かりません'
        dist = math.dist(cur, point)
        return f'BRが遠すぎます（あと約{(dist - near_mm) / 10:.0f}cm）' if dist > near_mm else None

    def _selected_spot_center(self) -> tuple[float, float]:
        origin = fc.BUILD_SPOTS[self._build_spot_index][3]
        return (origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)

    def _place_view(self) -> tuple[str, bool]:
        held = self._held('br')
        if held is None:
            return 'BRが何も持っていません', False
        if held[1] == 'mustika':
            return 'ムスティカは右のボタンで', False
        too_far = self._br_too_far(self._selected_spot_center())
        if too_far:
            return too_far, False
        return f'{_block_name(*held, short=True)}を置く', True

    def _flip_view(self) -> tuple[str, bool]:
        tower = self._towers.get(self._selected_spot()[0])
        if tower is None or not tower.block_types:
            return 'まだ何も置かれていません', False
        if tower.block_types[-1] != 'sky':
            return '一番上がスカイではありません', False
        too_far = self._br_too_far(self._selected_spot_center())
        if too_far:
            return too_far, False
        return f'上面 {_COLOR_NAMES.get(tower.top_colors[-1], "")} → 反対の色へ', True

    def _mustika_view(self) -> tuple[str, bool]:
        held = self._held('br')
        if held is None or held[1] != 'mustika':
            return 'BRがムスティカを持っていません', False
        too_far = self._br_too_far((fc.FIELD_CENTER, fc.FIELD_CENTER))
        if too_far:
            return too_far, False
        return '中央の柱に置く', True

    def _button_view(self, name: str) -> tuple[str, bool]:
        """ボタンの(補足/無効理由, 押せるか)。"""
        if self._pending is not None and name in ('grip', 'place', 'flip', 'mustika'):
            return '動作中です', False
        if name == 'grip':
            _, sub, enabled = self._grip_view()
            return sub, enabled
        if name == 'place':
            return self._place_view()
        if name == 'flip':
            return self._flip_view()
        if name == 'mustika':
            return self._mustika_view()
        return self._buttons[name].sub, True

    def _guidance(self) -> str:
        if self._match_ended:
            return '試合時間が終わったため操作は反映されません。シムを起動し直してください'
        if self._current_uv(self._active_robot) is None:
            return 'シミュレータの起動を待っています…'
        held = self._held(self._active_robot)
        if held is not None:
            name = _block_name(*held)
            if self._active_robot == 'tr':
                return f'{name}を運んでいます。受渡しエリアまで運んだら「はなす」'
            if held[1] == 'mustika':
                return '「ムスティカを置く」を押すと中央の柱に置きます'
            return '下の「建築」で置く場所を選び「ここに置く」を押してください'
        target = self._selected_candidate()
        if target is None:
            return '近くにつかめる物が見えません。移動して探してください'
        name = _block_name(target.id, target.block_type, target.top_color)
        if target.in_range:
            return f'「つかむ」を押すと {name} をつかみます'
        return f'{name}まで あと約{(target.dist - target.reach) / 10:.0f}cm。矢印ボタンかW/A/S/Dで近づいてください'

    # --- メインループ ---

    def _tick(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._request_quit()
            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event.key)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._handle_click(event.pos)

        if self._quit_requested:
            return

        self._update_grasp_candidates()
        self._check_pending()
        self._publish_cmd_vel()
        self._render()

    def _handle_keydown(self, key: int) -> None:
        if key == pygame.K_ESCAPE:
            self._request_quit()
        elif key == pygame.K_TAB:
            self._press('robot_br' if self._active_robot == 'tr' else 'robot_tr')
        elif key in _KEY_TO_BUTTON:
            self._press(_KEY_TO_BUTTON[key])

    def _handle_click(self, pos: tuple[int, int]) -> None:
        for name, button in self._buttons.items():
            if button.on_click is not None and button.rect.collidepoint(pos):
                self._press(name)
                return
        for rect, index in self._chip_rects:
            if rect.collidepoint(pos) and index < len(self._grasp_candidates):
                self._grasp_selection_index = index
                return

    def _press(self, name: str) -> None:
        button = self._buttons[name]
        button.flash_until = time.monotonic() + FLASH_SEC
        reason, enabled = self._button_view(name)
        if not enabled:
            self._notify(reason, _MSG_WARN)
            return
        if button.on_click is not None:
            button.on_click()

    def _notify(self, text: str, color: tuple[int, int, int]) -> None:
        self._message = (text, color, time.monotonic() + MESSAGE_SEC)

    def _set_active_robot(self, robot: str) -> None:
        if robot != self._active_robot:
            self._active_robot = robot
            self._grasp_selection_index = 0
            self._notify(f'{_ROBOT_NAMES[robot]} を操作します', _MSG_INFO)

    def _cycle_grasp_selection(self, direction: int) -> None:
        if not self._grasp_candidates:
            return
        self._grasp_selection_index = (self._grasp_selection_index + direction) % len(self._grasp_candidates)

    def _cycle_build_spot(self, direction: int) -> None:
        self._build_spot_index = (self._build_spot_index + direction) % len(fc.BUILD_SPOTS)

    def _toggle_precise_lock(self) -> None:
        self._precise_lock = not self._precise_lock
        self._notify('ゆっくり移動を固定しました' if self._precise_lock else 'ゆっくり移動の固定を解除しました',
                     _MSG_INFO)

    def _send_gripper(self, robot: str, open_: bool, target_id: str = '') -> None:
        self._gripper_open[robot] = open_
        self.pub_gripper_cmd[robot].publish(GripperCmd(
            open=open_, target_force=0.0 if open_ else 1.0, target_block_id=target_id))

    def _start_pending(self, done, on_done, ok_msg: str, fail_msg: str, wait_s: float, on_fail=None) -> None:
        self._pending = (done, on_done, ok_msg, fail_msg, time.monotonic() + wait_s + ACTION_CONFIRM_MARGIN_SEC, on_fail)

    def _check_pending(self) -> None:
        if self._pending is None:
            return
        done, on_done, ok_msg, fail_msg, deadline, on_fail = self._pending
        if done():
            self._pending = None
            if on_done:
                on_done()
            self._notify(ok_msg, _MSG_OK)
        elif time.monotonic() > deadline:
            self._pending = None
            if on_fail:
                on_fail()
            self._notify(fail_msg, _MSG_WARN)

    def _toggle_gripper(self) -> None:
        robot = self._active_robot
        spec = self._specs[robot]
        held = self._held(robot)
        if held is not None:
            name = _block_name(*held)
            self._send_gripper(robot, True)
            self._notify(f'{name}をはなしています…', _MSG_INFO)
            self._start_pending(lambda: self._held(robot) is None, None, f'{name}をはなしました',
                                f'{name}をはなせませんでした', spec.release_time_s)
            return
        target = self._selected_candidate()
        name = _block_name(target.id, target.block_type, target.top_color)
        self._send_gripper(robot, False, target.id)
        self._notify(f'{name}をつかんでいます…', _MSG_INFO)

        def grasped() -> bool:
            held_now = self._held(robot)
            return held_now is not None and held_now[0] == target.id

        # 失敗時は開き直す(閉じたままだと後でレンジに入った瞬間に勝手につかむため)
        self._start_pending(grasped, None, f'{name}をつかみました',
                            f'{name}をつかめませんでした。もう少し近づいてから押してください',
                            spec.grasp_time_s, on_fail=lambda: self._send_gripper(robot, True))

    def _send_build(self, action_type: str) -> None:
        self.pub_build_action.publish(
            BuildAction(action_type=action_type, target_build_spot_id=self._selected_spot()[0]))

    def _place_block(self) -> None:
        _, block_type, _ = held = self._held('br')
        name = _block_name(*held)
        spot = _spot_name(*self._selected_spot())
        # 置く動作が終わるまでグリッパーは閉じたまま(シム側も設置中の開指令は無視する)。
        # 置けたら開く(設置済みの物は再びつかまない)
        self._send_build('PLACE_SKY_BLOCK' if block_type == 'sky' else 'PLACE_EARTH_BLOCK')
        self._notify(f'{name}を {spot} に置いています…', _MSG_INFO)
        self._start_pending(lambda: self._held('br') is None, lambda: self._send_gripper('br', True),
                            f'{name}を {spot} に置きました', f'{name}を置けませんでした',
                            self._specs['br'].place_time_s)

    def _flip_sky(self) -> None:
        spot_id = self._selected_spot()[0]
        spot = _spot_name(*self._selected_spot())
        before = self._towers[spot_id].top_colors[-1]
        self._send_build('FLIP_SKY_BLOCK')
        self._notify(f'{spot} のスカイを裏返しています…', _MSG_INFO)
        self._start_pending(lambda: self._towers[spot_id].top_colors[-1] != before, None,
                            f'{spot} のスカイを裏返しました', 'スカイを裏返せませんでした',
                            self._specs['br'].flip_time_s)

    def _place_mustika(self) -> None:
        self._send_build('PLACE_MUSTIKA')
        self._notify('ムスティカを中央の柱に置いています…', _MSG_INFO)
        self._start_pending(lambda: self._held('br') is None, lambda: self._send_gripper('br', True),
                            'ムスティカを中央の柱に置きました', 'ムスティカを置けませんでした',
                            self._specs['br'].place_time_s)

    def _publish_cmd_vel(self) -> None:
        keys = pygame.key.get_pressed()
        mouse_down = pygame.mouse.get_pressed()[0]
        mouse_pos = pygame.mouse.get_pos()

        # 斜め移動(例: W+D)がまっすぐより速くならないよう正規化する。
        dir_x = 0.0
        dir_y = 0.0
        for name, (key, du, dv, _) in _MOVE_KEYS.items():
            held = keys[key] or (mouse_down and self._buttons[name].rect.collidepoint(mouse_pos))
            self._move_held[name] = held
            if held:
                dir_x += du
                dir_y += dv
        self._precise_active = self._precise_lock or keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        speed = self._specs[self._active_robot].max_speed_m_s * (
            PRECISION_SPEED_SCALE if self._precise_active else 1.0)

        norm = math.hypot(dir_x, dir_y)
        vx = dir_x / norm * speed if norm > 0.0 else 0.0
        vy = dir_y / norm * speed if norm > 0.0 else 0.0

        for robot in _ROBOT_IDS:
            twist = Twist()
            if robot == self._active_robot:
                twist.linear.x = vx
                twist.linear.y = vy
            self.pub_cmd_vel[robot].publish(twist)

    def _request_quit(self) -> None:
        # rclpy.spin(node)を使う標準的な形に合わせるため(下記main参照)、
        # ウィンドウを閉じる/Escで終了したい場合はrclpy.shutdown()を直接
        # 呼んでspin()自体を抜けさせる(rclpy.ok()がFalseになりループが終わる)。
        self._quit_requested = True
        if rclpy.ok():
            rclpy.shutdown()

    # --- 描画 ---

    def _text(self, text: str, font: pygame.font.Font, color: tuple[int, int, int],
              pos: tuple[int, int], anchor: str = 'topleft') -> None:
        img = font.render(text, True, color)
        self._screen.blit(img, img.get_rect(**{anchor: pos}))

    def _draw_button(self, button: _Button, now: float, lit: bool = False, enabled: bool = True,
                     label: str | None = None, sub: str | None = None,
                     base: tuple[int, int, int] | None = None) -> None:
        lit = enabled and (lit or button.flash_until > now)
        if lit:
            color = _BTN_LIT
        elif not enabled:
            color = _BTN_DISABLED
        else:
            color = base or _BTN
        pygame.draw.rect(self._screen, color, button.rect, border_radius=8)
        pygame.draw.rect(self._screen, _BTN_BORDER if enabled else _BTN_DISABLED, button.rect,
                         width=2, border_radius=8)
        label = button.label if label is None else label
        sub = button.sub if sub is None else sub
        text_color = _TEXT if enabled else _FAINT
        cx, cy = button.rect.center
        if sub:
            self._text(label, self._font, text_color, (cx, cy - 10), 'center')
            self._text(sub, self._font_tiny, _TEXT if lit else _DIM, (cx, cy + 14), 'center')
        else:
            self._text(label, self._font_large, text_color, (cx, cy), 'center')
        if button.key:
            self._text(button.key, self._font_tiny, _TEXT if lit else _FAINT,
                       (button.rect.right - 6, button.rect.top + 3), 'topright')

    def _draw_panel(self, rect: tuple[int, int, int, int], title: str) -> None:
        pygame.draw.rect(self._screen, _PANEL, rect, border_radius=10)
        self._text(title, self._font_small, _DIM, (rect[0] + 14, rect[1] + 7))

    def _render(self) -> None:
        now = time.monotonic()
        self._screen.fill(_BG)

        # ヘッダ: 操作するロボットの切替ボタンと得点
        for robot in _ROBOT_IDS:
            active = robot == self._active_robot
            self._draw_button(self._buttons[f'robot_{robot}'], now, lit=active,
                              sub='操作中' if active else 'クリックで切替')
        self._text(f'赤 {self._score["red"]}点', self._font_large, _TEAM_COLOR[fc.TeamColor.RED],
                   (530, 33), 'midright')
        self._text(f'青 {self._score["blue"]}点', self._font_large, _TEAM_COLOR[fc.TeamColor.BLUE],
                   (664, 33), 'midright')

        # 案内バー: 操作結果の通知があればそれを、無ければ「次にすること」を出す
        if self._message is not None and self._message[2] > now:
            text, color = self._message[0], self._message[1]
        else:
            text, color = self._guidance(), (_MSG_WARN if self._match_ended else _PANEL)
        bar = pygame.Rect(16, 64, 648, 48)
        pygame.draw.rect(self._screen, color, bar, border_radius=10)
        self._text(text, self._font, _TEXT, (bar.x + 16, bar.centery), 'midleft')

        cur = self._current_uv(self._active_robot)
        pos_text = f'({cur[0] / 1000:.2f}m, {cur[1] / 1000:.2f}m)' if cur is not None else '（待機中）'
        self._text(f'位置 {pos_text}', self._font_small, _DIM, (18, 124))
        self._text(f'持っている物: {self._held_name(self._active_robot)}', self._font_small, _TEXT, (300, 124))

        # 移動パッド / つかむ・ゆっくり
        self._draw_panel((16, 150, 316, 180), '移動（俯瞰画面の上下左右）')
        for name in _MOVE_KEYS:
            self._draw_button(self._buttons[name], now, lit=self._move_held[name])
        self._draw_panel((348, 150, 316, 180), '操作')
        grip_label, grip_sub, grip_enabled = self._grip_view()
        holding = self._held(self._active_robot) is not None
        self._draw_button(self._buttons['grip'], now, enabled=grip_enabled, label=grip_label, sub=grip_sub,
                          base=_BTN_HOLDING if holding else None)
        self._draw_button(self._buttons['precise'], now, lit=self._precise_active,
                          label='ゆっくり移動（固定中）' if self._precise_lock else 'ゆっくり移動')

        # つかむ対象の選択
        self._draw_panel((16, 342, 648, 140), 'つかむ対象を選ぶ（近い順）')
        self._draw_button(self._buttons['target_prev'], now)
        self._draw_button(self._buttons['target_next'], now)
        target = self._selected_candidate()
        if target is not None:
            self._text(_block_name(target.id, target.block_type, target.top_color), self._font_large, _TEXT,
                       (100, 370))
            self._text(f'距離 約{target.dist / 10:.0f}cm', self._font_small, _DIM, (102, 404))
            badge = pygame.Rect(440, 378, 140, 34)
            pygame.draw.rect(self._screen, _GREEN if target.in_range else _BTN, badge, border_radius=17)
            self._text('つかめる距離' if target.in_range else '遠い', self._font_small,
                       _BG if target.in_range else _DIM, badge.center, 'center')
        else:
            self._text('近くに物が見えません', self._font, _DIM, (100, 384))

        self._chip_rects = []
        for i, cand in enumerate(self._grasp_candidates[:MAX_CANDIDATE_CHIPS]):
            rect = pygame.Rect(32 + i * 125, 436, 116, 34)
            selected = i == self._grasp_selection_index
            pygame.draw.rect(self._screen, _BTN_LIT if selected else _BTN, rect, border_radius=6)
            label = f'{_block_name(cand.id, cand.block_type, short=True)} {cand.dist / 10:.0f}cm'
            self._text(label, self._font_tiny, _GREEN if cand.in_range and not selected else _TEXT,
                       rect.center, 'center')
            self._chip_rects.append((rect, i))

        # 建築スポットの選択と建築操作
        self._draw_panel((16, 494, 648, 170), '建築（建築ロボBRが行います）')
        self._draw_button(self._buttons['spot_prev'], now)
        self._draw_button(self._buttons['spot_next'], now)
        spot_id, level, team = self._selected_spot()
        self._text(f'置く場所: {_spot_name(spot_id, level, team)}', self._font_large,
                   _TEAM_COLOR.get(team, _TEXT), (340, 532), 'center')
        tower = self._towers.get(spot_id)
        if tower is not None and tower.block_types:
            layers = ' → '.join(
                _TYPE_NAMES.get(t, t) + (f'({_COLOR_NAMES.get(c, "")})' if t == 'sky' else '')
                for t, c in zip(tower.block_types, tower.top_colors))
            tower_text = f'いま積まれている物（下から）: {layers}'
        else:
            tower_text = 'いま積まれている物: なし'
        self._text(tower_text, self._font_tiny, _DIM, (340, 560), 'center')
        for name in ('place', 'flip', 'mustika'):
            sub, enabled = self._button_view(name)
            self._draw_button(self._buttons[name], now, enabled=enabled, sub=sub)

        self._text('Escで終了　／　キーボードでもマウスでも操作できます（ボタン右上がキー）',
                   self._font_tiny, _DIM, (WINDOW_WIDTH // 2, 682), 'center')

        pygame.display.flip()

    def should_quit(self) -> bool:
        return self._quit_requested


def main(args=None):
    rclpy.init(args=args)
    node = BrTeleopNode()
    try:
        # 他のノード(br_visualizer_node等)と同じくrclpy.spin(node)を使う。
        # 独自のspin_onceループ(timeout_sec=0.05)を_tickのタイマー周期
        # (1/CONTROL_HZ=0.05s)と偶然一致させていた旧実装は、タイミングが
        # ぴったり噛み合わずタイマーが安定して発火しない場合があった
        # (WASD操作が反応しないという報告で発覚)。ウィンドウを閉じる/Escでの
        # 終了は_request_quitが直接rclpy.shutdown()を呼ぶことで実現する。
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
