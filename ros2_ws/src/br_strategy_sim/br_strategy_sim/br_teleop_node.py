"""
ABU Robocon 2027 Phase 1 2Dシム: 手動操縦コンソール(br_teleop_node)。

意思決定ノード(br_decision_tr_node/br_decision_br_node)を起動しない状態
(`enable_decision:=false`)で、TR/BRを手動でキーボード操作するためのデバッグ用
ノード。br_visualizer_nodeと同じく独立したpygameウィンドウを持つ(別プロセス
として起動されるため自然に別ウィンドウになる)。可視化ウィンドウ(俯瞰の
グラフィカル表示)とは役割を分け、このウィンドウはテキストHUDでの詳細な
状態表示(自機位置・保持ブロック・把持候補・得点)に専念する。

トピック契約(docs/topic_contract.md)に定義済みの/tr_cmd_vel, /br_cmd_vel,
/tr_gripper_cmd, /br_gripper_cmd, /br_build_actionへpublishするだけで、
sim_bridge_node側の変更は不要(「ロボットへの指令は単一トピックで受ける」
設計のおかげで、意思決定ノードの代わりにこのノードが同じトピックへ
publishすれば手動操縦に差し替えられる)。状態表示のための購読は、意思決定
ノードと同じ規約(/true_state/*は購読禁止、観測トピックのみ)に従い
/tr_pose_estimated, /br_pose_estimated, /detected_blocks, /detected_mustika
を使う(/score/*は結果の発行専用トピックなので対象外)。

操作方法:
    Tab    : 操作対象をTR<->BRで切替
    W/A/S/D: 対象ロボットをフィールド座標(u,v)で移動(押している間だけ、
             斜め移動は正規化済み)
    Shift  : 押している間だけ低速(精密操作)モード
    Q / E  : 把持対象候補(距離順)の選択を前/次に切替
    Space  : 選択中の対象を把持 / 保持中のものを解放(トグル)
    Z / X  : 建築スポット(field_constants.BUILD_SPOTS)の選択を前/次に切替
    1/2/3/4: 選択中の建築スポットへBuildActionを送信
             (PLACE_EARTH_BLOCK/PLACE_SKY_BLOCK/FLIP_SKY_BLOCK/PLACE_MUSTIKA)
    Esc / ウィンドウを閉じる: 終了

BuildActionの実行可否(BRが実際に何か保持しているか等)はsim_bridge_node側が
安全にno-opする設計なので、ここでは「BRがアクティブかどうか」のガードは
行わない(誤操作しても実害がない)。
"""

from __future__ import annotations

import math

import pygame
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Int32

from br_msgs.msg import BuildAction, DetectedBlockArray, DetectedMustika, GripperCmd

from . import field_constants as fc
from .decision_common import CRUISE_SPEED_M_S, block_arrival_threshold_mm

CONTROL_HZ = 20.0
WINDOW_WIDTH = 520
WINDOW_HEIGHT = 460
MAX_CANDIDATES_SHOWN = 5
# Shift押下中の低速(精密操作)モードの速度倍率
PRECISION_SPEED_SCALE = 0.4

_ROBOT_IDS = ('tr', 'br')

# 距離->把持レンジ判定に使うブロック半径(decision_common.block_arrival_threshold_mmと
# 同じ考え方。'mustika'はDetectedBlockArrayには含まれない別トピックのため
# ここで個別に半径を持たせて候補リストへ合流させる)
_BLOCK_HALF_SIZE_MM = {
    'earth': fc.EARTH_BLOCK_SIZE / 2,
    'sky': fc.SKY_BLOCK_SIZE / 2,
    'mustika': fc.MUSTIKA_DIAMETER / 2,
}

_BUILD_ACTION_KEYS = {
    pygame.K_1: 'PLACE_EARTH_BLOCK',
    pygame.K_2: 'PLACE_SKY_BLOCK',
    pygame.K_3: 'FLIP_SKY_BLOCK',
    pygame.K_4: 'PLACE_MUSTIKA',
}
_BUILD_ACTION_LABELS = {
    'PLACE_EARTH_BLOCK': '1:PlaceEarth',
    'PLACE_SKY_BLOCK': '2:PlaceSky',
    'FLIP_SKY_BLOCK': '3:FlipSky',
    'PLACE_MUSTIKA': '4:PlaceMustika',
}


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
        self._font = pygame.font.SysFont(None, 20)
        self._screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('ABU Robocon 2027 BR Teleop Console')

        self._active_robot = 'tr'
        # ロボットごとに開閉状態を独立に保持する(Tabで切り替えても
        # 相手側の状態が変わらないようにするため)
        self._gripper_open = {'tr': True, 'br': True}
        self._quit_requested = False

        # 状態表示用(観測トピックから受け取る。/true_state/*は購読しない)
        self._pose: dict[str, PoseWithCovarianceStamped | None] = {'tr': None, 'br': None}
        self._detected_blocks = DetectedBlockArray()
        self._detected_mustika: DetectedMustika | None = None
        self._score = {'red': 0, 'blue': 0}

        # 把持のねらい撃ち: (id, block_type, distance_mm, in_range)を
        # 距離順に並べたもの。毎tick、アクティブ側ロボットの位置基準で作り直す
        self._grasp_candidates: list[tuple[str, str, float, bool]] = []
        self._grasp_selection_index = 0

        # 手動建築: field_constants.BUILD_SPOTSへのインデックス
        self._build_spot_index = 0

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
        self.create_subscription(Int32, '/score/red', self._on_score_red, 10)
        self.create_subscription(Int32, '/score/blue', self._on_score_blue, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    # --- 観測購読 ---

    def _on_tr_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose['tr'] = msg

    def _on_br_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose['br'] = msg

    def _on_blocks(self, msg: DetectedBlockArray) -> None:
        self._detected_blocks = msg

    def _on_mustika(self, msg: DetectedMustika) -> None:
        self._detected_mustika = msg

    def _on_score_red(self, msg: Int32) -> None:
        self._score['red'] = msg.data

    def _on_score_blue(self, msg: Int32) -> None:
        self._score['blue'] = msg.data

    def _current_uv(self, robot: str) -> tuple[float, float] | None:
        pose = self._pose[robot]
        if pose is None:
            return None
        p = pose.pose.pose.position
        return (p.x, p.y)

    # --- 把持候補 ---

    def _update_grasp_candidates(self) -> None:
        cur = self._current_uv(self._active_robot)
        candidates: list[tuple[str, str, float, bool]] = []
        if cur is not None:
            for block in self._detected_blocks.blocks:
                if block.held_by != 'none':
                    continue
                dist = math.hypot(block.position.x - cur[0], block.position.y - cur[1])
                half = _BLOCK_HALF_SIZE_MM.get(block.block_type, 0.0)
                candidates.append((block.id, block.block_type, dist, dist <= block_arrival_threshold_mm(half)))
            mustika = self._detected_mustika
            if mustika is not None and mustika.held_by == 'none':
                dist = math.hypot(mustika.position.x - cur[0], mustika.position.y - cur[1])
                in_range = dist <= block_arrival_threshold_mm(_BLOCK_HALF_SIZE_MM['mustika'])
                candidates.append(('mustika', 'mustika', dist, in_range))
        candidates.sort(key=lambda c: c[2])
        self._grasp_candidates = candidates
        if self._grasp_candidates:
            self._grasp_selection_index %= len(self._grasp_candidates)
        else:
            self._grasp_selection_index = 0

    def _selected_grasp_target_id(self) -> str | None:
        if not self._grasp_candidates:
            return None
        return self._grasp_candidates[self._grasp_selection_index][0]

    def _held_block_label(self, robot: str) -> str:
        for block in self._detected_blocks.blocks:
            if block.held_by == robot:
                return f'{block.id} ({block.block_type}/{block.top_color or "-"})'
        mustika = self._detected_mustika
        if mustika is not None and mustika.held_by == robot:
            return 'mustika'
        return 'none'

    # --- メインループ ---

    def _tick(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._request_quit()
            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event.key)

        if self._quit_requested:
            return

        self._update_grasp_candidates()
        self._publish_cmd_vel()
        self._render()

    def _handle_keydown(self, key: int) -> None:
        if key == pygame.K_ESCAPE:
            self._request_quit()
        elif key == pygame.K_TAB:
            self._active_robot = 'br' if self._active_robot == 'tr' else 'tr'
            self._grasp_selection_index = 0
        elif key == pygame.K_SPACE:
            self._toggle_gripper()
        elif key == pygame.K_q:
            self._cycle_grasp_selection(-1)
        elif key == pygame.K_e:
            self._cycle_grasp_selection(1)
        elif key == pygame.K_z:
            self._build_spot_index = (self._build_spot_index - 1) % len(fc.BUILD_SPOTS)
        elif key == pygame.K_x:
            self._build_spot_index = (self._build_spot_index + 1) % len(fc.BUILD_SPOTS)
        elif key in _BUILD_ACTION_KEYS:
            self._send_build_action(_BUILD_ACTION_KEYS[key])

    def _cycle_grasp_selection(self, direction: int) -> None:
        if not self._grasp_candidates:
            return
        self._grasp_selection_index = (self._grasp_selection_index + direction) % len(self._grasp_candidates)

    def _toggle_gripper(self) -> None:
        robot = self._active_robot
        self._gripper_open[robot] = not self._gripper_open[robot]
        # 閉じる場合のみQ/Eで選択中の対象idを乗せる(開ける場合は不要)。
        # 未選択(候補が無い)なら従来通り空文字で「レンジ内最近傍」に
        # フォールバックさせる。
        target_id = '' if self._gripper_open[robot] else (self._selected_grasp_target_id() or '')
        self.pub_gripper_cmd[robot].publish(GripperCmd(
            open=self._gripper_open[robot],
            target_force=0.0 if self._gripper_open[robot] else 1.0,
            target_block_id=target_id,
        ))

    def _send_build_action(self, action_type: str) -> None:
        spot_id = fc.BUILD_SPOTS[self._build_spot_index][0]
        self.pub_build_action.publish(BuildAction(action_type=action_type, target_build_spot_id=spot_id))

    def _publish_cmd_vel(self) -> None:
        keys = pygame.key.get_pressed()
        speed = CRUISE_SPEED_M_S
        if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
            speed *= PRECISION_SPEED_SCALE

        # W/A/S/Dで押した方向にフィールド座標(u,v)で移動する。
        # field_drawing.mm_to_pxはvが増えるほど画面上で上に描画するため、
        # W(画面上で上)=+v, S(下)=-v, A(左)=-u, D(右)=+uとする。
        # 斜め移動(例: W+D)がまっすぐより速くならないよう正規化する。
        dir_x = 0.0
        dir_y = 0.0
        if keys[pygame.K_d]:
            dir_x += 1.0
        if keys[pygame.K_a]:
            dir_x -= 1.0
        if keys[pygame.K_w]:
            dir_y += 1.0
        if keys[pygame.K_s]:
            dir_y -= 1.0
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

    def _render(self) -> None:
        # pygame.font.SysFont(None, ...)の既定フォントは日本語グリフを
        # 持たないため、画面表示は英語のみにする(field_drawing.pyの
        # 画面表示が全て英語なのも同じ理由。文字化けの原因になっていた)。
        self._screen.fill((30, 30, 35))

        cur = self._current_uv(self._active_robot)
        pos_text = f'({cur[0]:.0f}, {cur[1]:.0f})' if cur is not None else '(waiting for pose...)'

        lines = [
            f'ACTIVE: {self._active_robot.upper()}   (Tab to switch)',
            f'POS: {pos_text}    HELD: {self._held_block_label(self._active_robot)}',
            '',
            'Grasp target (Q/E select, Space grasp/release):',
        ]
        if not self._grasp_candidates:
            lines.append('  (none detected)')
        for i, (block_id, block_type, dist, in_range) in enumerate(self._grasp_candidates[:MAX_CANDIDATES_SHOWN]):
            marker = '> ' if i == self._grasp_selection_index else '  '
            range_text = '[in range]' if in_range else '[out of range]'
            lines.append(f'{marker}{block_id:14s} {block_type:8s} d={dist:.0f}mm  {range_text}')

        lines.append('')
        spot_id = fc.BUILD_SPOTS[self._build_spot_index][0]
        lines.append(f'Build spot (Z/X select): {spot_id}')
        lines.append('  ' + '  '.join(_BUILD_ACTION_LABELS[a] for a in _BUILD_ACTION_KEYS.values()))
        lines.append('')

        for robot in _ROBOT_IDS:
            state = 'OPEN' if self._gripper_open[robot] else 'CLOSED'
            marker = '>> ' if robot == self._active_robot else '   '
            lines.append(f'{marker}{robot.upper()}  gripper: {state}')

        lines.append('')
        lines.append('W/A/S/D: move (Shift=precise)    Esc: quit')
        lines.append(f'SCORE  RED:{self._score["red"]}  BLUE:{self._score["blue"]}')

        y = 14
        for line in lines:
            img = self._font.render(line, True, (230, 230, 230))
            self._screen.blit(img, (16, y))
            y += 22

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
