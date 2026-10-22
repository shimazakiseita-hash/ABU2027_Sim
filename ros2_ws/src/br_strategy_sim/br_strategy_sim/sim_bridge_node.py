"""
ABU Robocon 2027 Phase 1 2Dシム: ROS2ブリッジノード。

/br_cmd_vel, /tr_cmd_vel (geometry_msgs/Twist) を購読し、pymunkのBR/TR
bodyの速度に反映する。Twistの単位はROS標準(m/s, rad/s)のまま受け取り、
シム内部の単位(mm)に変換してbodyへ渡す(フィールド座標系(u,v)での速度
指令としてそのまま扱う簡略化。ロボット自身の向きに対する相対速度への
変換はしない。実機の足回り座標変換はbr_hw_bridge側の責務なので、シムの
ブリッジはトピック契約のインターフェースを満たせば十分という判断)。

起動時にストレージエリアのアースブロック20個・グラウンド共用エリアの
スカイブロック12個・ムスティカ1個をfield_constants.pyの座標でspawnし、
物理シミュレーションの真値を/true_state/*(RobotPose, BlockArray,
MustikaPose, TowerArray)として発行する(真値の発行元はこのノード。
br_referee_node/br_observation_nodeはこれを購読する)。

/br_gripper_cmd, /tr_gripper_cmd を購読し、機体仕様(config/robot_spec.yaml)の
把持時間・解放時間をかけてheld_byを判定する(_update_grasping参照)。
/br_build_action を購読し、機体仕様の動作時間をかけて(_update_build_job参照)、
PLACE_EARTH_BLOCK/PLACE_SKY_BLOCKならBRが現在
保持しているブロックを対象の建築スポットへ実際に配置し(位置・levelを更新、
held_by="none"・placed=True)、FLIP_SKY_BLOCKなら既に設置済みの最上段
スカイブロックの上面色を反転する(ルールブック3.5.8のタワー"stealing")。
いずれも/true_state/tower_stateへ反映する。

ロボットの加減速・旋回の上限と、柵(field_geometry.WALL_SEGMENTS)・地形による
階層と速度倍率は、物理ステップごとに反映する(_step_physics, robot_body.py参照)。

重要な前提: heldブロックはKINEMATICに切り替えて直接追従させる設計
(physics_blocks.py参照)なので、壁や他ブロックとの物理衝突をすり抜けられる。
区域侵犯・押出し違反はbr_referee_nodeが/true_state座標を見て判定する
前提であり、pymunkの衝突コールバックには依存しない。TRのRobotBodyにも
BRと同じlevel_filterを適用するが、この前提(violation判定をpymunk衝突
ベースにしない)を崩さないこと。

6.4場外の物理的な処理(アース/スカイブロックの遊技からの除外、ムスティカの
開始位置への復帰)は、真値そのものを所有するこのノードが担う
(_enforce_out_of_bounds参照)。違反の記録(Violation発行)自体はこれまで通り
br_referee_nodeの責務で、/true_state/blocksから該当ブロックが消えたことを
もって検出する(このノードから審判ノードへの逆方向のトピックは設けない)。

9.4.1「ブザーが鳴ったら直ちにロボットを停止する」への対応として、
br_referee_nodeが発行する/match_ended(std_msgs/Bool)を購読し、受信後は
cmd_vel/gripper_cmd/build_actionの処理を一切行わずTR/BRの速度を強制的に
ゼロに固定する(_step_physics参照)。得点自体はbr_referee_node側で
ブザーの瞬間の状態を使って確定済みのため、このノードが停止するタイミングが
多少ずれても最終得点には影響しない。
"""

from __future__ import annotations

import math

import pymunk
import rclpy
from geometry_msgs.msg import Point, Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from br_msgs.msg import Block, BlockArray, BuildAction, GripperCmd, MustikaPose, RobotPose, TowerArray, TowerState

from . import field_constants as fc
from . import field_geometry as fg
from .physics_blocks import (
    EVER_HELD_BLOCK_COLLISION_TYPE,
    LEVEL_GROUND,
    LEVEL_L2,
    ROBOT_COLLISION_TYPE,
    PhysicsBlock,
    make_earth_block,
    make_mustika,
    make_sky_block,
)
from .robot_body import M_TO_MM, RobotBody, add_static_field, make_robot_body
from .robot_spec import declare_robot_specs
from .steady_clock import steady_clock

PHYSICS_HZ = 60.0


# ストレージのアースブロック配置: 2D物理では実寸(350mm角)通りの2段積みを
# 表現できないため、1段に並べて配置する簡略化(要調整)。ピッチは350mm角の
# ブロック同士が余裕を持って離れるように設定する(狭すぎるとロボットが
# 突っ込んだ際に隣接ブロック同士が強く押し合い、pymunkの衝突解決が
# 数値的に不安定になる=NaN化することが統合テストで判明したため。380mmで
# 発生・550〜600mmで安定を確認済み。500mmは両者の間だが、ブロック辺同士の
# 隙間(500-350=150mm)は380mm時の隙間(30mm)よりずっと広いため安全に
# 倒せる値として採用)。
# ストレージエリアはフィールド外周(V=11000)の辺に接しているため、格子はその辺から
# フィールドの内側(-V方向)へ向かって並べる(+V方向に並べるとフィールド外にはみ出し、
# 開始直後に6.4場外として取り除かれてしまう)。5列×4行(20個ちょうど)で、U方向は
# ストレージの長辺2000mmにほぼ収まり、V方向は奥行き1000mmから少しはみ出す
# (2Dシムでは積み重ね(最大2段)を表現できないため、平置きで並べる)。
STORAGE_BLOCK_PITCH_U_MM = 500.0
STORAGE_BLOCK_PITCH_V_MM = 500.0
STORAGE_BLOCK_COLS = 5


def earth_block_layout() -> list[tuple[str, tuple[float, float]]]:
    """ストレージのアースブロックの(id, 初期位置)。2D/3Dシムで共有する。"""
    layout = []
    for i in range(fc.STORAGE_AREA_EARTH_BLOCK_COUNT):
        col = i % STORAGE_BLOCK_COLS
        row = i // STORAGE_BLOCK_COLS
        u = fc.STORAGE_AREA_ORIGIN[0] + fc.EARTH_BLOCK_SIZE / 2 + col * STORAGE_BLOCK_PITCH_U_MM
        v = (fc.STORAGE_AREA_ORIGIN[1] + fc.STORAGE_AREA_SIZE[1]
             - fc.EARTH_BLOCK_SIZE / 2 - row * STORAGE_BLOCK_PITCH_V_MM)
        layout.append((f'earth_red_{i:02d}', (u, v)))
    return layout


def _spawn_storage_earth_blocks(space: pymunk.Space) -> list[PhysicsBlock]:
    return [make_earth_block(space, block_id, uv, LEVEL_GROUND, fc.TeamColor.RED)
            for block_id, uv in earth_block_layout()]


def sky_block_layout() -> list[tuple[str, tuple[float, float], str]]:
    """
    ルール4.1.4: 列位置(C1〜C5)を基準に中央列(C3)に対して鏡映対称に配色する。
    「列cが赤上向きなら鏡像列(6-c)は青上向き。中央列(C3)の2個は赤1個・青1個」。
    ここではC1・C2側を赤、C4・C5側を青とする(列cとその鏡像列(6-c)が逆色になれば
    ルールを満たすため、どちらの半分を赤にするかはルール上未指定・実装上の選択)。
    C3の2個は出現順に赤/青を1個ずつ割り当てる。
    共用エリアのスカイブロックの(id, 初期位置, 上面の色)。2D/3Dシムで共有する。
    """
    layout = []
    rows = fc.SKY_BLOCK_GRID_ROWS
    cols = fc.SKY_BLOCK_GRID_COLS
    center_col = cols // 2  # 0-indexed。cols=5ならC3に相当
    cell_u = fc.GROUND_SHARED_AREA_SIZE[0] / cols
    cell_v = fc.GROUND_SHARED_AREA_SIZE[1] / rows
    index = 0
    center_col_next_is_red = True
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                continue  # 中央マスを含む偶数パリティは使わない(市松状に12個配置)
            u = fc.GROUND_SHARED_AREA_ORIGIN[0] + cell_u * col + cell_u / 2
            v = fc.GROUND_SHARED_AREA_ORIGIN[1] + cell_v * row + cell_v / 2
            if col == center_col:
                top_color = fc.TeamColor.RED if center_col_next_is_red else fc.TeamColor.BLUE
                center_col_next_is_red = not center_col_next_is_red
            elif col < center_col:
                top_color = fc.TeamColor.RED
            else:
                top_color = fc.TeamColor.BLUE
            layout.append((f'sky_{index:02d}', (u, v), top_color))
            index += 1
    return layout


def _spawn_shared_sky_blocks(space: pymunk.Space) -> list[PhysicsBlock]:
    return [make_sky_block(space, block_id, uv, LEVEL_GROUND, color) for block_id, uv, color in sky_block_layout()]


def _yaw_from_body(body: pymunk.Body) -> float:
    return body.angle


def _build_spot_by_id(build_spot_id: str):
    return next((s for s in fc.BUILD_SPOTS if s[0] == build_spot_id), None)


def _in_field_bounds(position: pymunk.Vec2d) -> bool:
    """6.4場外判定: フィールド(0〜GAME_FIELD_SIZE mm四方)の内側かどうか。"""
    return 0.0 <= position.x <= fc.GAME_FIELD_SIZE and 0.0 <= position.y <= fc.GAME_FIELD_SIZE


def _ignore_ever_held_block_vs_robot(arbiter: pymunk.Arbiter, space: pymunk.Space, data) -> None:
    # 一度でも保持されたブロックはTR/BRどちらとも以後物理衝突しない
    # (PhysicsBlock.set_held_by参照。解放直後の位置重複による爆発的な
    # 分離速度を避けるための恒久的な措置)
    arbiter.process_collision = False


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('br_sim_bridge_node')

        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.space.on_collision(
            ROBOT_COLLISION_TYPE, EVER_HELD_BLOCK_COLLISION_TYPE,
            pre_solve=_ignore_ever_held_block_vs_robot,
        )

        self._specs = declare_robot_specs(self)
        starts = fg.start_positions(self._specs)
        tr_start, br_start = starts['tr'], starts['br']
        self.br: RobotBody = make_robot_body(
            self.space, 'br', br_start, LEVEL_GROUND, self._specs['br'], fc.ROBOT_WEIGHT_MAX_KG)
        self.tr: RobotBody = make_robot_body(
            self.space, 'tr', tr_start, LEVEL_GROUND, self._specs['tr'], fc.ROBOT_WEIGHT_MAX_KG)
        add_static_field(self.space, (self.tr, self.br))

        self.blocks: list[PhysicsBlock] = []
        self.blocks += _spawn_storage_earth_blocks(self.space)
        self.blocks += _spawn_shared_sky_blocks(self.space)
        self.mustika: PhysicsBlock = make_mustika(self.space, 'mustika', fc.MUSTIKA_PILLAR_ORIGIN, LEVEL_GROUND)

        # グリッパー開閉状態(初期値は「開いている」=何も掴もうとしていない)
        self._br_gripper_open = True
        self._tr_gripper_open = True
        # 閉じる際に狙う対象ブロックid(空文字ならレンジ内最近傍にフォールバック)
        self._br_gripper_target_id = ''
        self._tr_gripper_target_id = ''

        # 機構の動作時間: 把持の進み具合(対象id, 経過秒)と、解放の経過秒
        self._grasp_progress: dict[str, tuple[str, float] | None] = {'tr': None, 'br': None}
        self._release_elapsed = {'tr': 0.0, 'br': 0.0}
        # 実行中のBuildAction({'action', 'elapsed', 'duration'})。_update_build_job参照
        self._build_job: dict | None = None

        # /br_build_actionの処理待ちキューと、建築スポットごとの積み上げ状態
        # (build_spot_id -> {"level": int, "team": str, "blocks": [(block_type, top_color), ...]})
        self._pending_build_action: BuildAction | None = None
        self._towers: dict[str, dict] = {}

        # 9.1/9.4.1: /match_ended受信後はcmd_vel/gripper/build_actionの処理を
        # 一切行わず、ロボットを強制停止する(_step_physics参照)
        self._match_ended = False

        self.create_subscription(Twist, '/br_cmd_vel', self._on_br_cmd_vel, 10)
        self.create_subscription(Twist, '/tr_cmd_vel', self._on_tr_cmd_vel, 10)
        self.create_subscription(GripperCmd, '/br_gripper_cmd', self._on_br_gripper_cmd, 10)
        self.create_subscription(GripperCmd, '/tr_gripper_cmd', self._on_tr_gripper_cmd, 10)
        self.create_subscription(BuildAction, '/br_build_action', self._on_build_action, 10)
        self.create_subscription(Bool, '/match_ended', self._on_match_ended, 10)

        self.pub_br_pose = self.create_publisher(RobotPose, '/true_state/br_pose', 10)
        self.pub_tr_pose = self.create_publisher(RobotPose, '/true_state/tr_pose', 10)
        self.pub_blocks = self.create_publisher(BlockArray, '/true_state/blocks', 10)
        self.pub_mustika_pose = self.create_publisher(MustikaPose, '/true_state/mustika_pose', 10)
        self.pub_tower_state = self.create_publisher(TowerArray, '/true_state/tower_state', 10)

        self.create_timer(1.0 / PHYSICS_HZ, self._step_physics, clock=steady_clock())

    def _on_br_cmd_vel(self, msg: Twist) -> None:
        if self._match_ended:
            return  # 9.4.1: 試合終了後の指令は反映しない(_step_physicsのゼロ化が
            # このコールバックの方が後に届いた指令で上書きされるのを防ぐ)
        self.br.set_cmd_vel(msg.linear.x * M_TO_MM, msg.linear.y * M_TO_MM, msg.angular.z)

    def _on_tr_cmd_vel(self, msg: Twist) -> None:
        if self._match_ended:
            return
        self.tr.set_cmd_vel(msg.linear.x * M_TO_MM, msg.linear.y * M_TO_MM, msg.angular.z)

    def _on_br_gripper_cmd(self, msg: GripperCmd) -> None:
        self._br_gripper_open = msg.open
        self._br_gripper_target_id = msg.target_block_id

    def _on_tr_gripper_cmd(self, msg: GripperCmd) -> None:
        self._tr_gripper_open = msg.open
        self._tr_gripper_target_id = msg.target_block_id

    def _on_build_action(self, msg: BuildAction) -> None:
        self._pending_build_action = msg

    def _on_match_ended(self, msg: Bool) -> None:
        # 一度trueになったら以後falseが来ても戻さない(想定上は一度しか
        # 発行されないが念のため単調にしておく)
        self._match_ended = self._match_ended or msg.data

    def _step_physics(self) -> None:
        dt = 1.0 / PHYSICS_HZ
        if self._match_ended:
            # 9.4.1: ロボットは直ちに停止。以後の指令(cmd_vel/gripper/
            # build_action)は一切反映しない(得点はbr_referee_node側で
            # ブザーの瞬間の状態を使って既に確定している)。
            self.br.stop_now()
            self.tr.stop_now()
            self.space.step(dt)
            self._publish_true_state()
            return
        for robot in (self.br, self.tr):
            u, v = robot.position
            robot.set_level(fg.level_at(u, v))
            robot.step(dt, fg.speed_scale_at(robot.spec, u, v))
        self.space.step(dt)
        # 建築の進行を把持の更新より先に処理する: 設置中はBRのグリッパー開指令を
        # 無視する(機構が動作中)ので、その判定に今回のtickの建築状態を使うため
        self._update_build_job(dt)
        self._update_grasping(dt)
        self._publish_true_state()
        self._enforce_out_of_bounds()

    def _enforce_out_of_bounds(self) -> None:
        """
        6.4 場外: フィールド外(0〜GAME_FIELD_SIZE mm四方)に出た未保持ブロックを
        処理する。アース/スカイブロックは"permanently removed from play"に
        従い、pymunk空間とself.blocksから完全に取り除く(以後/true_state/blocks
        にも現れなくなる)。ムスティカは"immediately returned to the Mustika
        Pillar"に従い、開始位置(MUSTIKA_PILLAR_ORIGIN, レベルGROUND)へ即時
        リセットする。held中(ロボットが運搬中)のオブジェクトは対象外
        (ロボットが自らの制御下で運んでいる間は「ノックアウトされた」状況
        ではないため)。

        _publish_true_state()の"後"に呼ぶこと: フィールド外に出た瞬間の位置を
        一度は/true_state/*として発行してから是正する(逆順にすると、
        br_referee_nodeがムスティカの場外位置を一度も観測できないまま
        ムスティカ柱への復帰だけを見ることになり、6.4違反として検出できない。
        ブロック側は消える/消えないの2値なので順序に依存しないが、
        統一のためこちらもこの順序に合わせる)。
        """
        remaining = []
        for pblock in self.blocks:
            if pblock.held_by == 'none' and not _in_field_bounds(pblock.body.position):
                self.space.remove(pblock.body, pblock.shape)
            else:
                remaining.append(pblock)
        self.blocks = remaining

        if self.mustika.held_by == 'none' and not _in_field_bounds(self.mustika.body.position):
            self.mustika.body.position = fc.MUSTIKA_PILLAR_ORIGIN
            self.mustika.body.velocity = (0, 0)
            self.mustika.set_level(LEVEL_GROUND)
            self.mustika.placed = False

    def _update_grasping(self, dt: float) -> None:
        """
        TR/BRそれぞれ同時に1個しか保持できない。グリッパーが閉じていて、対象が
        把持レンジ(機体仕様のgrasp_range_mm)内にある状態がgrasp_time_s続いたら保持し、
        保持中に開いている状態がrelease_time_s続いたら手を離す。設置済み(placed)の
        物はつかめない(置いた直後に閉じたままのグリッパーで再びつかまないように)。

        対象はGripperCmd.target_block_idで指定された1個に限る。以前は距離だけで最も
        近い1個を推測していたが、スカイ格子(240mmピッチ)で隣のブロックを取り違える、
        種別ごとにレンジが違うのでレンジ外の別種別を選ぶ、搬送中に転がってきた
        ムスティカを掴む、といった取り違えが統合テストで発覚したため。空文字の場合のみ
        後方互換としてレンジ内最近傍にフォールバックする。
        """
        all_blocks = [*self.blocks, self.mustika]
        self._update_robot_gripper(self.br, all_blocks, self._br_gripper_open, self._br_gripper_target_id, dt)
        self._update_robot_gripper(self.tr, all_blocks, self._tr_gripper_open, self._tr_gripper_target_id, dt)
        for pblock in all_blocks:
            if pblock.held_by == 'br':
                pblock.follow_gripper(self.br.position)
            elif pblock.held_by == 'tr':
                pblock.follow_gripper(self.tr.position)

    def _update_robot_gripper(self, robot: RobotBody, all_blocks, gripper_open: bool, target_id: str,
                              dt: float) -> None:
        rid = robot.robot_id
        holding = next((b for b in all_blocks if b.held_by == rid), None)
        if holding is not None:
            self._grasp_progress[rid] = None
            placing = (rid == 'br' and self._build_job is not None
                       and self._build_job['action'].action_type != 'FLIP_SKY_BLOCK')
            if gripper_open and not placing:
                self._release_elapsed[rid] += dt
                if self._release_elapsed[rid] >= robot.spec.release_time_s:
                    holding.set_held_by('none')
                    self._release_elapsed[rid] = 0.0
            else:
                self._release_elapsed[rid] = 0.0
            return
        self._release_elapsed[rid] = 0.0
        target = None if gripper_open else self._grasp_target(robot, all_blocks, target_id)
        if target is None:
            self._grasp_progress[rid] = None
            return
        progress = self._grasp_progress[rid]
        elapsed = (progress[1] if progress and progress[0] == target.id else 0.0) + dt
        if elapsed >= robot.spec.grasp_time_s:
            target.set_held_by(rid)
            self._grasp_progress[rid] = None
        else:
            self._grasp_progress[rid] = (target.id, elapsed)

    @staticmethod
    def _grasp_target(robot: RobotBody, all_blocks, target_id: str) -> PhysicsBlock | None:
        def in_range(b: PhysicsBlock) -> bool:
            return (b.body.position - robot.position).length <= robot.spec.grasp_range_mm(b.block_type)

        candidates = [b for b in all_blocks if b.held_by == 'none' and not b.placed]
        if target_id:
            target = next((b for b in candidates if b.id == target_id), None)
            return target if target is not None and in_range(target) else None
        in_reach = [b for b in candidates if in_range(b)]
        return min(in_reach, key=lambda b: (b.body.position - robot.position).length, default=None)

    def _update_build_job(self, dt: float) -> None:
        """
        /br_build_actionを機体仕様の動作時間(place_time_s/flip_time_s)をかけて実行する。
        受け付けるのは、BRが対象(建築スポット、ムスティカなら中央の柱)の近くにいて、
        必要な物を持っている(裏返しなら塔の一番上がスカイ)ときだけ。実行中は新しい
        指令を受け付けず、途中で条件が崩れたら(離れた等)中止する。意思決定ノードは
        完了まで同じ指令を送り続けるので、完了後に届いた再送は条件を満たさず捨てられる。
        """
        if self._build_job is None:
            action = self._pending_build_action
            self._pending_build_action = None
            if action is None:
                return
            duration = self._build_duration(action)
            if duration is None:
                return
            self._build_job = {'action': action, 'elapsed': 0.0, 'duration': duration}
        else:
            self._pending_build_action = None
            if self._build_duration(self._build_job['action']) is None:
                self._build_job = None
                return
        job = self._build_job
        job['elapsed'] += dt
        if job['elapsed'] >= job['duration']:
            self._build_job = None
            self._complete_build_action(job['action'])

    def _build_duration(self, action: BuildAction) -> float | None:
        """actionを今実行できるならその所要時間(秒)、できないならNone。"""
        spec = self.br.spec
        near_mm = spec.half_width_mm + spec.grasp_reach_mm + fc.BUILD_SPOT_SIZE[0] / 2

        def near(point) -> bool:
            return math.dist((self.br.position.x, self.br.position.y), point) <= near_mm

        if action.action_type == 'PLACE_MUSTIKA':
            ok = self.mustika.held_by == 'br' and near((fc.FIELD_CENTER, fc.FIELD_CENTER))
            return spec.place_time_s if ok else None
        target = _build_spot_by_id(action.target_build_spot_id)
        if target is None:
            return None
        spot_id, _level, _team, origin = target
        if not near((origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)):
            return None
        if action.action_type == 'FLIP_SKY_BLOCK':
            tower = self._towers.get(spot_id)
            ok = bool(tower and tower['blocks'] and tower['blocks'][-1].block_type == 'sky')
            return spec.flip_time_s if ok else None
        if action.action_type in ('PLACE_EARTH_BLOCK', 'PLACE_SKY_BLOCK'):
            ok = any(b.held_by == 'br' for b in self.blocks)
            return spec.place_time_s if ok else None
        return None

    def _complete_build_action(self, action: BuildAction) -> None:
        """
        所要時間を経過したBuildActionの結果を反映する。action_typeは以下の4種類:
        - PLACE_EARTH_BLOCK / PLACE_SKY_BLOCK: BRが現在保持しているブロックを
          対象の建築スポットへ配置する(block_typeとの厳密な整合性チェックは
          行わない。Phase1最初のゴールに向けた簡略化。要調整)。
        - FLIP_SKY_BLOCK: 対象の建築スポットに既に設置されている(最上段の)
          スカイブロックの上面色を反転する(ルールブック3.5.8の"stealing")。
        - PLACE_MUSTIKA: BRが保持しているムスティカを中央支柱(FIELD_CENTER,
          レベルL2)へ設置する(8.5)。建築スポットではなく専用のセントラル
          ピラーが対象のため、target_build_spot_idは無視して先に処理する。
        self._towersはbuild_spot_id -> {"level": int, "blocks": [PhysicsBlock, ...]}
        で、実際のPhysicsBlockオブジェクトの参照を積み上げ順に保持する
        (block_types/owner_teams/top_colorsはtower_state発行時に毎回そこから
        導出するので、スカイブロックをひっくり返した結果が即座に反映される)。
        """
        if action.action_type == 'PLACE_MUSTIKA':
            if self.mustika.held_by != 'br':
                return
            self.mustika.set_held_by('none')
            self.mustika.set_level(LEVEL_L2)
            self.mustika.body.position = (fc.FIELD_CENTER, fc.FIELD_CENTER)
            self.mustika.body.velocity = (0, 0)
            self.mustika.placed = True
            return

        target = _build_spot_by_id(action.target_build_spot_id)
        if target is None:
            return
        spot_id, level, _team, origin = target

        if action.action_type == 'FLIP_SKY_BLOCK':
            tower = self._towers.get(spot_id)
            if not tower or not tower['blocks']:
                return
            top_block = tower['blocks'][-1]
            if top_block.block_type != 'sky':
                return  # 最上段がスカイでなければ何もしない
            top_block.top_color = (
                fc.TeamColor.BLUE if top_block.top_color == fc.TeamColor.RED else fc.TeamColor.RED)
            return

        held_block = next((b for b in self.blocks if b.held_by == 'br'), None)
        if held_block is None:
            return

        center = (origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)
        held_block.set_held_by('none')
        held_block.set_level(level)
        held_block.body.position = center
        held_block.body.velocity = (0, 0)
        held_block.placed = True

        tower = self._towers.setdefault(spot_id, {'level': level, 'blocks': []})
        tower['blocks'].append(held_block)

    def _publish_true_state(self) -> None:
        self.pub_br_pose.publish(self._robot_to_pose_msg(self.br))
        self.pub_tr_pose.publish(self._robot_to_pose_msg(self.tr))

        block_array = BlockArray()
        block_array.blocks = [self._block_to_msg(b) for b in self.blocks]
        self.pub_blocks.publish(block_array)

        mustika_msg = MustikaPose()
        mustika_msg.position = Point(x=self.mustika.position.x, y=self.mustika.position.y, z=0.0)
        mustika_msg.level = self.mustika.level
        mustika_msg.held_by = self.mustika.held_by
        self.pub_mustika_pose.publish(mustika_msg)

        self.pub_tower_state.publish(self._tower_state_msg())

    def _tower_state_msg(self) -> TowerArray:
        msg = TowerArray()
        for spot_id, info in self._towers.items():
            tower = TowerState()
            tower.build_spot_id = spot_id
            tower.level = info['level']
            tower.block_types = [b.block_type for b in info['blocks']]
            # アースは設置した個人チームに固定(8.3.1)、スカイは所有権の概念が
            # ないため空文字(得点判定はtop_colorで行う。_recompute_tower_score参照)
            tower.owner_teams = [b.owner_team if b.block_type == 'earth' else '' for b in info['blocks']]
            tower.top_colors = [b.top_color for b in info['blocks']]
            msg.towers.append(tower)
        return msg

    @staticmethod
    def _robot_to_pose_msg(robot: RobotBody) -> RobotPose:
        msg = RobotPose()
        msg.pose = Pose2D(x=robot.position.x, y=robot.position.y, theta=_yaw_from_body(robot.body))
        msg.level = robot.level
        return msg

    @staticmethod
    def _block_to_msg(pblock: PhysicsBlock) -> Block:
        # PhysicsBlockのフィールドはBlock.msgに1対1対応させてあるので詰め替えるだけでよい
        msg = Block()
        msg.id = pblock.id
        msg.position = Point(x=pblock.position.x, y=pblock.position.y, z=0.0)
        msg.level = pblock.level
        msg.block_type = pblock.block_type
        msg.top_color = pblock.top_color
        msg.owner_team = pblock.owner_team
        msg.held_by = pblock.held_by
        msg.placed = pblock.placed
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
