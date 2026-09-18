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

/br_gripper_cmd, /tr_gripper_cmd を購読し、PhysicsBlock.update_holding
経由でheld_byを判定する(取り違え防止のロジックはphysics_blocks.py参照)。
/br_build_action を購読し、PLACE_EARTH_BLOCK/PLACE_SKY_BLOCKならBRが現在
保持しているブロックを対象の建築スポットへ実際に配置し(位置・levelを更新、
held_by="none"・placed=True)、FLIP_SKY_BLOCKなら既に設置済みの最上段
スカイブロックの上面色を反転する(ルールブック3.5.8のタワー"stealing")。
いずれも/true_state/tower_stateへ反映する。

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

import pymunk
import rclpy
from geometry_msgs.msg import Point, Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from br_msgs.msg import Block, BlockArray, BuildAction, GripperCmd, MustikaPose, RobotPose, TowerArray, TowerState

from . import field_constants as fc
from .physics_blocks import (
    EVER_HELD_BLOCK_COLLISION_TYPE,
    LEVEL_GROUND,
    LEVEL_L2,
    ROBOT_COLLISION_TYPE,
    PhysicsBlock,
    grasp_range_mm,
    make_earth_block,
    make_mustika,
    make_sky_block,
)
from .robot_body import RobotBody, make_robot_body

PHYSICS_HZ = 60.0
M_TO_MM = 1000.0

# BR/TRのfootprint(700mm角)同士がスタートゾーン境界で密着しないための余裕。
# 実際の大会では審判が手動配置するため密着はあり得ない想定の近似値。
START_SPAWN_GAP_MM = 200.0

# ストレージのアースブロック配置: 2D物理では実寸(350mm角)通りの2段積みを
# 表現できないため、1段に並べて配置する簡略化(要調整)。ピッチは350mm角の
# ブロック同士が余裕を持って離れるように設定する(狭すぎるとロボットが
# 突っ込んだ際に隣接ブロック同士が強く押し合い、pymunkの衝突解決が
# 数値的に不安定になる=NaN化することが統合テストで判明したため)。
STORAGE_BLOCK_PITCH_U_MM = 600.0
STORAGE_BLOCK_PITCH_V_MM = 550.0
STORAGE_BLOCK_COLS = 2


def _spawn_storage_earth_blocks(space: pymunk.Space) -> list[PhysicsBlock]:
    blocks = []
    count = fc.STORAGE_AREA_EARTH_BLOCK_COUNT
    for i in range(count):
        col = i % STORAGE_BLOCK_COLS
        row = i // STORAGE_BLOCK_COLS
        u = fc.STORAGE_AREA_ORIGIN[0] + fc.EARTH_BLOCK_SIZE / 2 + col * STORAGE_BLOCK_PITCH_U_MM
        v = fc.STORAGE_AREA_ORIGIN[1] + fc.EARTH_BLOCK_SIZE / 2 + row * STORAGE_BLOCK_PITCH_V_MM
        blocks.append(make_earth_block(space, f'earth_red_{i:02d}', (u, v), LEVEL_GROUND, fc.TeamColor.RED))
    return blocks


def _spawn_shared_sky_blocks(space: pymunk.Space) -> list[PhysicsBlock]:
    """
    ルール4.1.4: 列位置(C1〜C5)を基準に中央列(C3)に対して鏡映対称に配色する。
    「列cが赤上向きなら鏡像列(6-c)は青上向き。中央列(C3)の2個は赤1個・青1個」。
    ここではC1・C2側を赤、C4・C5側を青とする(列cとその鏡像列(6-c)が逆色になれば
    ルールを満たすため、どちらの半分を赤にするかはルール上未指定・実装上の選択)。
    C3の2個は出現順に赤/青を1個ずつ割り当てる。
    """
    blocks = []
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
            blocks.append(make_sky_block(space, f'sky_{index:02d}', (u, v), LEVEL_GROUND, top_color))
            index += 1
    return blocks


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

        br_start = (
            fc.START_ZONE_1_ORIGIN[0] + fc.START_ZONE_SIZE[0] / 2 - START_SPAWN_GAP_MM / 2,
            fc.START_ZONE_1_ORIGIN[1] + fc.START_ZONE_SIZE[1] / 2,
        )
        tr_start = (
            fc.START_ZONE_2_ORIGIN[0] + fc.START_ZONE_SIZE[0] / 2 + START_SPAWN_GAP_MM / 2,
            fc.START_ZONE_2_ORIGIN[1] + fc.START_ZONE_SIZE[1] / 2,
        )
        self.br: RobotBody = make_robot_body(self.space, 'br', br_start, LEVEL_GROUND)
        self.tr: RobotBody = make_robot_body(self.space, 'tr', tr_start, LEVEL_GROUND)

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

        self.create_timer(1.0 / PHYSICS_HZ, self._step_physics)

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
        self.space.step(1.0 / PHYSICS_HZ)
        if self._match_ended:
            # 9.4.1: ロボットは直ちに停止。以後の指令(cmd_vel/gripper/
            # build_action)は一切反映しない(得点はbr_referee_node側で
            # ブザーの瞬間の状態を使って既に確定している)。
            self.br.set_cmd_vel(0.0, 0.0, 0.0)
            self.tr.set_cmd_vel(0.0, 0.0, 0.0)
            self._publish_true_state()
            return
        # BuildActionの実行を先に行う: br_decisionはRELEASE_AND_BUILD状態で
        # BuildActionとgripper open(release)を同一tickで送るため、
        # _update_grasping()を先に呼ぶとgripper open処理でheld_by="none"に
        # なってしまい、直後のBuildAction実行時に「BRが何も保持していない」
        # と判定されてビルドが成立しない(統合テストで発覚)。
        self._execute_pending_build_action()
        self._update_grasping()
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

    def _update_grasping(self) -> None:
        """
        TR/BRそれぞれ同時に1個しか保持できないようにする。グリッパーが
        閉じている場合、GripperCmd.target_block_idで指定された特定の1個
        だけを対象に把持判定する(_try_grasp参照)。

        以前は「距離だけで最も近い1個」を推測で選んでいたが、これだと
        以下のような取り違えが起こり得た:
        - グラウンド共用エリアのスカイブロック格子(240mmピッチ)は把持レンジ
          (600mm)より隣接間隔が狭く、意思決定ノードが実際に狙っている
          ブロックの隣にある別のブロックの方が僅かに近いだけで誤って
          選ばれてしまう
        - ブロック種別ごとに把持レンジが異なるため、距離だけでnearestを
          決めると、意思決定ノードが実際に接近している対象(その種別の
          レンジ内)より、レンジ外の別種別のブロックの方が近いというだけで
          選ばれてしまう
        - 共用エリア内の衝突で弾かれて偶然ブロック搬送経路の近くまで転がって
          きたムスティカを、搬送中のグリッパー閉状態のロボットが意図せず
          掴んでしまう
        いずれも複数塔の統合テストで発覚した。意思決定ノード自身が
        /detected_blocksで対象のidを既に把握しているため、そのidを
        GripperCmdに乗せて物理層に直接伝える方が、距離ベースの推測より
        堅牢(target_block_idが空文字の場合のみ、後方互換としてレンジ内
        最近傍へのフォールバックを行う)。
        """
        all_blocks = [*self.blocks, self.mustika]
        self._try_grasp(all_blocks, self.br.position, self._br_gripper_open, 'br', self._br_gripper_target_id)
        self._try_grasp(all_blocks, self.tr.position, self._tr_gripper_open, 'tr', self._tr_gripper_target_id)

        for pblock in all_blocks:
            if pblock.held_by == 'br':
                pblock.follow_gripper(self.br.position)
            elif pblock.held_by == 'tr':
                pblock.follow_gripper(self.tr.position)

    @staticmethod
    def _try_grasp(all_blocks, gripper_point, gripper_open: bool, holder_id: str, target_block_id: str) -> None:
        holding = next((b for b in all_blocks if b.held_by == holder_id), None)
        if gripper_open:
            if holding is not None:
                holding.update_holding(gripper_point, True, holder_id)
            return
        if holding is not None:
            return  # 既に何か保持中なら新たに掴みにいかない
        candidates = [b for b in all_blocks if b.held_by == 'none']
        if not candidates:
            return
        if target_block_id:
            target = next((b for b in candidates if b.id == target_block_id), None)
            if target is not None:
                target.update_holding(gripper_point, False, holder_id)
            return
        # target_block_id未指定時のみ、後方互換としてレンジ内最近傍にフォールバックする。
        candidates.sort(key=lambda b: (b.body.position - gripper_point).length)
        for candidate in candidates:
            distance = (candidate.body.position - gripper_point).length
            if distance <= grasp_range_mm(candidate.block_type):
                candidate.update_holding(gripper_point, False, holder_id)
                return

    def _execute_pending_build_action(self) -> None:
        """
        /br_build_actionを実行する。action_typeは以下の4種類:
        - PLACE_EARTH_BLOCK / PLACE_SKY_BLOCK: BRが現在保持しているブロックを
          対象の建築スポットへ配置する(block_typeとの厳密な整合性チェックは
          行わない。Phase1最初のゴールに向けた簡略化。要調整)。
        - FLIP_SKY_BLOCK: 対象の建築スポットに既に設置されている(最上段の)
          スカイブロックの上面色を反転する(ルールブック3.5.8の"stealing"。
          BRが保持している必要はなく、位置的な近さも今は問わない簡略化)。
        - PLACE_MUSTIKA: BRが保持しているムスティカを中央支柱(FIELD_CENTER,
          レベルL2)へ設置する(8.5)。建築スポットではなく専用のセントラル
          ピラーが対象のため、target_build_spot_idは無視して先に処理する。
        self._towersはbuild_spot_id -> {"level": int, "blocks": [PhysicsBlock, ...]}
        で、実際のPhysicsBlockオブジェクトの参照を積み上げ順に保持する
        (block_types/owner_teams/top_colorsはtower_state発行時に毎回そこから
        導出するので、スカイブロックをひっくり返した結果が即座に反映される)。
        """
        if self._pending_build_action is None:
            return
        action = self._pending_build_action
        self._pending_build_action = None

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
