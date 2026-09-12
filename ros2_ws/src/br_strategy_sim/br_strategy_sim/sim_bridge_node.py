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
"""

from __future__ import annotations

import pymunk
import rclpy
from geometry_msgs.msg import Point, Pose2D, Twist
from rclpy.node import Node

from br_msgs.msg import Block, BlockArray, BuildAction, GripperCmd, MustikaPose, RobotPose, TowerArray, TowerState

from . import field_constants as fc
from .physics_blocks import (
    EVER_HELD_BLOCK_COLLISION_TYPE,
    LEVEL_GROUND,
    ROBOT_COLLISION_TYPE,
    PhysicsBlock,
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

        # /br_build_actionの処理待ちキューと、建築スポットごとの積み上げ状態
        # (build_spot_id -> {"level": int, "team": str, "blocks": [(block_type, top_color), ...]})
        self._pending_build_action: BuildAction | None = None
        self._towers: dict[str, dict] = {}

        self.create_subscription(Twist, '/br_cmd_vel', self._on_br_cmd_vel, 10)
        self.create_subscription(Twist, '/tr_cmd_vel', self._on_tr_cmd_vel, 10)
        self.create_subscription(GripperCmd, '/br_gripper_cmd', self._on_br_gripper_cmd, 10)
        self.create_subscription(GripperCmd, '/tr_gripper_cmd', self._on_tr_gripper_cmd, 10)
        self.create_subscription(BuildAction, '/br_build_action', self._on_build_action, 10)

        self.pub_br_pose = self.create_publisher(RobotPose, '/true_state/br_pose', 10)
        self.pub_tr_pose = self.create_publisher(RobotPose, '/true_state/tr_pose', 10)
        self.pub_blocks = self.create_publisher(BlockArray, '/true_state/blocks', 10)
        self.pub_mustika_pose = self.create_publisher(MustikaPose, '/true_state/mustika_pose', 10)
        self.pub_tower_state = self.create_publisher(TowerArray, '/true_state/tower_state', 10)

        self.create_timer(1.0 / PHYSICS_HZ, self._step_physics)

    def _on_br_cmd_vel(self, msg: Twist) -> None:
        self.br.set_cmd_vel(msg.linear.x * M_TO_MM, msg.linear.y * M_TO_MM, msg.angular.z)

    def _on_tr_cmd_vel(self, msg: Twist) -> None:
        self.tr.set_cmd_vel(msg.linear.x * M_TO_MM, msg.linear.y * M_TO_MM, msg.angular.z)

    def _on_br_gripper_cmd(self, msg: GripperCmd) -> None:
        self._br_gripper_open = msg.open

    def _on_tr_gripper_cmd(self, msg: GripperCmd) -> None:
        self._tr_gripper_open = msg.open

    def _on_build_action(self, msg: BuildAction) -> None:
        self._pending_build_action = msg

    def _step_physics(self) -> None:
        self.space.step(1.0 / PHYSICS_HZ)
        # BuildActionの実行を先に行う: br_decisionはRELEASE_AND_BUILD状態で
        # BuildActionとgripper open(release)を同一tickで送るため、
        # _update_grasping()を先に呼ぶとgripper open処理でheld_by="none"に
        # なってしまい、直後のBuildAction実行時に「BRが何も保持していない」
        # と判定されてビルドが成立しない(統合テストで発覚)。
        self._execute_pending_build_action()
        self._update_grasping()
        self._publish_true_state()

    def _update_grasping(self) -> None:
        """
        TR/BRそれぞれ同時に1個しか保持できないようにする。update_holding自体は
        ブロック単位でしか判定できず「このロボットは既に別のブロックを保持中か」
        を知らないため、ここ(全ブロックを見渡せる場所)でロボットが既に何か
        保持しているかを事前に調べ、保持中はそのブロック以外へのupdate_holding
        呼び出しをスキップする(統合テストで、受渡し直後にたまたま近くにあった
        配達済みブロックを再度掴んでしまい2個同時保持になる不具合が発覚)。
        """
        all_blocks = [*self.blocks, self.mustika]
        br_holds = next((b for b in all_blocks if b.held_by == 'br'), None)
        tr_holds = next((b for b in all_blocks if b.held_by == 'tr'), None)
        for pblock in all_blocks:
            if br_holds is None or pblock is br_holds:
                pblock.update_holding(self.br.position, self._br_gripper_open, 'br')
                if pblock.held_by == 'br':
                    br_holds = pblock  # 同一tick内で他のブロックまで掴まないよう即座に反映
            if tr_holds is None or pblock is tr_holds:
                pblock.update_holding(self.tr.position, self._tr_gripper_open, 'tr')
                if pblock.held_by == 'tr':
                    tr_holds = pblock
            if pblock.held_by == 'br':
                pblock.follow_gripper(self.br.position)
            elif pblock.held_by == 'tr':
                pblock.follow_gripper(self.tr.position)

    def _execute_pending_build_action(self) -> None:
        """
        /br_build_actionを実行する。action_typeは以下の2種類:
        - PLACE_EARTH_BLOCK / PLACE_SKY_BLOCK: BRが現在保持しているブロックを
          対象の建築スポットへ配置する(block_typeとの厳密な整合性チェックは
          行わない。Phase1最初のゴールに向けた簡略化。要調整)。
        - FLIP_SKY_BLOCK: 対象の建築スポットに既に設置されている(最上段の)
          スカイブロックの上面色を反転する(ルールブック3.5.8の"stealing"。
          BRが保持している必要はなく、位置的な近さも今は問わない簡略化)。
        self._towersはbuild_spot_id -> {"level": int, "blocks": [PhysicsBlock, ...]}
        で、実際のPhysicsBlockオブジェクトの参照を積み上げ順に保持する
        (block_types/owner_teams/top_colorsはtower_state発行時に毎回そこから
        導出するので、スカイブロックをひっくり返した結果が即座に反映される)。
        """
        if self._pending_build_action is None:
            return
        action = self._pending_build_action
        self._pending_build_action = None

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
