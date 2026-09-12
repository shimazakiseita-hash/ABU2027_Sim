"""
ABU Robocon 2027 Phase 1 2Dシム: 審判ノード(br_referee_node)。

/true_state/* と /br_build_action を購読し、違反判定(ルールブック6章・7.2)と
得点計算(8章、field_constants.pyのSCORE_*定数を使用)を行う。意思決定ノード
(br_decision)はこのノードの出力(/score/*, /violation)だけを見て、
/true_state/*を直接購読してはいけない(topic_contract.mdの規約)。

判定ロジックの根拠と既知の未対応点は topic_contract.md「5. 審判ノード」参照。
得点計算(8章)はSCORE_*定数が確定値として与えられているため信頼度が高い。
違反判定(6章・7.2)は条文通り実装したが、以下は未対応(要調整):
- 6.2.2「非共用のL1」全体の正確な境界(共用エリアとの切り分け線)が未確定のため、
  相手側スタートゾーン・ストレージエリア・建築スポットの侵入のみ判定する
- 6.6 妨害(共用区域): 5秒間の意図的な進路妨害の判定にはタイマー管理が必要で未実装
- 6.4 場外・6.5 落下ブロックの扱いは未実装
- 6.3/7.2の判定は/br_build_actionの「直近受信」を建築行為のヒントとして使う
  簡易実装で、複数ブロックを連続建築する場合の取り違えは未対応
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32

from br_msgs.msg import BlockArray, BuildAction, MustikaPose, RobotPose, TowerArray, Violation

from . import field_constants as fc
from .physics_blocks import LEVEL_GROUND, LEVEL_L1, LEVEL_L2

# 6.1押出し・7.2設置済み移動の判定: /true_state/blocks受信間隔での位置変化(mm)の閾値。
# 発行周期が固定でない前提のため「速度」ではなく「1メッセージあたりの変位」として扱う(要調整)。
PUSH_DISPLACEMENT_THRESHOLD_MM = 30.0
# 違反したロボットを推定する際、ブロックとロボット中心の距離がこれ以内なら「関与した」とみなす
NEAREST_ROBOT_MAX_DIST_MM = 800.0


def _point_in_rect(x: float, y: float, origin: tuple[float, float], size: tuple[float, float]) -> bool:
    return origin[0] <= x <= origin[0] + size[0] and origin[1] <= y <= origin[1] + size[1]


def _transfer_area_origin() -> tuple[float, float]:
    off = fc.TRANSFER_AREA_L1_CORNER_OFFSET
    return (fc.L1_ORIGIN[0] + off, fc.L1_ORIGIN[1] + off)


class BrRefereeNode(Node):
    def __init__(self):
        super().__init__('br_referee_node')

        # Phase1は自チーム1台構成のため、TR/BRの区域侵犯をどちらのteamとして
        # 報告するか、また相手チームがどちらかをパラメータで持たせる
        self.declare_parameter('team', 'red')
        self._own_team = self.get_parameter('team').value

        self._tr_pose: RobotPose | None = None
        self._br_pose: RobotPose | None = None
        self._mustika_pose: MustikaPose | None = None
        self._tower_state = TowerArray()
        self._prev_block_position: dict[str, Point] = {}
        self._prev_block_held: dict[str, str] = {}
        self._prev_block_placed: dict[str, bool] = {}
        # 直近に受信した/br_build_actionを「保留中の建築行為」として覚えておき、
        # 次のブロック解放が建築設置か受渡しかの区別に使う(1回消費でクリア)
        self._pending_build_action: BuildAction | None = None

        # 得点は「tower_state/mustikaから毎回全体を再計算する分」と
        # 「受渡し成功イベントで積み増す分」を別に持ち、発行時に合算する
        # (両方を1つのキャッシュに混ぜると、片方の更新でもう片方を消してしまうため)
        self._tower_score = {fc.TeamColor.RED: 0, fc.TeamColor.BLUE: 0}
        self._transfer_score = {fc.TeamColor.RED: 0, fc.TeamColor.BLUE: 0}

        self.pub_score_red = self.create_publisher(Int32, '/score/red', 10)
        self.pub_score_blue = self.create_publisher(Int32, '/score/blue', 10)
        self.pub_violation = self.create_publisher(Violation, '/violation', 10)

        self.create_subscription(RobotPose, '/true_state/tr_pose', self._on_tr_pose, 10)
        self.create_subscription(RobotPose, '/true_state/br_pose', self._on_br_pose, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_subscription(MustikaPose, '/true_state/mustika_pose', self._on_mustika_pose, 10)
        self.create_subscription(TowerArray, '/true_state/tower_state', self._on_tower_state, 10)
        self.create_subscription(BuildAction, '/br_build_action', self._on_build_action, 10)

    # --- 区域侵犯 (6.2) ---

    def _on_tr_pose(self, msg: RobotPose) -> None:
        self._tr_pose = msg
        if msg.level in (LEVEL_L1, LEVEL_L2):
            # 6.2.1: TRは受渡しエリアを越えてL1/L2の鉛直境界内へ入ってはならない
            self._publish_violation('zone', 'tr', self._own_team, forced_retry=True)
            return
        if self._in_opponent_zone(msg.pose.x, msg.pose.y, msg.level):
            self._publish_violation('zone', 'tr', self._own_team, forced_retry=True)

    def _on_br_pose(self, msg: RobotPose) -> None:
        # BRは建築のためL1/L2に登る前提なので、レベル侵入自体は違反にしない(6.2.1はTR限定)
        self._br_pose = msg
        if self._in_opponent_zone(msg.pose.x, msg.pose.y, msg.level):
            self._publish_violation('zone', 'br', self._own_team, forced_retry=True)

    def _in_opponent_zone(self, x: float, y: float, level: int) -> bool:
        """6.2.2: 相手専用区域(スタートゾーン/ストレージエリア/建築スポット)への侵入。"""
        for origin, size in self._opponent_ground_zones():
            if _point_in_rect(x, y, origin, size):
                return True
        opponent = self._opponent_team()
        for _spot_id, spot_level, spot_team, spot_origin in fc.BUILD_SPOTS:
            if spot_level == level and spot_team == opponent:
                if _point_in_rect(x, y, spot_origin, fc.BUILD_SPOT_SIZE):
                    return True
        return False

    def _opponent_team(self) -> str:
        return fc.TeamColor.BLUE if self._own_team == fc.TeamColor.RED else fc.TeamColor.RED

    def _opponent_ground_zones(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        # field_constants.pyのSTART_ZONE_*/STORAGE_AREA_*は赤チーム側の座標として
        # 定義されているので、自チームが赤なら相手(青)側は鏡映、青ならそのまま使う
        red_side_zones = [
            (fc.START_ZONE_1_ORIGIN, fc.START_ZONE_SIZE),
            (fc.START_ZONE_2_ORIGIN, fc.START_ZONE_SIZE),
            (fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE),
        ]
        if self._own_team == fc.TeamColor.RED:
            return [(fc.mirror_rect_origin(o, s), s) for o, s in red_side_zones]
        return red_side_zones

    # --- ブロックの状態遷移: 6.1押出し / 7.2設置済み移動(失格) / 6.3受渡し ---

    def _on_blocks(self, msg: BlockArray) -> None:
        for block in msg.blocks:
            self._check_block_movement(block)
            self._check_release(block)
            self._prev_block_position[block.id] = block.position
            self._prev_block_held[block.id] = block.held_by
            self._prev_block_placed[block.id] = block.placed

    def _check_block_movement(self, block) -> None:
        """held中(正当な運搬)以外でブロックが動いた場合の判定(6.1押出し禁止 / 7.2失格)。"""
        if block.held_by != "none":
            return
        prev = self._prev_block_position.get(block.id)
        if prev is None:
            return
        displacement = math.hypot(block.position.x - prev.x, block.position.y - prev.y)
        if displacement <= PUSH_DISPLACEMENT_THRESHOLD_MM:
            return

        was_placed = self._prev_block_placed.get(block.id, False)
        robot = self._nearest_robot(block.position.x, block.position.y)
        if was_placed:
            # 7.2: 設置済みブロックが動いた。相手の設置物を動かした場合のみ即失格
            # (Phase1で物理的に動かせるのは自チームのロボットだけなので、
            # 「自チームが相手の設置物を動かした」場合のみ判定できる)
            if block.owner_team and block.owner_team != self._own_team:
                self._publish_violation(
                    'disqualification', robot or 'br', self._own_team, forced_retry=False)
            return
        # 6.1: 未設置・未保持のブロックが動いた -> 押出し禁止違反
        if robot is not None:
            self._publish_violation('push', robot, block.owner_team, forced_retry=True)

    def _check_release(self, block) -> None:
        """6.3受渡し違反 / 受渡し点。held=True->Falseの遷移のうち、建築設置ではないものが対象。"""
        was_held = self._prev_block_held.get(block.id, "none") != "none"
        if not was_held or block.held_by != "none":
            return  # 「保持中->解放」の遷移でなければ対象外
        if self._pending_build_action is not None:
            # 直近にBuildActionがあった解放は建築設置とみなし、受渡し判定はしない
            self._pending_build_action = None
            return
        if block.level != LEVEL_L1:
            return
        origin = _transfer_area_origin()
        if _point_in_rect(block.position.x, block.position.y, origin, fc.TRANSFER_AREA_SIZE):
            self._add_score(block.owner_team, fc.SCORE_TRANSFER_PER_BLOCK)
        else:
            self._publish_violation('transfer', 'br', block.owner_team, forced_retry=True)

    def _on_build_action(self, msg: BuildAction) -> None:
        self._pending_build_action = msg

    def _nearest_robot(self, x: float, y: float) -> str | None:
        candidates = []
        if self._tr_pose is not None:
            candidates.append(('tr', self._tr_pose.pose.x, self._tr_pose.pose.y))
        if self._br_pose is not None:
            candidates.append(('br', self._br_pose.pose.x, self._br_pose.pose.y))
        best_robot = None
        best_dist = NEAREST_ROBOT_MAX_DIST_MM
        for robot, rx, ry in candidates:
            dist = math.hypot(x - rx, y - ry)
            if dist <= best_dist:
                best_robot = robot
                best_dist = dist
        return best_robot

    def _publish_violation(self, vtype: str, robot: str, team: str, forced_retry: bool) -> None:
        msg = Violation()
        msg.type = vtype
        msg.robot = robot
        msg.team = team
        msg.forced_retry = forced_retry
        self.pub_violation.publish(msg)

    # --- 得点計算 (8章) ---

    def _on_mustika_pose(self, msg: MustikaPose) -> None:
        self._mustika_pose = msg
        self._recompute_tower_score()

    def _on_tower_state(self, msg: TowerArray) -> None:
        self._tower_state = msg
        self._recompute_tower_score()

    def _recompute_tower_score(self) -> None:
        red = 0
        blue = 0
        for tower in self._tower_state.towers:
            points = sum(
                self._block_score(block_type, tower.level, layer=i + 1)
                for i, block_type in enumerate(tower.block_types)
            )
            if tower.team == fc.TeamColor.RED:
                red += points
            elif tower.team == fc.TeamColor.BLUE:
                blue += points
            # team未確定("")の建築スポットは得点計上しない(要確認)

        if self._mustika_on_central_pillar():
            # ムスティカ奉納点の帰属チームを判別する情報が契約上無いため、
            # 暫定的に自チームに加算する(要確認)
            if self._own_team == fc.TeamColor.RED:
                red += fc.SCORE_MUSTIKA
            else:
                blue += fc.SCORE_MUSTIKA

        self._tower_score = {fc.TeamColor.RED: red, fc.TeamColor.BLUE: blue}
        self._publish_scores()

    def _mustika_on_central_pillar(self) -> bool:
        if self._mustika_pose is None or self._mustika_pose.level != LEVEL_L2:
            return False
        dx = self._mustika_pose.position.x - fc.FIELD_CENTER
        dy = self._mustika_pose.position.y - fc.FIELD_CENTER
        return math.hypot(dx, dy) <= fc.CENTRAL_PILLAR_SOCKET_DIAMETER / 2

    def _block_score(self, block_type: str, level: int, layer: int) -> int:
        if block_type == 'earth':
            if level == LEVEL_L1:
                return fc.SCORE_EARTH_BLOCK_L1_LAYER1 if layer == 1 else fc.SCORE_EARTH_BLOCK_L1_LAYER2
            if level == LEVEL_L2:
                return fc.SCORE_EARTH_BLOCK_L2_LAYER1 if layer == 1 else fc.SCORE_EARTH_BLOCK_L2_LAYER2
        elif block_type == 'sky':
            return fc.SCORE_SKY_BLOCK_L1 if level == LEVEL_L1 else fc.SCORE_SKY_BLOCK_L2
        return 0

    def _add_score(self, team: str, points: int) -> None:
        if team not in self._transfer_score:
            return  # owner_team未確定("")のブロックは受渡し点を計上しない
        self._transfer_score[team] += points
        self._publish_scores()

    def _publish_scores(self) -> None:
        red = self._tower_score[fc.TeamColor.RED] + self._transfer_score[fc.TeamColor.RED]
        blue = self._tower_score[fc.TeamColor.BLUE] + self._transfer_score[fc.TeamColor.BLUE]
        self.pub_score_red.publish(Int32(data=red))
        self.pub_score_blue.publish(Int32(data=blue))


def main(args=None):
    rclpy.init(args=args)
    node = BrRefereeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
