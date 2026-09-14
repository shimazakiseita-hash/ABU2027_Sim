"""
ABU Robocon 2027 Phase 1 2Dシム: BR側の意思決定ノード(br_decision_br_node)。

最小限のステートマシン:
    IDLE -> APPROACH_TRANSFER_AREA -> WAIT_FOR_BLOCK
         -> GRASP_FROM_TRANSFER -> APPROACH_BUILD_SPOT -> RELEASE_AND_BUILD -> (ループ)

/br_pose_estimated, /detected_blocks(観測トピック)だけを見て動く。
/true_state/*は購読しない(topic_contract.mdの規約)。TR側(br_decision_tr_node)
とは完全に独立したノード/ステートマシンで、通信プロトコルは持たない。
WAIT_FOR_BLOCKは/detected_blocksをポーリングし、受渡しエリア内に
held_by="none"のブロックが現れるのを待つだけの単純な実装(HANDOFF方針通り)。

BUILD_SPOT_PLANで指定した複数の建築スポットに、順番に完成塔(アース2段+
スカイ1段)を作っていく。「今何番目の塔を、その塔のどの段まで作っているか」を
(_tower_index, _layer_index)として保持し、層ごとに「受渡しエリアで待つ対象の
種別」と「/br_build_actionのaction_type」を切り替える。TR側
(br_decision_tr_node.DELIVERY_SEQUENCE)と同じ順序・同じ塔数を独立に前提として
動く(通信プロトコルは持たない設計のため、両者とも自分のカウンタだけを頼りに
同期する)。全ての塔が完成したらPLAN_COMPLETEへ遷移し、以降は停止する。
"""

from __future__ import annotations

from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

from br_msgs.msg import BuildAction, DetectedBlockArray, GripperCmd

from . import field_constants as fc
from .decision_common import br_transfer_wait_point, drive_toward, point_in_rect, transfer_area_rect

CONTROL_HZ = 10.0
GRASP_TIMEOUT_TICKS = int(CONTROL_HZ * 5)
BUILD_SETTLE_TICKS = int(CONTROL_HZ * 0.5)

# 順番に完成塔を作る建築スポット。L2はルールブック上「全体が共用エリア」
# (Building Spot: located on L1 (exclusive and shared) and L2 (all shared))
# なので、l2_red_1を含めることで秘蹟の要件(Sanctuary Mandate: 完成塔2つ、
# うち1つは共有エリア)を自然に満たせる計画にしてある。
BUILD_SPOT_PLAN = ('l1_red_1', 'l2_red_1')

# 塔の構成順(アース2段->スカイ1段)。各層で「待つ対象の種別」と
# 「/br_build_actionのaction_type」の組を持つ。
# br_decision_tr_node.DELIVERY_SEQUENCEと対応させること(要素数・種別の順序を一致させる)。
BUILD_SEQUENCE = (
    ('earth', 'PLACE_EARTH_BLOCK'),
    ('earth', 'PLACE_EARTH_BLOCK'),
    ('sky', 'PLACE_SKY_BLOCK'),
)


class BrState(Enum):
    IDLE = auto()
    APPROACH_TRANSFER_AREA = auto()
    WAIT_FOR_BLOCK = auto()
    GRASP_FROM_TRANSFER = auto()
    APPROACH_BUILD_SPOT = auto()
    RELEASE_AND_BUILD = auto()
    PLAN_COMPLETE = auto()


def _build_spot_center(build_spot_id: str) -> tuple[float, float]:
    for spot_id, _level, _team, origin in fc.BUILD_SPOTS:
        if spot_id == build_spot_id:
            return (origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)
    raise ValueError(f'unknown build_spot_id: {build_spot_id}')


class BrDecisionBrNode(Node):
    def __init__(self):
        super().__init__('br_decision_br_node')

        self._pose: PoseWithCovarianceStamped | None = None
        self._blocks = DetectedBlockArray()
        self._state = BrState.IDLE
        self._target_block_id: str | None = None
        self._timeout_counter = 0
        self._tower_index = 0
        self._layer_index = 0

        self.pub_cmd_vel = self.create_publisher(Twist, '/br_cmd_vel', 10)
        self.pub_gripper = self.create_publisher(GripperCmd, '/br_gripper_cmd', 10)
        self.pub_build_action = self.create_publisher(BuildAction, '/br_build_action', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/br_pose_estimated', self._on_pose, 10)
        self.create_subscription(DetectedBlockArray, '/detected_blocks', self._on_blocks, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose = msg

    def _on_blocks(self, msg: DetectedBlockArray) -> None:
        self._blocks = msg

    def _current_uv(self) -> tuple[float, float]:
        p = self._pose.pose.pose.position
        return (p.x, p.y)

    def _find_block(self, block_id: str):
        return next((b for b in self._blocks.blocks if b.id == block_id), None)

    def _current_build_spot_id(self) -> str:
        return BUILD_SPOT_PLAN[self._tower_index]

    def _find_waiting_block(self):
        expected_type, _action_type = BUILD_SEQUENCE[self._layer_index]
        origin, size = transfer_area_rect()
        for b in self._blocks.blocks:
            if (b.held_by == 'none' and b.block_type == expected_type
                    and point_in_rect(b.position.x, b.position.y, origin, size)):
                return b
        return None

    def _tick(self) -> None:
        if self._pose is None:
            return

        if self._state == BrState.IDLE:
            self._state = BrState.APPROACH_TRANSFER_AREA
            return

        if self._state == BrState.APPROACH_TRANSFER_AREA:
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            twist, arrived = drive_toward(self._current_uv(), br_transfer_wait_point())
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = BrState.WAIT_FOR_BLOCK

        elif self._state == BrState.WAIT_FOR_BLOCK:
            self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            target = self._find_waiting_block()
            if target is not None:
                self._target_block_id = target.id
                self._state = BrState.GRASP_FROM_TRANSFER
                self._timeout_counter = 0

        elif self._state == BrState.GRASP_FROM_TRANSFER:
            target = self._find_block(self._target_block_id)
            self._timeout_counter += 1
            if target is None:
                # TR自身との接触等で一時的に検知が途切れることがあるため、
                # 一度見失っただけでは諦めず、その場で待って様子を見る
                # (タイムアウトまで見つからなければ待機に戻る)
                self.pub_cmd_vel.publish(Twist())
                self.pub_gripper.publish(GripperCmd(
                    open=False, target_force=1.0, target_block_id=self._target_block_id or ''))
                if self._timeout_counter > GRASP_TIMEOUT_TICKS:
                    self._target_block_id = None
                    self._state = BrState.WAIT_FOR_BLOCK
                return
            twist, _arrived = drive_toward(self._current_uv(), (target.position.x, target.position.y))
            self.pub_cmd_vel.publish(twist)
            self.pub_gripper.publish(GripperCmd(
                open=False, target_force=1.0, target_block_id=self._target_block_id or ''))
            if target.held_by == 'br':
                self._state = BrState.APPROACH_BUILD_SPOT
            elif self._timeout_counter > GRASP_TIMEOUT_TICKS:
                self._target_block_id = None
                self._state = BrState.WAIT_FOR_BLOCK

        elif self._state == BrState.APPROACH_BUILD_SPOT:
            self.pub_gripper.publish(GripperCmd(
                open=False, target_force=1.0, target_block_id=self._target_block_id or ''))
            twist, arrived = drive_toward(self._current_uv(), _build_spot_center(self._current_build_spot_id()))
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = BrState.RELEASE_AND_BUILD
                self._timeout_counter = 0

        elif self._state == BrState.RELEASE_AND_BUILD:
            self.pub_cmd_vel.publish(Twist())
            _expected_type, action_type = BUILD_SEQUENCE[self._layer_index]
            self.pub_build_action.publish(
                BuildAction(action_type=action_type, target_build_spot_id=self._current_build_spot_id()))
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            self._timeout_counter += 1
            if self._timeout_counter > BUILD_SETTLE_TICKS:
                self._target_block_id = None
                self._layer_index += 1
                if self._layer_index >= len(BUILD_SEQUENCE):
                    self._layer_index = 0
                    self._tower_index += 1
                    if self._tower_index >= len(BUILD_SPOT_PLAN):
                        self._state = BrState.PLAN_COMPLETE
                    else:
                        self._state = BrState.APPROACH_TRANSFER_AREA
                else:
                    self._state = BrState.APPROACH_TRANSFER_AREA

        elif self._state == BrState.PLAN_COMPLETE:
            # BUILD_SPOT_PLAN全ての塔の設置が完了した。以降は何もせず停止する。
            self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = BrDecisionBrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
