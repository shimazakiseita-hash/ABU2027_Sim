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

最初のゴールはアースブロック1個を受渡しエリアで拾い、建築スポットへ設置して
得点が入るところまで。塔の完成(アース2段+スカイ1段)は、このループが証明
できてから同じ処理を繰り返す形で後回しにする(現状はBUILD_SPOT_IDを固定した
1個限りの実装)。
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

# 最初のゴールでは固定の建築スポットへ運ぶ(複数スポットへの割り振りは後回し)
BUILD_SPOT_ID = 'l1_red_1'


class BrState(Enum):
    IDLE = auto()
    APPROACH_TRANSFER_AREA = auto()
    WAIT_FOR_BLOCK = auto()
    GRASP_FROM_TRANSFER = auto()
    APPROACH_BUILD_SPOT = auto()
    RELEASE_AND_BUILD = auto()


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

    def _find_waiting_block(self):
        origin, size = transfer_area_rect()
        for b in self._blocks.blocks:
            if b.held_by == 'none' and point_in_rect(b.position.x, b.position.y, origin, size):
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
                self.pub_gripper.publish(GripperCmd(open=False, target_force=1.0))
                if self._timeout_counter > GRASP_TIMEOUT_TICKS:
                    self._target_block_id = None
                    self._state = BrState.WAIT_FOR_BLOCK
                return
            twist, _arrived = drive_toward(self._current_uv(), (target.position.x, target.position.y))
            self.pub_cmd_vel.publish(twist)
            self.pub_gripper.publish(GripperCmd(open=False, target_force=1.0))
            if target.held_by == 'br':
                self._state = BrState.APPROACH_BUILD_SPOT
            elif self._timeout_counter > GRASP_TIMEOUT_TICKS:
                self._target_block_id = None
                self._state = BrState.WAIT_FOR_BLOCK

        elif self._state == BrState.APPROACH_BUILD_SPOT:
            self.pub_gripper.publish(GripperCmd(open=False, target_force=1.0))
            twist, arrived = drive_toward(self._current_uv(), _build_spot_center(BUILD_SPOT_ID))
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = BrState.RELEASE_AND_BUILD
                self._timeout_counter = 0

        elif self._state == BrState.RELEASE_AND_BUILD:
            self.pub_cmd_vel.publish(Twist())
            self.pub_build_action.publish(
                BuildAction(action_type='PLACE_EARTH_BLOCK', target_build_spot_id=BUILD_SPOT_ID))
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            self._timeout_counter += 1
            if self._timeout_counter > BUILD_SETTLE_TICKS:
                self._target_block_id = None
                self._state = BrState.APPROACH_TRANSFER_AREA


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
