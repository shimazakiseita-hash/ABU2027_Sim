"""
ABU Robocon 2027 Phase 1 2Dシム: 観測ノード(認識・自己位置推定の簡易モック)。

/true_state/* を購読し、ノイズ有り観測トピック(/br_pose_estimated,
/tr_pose_estimated, /detected_blocks, /detected_mustika, /detected_towers)
を発行する。意思決定ノード(br_decision)はこちらを購読し、/true_state/*は
直接見ない(topic_contract.mdの規約)。

/detected_towers(完成塔の一覧)は視野判定を行わない。塔はブロックと違って
一度組み上がると動かない大型の構造物であり、フィールド上のどこからでも
「何段組み上がっているか」を目視できるという前提(要調整: 厳密なカメラ
画角を考慮すればもっと近くまで寄らないと段数までは判別できないはずだが、
Phase1では簡略化する)。TR側の秘蹟の要件(Sanctuary Mandate)判定に使う。

HANDOFFの指示通り、Phase1は「視野内のブロックをそのまま返す」簡易実装。
視野は簡略化して、BR/TRそれぞれの位置からの円形検出範囲(DETECTION_RANGE_MM)
とした(実際のカメラ画角・向きは考慮していない。要調整)。

observation_noiseパラメータ(bool, default False)がTrueのとき、位置に
ガウスノイズ(標準偏差POSITION_NOISE_STD_MM)を乗せる。起動時は
`ros2 run br_strategy_sim br_observation_node --ros-args -p observation_noise:=true`
のように指定する(launchファイルからの`observation_noise:=true`引数連携は
launchファイル作成時に対応する)。
"""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node

from br_msgs.msg import (
    BlockArray,
    DetectedBlock,
    DetectedBlockArray,
    DetectedMustika,
    MustikaPose,
    RobotPose,
    TowerArray,
)

from . import field_constants as fc
from .physics_blocks import LEVEL_GROUND, LEVEL_L1, LEVEL_L2

# 視野の簡略化: BR/TRそれぞれの位置から半径この距離以内を「視野内」とする
DETECTION_RANGE_MM = 2000.0
POSITION_NOISE_STD_MM = 20.0

_LEVEL_Z_MM = {LEVEL_GROUND: fc.GROUND_Z, LEVEL_L1: fc.L1_Z, LEVEL_L2: fc.L2_Z}


def _level_to_z_mm(level: int) -> float:
    # field_constants.pyのGROUND_Z/L1_Z/L2_Zはint定数なので、
    # geometry_msgs/Point(float64)への代入に備えて明示的にfloat化する
    return float(_LEVEL_Z_MM.get(level, fc.GROUND_Z))


def _yaw_to_quaternion(theta: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(theta / 2.0), w=math.cos(theta / 2.0))


class BrObservationNode(Node):
    def __init__(self):
        super().__init__('br_observation_node')

        self.declare_parameter('observation_noise', False)
        self._noise_enabled = self.get_parameter('observation_noise').value

        self._tr_pose: RobotPose | None = None
        self._br_pose: RobotPose | None = None

        self.pub_br_pose_estimated = self.create_publisher(
            PoseWithCovarianceStamped, '/br_pose_estimated', 10)
        self.pub_tr_pose_estimated = self.create_publisher(
            PoseWithCovarianceStamped, '/tr_pose_estimated', 10)
        self.pub_detected_blocks = self.create_publisher(DetectedBlockArray, '/detected_blocks', 10)
        self.pub_detected_mustika = self.create_publisher(DetectedMustika, '/detected_mustika', 10)
        self.pub_detected_towers = self.create_publisher(TowerArray, '/detected_towers', 10)

        self.create_subscription(RobotPose, '/true_state/tr_pose', self._on_tr_pose, 10)
        self.create_subscription(RobotPose, '/true_state/br_pose', self._on_br_pose, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_subscription(MustikaPose, '/true_state/mustika_pose', self._on_mustika_pose, 10)
        self.create_subscription(TowerArray, '/true_state/tower_state', self._on_tower_state, 10)

    def _add_noise(self, value: float) -> float:
        if not self._noise_enabled:
            return value
        return value + random.gauss(0.0, POSITION_NOISE_STD_MM)

    # --- 自己位置推定 ---

    def _on_tr_pose(self, msg: RobotPose) -> None:
        self._tr_pose = msg
        self._publish_pose_estimate(self.pub_tr_pose_estimated, msg)

    def _on_br_pose(self, msg: RobotPose) -> None:
        self._br_pose = msg
        self._publish_pose_estimate(self.pub_br_pose_estimated, msg)

    def _publish_pose_estimate(self, publisher, true_pose: RobotPose) -> None:
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'field'
        out.pose.pose.position.x = self._add_noise(true_pose.pose.x)
        out.pose.pose.position.y = self._add_noise(true_pose.pose.y)
        out.pose.pose.position.z = _level_to_z_mm(true_pose.level)
        out.pose.pose.orientation = _yaw_to_quaternion(true_pose.pose.theta)
        variance = (POSITION_NOISE_STD_MM ** 2) if self._noise_enabled else 0.0
        out.pose.covariance[0] = variance  # x
        out.pose.covariance[7] = variance  # y
        publisher.publish(out)

    # --- ブロック/ムスティカ検出 ---

    def _is_in_view(self, x: float, y: float) -> bool:
        for pose in (self._tr_pose, self._br_pose):
            if pose is None:
                continue
            if math.hypot(x - pose.pose.x, y - pose.pose.y) <= DETECTION_RANGE_MM:
                return True
        return False

    def _on_blocks(self, msg: BlockArray) -> None:
        out = DetectedBlockArray()
        for block in msg.blocks:
            if not self._is_in_view(block.position.x, block.position.y):
                continue
            detected = DetectedBlock()
            detected.id = block.id
            detected.position = Point(
                x=self._add_noise(block.position.x),
                y=self._add_noise(block.position.y),
                z=block.position.z,
            )
            detected.level = block.level
            detected.block_type = block.block_type
            detected.top_color = block.top_color
            detected.owner_team = block.owner_team
            detected.held_by = block.held_by
            out.blocks.append(detected)
        self.pub_detected_blocks.publish(out)

    def _on_mustika_pose(self, msg: MustikaPose) -> None:
        if not self._is_in_view(msg.position.x, msg.position.y):
            return  # 視野外は発行しない
        out = DetectedMustika()
        out.position = Point(
            x=self._add_noise(msg.position.x),
            y=self._add_noise(msg.position.y),
            z=msg.position.z,
        )
        out.held_by = msg.held_by
        self.pub_detected_mustika.publish(out)

    def _on_tower_state(self, msg: TowerArray) -> None:
        self.pub_detected_towers.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
