"""
推測航法(本番でも使う)。ハードウェア境界の/<r>/wheel_deltas(4輪メカナムのエンコーダ差分)と
/<r>/imu(向き)から、フィールド座標の位置を積算して/<r>_pose_estimatedへ出す。
開始位置はスタートゾーンの既知の位置(field_geometry.start_positions)。滑りの分だけ誤差が溜まる
(LiDAR等での補正は次の段)。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray

from br_strategy_sim import field_geometry as fg
from br_strategy_sim.robot_spec import body_axes, declare_robot_specs, mecanum_body_velocity

from .common import declare_robot, quaternion_from_yaw, yaw_from_quaternion

# 進んだ距離あたりに増やす位置の分散(mm^2/mm)。推定の不確かさの目安
VARIANCE_PER_MM = 0.5


class OdometryNode(Node):
    def __init__(self, robot: str | None = None):
        # 1プロセスでTR用・BR用を両方動かすときにノード名が重ならないよう、ロボットを名前に含める
        super().__init__(f'{robot}_odometry' if robot else 'odometry_node')
        self._robot = declare_robot(self, robot)
        specs = declare_robot_specs(self)
        self._spec = specs[self._robot]
        self._u, self._v = fg.start_positions(specs)[self._robot]
        self._yaw = 0.0
        self._variance = 0.0
        self._rad_per_count = 2 * math.pi / self._spec.encoder_counts_per_rev
        self.pub = self.create_publisher(PoseWithCovarianceStamped, f'/{self._robot}_pose_estimated', 10)
        self.create_subscription(Int16MultiArray, f'/{self._robot}/wheel_deltas', self._on_wheels, 10)
        self.create_subscription(Imu, f'/{self._robot}/imu', self._on_imu, 10)

    def _on_imu(self, msg: Imu) -> None:
        self._yaw = yaw_from_quaternion(msg.orientation)

    def _on_wheels(self, msg: Int16MultiArray) -> None:
        # 車輪の回転角の増分を順運動学に通すと、機体座標での移動量(mm)になる(線形なため)
        dx, dy, _dyaw = mecanum_body_velocity(self._spec, [c * self._rad_per_count for c in msg.data])
        forward, left = body_axes(self._yaw)
        self._u += dx * forward[0] + dy * left[0]
        self._v += dx * forward[1] + dy * left[1]
        self._variance += VARIANCE_PER_MM * math.hypot(dx, dy)
        self._publish()

    def _publish(self) -> None:
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'field'
        out.pose.pose.position.x = self._u
        out.pose.pose.position.y = self._v
        out.pose.pose.position.z = fg.floor_height_at(self._u, self._v)
        out.pose.pose.orientation = quaternion_from_yaw(self._yaw)
        out.pose.covariance[0] = self._variance
        out.pose.covariance[7] = self._variance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
