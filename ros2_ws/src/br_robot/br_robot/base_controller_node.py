"""
足回り制御(本番でも使う)。意思決定ノードのフィールド座標の速度指令/<r>_cmd_velを、推定した
向きで機体座標に直してハードウェア境界の/<r>/base_cmdへ渡す(実機ではcan_nodeがSTM32へ送る)。
指令が途絶えたら止める(通信切れで走り続けないため)。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

from br_strategy_sim.robot_spec import body_axes

from .common import declare_robot, yaw_from_quaternion

RATE_HZ = 20.0
# この時間指令が来なければ止める
CMD_TIMEOUT_S = 0.5


class BaseControllerNode(Node):
    def __init__(self, robot: str | None = None):
        # 1プロセスでTR用・BR用を両方動かすときにノード名が重ならないよう、ロボットを名前に含める
        super().__init__(f'{robot}_base_controller' if robot else 'base_controller_node')
        self._robot = declare_robot(self, robot)
        self._yaw = 0.0
        self._cmd = Twist()
        self._cmd_time = None
        self.pub = self.create_publisher(Twist, f'/{self._robot}/base_cmd', 10)
        self.create_subscription(Twist, f'/{self._robot}_cmd_vel', self._on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, f'/{self._robot}_pose_estimated', self._on_pose, 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg
        self._cmd_time = self.get_clock().now()
        self._publish()

    def _tick(self) -> None:
        if self._cmd_time is not None and \
                (self.get_clock().now() - self._cmd_time).nanoseconds / 1e9 > CMD_TIMEOUT_S:
            self._cmd = Twist()
            self._cmd_time = None
        self._publish()

    def _publish(self) -> None:
        forward, left = body_axes(self._yaw)
        vu, vv = self._cmd.linear.x, self._cmd.linear.y
        out = Twist()
        out.linear.x = vu * forward[0] + vv * forward[1]
        out.linear.y = vu * left[0] + vv * left[1]
        out.angular.z = self._cmd.angular.z
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
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
