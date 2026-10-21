"""
機構制御(本番でも使う)。意思決定ノードのGripperCmd(対象のブロックid)・BuildAction(建築スポットid)を、
推定位置から見た機体座標の目標位置に直してハードウェア境界の/<r>/mechanism_cmdへ渡す。
実機のグリッパーはIDを知らないので、IDから位置への変換はここで行う(ブロックは/detected_blocks、
建築スポットと中央の柱はフィールド図)。推定位置がずれていれば、そのまま狙う位置もずれる。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.node import Node

from br_msgs.msg import BuildAction, DetectedBlockArray, DetectedMustika, GripperCmd, MechanismCmd
from br_strategy_sim import field_constants as fc
from br_strategy_sim.robot_spec import body_axes

from .common import declare_robot, yaw_from_quaternion

_PLACE_ACTIONS = ('PLACE_EARTH_BLOCK', 'PLACE_SKY_BLOCK')


def _spot_center(spot_id: str):
    for sid, _level, _team, origin in fc.BUILD_SPOTS:
        if sid == spot_id:
            return (origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)
    return None


class MechanismControllerNode(Node):
    def __init__(self, robot: str | None = None):
        # 1プロセスでTR用・BR用を両方動かすときにノード名が重ならないよう、ロボットを名前に含める
        super().__init__(f'{robot}_mechanism_controller' if robot else 'mechanism_controller_node')
        self._robot = declare_robot(self, robot)
        self._pose: tuple[float, float, float] | None = None
        self._blocks = DetectedBlockArray()
        self._mustika: DetectedMustika | None = None
        self.pub = self.create_publisher(MechanismCmd, f'/{self._robot}/mechanism_cmd', 10)
        self.create_subscription(PoseWithCovarianceStamped, f'/{self._robot}_pose_estimated', self._on_pose, 10)
        self.create_subscription(DetectedBlockArray, '/detected_blocks', self._on_blocks, 10)
        self.create_subscription(DetectedMustika, '/detected_mustika', self._on_mustika, 10)
        self.create_subscription(GripperCmd, f'/{self._robot}_gripper_cmd', self._on_gripper, 10)
        if self._robot == 'br':
            self.create_subscription(BuildAction, '/br_build_action', self._on_build_action, 10)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_blocks(self, msg: DetectedBlockArray) -> None:
        self._blocks = msg

    def _on_mustika(self, msg: DetectedMustika) -> None:
        self._mustika = msg

    def _to_body(self, uv) -> Point | None:
        if self._pose is None:
            return None
        u, v, yaw = self._pose
        forward, left = body_axes(yaw)
        du, dv = uv[0] - u, uv[1] - v
        return Point(x=du * forward[0] + dv * forward[1], y=du * left[0] + dv * left[1], z=0.0)

    def _send(self, command: str, uv) -> None:
        target = self._to_body(uv) if uv is not None else Point()
        if target is not None:
            self.pub.publish(MechanismCmd(command=command, target=target))

    def _object_position(self, object_id: str):
        if object_id == 'mustika':
            m = self._mustika
            return (m.position.x, m.position.y) if m is not None else None
        if not object_id:
            # 対象の指定が無いときは、見えている物のうち一番近いもの
            if self._pose is None or not self._blocks.blocks:
                return None
            free = [b for b in self._blocks.blocks if b.held_by == 'none' and b.level == 0]
            if not free:
                return None
            b = min(free, key=lambda b: math.hypot(b.position.x - self._pose[0], b.position.y - self._pose[1]))
            return (b.position.x, b.position.y)
        b = next((b for b in self._blocks.blocks if b.id == object_id), None)
        return (b.position.x, b.position.y) if b is not None else None

    def _on_gripper(self, msg: GripperCmd) -> None:
        if msg.open:
            self._send('OPEN', None)
            return
        position = self._object_position(msg.target_block_id)
        if position is not None:
            self._send('CLOSE', position)

    def _on_build_action(self, msg: BuildAction) -> None:
        if msg.action_type == 'PLACE_MUSTIKA':
            self._send('PLACE', (fc.FIELD_CENTER, fc.FIELD_CENTER))
            return
        center = _spot_center(msg.target_build_spot_id)
        if center is None:
            return
        if msg.action_type in _PLACE_ACTIONS:
            self._send('PLACE', center)
        elif msg.action_type == 'FLIP_SKY_BLOCK':
            self._send('FLIP', center)


def main(args=None):
    rclpy.init(args=args)
    node = MechanismControllerNode()
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
