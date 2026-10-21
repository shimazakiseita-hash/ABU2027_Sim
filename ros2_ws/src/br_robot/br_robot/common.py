"""br_robotの各ノードで共通の小物。"""

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    """z軸まわりの回転角(このプロジェクトの約束: 0で機体の前方が+V)。"""
    return 2.0 * math.atan2(q.z, q.w)


def quaternion_from_yaw(theta: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(theta / 2), w=math.cos(theta / 2))


def declare_robot(node, default: str | None = None) -> str:
    """どちらのロボット用のノードか(パラメータrobot: 'tr' | 'br')。"""
    node.declare_parameter('robot', default or 'br')
    robot = node.get_parameter('robot').value
    if robot not in ('tr', 'br'):
        raise ValueError(f"robot must be 'tr' or 'br': {robot}")
    return robot
