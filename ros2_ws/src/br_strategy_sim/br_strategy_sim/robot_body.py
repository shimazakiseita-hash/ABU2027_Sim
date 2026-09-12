"""
ABU Robocon 2027 Phase 1 2Dシム: BR/TRの簡易矩形表現(pymunk body/shape)。

展開後最大寸法(1000x1400x1200mm)ではなく、両ロボット共通の初期寸法制約
(ROBOT_INITIAL_MAX: 700x700x700mm)の正方形footprintで近似する
(HANDOFFの「簡易矩形」指示に対する具体化。展開時の実寸を反映したい場合は
ここのサイズ選択を見直す)。

physics_blocks.pyと同じ理由(区域侵犯・押出し禁止はbr_referee_nodeが
/true_state座標を見て判定する設計)で、pymunk衝突はブロックとの
押し出し力学の再現に使うのみで、壁との衝突判定には依存しない。
"""

from __future__ import annotations

import pymunk

from . import field_constants as fc
from .physics_blocks import ROBOT_COLLISION_TYPE, level_filter


class RobotBody:
    """BR/TR共通の簡易矩形ラッパー。"""

    def __init__(self, robot_id: str, body: pymunk.Body, shape: pymunk.Shape, level: int):
        self.robot_id = robot_id
        self.body = body
        self.shape = shape
        self.level = level

    @property
    def position(self) -> pymunk.Vec2d:
        return self.body.position

    def set_level(self, level: int) -> None:
        self.level = level
        self.shape.filter = level_filter(level)

    def set_cmd_vel(self, vx_mm_s: float, vy_mm_s: float, omega_rad_s: float) -> None:
        """/br_cmd_vel, /tr_cmd_velの速度指令(mm/s, rad/s)をbodyに反映する。"""
        self.body.velocity = (vx_mm_s, vy_mm_s)
        self.body.angular_velocity = omega_rad_s


def make_robot_body(
    space: pymunk.Space,
    robot_id: str,
    position: tuple[float, float],
    level: int,
) -> RobotBody:
    size = fc.ROBOT_INITIAL_MAX[0]
    mass = fc.ROBOT_WEIGHT_MAX_KG
    body = pymunk.Body(mass, pymunk.moment_for_box(mass, (size, size)))
    body.position = position
    shape = pymunk.Poly.create_box(body, (size, size))
    shape.friction = 0.8
    shape.filter = level_filter(level)
    # 一度でも保持されたブロックとは以後衝突しないようにするpre_solveハンドラの
    # 対象識別に使う(physics_blocks.PhysicsBlock.set_held_by参照)
    shape.collision_type = ROBOT_COLLISION_TYPE
    space.add(body, shape)
    return RobotBody(robot_id, body, shape, level)
