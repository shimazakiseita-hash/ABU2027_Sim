"""
ABU Robocon 2027 Phase 1 2Dシム: BR/TRの簡易矩形表現(pymunk body/shape)。

車体の寸法・最高速度・加減速・旋回の上限は機体仕様(robot_spec.RobotSpec)から取る。
/cmd_velは目標速度として保存するだけで、物理ステップごとにstep()が上限加速度で
現在速度を目標へ近づける(速度が瞬時に変わらない)。向きは指令した角速度でだけ変わる
(慣性モーメントを無限大にして、衝突では回らない。実機がジャイロ等で向きを保つ想定)。

衝突フィルタ: ロボットは同じ階層のブロック・ロボットと、柵(WALL_CATEGORY)と、
自分だけが入れない地形(TR_ONLY/BR_ONLY_CATEGORY)にぶつかる。ブロックは柵を
すり抜ける(場外判定は位置で行う簡略化のまま)。区域侵犯・押出し違反は
br_referee_nodeが/true_state座標を見て判定する前提で、pymunkの衝突には依存しない。
"""

from __future__ import annotations

import math

import pymunk

from . import field_geometry as fg
from .physics_blocks import _LEVEL_CATEGORY, BOUNDARY_CATEGORY, ROBOT_COLLISION_TYPE

M_TO_MM = 1000.0

WALL_CATEGORY = 0b0001000
_ONLY_CATEGORY = {'tr': 0b0010000, 'br': 0b0100000}
_ALL_LEVELS = 0b0000111


def wall_filter(kind: str) -> pymunk.ShapeFilter:
    """柵の衝突フィルタ。外周('boundary')はブロックも止め、それ以外はロボットだけを止める。"""
    category = BOUNDARY_CATEGORY if kind == 'boundary' else WALL_CATEGORY
    return pymunk.ShapeFilter(categories=category, mask=_ALL_LEVELS)


def robot_only_filter(robot_id: str) -> pymunk.ShapeFilter:
    """robot_idのロボットだけがぶつかる形状(そのロボットが通れない地形)用。"""
    return pymunk.ShapeFilter(categories=_ONLY_CATEGORY[robot_id], mask=_ALL_LEVELS)


def _robot_filter(robot_id: str, level: int) -> pymunk.ShapeFilter:
    category = _LEVEL_CATEGORY[level]
    return pymunk.ShapeFilter(
        categories=category, mask=category | WALL_CATEGORY | BOUNDARY_CATEGORY | _ONLY_CATEGORY[robot_id])


def _approach(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


class RobotBody:
    """BR/TR共通の簡易矩形ラッパー。"""

    def __init__(self, robot_id: str, body: pymunk.Body, shape: pymunk.Shape, level: int, spec):
        self.robot_id = robot_id
        self.body = body
        self.shape = shape
        self.spec = spec
        self.level = level
        self._cmd = (0.0, 0.0, 0.0)  # 目標速度(mm/s, mm/s, rad/s)
        self._omega = 0.0

    @property
    def position(self) -> pymunk.Vec2d:
        return self.body.position

    def set_level(self, level: int) -> None:
        if level != self.level:
            self.level = level
            self.shape.filter = _robot_filter(self.robot_id, level)

    def set_cmd_vel(self, vx_mm_s: float, vy_mm_s: float, omega_rad_s: float) -> None:
        """/br_cmd_vel, /tr_cmd_velの速度指令(mm/s, rad/s)を目標速度として保存する。"""
        self._cmd = (vx_mm_s, vy_mm_s, omega_rad_s)

    def stop_now(self) -> None:
        """9.4.1: 試合終了時は加減速を待たずに止める。"""
        self._cmd = (0.0, 0.0, 0.0)
        self.body.velocity = (0, 0)
        self.body.angular_velocity = 0.0
        self._omega = 0.0

    def step(self, dt: float, speed_scale: float) -> None:
        """目標速度へ上限加速度で近づける。speed_scaleは地形(スロープ/階段)の速度倍率。"""
        vmax = self.spec.max_speed_m_s * M_TO_MM * speed_scale
        tx, ty, tw = self._cmd
        norm = math.hypot(tx, ty)
        if norm > vmax:
            tx, ty = tx / norm * vmax, ty / norm * vmax
        vx, vy = self.body.velocity
        dvx, dvy = tx - vx, ty - vy
        dv = math.hypot(dvx, dvy)
        max_dv = self.spec.max_accel_m_s2 * M_TO_MM * dt
        if dv > max_dv:
            dvx, dvy = dvx / dv * max_dv, dvy / dv * max_dv
        self.body.velocity = (vx + dvx, vy + dvy)

        tw = max(-self.spec.max_yaw_rate_rad_s, min(self.spec.max_yaw_rate_rad_s, tw))
        # 衝突で付いた角速度を引き継がず、前回の指令由来の角速度から上限で近づける
        self._omega = _approach(self._omega, tw, self.spec.max_yaw_accel_rad_s2 * dt)
        self.body.angular_velocity = self._omega


def make_robot_body(
    space: pymunk.Space,
    robot_id: str,
    position: tuple[float, float],
    level: int,
    spec,
    mass_kg: float,
) -> RobotBody:
    length, width = spec.footprint_mm
    # 車体の長さ方向を前(+v, theta=0)に合わせる
    size = (width, length)
    body = pymunk.Body(mass_kg, float('inf'))
    body.position = position
    shape = pymunk.Poly.create_box(body, size)
    shape.friction = 0.8
    shape.filter = _robot_filter(robot_id, level)
    # 一度でも保持されたブロックとは以後衝突しないようにするpre_solveハンドラの
    # 対象識別に使う(physics_blocks.PhysicsBlock.set_held_by参照)
    shape.collision_type = ROBOT_COLLISION_TYPE
    space.add(body, shape)
    return RobotBody(robot_id, body, shape, level, spec)


def add_static_field(space: pymunk.Space, robots) -> None:
    """柵(field_geometry.WALL_SEGMENTS)と、ロボットごとの通れない地形を静的形状として置く。"""
    static = space.static_body
    for kind, p0, p1 in fg.WALL_SEGMENTS:
        segment = pymunk.Segment(static, p0, p1, fg.WALL_THICKNESS_MM / 2)
        segment.filter = wall_filter(kind)
        segment.friction = 0.5
        space.add(segment)
    for robot in robots:
        for (u0, v0), (w, h) in fg.forbidden_terrain_rects(robot.spec):
            poly = pymunk.Poly(static, [(u0, v0), (u0 + w, v0), (u0 + w, v0 + h), (u0, v0 + h)])
            poly.filter = robot_only_filter(robot.robot_id)
            space.add(poly)
