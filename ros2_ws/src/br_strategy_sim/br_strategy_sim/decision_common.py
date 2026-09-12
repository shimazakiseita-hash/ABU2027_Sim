"""
br_decision_tr_node / br_decision_br_nodeで共通して使う簡易移動コントローラ。

経路計画や障害物回避は行わない、目標点への直線的な速度指令を返すだけの
最小実装(最初のゴールである「アースブロック1個をTR->BR->建築スポットへ
運ぶ」ループを通すための割り切り。要調整)。
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist

from . import field_constants as fc

CRUISE_SPEED_M_S = 0.3
ARRIVAL_THRESHOLD_MM = 150.0

# ロボット中心-ブロック中心間で実際に到達できる最短距離の余裕。
# physics_blocks.GRASP_MARGIN_MMと同じ考え方(ロボットは無視できない大きさの
# footprintを持つので、ブロックへ"到着"したとみなす距離はブロック中心そのもの
# ではなく、ロボット半幅+ブロック半径相当まで)。
_ROBOT_HALF_WIDTH_MM = fc.ROBOT_INITIAL_MAX[0] / 2
# physics_blocks.GRASP_MARGIN_MMと同じ値に保つこと
_BLOCK_APPROACH_MARGIN_MM = 150.0


def block_arrival_threshold_mm(block_half_size_mm: float) -> float:
    """ブロック(半径相当block_half_size_mm)に物理的に接触できる距離を返す。
    ロボット自身の中心を"グリッパー位置"として使う簡略化のため、ブロック中心
    そのものには到達できない(physics_blocks._grasp_range_mmと同じ考え方)。"""
    return _ROBOT_HALF_WIDTH_MM + _BLOCK_APPROACH_MARGIN_MM + block_half_size_mm


def transfer_area_rect() -> tuple[tuple[float, float], tuple[float, float]]:
    return fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE


def point_in_rect(x: float, y: float, origin: tuple[float, float], size: tuple[float, float]) -> bool:
    return origin[0] <= x <= origin[0] + size[0] and origin[1] <= y <= origin[1] + size[1]


# TR/BRが受渡しエリア内で完全に同じ点を目指すと、ロボット同士(700mm角)が
# 正面衝突して想定位置から押し出されてしまう(統合テストで発覚: 受渡し後に
# BRが把持レンジ外まで押し出され、いつまでも受渡しブロックを掴めなかった)。
# エリア境界内(1000x1000mm)に収まる範囲で、TR/BRそれぞれに少しずらした
# 待ち合わせ地点を与えて正面衝突を避ける。
_TRANSFER_TARGET_OFFSET_MM = 350.0


def tr_transfer_release_point() -> tuple[float, float]:
    origin, size = transfer_area_rect()
    center = (origin[0] + size[0] / 2, origin[1] + size[1] / 2)
    return (center[0] - _TRANSFER_TARGET_OFFSET_MM, center[1] - _TRANSFER_TARGET_OFFSET_MM)


def br_transfer_wait_point() -> tuple[float, float]:
    origin, size = transfer_area_rect()
    center = (origin[0] + size[0] / 2, origin[1] + size[1] / 2)
    return (center[0] + _TRANSFER_TARGET_OFFSET_MM, center[1] + _TRANSFER_TARGET_OFFSET_MM)


def drive_toward(
    current_uv: tuple[float, float],
    target_uv: tuple[float, float],
    arrival_threshold_mm: float = ARRIVAL_THRESHOLD_MM,
) -> tuple[Twist, bool]:
    """current_uvからtarget_uvへ向かう速度指令(Twist, m/s)を返す。2値目は到着済みか。"""
    dx = target_uv[0] - current_uv[0]
    dy = target_uv[1] - current_uv[1]
    distance = math.hypot(dx, dy)
    twist = Twist()
    if distance <= arrival_threshold_mm:
        return twist, True
    twist.linear.x = (dx / distance) * CRUISE_SPEED_M_S
    twist.linear.y = (dy / distance) * CRUISE_SPEED_M_S
    return twist, False
