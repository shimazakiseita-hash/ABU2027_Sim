"""
br_decision_tr_node / br_decision_br_nodeで共通して使う移動・受渡しの補助。

移動そのもの(経路計画と追従)はnavigation.Navigatorが行う。ここには、受渡しの
待ち合わせ位置、秘蹟の要件の判定、目標のすぐ近くでの微調整用のdrive_towardを置く。
速度・加速度・到着距離は機体仕様(robot_spec.RobotSpec)から取る。
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist

from . import field_constants as fc

ARRIVAL_THRESHOLD_MM = 150.0


def transfer_area_rect() -> tuple[tuple[float, float], tuple[float, float]]:
    return fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE


def point_in_rect(x: float, y: float, origin: tuple[float, float], size: tuple[float, float]) -> bool:
    return origin[0] <= x <= origin[0] + size[0] and origin[1] <= y <= origin[1] + size[1]


# tr_transfer_release_point/br_transfer_wait_pointへの接近には、デフォルトの
# ARRIVAL_THRESHOLD_MM(150mm)ではなくこちらを使うこと。到着判定が成立する
# 瞬間の位置によってはブロック/ムスティカの解放位置が受渡しエリアの境界
# ギリギリになり得て、受け手側のpoint_in_rect判定が恒久的に満たされず
# 無限待機してしまう(複数塔+ムスティカの統合テストで発覚)ため、小さい閾値を使う。
TRANSFER_POINT_ARRIVAL_THRESHOLD_MM = 80.0


def _transfer_center() -> tuple[float, float]:
    origin, size = transfer_area_rect()
    return (origin[0] + size[0] / 2, origin[1] + size[1] / 2)


def tr_transfer_release_point() -> tuple[float, float]:
    """TRが物を置く位置。受渡しエリアは幅1000mmで車体1台分しか入らないので中央。"""
    return _transfer_center()


def br_transfer_wait_point(br_spec) -> tuple[float, float]:
    """BRの待ち位置。TRとぶつからないよう、受渡しエリアに面したL1の開口のすぐ内側で待つ。"""
    return (fc.L1_ORIGIN[0] + br_spec.half_width_mm + 150.0, _transfer_center()[1])


_SHARED_BUILD_SPOT_IDS = {spot_id for spot_id, _level, team, _origin in fc.BUILD_SPOTS if team == ''}


def sanctuary_mandate_satisfied(towers, required_layers: int) -> bool:
    """3.6/4.5.1(秘蹟の要件, Sanctuary Mandate): 完成塔(積み上げ段数が
    required_layers以上)が2つ以上、かつそのうち少なくとも1つが共有エリアの
    建築スポットにあること。これを満たすまでTRはムスティカを回収できない。
    共有エリアの建築スポットは、L2の全スポット(ルールブック上「全体が共用
    エリア」)とL1中央線上の共用スポット(field_constants.BUILD_SPOTSでteam="")。
    towersはTowerState(build_spot_id, level, block_types, ...)相当のリスト。"""
    complete = [t for t in towers if len(t.block_types) >= required_layers]
    if len(complete) < 2:
        return False
    return any(t.level == 2 or t.build_spot_id in _SHARED_BUILD_SPOT_IDS for t in complete)


def drive_toward(
    current_uv: tuple[float, float],
    target_uv: tuple[float, float],
    spec,
    arrival_threshold_mm: float = ARRIVAL_THRESHOLD_MM,
) -> tuple[Twist, bool]:
    """current_uvからtarget_uvへ直進する速度指令(Twist, m/s)と、到着済みかを返す。
    柵を考えないので、物をつかむ直前など目標のすぐ近くでだけ使う(遠くへはNavigatorを使う)。
    上限加速度で止まりきれる速さまで手前から減速する。"""
    dx = target_uv[0] - current_uv[0]
    dy = target_uv[1] - current_uv[1]
    distance = math.hypot(dx, dy)
    twist = Twist()
    if distance <= arrival_threshold_mm:
        return twist, True
    speed = min(spec.max_speed_m_s, math.sqrt(2.0 * spec.max_accel_m_s2 * distance / 1000.0))
    twist.linear.x = (dx / distance) * speed
    twist.linear.y = (dy / distance) * speed
    return twist, False
