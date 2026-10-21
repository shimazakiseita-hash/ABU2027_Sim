"""
ABU Robocon 2027 Phase 1 2Dシム: TR/BRの機体仕様(config/robot_spec.yaml)の読み込み。

速度・加速度・車体寸法・把持距離・機構の動作時間・通れる地形をここに集め、
シム・意思決定ノード・操縦コンソール・審判が同じ値を使う。機体要件が決まったら
YAMLを書き換える(または起動引数 robot_spec_file で別ファイルを渡す)だけで
すべてに反映される。
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

from . import field_constants as fc

ROBOT_IDS = ('tr', 'br')

_BLOCK_HALF_SIZE_MM = {
    'earth': fc.EARTH_BLOCK_SIZE / 2,
    'sky': fc.SKY_BLOCK_SIZE / 2,
    'mustika': fc.MUSTIKA_DIAMETER / 2,
}


@dataclass(frozen=True)
class RobotSpec:
    footprint_mm: tuple[float, float]
    max_speed_m_s: float
    max_accel_m_s2: float
    max_yaw_rate_rad_s: float
    max_yaw_accel_rad_s2: float
    ramp_speed_scale: float
    stairs_speed_scale: float
    can_use_ramp: bool
    can_use_stairs: bool
    grasp_reach_mm: float
    grasp_time_s: float
    release_time_s: float
    place_time_s: float
    flip_time_s: float

    @property
    def half_width_mm(self) -> float:
        """経路計画・到着判定に使う車体の半幅(長さと幅の大きい方の半分)。"""
        return max(self.footprint_mm) / 2

    def grasp_range_mm(self, block_type: str) -> float:
        """ロボット中心からこの距離以内の物をつかめる(グリッパーを車体中心に置く簡略化)。"""
        return self.half_width_mm + self.grasp_reach_mm + _BLOCK_HALF_SIZE_MM.get(block_type, 0.0)


def default_spec_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('br_strategy_sim'), 'config', 'robot_spec.yaml')


def load_robot_specs(path: str | None = None) -> dict[str, RobotSpec]:
    with open(path or default_spec_path(), encoding='utf-8') as f:
        data = yaml.safe_load(f)
    specs = {}
    for robot in ROBOT_IDS:
        raw = dict(data[robot])
        raw['footprint_mm'] = tuple(float(x) for x in raw['footprint_mm'])
        specs[robot] = RobotSpec(**raw)
    return specs


def declare_robot_specs(node) -> dict[str, RobotSpec]:
    """ノードにrobot_spec_fileパラメータを宣言し、その仕様(空なら既定ファイル)を読む。"""
    node.declare_parameter('robot_spec_file', '')
    path = node.get_parameter('robot_spec_file').value or None
    return load_robot_specs(path)
