"""
ABU Robocon 2027 Phase 1 2Dシム: 柵と地形(段差)の簡易モデル。

2Dシムは高さを持たないので、段差は次の2つで表す:
- 柵(WALL_SEGMENTS): ロボットが越えられない線分。L1/L2の外周、スロープ・階段・
  受渡しエリアの側面、中央の仕切り、フィールド外周。開口はスロープの下端、
  L1階段の下端、受渡しエリア/踊り場とL1の境目、L2階段の上端だけ。
- 地形(region_at/level_at): 位置からその場所の種類と高さ(階層)を決める。
  スロープ・階段は速度倍率を掛ける場所として扱う(robot_spec参照)。

座標はfield_constantsの赤チーム側で定義し、青チーム側は中央線で鏡映する。
シム(柵の物理・ロボットの階層)、経路計画、描画がこのモジュールを共有する。
"""

from __future__ import annotations

from . import field_constants as fc
from .physics_blocks import LEVEL_GROUND, LEVEL_L1, LEVEL_L2

Point = tuple[float, float]
Rect = tuple[tuple[float, float], tuple[float, float]]  # (原点, サイズ)

_L1_U0, _L1_V0 = fc.L1_ORIGIN
_L1_U1, _L1_V1 = _L1_U0 + fc.L1_SIZE, _L1_V0 + fc.L1_SIZE
_L2_U0, _L2_V0 = fc.L2_ORIGIN
_L2_U1, _L2_V1 = _L2_U0 + fc.L2_SIZE, _L2_V0 + fc.L2_SIZE
_T_U0 = fc.TRANSITION_U  # スロープ・踊り場・受渡し・L1階段の外側の辺
_STAIRS_TOP_V = fc.L1_STAIRS_ORIGIN[1] + fc.L1_STAIRS_SIZE[1]
_LANDING_TOP_V = fc.RAMP_LANDING_ORIGIN[1] + fc.RAMP_LANDING_SIZE[1]
_L2S_U0, _L2S_V0 = fc.L2_STAIRS_ORIGIN
_L2S_V1 = _L2S_V0 + fc.L2_STAIRS_SIZE[1]
_MID = fc.FIELD_CENTER


def _mirror_point(p: Point) -> Point:
    return fc.mirror_to_blue(*p)


def mirror_rect(rect: Rect) -> Rect:
    origin, size = rect
    return (fc.mirror_rect_origin(origin, size), size)


def _red_side_walls() -> list[tuple[str, Point, Point]]:
    return [
        # L1外周(U=0側の辺は、受渡しエリア+踊り場に面する部分だけ開口)
        ('barrier', (_L1_U0, _L1_V0), (_L1_U0, _STAIRS_TOP_V)),
        ('barrier', (_L1_U0, _LANDING_TOP_V), (_L1_U0, _L1_V1)),
        ('barrier', (_L1_U0, _L1_V0), (_MID, _L1_V0)),
        ('barrier', (_L1_U0, _L1_V1), (_MID, _L1_V1)),
        # スロープ・踊り場・受渡し・L1階段の外側の辺(スロープ下端と階段下端は開口)
        ('barrier', (_T_U0, _L1_V0), (_T_U0, _L1_V1)),
        # L2外周(L2階段の上端だけ開口)
        ('barrier', (_L2_U0, _L2_V0), (_L2_U0, _L2S_V0)),
        ('barrier', (_L2_U0, _L2S_V1), (_L2_U0, _L2_V1)),
        ('barrier', (_L2_U0, _L2_V0), (_MID, _L2_V0)),
        ('barrier', (_L2_U0, _L2_V1), (_MID, _L2_V1)),
        # L2階段の側面(L1側から上る)
        ('barrier', (_L2S_U0, _L2S_V0), (_L2_U0, _L2S_V0)),
        ('barrier', (_L2S_U0, _L2S_V1), (_L2_U0, _L2S_V1)),
    ]


def _center_divider() -> list[tuple[str, Point, Point]]:
    """グラウンドとL1の中心線の仕切り。L2と、中央線上の共用エリアの部分は途切れる。"""
    gaps = sorted([
        (fc.GROUND_SHARED_AREA_ORIGIN[1], fc.GROUND_SHARED_AREA_ORIGIN[1] + fc.GROUND_SHARED_AREA_SIZE[1]),
        (_L2_V0, _L2_V1),
        (fc.MUSTIKA_SHARED_AREA_ORIGIN[1], fc.MUSTIKA_SHARED_AREA_ORIGIN[1] + fc.MUSTIKA_SHARED_AREA_SIZE[1]),
        *((o[1], o[1] + fc.L1_SHARED_AREA_SIZE[1]) for o in fc.L1_SHARED_AREA_ORIGINS),
    ])
    segments = []
    v = 0.0
    for start, end in gaps + [(fc.GAME_FIELD_SIZE, fc.GAME_FIELD_SIZE)]:
        if start > v:
            segments.append(('divider', (_MID, v), (_MID, start)))
        v = max(v, end)
    return segments


def _build_walls() -> list[tuple[str, Point, Point]]:
    n = fc.GAME_FIELD_SIZE
    walls = [
        ('boundary', (0, 0), (n, 0)),
        ('boundary', (n, 0), (n, n)),
        ('boundary', (n, n), (0, n)),
        ('boundary', (0, n), (0, 0)),
    ]
    for kind, p0, p1 in _red_side_walls():
        walls.append((kind, p0, p1))
        walls.append((kind, _mirror_point(p0), _mirror_point(p1)))
    return walls + _center_divider()


# (種類, 始点, 終点)。種類は'boundary'(外周)|'divider'(中央の仕切り)|'barrier'(段差の縁)
WALL_SEGMENTS: list[tuple[str, Point, Point]] = _build_walls()


_RED_REGIONS: list[tuple[str, Rect]] = [
    ('l2_stairs', (fc.L2_STAIRS_ORIGIN, fc.L2_STAIRS_SIZE)),
    ('ramp', (fc.RAMP_ORIGIN, fc.RAMP_SIZE)),
    ('landing', (fc.RAMP_LANDING_ORIGIN, fc.RAMP_LANDING_SIZE)),
    ('transfer', (fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE)),
    ('l1_stairs', (fc.L1_STAIRS_ORIGIN, fc.L1_STAIRS_SIZE)),
]
_REGION_LEVEL = {
    'ground': LEVEL_GROUND,
    'ramp': LEVEL_L1, 'landing': LEVEL_L1, 'transfer': LEVEL_L1, 'l1_stairs': LEVEL_L1, 'l1': LEVEL_L1,
    'l2_stairs': LEVEL_L2, 'l2': LEVEL_L2,
}


def _in_rect(u: float, v: float, rect: Rect) -> bool:
    (u0, v0), (w, h) = rect
    return u0 <= u <= u0 + w and v0 <= v <= v0 + h


def region_at(u: float, v: float) -> str:
    """その位置の地形: ground|ramp|landing|transfer|l1_stairs|l1|l2_stairs|l2。"""
    if u > _MID:
        u = fc.GAME_FIELD_SIZE - u  # 青側は鏡映して赤側の定義で判定する
    if _L2_U0 <= u and _L2_V0 <= v <= _L2_V1:
        return 'l2'
    for name, rect in _RED_REGIONS:
        if _in_rect(u, v, rect):
            return name
    if _L1_U0 <= u and _L1_V0 <= v <= _L1_V1:
        return 'l1'
    return 'ground'


def level_at(u: float, v: float) -> int:
    """その位置の高さ(階層)。スロープ・階段・踊り場・受渡しエリアは上の階層として扱う。"""
    return _REGION_LEVEL[region_at(u, v)]


def speed_scale_at(spec, u: float, v: float) -> float:
    region = region_at(u, v)
    if region == 'ramp':
        return spec.ramp_speed_scale
    if region in ('l1_stairs', 'l2_stairs'):
        return spec.stairs_speed_scale
    return 1.0


def forbidden_terrain_rects(spec) -> list[Rect]:
    """機体仕様上そのロボットが入れない地形(階段/スロープ)の矩形。赤青両側。"""
    rects: list[Rect] = []
    if not spec.can_use_stairs:
        rects += [(fc.L1_STAIRS_ORIGIN, fc.L1_STAIRS_SIZE), (fc.L2_STAIRS_ORIGIN, fc.L2_STAIRS_SIZE)]
    if not spec.can_use_ramp:
        rects += [(fc.RAMP_ORIGIN, fc.RAMP_SIZE)]
    return rects + [mirror_rect(r) for r in rects]


# 6.2.1: TRが入ってはならないL1/L2の鉛直境界
TR_KEEP_OUT_RECTS: list[Rect] = [
    (fc.L1_ORIGIN, (fc.L1_SIZE, fc.L1_SIZE)),
    (fc.L2_ORIGIN, (fc.L2_SIZE, fc.L2_SIZE)),
]


# 柵(フィールド外周等)の太さ。シムの物理(2D/3D)で使う
WALL_THICKNESS_MM = 10.0
# スタートゾーンに置くとき、柵や隣のロボットと重ならないための隙間
START_SPAWN_CLEARANCE_MM = 10.0


def start_positions(specs) -> dict[str, tuple[float, float]]:
    """TR/BRの開始位置(シムの初期配置と、本番の推測航法の初期値で共有)。
    スタートゾーン(原点の角から2個並ぶ)のうち、どちらをTR/BRに使うかはチームの選択で、
    角側にTR(北のストレージへまっすぐ向かう)、その隣にBR(北東のL1階段へ向かう)を置くと
    出だしの経路が交差しない。外周の柵と互いに重ならないよう少し離す。"""
    tr_half = specs['tr'].half_width_mm
    br_half = specs['br'].half_width_mm
    edge = WALL_THICKNESS_MM / 2 + START_SPAWN_CLEARANCE_MM
    tr = (edge + tr_half, edge + tr_half)
    br = (tr[0] + tr_half + START_SPAWN_CLEARANCE_MM + br_half, edge + br_half)
    return {'tr': tr, 'br': br}


def floor_height_at(u: float, v: float) -> float:
    """その位置の床の高さ(mm)。スロープは下端0→上端L1の直線、L1階段は1段ごとに上がる。"""
    region = region_at(u, v)
    if region == 'ground':
        return 0.0
    if region in ('l1', 'transfer', 'landing'):
        return float(fc.L1_Z)
    if region == 'l2':
        return float(fc.L2_Z)
    if region == 'l2_stairs':
        return float(fc.L1_Z + fc.STAIR_STEP_HEIGHT)
    if region == 'ramp':
        top_v = fc.RAMP_ORIGIN[1]  # 受渡しエリア側(L1の高さ)
        return max(0.0, min(float(fc.L1_Z), fc.L1_Z * (1.0 - (v - top_v) / fc.RAMP_LENGTH)))
    # l1_stairs: スタートゾーン側(V小)が下。1段目150, 2段目300, 3段目450
    step = min(fc.L1_STAIR_STEP_COUNT - 1, max(0, int((v - fc.L1_STAIRS_ORIGIN[1]) // fc.STAIR_STEP_LENGTH)))
    return float((step + 1) * fc.STAIR_STEP_HEIGHT)
