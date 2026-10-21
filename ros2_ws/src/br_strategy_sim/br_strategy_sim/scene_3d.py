"""
ABU Robocon 2027 3Dシム: フィールド・ロボット・競技物のMuJoCoモデル(MJCF)を作る。

座標はfield_constantsと同じu,v(mm)をMuJoCoのx,y(m)に、高さをzにとる。
静的な構造物(台・階段・スロープ・柱・柵)は当たり判定あり、床の塗り分けや
建築スポットは見た目だけの薄い板。ロボットは機体仕様の寸法の箱(mocap: 位置を
シムが直接与える)、ブロックとムスティカは自由に動く剛体。

衝突の組み合わせ(contype / conaffinity):
- 静的な構造物 1/1、まだ保持されたことのない物 1/3、ロボット 2/0
  → 物は構造物・物どうし・ロボットとぶつかる。ロボットは構造物・ロボットとはぶつからない
    (ロボットの平面の動きと柵は2Dと同じ平面モデルで扱うため)
- 一度でも保持された物は 4/5 に切り替える(sim_3d_node)→ ロボットとはぶつからない
  (2Dと同じ。解放直後の重なりで弾け飛ぶのを防ぐ)

形状のグループ(LiDARのレイの対象を選ぶのに使う): 0=構造物, 1=物, 2=ロボット, 3=見た目だけ。
"""

from __future__ import annotations

import math

from . import field_constants as fc
from . import field_geometry as fg
from .sim_bridge_node import earth_block_layout, sky_block_layout

MM = 0.001

GROUP_STATIC = 0
GROUP_OBJECT = 1
GROUP_ROBOT = 2
GROUP_VISUAL = 3

# 柵の高さはルールブックに記載が無いので仮値(床からの高さ)
BOUNDARY_HEIGHT_MM = 200.0
DIVIDER_HEIGHT_MM = 200.0
# 段差の縁の柵(L1/L2外周等)は、高い側の床からこれだけ上に出す
BARRIER_LIP_MM = 100.0
# 柵を高さを変えながら区切る長さ(スロープ脇などで高さが変わるため)
WALL_CHUNK_MM = 500.0
WALL_THICKNESS_MM = 20.0

# 中央の柱の上の受け(浅いくぼみ)を、柱の上面に並べた小さなカプセルの輪で近似する
SOCKET_RING_RADIUS_MM = fc.CENTRAL_PILLAR_SOCKET_DIAMETER / 2
SOCKET_RING_COUNT = 8
SOCKET_RING_THICKNESS_MM = 15.0

_COLORS = {
    'ground_red': (240, 210, 210), 'ground_blue': (170, 210, 230),
    'l1_red': (235, 180, 160), 'l1_blue': (150, 215, 220), 'l2': (190, 190, 185),
    'shared': (245, 240, 200), 'start_red': (223, 34, 34), 'start_blue': (50, 0, 255),
    'transfer_red': (245, 170, 60), 'transfer_blue': (60, 170, 245),
    'ramp_red': (200, 150, 140), 'ramp_blue': (130, 180, 200),
    'spot': (40, 100, 50), 'pillar': (100, 62, 0), 'wall': (110, 100, 95), 'boundary': (100, 62, 0),
    'earth_red': (223, 34, 34), 'earth_blue': (50, 0, 255), 'sky_red': (223, 34, 34), 'sky_blue': (50, 0, 255),
    'mustika': (230, 195, 40), 'robot_tr': (120, 122, 130), 'robot_br': (50, 52, 60),
}


def rgba(name: str, alpha: float = 1.0) -> str:
    r, g, b = _COLORS[name]
    return f'{r / 255:.3f} {g / 255:.3f} {b / 255:.3f} {alpha}'


def color_rgba(name: str) -> tuple[float, float, float, float]:
    r, g, b = _COLORS[name]
    return (r / 255, g / 255, b / 255, 1.0)


def _v(*values_mm) -> str:
    return ' '.join(f'{x * MM:.4f}' for x in values_mm)


class _Builder:
    def __init__(self):
        self.lines: list[str] = []
        self._count = 0

    def _name(self, prefix: str) -> str:
        self._count += 1
        return f'{prefix}_{self._count}'

    def box(self, center, half, color, static=True, euler=None, prefix='geom'):
        """center/halfはmm。static=Falseなら見た目だけ(当たり判定なし)。"""
        cls = 'static' if static else 'visual'
        rot = f' euler="{euler[0]:.5f} {euler[1]:.5f} {euler[2]:.5f}"' if euler else ''
        self.lines.append(f'<geom name="{self._name(prefix)}" class="{cls}" type="box" pos="{_v(*center)}" '
                          f'size="{_v(*half)}" rgba="{rgba(color)}"{rot}/>')

    def rect_slab(self, origin, size, z0, z1, color, static=True, prefix='geom'):
        (u0, v0), (w, h) = origin, size
        self.box((u0 + w / 2, v0 + h / 2, (z0 + z1) / 2), (w / 2, h / 2, (z1 - z0) / 2), color, static, prefix=prefix)

    def plate(self, origin, size, z, color, prefix='floor'):
        """床の塗り分け・区画の印など、見た目だけの薄い板(zはその床の高さ)。"""
        self.rect_slab(origin, size, z + 0.5, z + 1.5, color, static=False, prefix=prefix)

    def cylinder(self, center_uv, radius, z0, z1, color, prefix='pillar'):
        self.lines.append(f'<geom name="{self._name(prefix)}" class="static" type="cylinder" '
                          f'pos="{_v(center_uv[0], center_uv[1], (z0 + z1) / 2)}" size="{_v(radius, (z1 - z0) / 2)}" '
                          f'rgba="{rgba(color)}"/>')


def _both(origin, size):
    return [('red', origin), ('blue', fc.mirror_rect_origin(origin, size))]


def _static_field(b: _Builder) -> None:
    n = fc.GAME_FIELD_SIZE
    half = fc.FIELD_CENTER
    # 床の塗り分け(見た目)
    b.plate((0, 0), (half, n), 0, 'ground_red')
    b.plate((half, 0), (half, n), 0, 'ground_blue')
    for zone in (fc.START_ZONE_1_ORIGIN, fc.START_ZONE_2_ORIGIN):
        for team, origin in _both(zone, fc.START_ZONE_SIZE):
            b.plate(origin, fc.START_ZONE_SIZE, 1.0, f'start_{team}')
    b.plate(fc.GROUND_SHARED_AREA_ORIGIN, fc.GROUND_SHARED_AREA_SIZE, 1.0, 'shared')
    b.plate(fc.MUSTIKA_SHARED_AREA_ORIGIN, fc.MUSTIKA_SHARED_AREA_SIZE, 1.0, 'shared')

    # L1台(赤/青で色分け)と、L1の上の区画
    l1_red_w = half - fc.L1_ORIGIN[0]
    b.rect_slab(fc.L1_ORIGIN, (l1_red_w, fc.L1_SIZE), 0, fc.L1_Z, 'l1_red', prefix='l1')
    b.rect_slab((half, fc.L1_ORIGIN[1]), (fc.L1_SIZE - l1_red_w, fc.L1_SIZE), 0, fc.L1_Z, 'l1_blue', prefix='l1')
    for origin in fc.L1_SHARED_AREA_ORIGINS:
        b.plate(origin, fc.L1_SHARED_AREA_SIZE, fc.L1_Z, 'shared')
    for team, origin in _both(fc.L1_RETRY_ZONE_ORIGIN, fc.L1_RETRY_ZONE_SIZE):
        b.plate(origin, fc.L1_RETRY_ZONE_SIZE, fc.L1_Z + 1, f'start_{team}')

    # L1の外側: 受渡しエリア+踊り場の台、L1階段、スロープ(赤/青)
    for team, origin in _both(fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE):
        b.rect_slab(origin, fc.TRANSFER_AREA_SIZE, 0, fc.L1_Z, f'transfer_{team}', prefix='transfer')
    for team, origin in _both(fc.RAMP_LANDING_ORIGIN, fc.RAMP_LANDING_SIZE):
        b.rect_slab(origin, fc.RAMP_LANDING_SIZE, 0, fc.L1_Z, f'l1_{team}', prefix='landing')
    for k in range(fc.L1_STAIR_STEP_COUNT):
        step_origin = (fc.L1_STAIRS_ORIGIN[0], fc.L1_STAIRS_ORIGIN[1] + k * fc.STAIR_STEP_LENGTH)
        step_size = (fc.L1_STAIRS_SIZE[0], fc.STAIR_STEP_LENGTH)
        for team, origin in _both(step_origin, step_size):
            b.rect_slab(origin, step_size, 0, (k + 1) * fc.STAIR_STEP_HEIGHT, f'l1_{team}', prefix='stairs')
    # スロープ: 下端(V大)が床、上端(V小)がL1の高さの傾いた板
    slope = math.atan2(fc.L1_Z, fc.RAMP_LENGTH)
    half_len = math.hypot(fc.L1_Z, fc.RAMP_LENGTH) / 2
    thick = 20.0
    for team, origin in _both(fc.RAMP_ORIGIN, fc.RAMP_SIZE):
        cu = origin[0] + fc.RAMP_SIZE[0] / 2
        cv = origin[1] + fc.RAMP_SIZE[1] / 2
        # 板の上面が斜面に一致するよう、厚みの半分だけ法線方向に下げる
        cz = fc.L1_Z / 2 - thick / 2 * math.cos(slope)
        b.box((cu, cv, cz), (fc.RAMP_SIZE[0] / 2, half_len, thick / 2), f'ramp_{team}', euler=(-slope, 0, 0),
              prefix='ramp')

    # L2台とL2階段
    b.rect_slab(fc.L2_ORIGIN, (fc.L2_SIZE, fc.L2_SIZE), fc.L1_Z, fc.L2_Z, 'l2', prefix='l2')
    for team, origin in _both(fc.L2_STAIRS_ORIGIN, fc.L2_STAIRS_SIZE):
        b.rect_slab(origin, fc.L2_STAIRS_SIZE, fc.L1_Z, fc.L1_Z + fc.STAIR_STEP_HEIGHT, f'l1_{team}', prefix='l2stairs')

    # 建築スポット(見た目)
    for _spot_id, level, _team, origin in fc.BUILD_SPOTS:
        z = fc.L1_Z if level == 1 else fc.L2_Z
        b.plate(origin, fc.BUILD_SPOT_SIZE, z + 1, 'spot')

    # 柱と、中央の柱の上の受け
    b.cylinder(fc.MUSTIKA_PILLAR_ORIGIN, fc.MUSTIKA_PILLAR_DIAMETER / 2, 0, fc.MUSTIKA_PILLAR_HEIGHT, 'pillar')
    top = fc.L2_Z + fc.CENTRAL_PILLAR_HEIGHT
    b.cylinder((fc.FIELD_CENTER, fc.FIELD_CENTER), fc.CENTRAL_PILLAR_DIAMETER / 2, fc.L2_Z, top, 'pillar')
    for k in range(SOCKET_RING_COUNT):
        a = 2 * math.pi * k / SOCKET_RING_COUNT
        cu = fc.FIELD_CENTER + SOCKET_RING_RADIUS_MM * math.cos(a)
        cv = fc.FIELD_CENTER + SOCKET_RING_RADIUS_MM * math.sin(a)
        b.box((cu, cv, top + SOCKET_RING_THICKNESS_MM), (SOCKET_RING_THICKNESS_MM,) * 3, 'pillar', prefix='socket')

    # 柵: 2Dと同じ線分を、場所ごとの床の高さに合わせた壁にする
    for kind, p0, p1 in fg.WALL_SEGMENTS:
        _wall(b, kind, p0, p1)


def _wall(b: _Builder, kind: str, p0, p1) -> None:
    length = math.dist(p0, p1)
    chunks = max(1, math.ceil(length / WALL_CHUNK_MM))
    du, dv = (p1[0] - p0[0]) / chunks, (p1[1] - p0[1]) / chunks
    horizontal = abs(dv) < 1e-6
    for k in range(chunks):
        a = (p0[0] + du * k, p0[1] + dv * k)
        c = (a[0] + du / 2, a[1] + dv / 2)
        # 壁の両側の床の高さ(柵のすぐ内側/外側)のうち高い方を基準にする
        off = (0, 60) if horizontal else (60, 0)
        floors = [fg.floor_height_at(min(max(c[0] + s * off[0], 1), fc.GAME_FIELD_SIZE - 1),
                                     min(max(c[1] + s * off[1], 1), fc.GAME_FIELD_SIZE - 1)) for s in (-1, 1)]
        top = max(floors) + {'boundary': BOUNDARY_HEIGHT_MM, 'divider': DIVIDER_HEIGHT_MM}.get(kind, BARRIER_LIP_MM)
        half_u = abs(du) / 2 + WALL_THICKNESS_MM / 2
        half_v = abs(dv) / 2 + WALL_THICKNESS_MM / 2
        color = 'boundary' if kind != 'barrier' else 'wall'
        b.box((c[0], c[1], top / 2), (half_u, half_v, top / 2), color, prefix='wall')


def object_specs():
    """3Dシムで動かす物の(id, 種類, 初期位置uvz(mm), 半径相当(mm), 色, 所有チーム, 上面の色)。"""
    objects = []
    for block_id, (u, v) in earth_block_layout():
        objects.append((block_id, 'earth', (u, v, fc.EARTH_BLOCK_SIZE / 2), fc.EARTH_BLOCK_SIZE / 2,
                        'earth_red', fc.TeamColor.RED, ''))
    for block_id, (u, v), color in sky_block_layout():
        objects.append((block_id, 'sky', (u, v, fc.SKY_BLOCK_SIZE / 2), fc.SKY_BLOCK_SIZE / 2,
                        f'sky_{color}', '', color))
    pu, pv = fc.MUSTIKA_PILLAR_ORIGIN
    objects.append(('mustika', 'mustika', (pu, pv, fc.MUSTIKA_PILLAR_HEIGHT + fc.MUSTIKA_DIAMETER / 2),
                    fc.MUSTIKA_DIAMETER / 2, 'mustika', '', ''))
    return objects


def _mass_kg(weight_range_g) -> float:
    return sum(weight_range_g) / 2 / 1000


def build_scene_xml(specs, timestep_s: float) -> str:
    b = _Builder()
    _static_field(b)
    static_xml = '\n      '.join(b.lines)

    robots = []
    starts = fg.start_positions(specs)
    for robot_id in ('tr', 'br'):
        spec = specs[robot_id]
        length, width = spec.footprint_mm
        u, v = starts[robot_id]
        z = spec.body_height_mm / 2
        robots.append(f'''
    <body name="{robot_id}" mocap="true" pos="{_v(u, v, z)}">
      <geom name="{robot_id}_body" class="robot" type="box" size="{_v(width / 2, length / 2, spec.body_height_mm / 2)}" rgba="{rgba(f'robot_{robot_id}')}"/>
      <geom name="{robot_id}_front" class="robot_marker" type="box" pos="{_v(0, length / 2 - 60, spec.body_height_mm / 2 + 5)}" size="{_v(width / 4, 50, 5)}" rgba="0.95 0.95 0.95 1"/>
    </body>''')

    objects = []
    welds = []
    masses = {'earth': _mass_kg(fc.EARTH_BLOCK_WEIGHT_RANGE_G), 'sky': _mass_kg(fc.SKY_BLOCK_WEIGHT_RANGE_G),
              'mustika': _mass_kg(fc.MUSTIKA_WEIGHT_RANGE_G)}
    for obj_id, kind, (u, v, z), half, color, _team, _top in object_specs():
        if kind == 'mustika':
            geom = f'type="sphere" size="{_v(half)}"'
        else:
            geom = f'type="box" size="{_v(half, half, half)}"'
        objects.append(f'''
    <body name="{obj_id}" pos="{_v(u, v, z)}">
      <freejoint/>
      <geom name="{obj_id}" class="object" {geom} mass="{masses[kind]:.4f}" rgba="{rgba(color)}"/>
    </body>''')
        for robot_id in ('tr', 'br'):
            welds.append(f'<weld name="hold_{robot_id}_{obj_id}" body1="{robot_id}" body2="{obj_id}" active="false"/>')
    welds.append('<weld name="fix_mustika" body1="mustika" active="false"/>')

    return f'''<mujoco model="abu2027_field">
  <compiler angle="radian"/>
  <option timestep="{timestep_s:.6f}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="2048"/>
  </visual>
  <default>
    <geom friction="0.6 0.005 0.0001"/>
    <default class="static"><geom contype="1" conaffinity="1" group="{GROUP_STATIC}"/></default>
    <default class="visual"><geom contype="0" conaffinity="0" group="{GROUP_VISUAL}"/></default>
    <default class="object"><geom contype="1" conaffinity="3" group="{GROUP_OBJECT}" condim="4"/></default>
    <default class="robot"><geom contype="2" conaffinity="0" group="{GROUP_ROBOT}"/></default>
    <default class="robot_marker"><geom contype="0" conaffinity="0" group="{GROUP_VISUAL}"/></default>
  </default>
  <worldbody>
    <light pos="5.5 5.5 12" dir="0 0 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <light pos="0 0 8" dir="0.5 0.5 -1" diffuse="0.3 0.3 0.3" castshadow="false"/>
    <geom name="floor" class="static" type="plane" pos="5.5 5.5 0" size="8 8 0.1" rgba="0.35 0.36 0.4 1"/>
      {static_xml}
    {''.join(robots)}
    {''.join(objects)}
  </worldbody>
  <equality>
    {chr(10).join('    ' + w for w in welds)}
  </equality>
</mujoco>
'''
