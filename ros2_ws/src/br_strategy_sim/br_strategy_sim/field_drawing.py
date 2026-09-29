"""
ABU Robocon 2027 Phase 1 2Dシム: pygame描画の共通処理。

reference/draw_field_preview.py由来のフィールド静的要素の描画と、
座標変換(mm<->px)を、scripts/main.py(静止プレビュー)とbr_visualizer_node
(動的可視化)の両方から共有するためのモジュール。ROS2やpymunkには依存しない。

フィールドの色は公式ルールブック14章(Colors and Materials Specification)の値。
画面の文字は日本語(操縦コンソールと同じくソフトに詳しくないメンバーも見るため)。
"""

from __future__ import annotations

import math
import os

import pygame

from . import field_constants as fc

MARGIN = 60
SCALE = 0.09  # 1mm -> 0.09px  (11000mm -> 約990px)
WINDOW_SIZE = int(fc.GAME_FIELD_SIZE * SCALE) + MARGIN * 2

# フィールド外(画面の余白)。フィールドが浮いて見えるよう暗めにする
COLOR_BG = (58, 60, 66)
COLOR_TEXT = (20, 20, 20)
COLOR_TEXT_LIGHT = (235, 235, 238)
COLOR_LABEL_BG = (255, 255, 255, 190)

COLOR_GROUND = {fc.TeamColor.RED: (240, 210, 210), fc.TeamColor.BLUE: (170, 210, 230)}
COLOR_STORAGE_EDGE = {fc.TeamColor.RED: (205, 150, 150), fc.TeamColor.BLUE: (110, 160, 195)}
COLOR_L1 = {fc.TeamColor.RED: (235, 180, 160), fc.TeamColor.BLUE: (150, 215, 220)}
COLOR_L1_BARRIER = (190, 190, 185)
COLOR_L2 = (190, 190, 185)
COLOR_L2_EDGE = (140, 140, 135)
COLOR_START = (223, 34, 34)
COLOR_START_BLUE = (50, 0, 255)
COLOR_SHARED = (245, 240, 200)
COLOR_BUILD_SPOT = (40, 100, 50)
COLOR_PILLAR = (100, 62, 0)
COLOR_BOUNDARY = (100, 62, 0)
COLOR_TRANSFER = {fc.TeamColor.RED: (245, 170, 60), fc.TeamColor.BLUE: (60, 170, 245)}
COLOR_RAMP = {fc.TeamColor.RED: (200, 150, 140), fc.TeamColor.BLUE: (130, 180, 200)}
# 階段(L1/L2とも)は公式にはその段のL1と同じ色。段の境目が見えるよう線を引く
COLOR_STAIRS = COLOR_L1
COLOR_STAIR_EDGE = (120, 110, 105)
COLOR_RETRY = {fc.TeamColor.RED: COLOR_START, fc.TeamColor.BLUE: COLOR_START_BLUE}
COLOR_SPOT_SHARED_EDGE = (150, 150, 150)

# 動的要素(競技物・ロボット)の描画色。アース/スカイの赤青は公式ルールブック
# 14章のGame Objects表と同じ値(Start Zoneの赤青とも同色)。
COLOR_EARTH_RED = (223, 34, 34)
COLOR_EARTH_BLUE = (50, 0, 255)
COLOR_SKY_RED = (223, 34, 34)
COLOR_SKY_BLUE = (50, 0, 255)
COLOR_SKY_EDGE = (255, 255, 255)
COLOR_BLOCK_OUTLINE = (40, 20, 20)
# ムスティカ(公式には"Original ball color"としか指定が無いバレーボールなので
# 独自の近似色。ロボットの色も指定が無く独自の選択)
COLOR_MUSTIKA = (230, 195, 40)
COLOR_TEAM = {fc.TeamColor.RED: COLOR_START, fc.TeamColor.BLUE: COLOR_START_BLUE}
COLOR_ROBOT_BODY = {'tr': (120, 122, 130), 'br': (50, 52, 60)}
COLOR_ROBOT_ARROW = (240, 240, 240)
COLOR_GRIPPER_OPEN = (215, 215, 215)
COLOR_GRIPPER_CLOSED = (245, 170, 40)

_TEAM_NAMES = {fc.TeamColor.RED: '赤', fc.TeamColor.BLUE: '青', '': '共用'}

# pygameの既定フォントは日本語グリフを持たないため日本語フォントを探す。
# WSL環境ではLinux側に日本語フォントが無いことが多いので、Windows側のフォントも候補にする
_JP_FONT_NAMES = ('notosanscjkjp', 'notosansjp', 'notosanscjk', 'ipaexgothic', 'ipagothic',
                  'takaogothic', 'bizudgothic', 'meiryo')
_JP_FONT_FILES = (
    '/mnt/c/Windows/Fonts/BIZ-UDGothicR.ttc',
    '/mnt/c/Windows/Fonts/meiryo.ttc',
    '/mnt/c/Windows/Fonts/YuGothM.ttc',
    '/mnt/c/Windows/Fonts/msgothic.ttc',
)


def find_japanese_font() -> str | None:
    for name in _JP_FONT_NAMES:
        path = pygame.font.match_font(name)
        if path:
            return path
    return next((p for p in _JP_FONT_FILES if os.path.exists(p)), None)


def load_font(size: int) -> pygame.font.Font:
    """日本語フォント(見つからなければpygame既定フォント)を読み込む。"""
    return pygame.font.Font(find_japanese_font(), size)


def mm_to_px(u, v):
    """フィールド座標(mm) -> 画面座標(px)。V軸は画面上で上向きが正になるよう反転。"""
    x = MARGIN + u * SCALE
    y = MARGIN + (fc.GAME_FIELD_SIZE - v) * SCALE
    return (x, y)


def draw_rect_mm(surface, color, origin_uv, size_uv, width=0):
    x, y = mm_to_px(origin_uv[0], origin_uv[1] + size_uv[1])
    w = size_uv[0] * SCALE
    h = size_uv[1] * SCALE
    pygame.draw.rect(surface, color, (x, y, w, h), width)


def draw_square_center_mm(surface, color, center_uv, size_mm, width=0):
    origin = (center_uv[0] - size_mm / 2, center_uv[1] - size_mm / 2)
    draw_rect_mm(surface, color, origin, (size_mm, size_mm), width)


def draw_circle_mm(surface, color, center_uv, diameter_mm, width=0):
    x, y = mm_to_px(*center_uv)
    r = (diameter_mm / 2) * SCALE
    pygame.draw.circle(surface, color, (x, y), r, width)


def draw_label(surface, font, text, uv, anchor='topleft', offset=(0, 0)):
    """背景付きの読みやすいラベルを描く(フィールド上の物に重なっても読めるように)。"""
    x, y = mm_to_px(*uv)
    img = font.render(text, True, COLOR_TEXT)
    rect = img.get_rect(**{anchor: (x + offset[0], y + offset[1])})
    bg = pygame.Surface(rect.inflate(8, 2).size, pygame.SRCALPHA)
    pygame.draw.rect(bg, COLOR_LABEL_BG, bg.get_rect(), border_radius=4)
    surface.blit(bg, rect.inflate(8, 2).topleft)
    surface.blit(img, rect)


def spot_center(origin) -> tuple[float, float]:
    return (origin[0] + fc.BUILD_SPOT_SIZE[0] / 2, origin[1] + fc.BUILD_SPOT_SIZE[1] / 2)


def spot_short_name(spot_id: str, level: int, team: str) -> str:
    """建築スポットの短い名前(操縦コンソールの「L1・赤・1番」と対応)。"""
    return f'L{level}{_TEAM_NAMES.get(team, team)}{spot_id.rsplit("_", 1)[-1]}'


def _both_sides(origin, size):
    """赤側の矩形と、それを青側へ鏡映した矩形を(チーム, 原点)の組で返す。"""
    return ((fc.TeamColor.RED, origin), (fc.TeamColor.BLUE, fc.mirror_rect_origin(origin, size)))


def _draw_stairs(surface, color, origin, size, steps, along_u):
    draw_rect_mm(surface, color, origin, size)
    for i in range(1, steps):
        if along_u:
            u = origin[0] + size[0] * i / steps
            a, b = mm_to_px(u, origin[1]), mm_to_px(u, origin[1] + size[1])
        else:
            v = origin[1] + size[1] * i / steps
            a, b = mm_to_px(origin[0], v), mm_to_px(origin[0] + size[0], v)
        pygame.draw.line(surface, COLOR_STAIR_EDGE, a, b, 1)
    draw_rect_mm(surface, COLOR_STAIR_EDGE, origin, size, width=1)


def _draw_center_divider(surface):
    """中央の仕切り(グラウンドとL1の中心線)。L2と、中央線上の共用エリアの部分は途切れる。"""
    gaps = [
        (fc.GROUND_SHARED_AREA_ORIGIN[1], fc.GROUND_SHARED_AREA_ORIGIN[1] + fc.GROUND_SHARED_AREA_SIZE[1]),
        (fc.L2_ORIGIN[1], fc.L2_ORIGIN[1] + fc.L2_SIZE),
        (fc.MUSTIKA_SHARED_AREA_ORIGIN[1], fc.MUSTIKA_SHARED_AREA_ORIGIN[1] + fc.MUSTIKA_SHARED_AREA_SIZE[1]),
        *((o[1], o[1] + fc.L1_SHARED_AREA_SIZE[1]) for o in fc.L1_SHARED_AREA_ORIGINS),
    ]
    v = 0.0
    for start, end in sorted(gaps) + [(fc.GAME_FIELD_SIZE, fc.GAME_FIELD_SIZE)]:
        if start > v:
            pygame.draw.line(surface, COLOR_BOUNDARY, mm_to_px(fc.FIELD_CENTER, v), mm_to_px(fc.FIELD_CENTER, start), 3)
        v = max(v, end)


def draw_static_field(surface, font):
    """フィールドの静的要素(床の塗り分け・段差・階段・建築スポット・柱・仕切り)を描く。文字はdraw_field_labelsで別に描く。"""
    surface.fill(COLOR_BG)
    half = fc.FIELD_CENTER

    # グラウンド(中央線で赤/青に塗り分け)
    draw_rect_mm(surface, COLOR_GROUND[fc.TeamColor.RED], (0, 0), (half, fc.GAME_FIELD_SIZE))
    draw_rect_mm(surface, COLOR_GROUND[fc.TeamColor.BLUE], (half, 0), (half, fc.GAME_FIELD_SIZE))

    for team, origin in _both_sides(fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE):
        draw_rect_mm(surface, COLOR_STORAGE_EDGE[team], origin, fc.STORAGE_AREA_SIZE, width=2)
    for zone in (fc.START_ZONE_1_ORIGIN, fc.START_ZONE_2_ORIGIN):
        for team, origin in _both_sides(zone, fc.START_ZONE_SIZE):
            draw_rect_mm(surface, COLOR_TEAM[team], origin, fc.START_ZONE_SIZE)

    draw_rect_mm(surface, COLOR_SHARED, fc.GROUND_SHARED_AREA_ORIGIN, fc.GROUND_SHARED_AREA_SIZE)
    draw_rect_mm(surface, COLOR_SHARED, fc.MUSTIKA_SHARED_AREA_ORIGIN, fc.MUSTIKA_SHARED_AREA_SIZE)

    # L1(中央線で赤/青に塗り分け、外周に柵)
    l1_red_width = half - fc.L1_ORIGIN[0]
    draw_rect_mm(surface, COLOR_L1[fc.TeamColor.RED], fc.L1_ORIGIN, (l1_red_width, fc.L1_SIZE))
    draw_rect_mm(surface, COLOR_L1[fc.TeamColor.BLUE], (half, fc.L1_ORIGIN[1]),
                 (fc.L1_SIZE - l1_red_width, fc.L1_SIZE))
    draw_rect_mm(surface, COLOR_L1_BARRIER, fc.L1_ORIGIN, (fc.L1_SIZE, fc.L1_SIZE), width=4)
    for origin in fc.L1_SHARED_AREA_ORIGINS:
        draw_rect_mm(surface, COLOR_SHARED, origin, fc.L1_SHARED_AREA_SIZE)
    for team, origin in _both_sides(fc.L1_RETRY_ZONE_ORIGIN, fc.L1_RETRY_ZONE_SIZE):
        draw_rect_mm(surface, COLOR_RETRY[team], origin, fc.L1_RETRY_ZONE_SIZE)

    # L1の外側: スロープ -> 踊り場 -> 受渡しエリア -> 階段
    for team, origin in _both_sides(fc.RAMP_ORIGIN, fc.RAMP_SIZE):
        draw_rect_mm(surface, COLOR_RAMP[team], origin, fc.RAMP_SIZE)
    for team, origin in _both_sides(fc.RAMP_LANDING_ORIGIN, fc.RAMP_LANDING_SIZE):
        draw_rect_mm(surface, COLOR_STAIRS[team], origin, fc.RAMP_LANDING_SIZE)
    for team, origin in _both_sides(fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE):
        draw_rect_mm(surface, COLOR_TRANSFER[team], origin, fc.TRANSFER_AREA_SIZE)
    for team, origin in _both_sides(fc.L1_STAIRS_ORIGIN, fc.L1_STAIRS_SIZE):
        _draw_stairs(surface, COLOR_STAIRS[team], origin, fc.L1_STAIRS_SIZE, fc.L1_STAIR_STEP_COUNT, along_u=False)

    # L2とL1->L2の階段
    draw_rect_mm(surface, COLOR_L2, fc.L2_ORIGIN, (fc.L2_SIZE, fc.L2_SIZE))
    draw_rect_mm(surface, COLOR_L2_EDGE, fc.L2_ORIGIN, (fc.L2_SIZE, fc.L2_SIZE), width=2)
    for team, origin in _both_sides(fc.L2_STAIRS_ORIGIN, fc.L2_STAIRS_SIZE):
        _draw_stairs(surface, COLOR_STAIRS[team], origin, fc.L2_STAIRS_SIZE, 1, along_u=True)

    # 建築スポット(緑)。どちらのチームのスポットか(共用なら灰)を枠の色で示す
    for _spot_id, _level, team, origin in fc.BUILD_SPOTS:
        draw_rect_mm(surface, COLOR_BUILD_SPOT, origin, fc.BUILD_SPOT_SIZE)
        draw_rect_mm(surface, COLOR_TEAM.get(team, COLOR_SPOT_SHARED_EDGE), origin, fc.BUILD_SPOT_SIZE, width=3)

    draw_circle_mm(surface, COLOR_PILLAR, (fc.FIELD_CENTER, fc.FIELD_CENTER), fc.CENTRAL_PILLAR_DIAMETER)
    draw_circle_mm(surface, COLOR_PILLAR, fc.MUSTIKA_PILLAR_ORIGIN, fc.MUSTIKA_PILLAR_DIAMETER)

    _draw_center_divider(surface)
    draw_rect_mm(surface, COLOR_BOUNDARY, (0, 0), (fc.GAME_FIELD_SIZE, fc.GAME_FIELD_SIZE), width=4)


def draw_field_labels(surface, font):
    """エリア名・建築スポット名のラベル。競技物やロボットに隠れないよう最後に重ねて描く。"""
    margin = fc.BUILD_SPOT_SIZE[0] + 150
    draw_label(surface, font, 'L1', (fc.L1_ORIGIN[0] + margin, fc.L1_ORIGIN[1]), anchor='bottomleft', offset=(0, -4))
    draw_label(surface, font, 'L2', (fc.L2_ORIGIN[0] + margin, fc.L2_ORIGIN[1] + fc.L2_SIZE), offset=(0, 4))
    for _team, origin in _both_sides(fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE):
        draw_label(surface, font, 'ストレージ', origin, anchor='bottomleft', offset=(4, -4))
    for text, origin, size in (('受渡し', fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE),
                               ('スロープ', fc.RAMP_ORIGIN, fc.RAMP_SIZE),
                               ('リトライ', fc.L1_RETRY_ZONE_ORIGIN, fc.L1_RETRY_ZONE_SIZE)):
        for _team, o in _both_sides(origin, size):
            draw_label(surface, font, text, (o[0] + size[0] / 2, o[1] + size[1] / 2), anchor='center')
    draw_label(surface, font, 'スカイ置き場', fc.GROUND_SHARED_AREA_ORIGIN, offset=(0, 2))
    draw_label(surface, font, '中央の柱', (fc.FIELD_CENTER, fc.FIELD_CENTER), anchor='midtop',
               offset=(0, fc.CENTRAL_PILLAR_DIAMETER / 2 * SCALE + 2))
    draw_label(surface, font, 'ムスティカ台', fc.MUSTIKA_PILLAR_ORIGIN, anchor='midtop',
               offset=(0, fc.MUSTIKA_PILLAR_DIAMETER / 2 * SCALE + 2))

    # 建築スポット名は、スポットの外側(フィールド中心から遠い側)の上下に置く。
    # 中央線上の共用スポットは外側にムスティカ台/スカイ置き場があるので内側に置く
    for spot_id, level, team, origin in fc.BUILD_SPOTS:
        cu, cv = spot_center(origin)
        above = (cv > fc.FIELD_CENTER) != (team == '')
        edge_v = origin[1] + fc.BUILD_SPOT_SIZE[1] if above else origin[1]
        draw_label(surface, font, spot_short_name(spot_id, level, team), (cu, edge_v),
                   anchor='midbottom' if above else 'midtop', offset=(0, -2 if above else 2))


def _block_color(block_type, team_or_top_color):
    if block_type == 'sky':
        return COLOR_SKY_BLUE if team_or_top_color == fc.TeamColor.BLUE else COLOR_SKY_RED
    return COLOR_EARTH_BLUE if team_or_top_color == fc.TeamColor.BLUE else COLOR_EARTH_RED


def _draw_block_shape(surface, block_type, color, center_uv) -> None:
    """アースはベタ塗りの大きい四角、スカイは白縁付きの小さい四角(色=上面の色)。"""
    if block_type == 'sky':
        draw_square_center_mm(surface, COLOR_SKY_EDGE, center_uv, fc.SKY_BLOCK_SIZE + 60)
        draw_square_center_mm(surface, color, center_uv, fc.SKY_BLOCK_SIZE - 20)
    else:
        draw_square_center_mm(surface, color, center_uv, fc.EARTH_BLOCK_SIZE)
        draw_square_center_mm(surface, COLOR_BLOCK_OUTLINE, center_uv, fc.EARTH_BLOCK_SIZE, width=1)


def draw_block(surface, block) -> None:
    """br_msgs.Block(またはDetectedBlock)を種別・色に応じて描画する。"""
    color_key = block.top_color if block.block_type == 'sky' else block.owner_team
    _draw_block_shape(surface, block.block_type, _block_color(block.block_type, color_key),
                      (block.position.x, block.position.y))


def draw_mustika(surface, position_uv) -> None:
    draw_circle_mm(surface, COLOR_MUSTIKA, position_uv, fc.MUSTIKA_DIAMETER)
    draw_circle_mm(surface, COLOR_BLOCK_OUTLINE, position_uv, fc.MUSTIKA_DIAMETER, width=1)


def draw_tower(surface, font, tower) -> None:
    """
    br_msgs.TowerStateを、下の段から右上へ少しずつずらして重ねた形で描き、段数を添える。
    上から見ると一番上しか見えない塔の中身(何が何段)を分かるようにするため。
    """
    spot = next((s for s in fc.BUILD_SPOTS if s[0] == tower.build_spot_id), None)
    if spot is None or not tower.block_types:
        return
    cu, cv = spot_center(spot[3])
    step_mm = 70
    n = len(tower.block_types)
    for i, (block_type, owner, top) in enumerate(zip(tower.block_types, tower.owner_teams, tower.top_colors)):
        shift = (i - (n - 1) / 2) * step_mm
        color = _block_color(block_type, top if block_type == 'sky' else owner)
        _draw_block_shape(surface, block_type, color, (cu + shift, cv + shift))
    x, y = mm_to_px(cu + fc.BUILD_SPOT_SIZE[0] / 2, cv + fc.BUILD_SPOT_SIZE[1] / 2)
    img = font.render(f'{n}段', True, COLOR_TEXT_LIGHT)
    badge = img.get_rect(center=(x, y)).inflate(8, 2)
    pygame.draw.rect(surface, COLOR_TEXT, badge, border_radius=8)
    surface.blit(img, img.get_rect(center=badge.center))


def _local_to_px(center_uv, theta, x, y):
    """ロボット座標系(mm, theta=0で+vが前方)の点を画面座標(px)へ変換する。"""
    c, s = math.cos(theta), math.sin(theta)
    return mm_to_px(center_uv[0] + c * x - s * y, center_uv[1] + s * x + c * y)


def _local_rect_px(center_uv, theta, x0, y0, x1, y1):
    return [_local_to_px(center_uv, theta, x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def draw_robot(surface, font, robot_id, team, position_uv, theta, gripper_closed) -> None:
    """ロボットを上から見た記号で描く(チーム色の枠・前方向の矢印・前面のグリッパー・TR/BRの名札)。"""
    half = fc.ROBOT_INITIAL_MAX[0] / 2
    team_color = COLOR_TEAM.get(team, COLOR_TEXT)

    body = _local_rect_px(position_uv, theta, -half, -half, half, half)
    pygame.draw.polygon(surface, COLOR_ROBOT_BODY[robot_id], body)
    pygame.draw.polygon(surface, team_color, body, 3)
    # 同じ色の床(スタートゾーン等)の上でも輪郭が消えないよう黒線で縁取る
    outline = _local_rect_px(position_uv, theta, -half - 25, -half - 25, half + 25, half + 25)
    pygame.draw.polygon(surface, COLOR_TEXT, outline, 1)

    arrow = [_local_to_px(position_uv, theta, x, y)
             for x, y in ((0, half * 0.92), (-half * 0.28, half * 0.6), (half * 0.28, half * 0.6))]
    pygame.draw.polygon(surface, COLOR_ROBOT_ARROW, arrow)

    # グリッパーの爪: 閉じると内側に寄り色が変わる
    gap = half * (0.2 if gripper_closed else 0.6)
    jaw_color = COLOR_GRIPPER_CLOSED if gripper_closed else COLOR_GRIPPER_OPEN
    for sign in (-1, 1):
        x = sign * gap
        jaw = _local_rect_px(position_uv, theta, x - 30, half, x + 30, half + 180)
        pygame.draw.polygon(surface, jaw_color, jaw)
        pygame.draw.polygon(surface, COLOR_TEXT, jaw, 1)

    # 名札は車体の後ろ半分(矢印と反対側)の内側に、回転させずに描く。外に出すと
    # 2台が近づいたとき相手に重なり、中央は保持中のブロックで隠れるため
    img = font.render(robot_id.upper(), True, COLOR_ROBOT_ARROW)
    tag = img.get_rect(center=_local_to_px(position_uv, theta, 0, -half * 0.68))
    pygame.draw.rect(surface, team_color, tag.inflate(6, 0), border_radius=4)
    surface.blit(img, tag)


def draw_header(surface, font_large, font_small, score_red, score_blue) -> None:
    """画面上の余白に得点、下の余白に凡例を描く。"""
    red = font_large.render(f'赤 {score_red}点', True, (255, 120, 120))
    blue = font_large.render(f'青 {score_blue}点', True, (130, 160, 255))
    surface.blit(red, red.get_rect(midleft=(MARGIN, MARGIN // 2)))
    surface.blit(blue, blue.get_rect(midleft=(MARGIN + red.get_width() + 40, MARGIN // 2)))

    y = WINDOW_SIZE - MARGIN // 2
    x = MARGIN
    items = (
        ('earth', 'アース'),
        ('sky', 'スカイ（色=上面）'),
        ('mustika', 'ムスティカ'),
    )
    for kind, text in items:
        cu = (x - MARGIN) / SCALE + 150
        cv = -(y - MARGIN) / SCALE + fc.GAME_FIELD_SIZE
        if kind == 'mustika':
            draw_mustika(surface, (cu, cv))
        else:
            _draw_block_shape(surface, kind, COLOR_EARTH_RED, (cu, cv))
        img = font_small.render(text, True, COLOR_TEXT_LIGHT)
        surface.blit(img, img.get_rect(midleft=(x + 30, y)))
        x += 30 + img.get_width() + 30
    jaw = font_small.render('爪 灰=開 / 橙=閉　枠の色=チーム', True, COLOR_TEXT_LIGHT)
    surface.blit(jaw, jaw.get_rect(midleft=(x, y)))
