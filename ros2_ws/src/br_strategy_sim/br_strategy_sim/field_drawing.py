"""
ABU Robocon 2027 Phase 1 2Dシム: pygame描画の共通処理。

reference/draw_field_preview.py由来のフィールド静的要素の描画と、
座標変換(mm<->px)を、scripts/main.py(静止プレビュー)とbr_visualizer_node
(動的可視化)の両方から共有するためのモジュール。ROS2やpymunkには依存しない。
"""

from __future__ import annotations

import pygame

from . import field_constants as fc

MARGIN = 60
SCALE = 0.09  # 1mm -> 0.09px  (11000mm -> 約990px)
WINDOW_SIZE = int(fc.GAME_FIELD_SIZE * SCALE) + MARGIN * 2

COLOR_BG = (200, 200, 195)
COLOR_FIELD = (250, 235, 225)
COLOR_L1 = (245, 200, 180)
COLOR_L2 = (190, 190, 185)
COLOR_START = (223, 34, 34)
COLOR_START_BLUE = (50, 0, 255)
COLOR_STORAGE = (240, 210, 210)
COLOR_STORAGE_BLUE = (170, 210, 230)
COLOR_SHARED = (245, 240, 200)
COLOR_BUILD_SPOT = (40, 100, 50)
COLOR_PILLAR = (100, 62, 0)
COLOR_TEXT = (20, 20, 20)
# 公式ルールブック14章のColors and Materials Specificationより
COLOR_TRANSFER_RED = (245, 170, 60)
COLOR_TRANSFER_BLUE = (60, 170, 245)

# 動的要素(競技物・ロボット)の描画色。アース/スカイの赤青は公式ルールブック
# 14章のGame Objects表と同じ値(Start Zoneの赤青とも同色)。
COLOR_EARTH_RED = (223, 34, 34)
COLOR_EARTH_BLUE = (50, 0, 255)
COLOR_SKY_RED = (223, 34, 34)
COLOR_SKY_BLUE = (50, 0, 255)
# ムスティカ(公式には"Original ball color"としか指定が無いバレーボールなので
# 独自の近似色。ロボットの色も指定が無く独自の選択)
COLOR_MUSTIKA = (180, 150, 40)
COLOR_ROBOT_BR = (30, 30, 30)
COLOR_ROBOT_TR = (90, 90, 90)


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


def draw_label(surface, font, text, uv):
    x, y = mm_to_px(*uv)
    img = font.render(text, True, COLOR_TEXT)
    surface.blit(img, (x, y))


def draw_static_field(surface, font):
    """フィールドの静的要素(グラウンド/L1/L2/建築スポット/柱等)を描く。"""
    surface.fill(COLOR_BG)

    draw_rect_mm(surface, COLOR_FIELD, (0, 0), (fc.GAME_FIELD_SIZE, fc.GAME_FIELD_SIZE), width=2)

    draw_rect_mm(surface, COLOR_START, fc.START_ZONE_1_ORIGIN, fc.START_ZONE_SIZE)
    draw_rect_mm(surface, COLOR_START, fc.START_ZONE_2_ORIGIN, fc.START_ZONE_SIZE)

    draw_rect_mm(surface, COLOR_STORAGE, fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE)
    draw_label(surface, font, "Storage(R)", fc.STORAGE_AREA_ORIGIN)

    draw_rect_mm(surface, COLOR_L1, fc.L1_ORIGIN, (fc.L1_SIZE, fc.L1_SIZE), width=2)
    draw_label(surface, font, "L1 (6000x6000)", (fc.L1_ORIGIN[0], fc.L1_ORIGIN[1] + fc.L1_SIZE))

    for _spot_id, _level, _team, origin in fc.BUILD_SPOTS:
        draw_rect_mm(surface, COLOR_BUILD_SPOT, origin, fc.BUILD_SPOT_SIZE)

    draw_rect_mm(surface, COLOR_TRANSFER_RED, fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE, width=2)
    draw_label(surface, font, "Transfer", fc.TRANSFER_AREA_ORIGIN)

    draw_rect_mm(surface, COLOR_L2, fc.L2_ORIGIN, (fc.L2_SIZE, fc.L2_SIZE), width=2)
    draw_label(surface, font, "L2 (3000x3000)", (fc.L2_ORIGIN[0], fc.L2_ORIGIN[1] + fc.L2_SIZE))

    draw_circle_mm(surface, COLOR_PILLAR, (fc.FIELD_CENTER, fc.FIELD_CENTER), fc.CENTRAL_PILLAR_DIAMETER)
    draw_label(surface, font, "Central Pillar", (fc.FIELD_CENTER, fc.FIELD_CENTER))

    draw_circle_mm(surface, COLOR_PILLAR, fc.MUSTIKA_PILLAR_ORIGIN, fc.MUSTIKA_PILLAR_DIAMETER)
    draw_label(surface, font, "Mustika Pillar", fc.MUSTIKA_PILLAR_ORIGIN)

    draw_rect_mm(surface, COLOR_SHARED, fc.GROUND_SHARED_AREA_ORIGIN, fc.GROUND_SHARED_AREA_SIZE, width=2)
    draw_label(surface, font, "Sky Blocks", fc.GROUND_SHARED_AREA_ORIGIN)

    # --- 青チーム側(赤を鏡映) ---
    blue_start1 = fc.mirror_to_blue(*fc.START_ZONE_1_ORIGIN)
    blue_start2 = fc.mirror_to_blue(*fc.START_ZONE_2_ORIGIN)
    # mirrorはUのみ反転するので、矩形の描画原点も反転後に幅ぶんズラす必要がある
    draw_rect_mm(surface, COLOR_START_BLUE,
                 (blue_start1[0] - fc.START_ZONE_SIZE[0], blue_start1[1]), fc.START_ZONE_SIZE)
    draw_rect_mm(surface, COLOR_START_BLUE,
                 (blue_start2[0] - fc.START_ZONE_SIZE[0], blue_start2[1]), fc.START_ZONE_SIZE)

    blue_storage_origin = fc.mirror_to_blue(
        fc.STORAGE_AREA_ORIGIN[0] + fc.STORAGE_AREA_SIZE[0], fc.STORAGE_AREA_ORIGIN[1])
    draw_rect_mm(surface, COLOR_STORAGE_BLUE, blue_storage_origin, fc.STORAGE_AREA_SIZE)
    draw_label(surface, font, "Storage(B)", blue_storage_origin)


def draw_block(surface, block) -> None:
    """br_msgs.Block(またはDetectedBlock)を種別・色に応じて描画する。"""
    uv = (block.position.x, block.position.y)
    if block.block_type == "earth":
        color = COLOR_EARTH_BLUE if block.owner_team == fc.TeamColor.BLUE else COLOR_EARTH_RED
        draw_square_center_mm(surface, color, uv, fc.EARTH_BLOCK_SIZE)
    elif block.block_type == "sky":
        color = COLOR_SKY_BLUE if block.top_color == fc.TeamColor.BLUE else COLOR_SKY_RED
        draw_square_center_mm(surface, color, uv, fc.SKY_BLOCK_SIZE)


def draw_mustika(surface, position_uv) -> None:
    draw_circle_mm(surface, COLOR_MUSTIKA, position_uv, fc.MUSTIKA_DIAMETER)


def draw_robot(surface, color, position_uv) -> None:
    draw_square_center_mm(surface, color, position_uv, fc.ROBOT_INITIAL_MAX[0])
