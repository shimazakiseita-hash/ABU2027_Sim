"""
ABU Robocon 2027 フィールド俯瞰プレビュー（最小実装）

field_constants.py の座標定義を、実際にpygameで描画して目視確認するための
最小スクリプト。まだROS2やpymunkとは繋がっていない、純粋な描画確認用。

実行方法:
    python3 draw_field_preview.py

ウィンドウが出ない環境(このサンドボックス等)では、SDL_VIDEODRIVER=dummy を
設定した上でスクリーンショットをファイル保存するモードで動く。
"""

import os
import pygame

import field_constants as fc

# --- 描画設定 ---
MARGIN = 60
SCALE = 0.09  # 1mm -> 0.09px  (11000mm -> 約990px)
WINDOW_SIZE = int(fc.GAME_FIELD_SIZE * SCALE) + MARGIN * 2

COLOR_BG = (200, 200, 195)
COLOR_FIELD = (250, 235, 225)   # 赤ゾーン地色(薄め)
COLOR_FIELD_BLUE = (215, 235, 245)  # 青ゾーン地色(薄め)
COLOR_L1 = (245, 200, 180)
COLOR_L1_BLUE = (170, 220, 225)
COLOR_L2 = (190, 190, 185)
COLOR_START = (223, 34, 34)
COLOR_START_BLUE = (50, 0, 255)
COLOR_STORAGE = (240, 210, 210)
COLOR_STORAGE_BLUE = (170, 210, 230)
COLOR_SHARED = (245, 240, 200)
COLOR_BUILD_SPOT = (40, 100, 50)
COLOR_TRANSFER_RED = (245, 170, 60)
COLOR_TRANSFER_BLUE = (60, 170, 245)
COLOR_PILLAR = (100, 62, 0)
COLOR_TEXT = (20, 20, 20)


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


def draw_circle_mm(surface, color, center_uv, diameter_mm, width=0):
    x, y = mm_to_px(*center_uv)
    r = (diameter_mm / 2) * SCALE
    pygame.draw.circle(surface, color, (x, y), r, width)


def draw_label(surface, font, text, uv):
    x, y = mm_to_px(*uv)
    img = font.render(text, True, COLOR_TEXT)
    surface.blit(img, (x, y))


def draw_field(surface, font):
    surface.fill(COLOR_BG)

    # フィールド外枠
    draw_rect_mm(surface, COLOR_FIELD, (0, 0), (fc.GAME_FIELD_SIZE, fc.GAME_FIELD_SIZE), width=2)

    # --- 赤チーム側 ---
    # スタートゾーン x2
    draw_rect_mm(surface, COLOR_START, fc.START_ZONE_1_ORIGIN, fc.START_ZONE_SIZE)
    draw_rect_mm(surface, COLOR_START, fc.START_ZONE_2_ORIGIN, fc.START_ZONE_SIZE)

    # ストレージエリア
    draw_rect_mm(surface, COLOR_STORAGE, fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE)
    draw_label(surface, font, "Storage(R)", fc.STORAGE_AREA_ORIGIN)

    # --- 中央: L1 (簡略化して中央に正方形配置。中心=フィールド中心) ---
    l1_origin = (fc.FIELD_CENTER - fc.L1_SIZE / 2, fc.FIELD_CENTER - fc.L1_SIZE / 2)
    draw_rect_mm(surface, COLOR_L1, l1_origin, (fc.L1_SIZE, fc.L1_SIZE), width=2)
    draw_label(surface, font, "L1 (6000x6000)", (l1_origin[0], l1_origin[1] + fc.L1_SIZE))

    # L1建築スポット x4(四隅)
    bs = fc.BUILD_SPOT_SIZE
    corners = [
        (l1_origin[0], l1_origin[1]),
        (l1_origin[0] + fc.L1_SIZE - bs[0], l1_origin[1]),
        (l1_origin[0], l1_origin[1] + fc.L1_SIZE - bs[1]),
        (l1_origin[0] + fc.L1_SIZE - bs[0], l1_origin[1] + fc.L1_SIZE - bs[1]),
    ]
    for c in corners:
        draw_rect_mm(surface, COLOR_BUILD_SPOT, c, bs)

    # --- L2 (L1の中に入れ子で中央配置) ---
    l2_origin = (fc.FIELD_CENTER - fc.L2_SIZE / 2, fc.FIELD_CENTER - fc.L2_SIZE / 2)
    draw_rect_mm(surface, COLOR_L2, l2_origin, (fc.L2_SIZE, fc.L2_SIZE), width=2)
    draw_label(surface, font, "L2 (3000x3000)", (l2_origin[0], l2_origin[1] + fc.L2_SIZE))

    for c in [
        (l2_origin[0], l2_origin[1]),
        (l2_origin[0] + fc.L2_SIZE - bs[0], l2_origin[1]),
        (l2_origin[0], l2_origin[1] + fc.L2_SIZE - bs[1]),
        (l2_origin[0] + fc.L2_SIZE - bs[0], l2_origin[1] + fc.L2_SIZE - bs[1]),
    ]:
        draw_rect_mm(surface, COLOR_BUILD_SPOT, c, bs)

    # 中央柱
    draw_circle_mm(surface, COLOR_PILLAR, (fc.FIELD_CENTER, fc.FIELD_CENTER), fc.CENTRAL_PILLAR_DIAMETER)
    draw_label(surface, font, "Central Pillar", (fc.FIELD_CENTER, fc.FIELD_CENTER))

    # ムスティカ台(L1手前、グラウンド面)
    mustika_uv = (l1_origin[0] - 500, l1_origin[1] - 500)
    draw_circle_mm(surface, COLOR_PILLAR, mustika_uv, fc.MUSTIKA_PILLAR_DIAMETER)
    draw_label(surface, font, "Mustika Pillar", mustika_uv)

    # グラウンド共用エリア(スカイブロック格子)
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


def main():
    pygame.init()
    font = pygame.font.SysFont(None, 16)
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("ABU Robocon 2027 Field Preview")

    headless = os.environ.get("SDL_VIDEODRIVER") == "dummy"
    clock = pygame.time.Clock()
    running = True
    frame = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        draw_field(screen, font)
        pygame.display.flip()

        if headless:
            frame += 1
            if frame > 2:
                pygame.image.save(screen, "field_preview.png")
                running = False

        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
