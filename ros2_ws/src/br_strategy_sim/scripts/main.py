"""
ABU Robocon 2027 フィールド俯瞰プレビュー（描画ループの骨格）

reference/draw_field_preview.py をベースに、br_strategy_sim パッケージの
field_drawing(共通描画処理)を使って同じ描画を行う。まだROS2ノード化は
していない、純粋な描画ループの骨格。動的な競技物・ロボットの可視化は
br_visualizer_node(ROS2ノード)側で行う。

実行方法:
    python3 scripts/main.py

ウィンドウが出ない環境では、SDL_VIDEODRIVER=dummy を設定した上で
スクリーンショットをファイル保存するモードで動く。
"""

import os
import sys
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from br_strategy_sim import field_drawing as fd


def main():
    pygame.init()
    font = pygame.font.SysFont(None, 16)
    screen = pygame.display.set_mode((fd.WINDOW_SIZE, fd.WINDOW_SIZE))
    pygame.display.set_caption("ABU Robocon 2027 BR Strategy Sim")

    headless = os.environ.get("SDL_VIDEODRIVER") == "dummy"
    clock = pygame.time.Clock()
    running = True
    frame = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        fd.draw_static_field(screen, font)
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
