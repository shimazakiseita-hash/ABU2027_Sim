"""
ABU Robocon 2027 Phase 1 2Dシム: デバッグ用可視化ノード(br_visualizer_node)。

/true_state/* と /score/* を購読し、field_drawing(共通描画処理)を使って
フィールド・ロボット・競技物・得点をpygameで描画する。HANDOFFの
run_br_visualizer.sh(デバッグ用可視化、得点表示等)に対応するノード。
意思決定ロジックのデバッグ用であり、意思決定ノード自身は
/true_state/*を直接見ない(topic_contract.mdの規約)。

ウィンドウが出ない環境(SDL_VIDEODRIVER=dummy)では、screenshot_pathパラメータ
で指定したパスに最新フレームを毎ティック上書き保存する(目視確認用)。
"""

from __future__ import annotations

import os

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from br_msgs.msg import BlockArray, MustikaPose, RobotPose

from . import field_drawing as fd

RENDER_HZ = 30.0


class BrVisualizerNode(Node):
    def __init__(self):
        super().__init__('br_visualizer_node')

        self.declare_parameter('screenshot_path', '')
        self._screenshot_path = self.get_parameter('screenshot_path').value

        pygame.init()
        self._font = pygame.font.SysFont(None, 16)
        self._screen = pygame.display.set_mode((fd.WINDOW_SIZE, fd.WINDOW_SIZE))
        pygame.display.set_caption('ABU Robocon 2027 BR Strategy Sim Visualizer')
        self._headless = os.environ.get('SDL_VIDEODRIVER') == 'dummy'

        self._br_pose: RobotPose | None = None
        self._tr_pose: RobotPose | None = None
        self._blocks = BlockArray()
        self._mustika: MustikaPose | None = None
        self._score_red = 0
        self._score_blue = 0

        self.create_subscription(RobotPose, '/true_state/br_pose', self._on_br_pose, 10)
        self.create_subscription(RobotPose, '/true_state/tr_pose', self._on_tr_pose, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_subscription(MustikaPose, '/true_state/mustika_pose', self._on_mustika, 10)
        self.create_subscription(Int32, '/score/red', self._on_score_red, 10)
        self.create_subscription(Int32, '/score/blue', self._on_score_blue, 10)

        self.create_timer(1.0 / RENDER_HZ, self._render)

    def _on_br_pose(self, msg: RobotPose) -> None:
        self._br_pose = msg

    def _on_tr_pose(self, msg: RobotPose) -> None:
        self._tr_pose = msg

    def _on_blocks(self, msg: BlockArray) -> None:
        self._blocks = msg

    def _on_mustika(self, msg: MustikaPose) -> None:
        self._mustika = msg

    def _on_score_red(self, msg: Int32) -> None:
        self._score_red = msg.data

    def _on_score_blue(self, msg: Int32) -> None:
        self._score_blue = msg.data

    def _render(self) -> None:
        pygame.event.pump()

        fd.draw_static_field(self._screen, self._font)

        for block in self._blocks.blocks:
            fd.draw_block(self._screen, block)
        if self._mustika is not None:
            fd.draw_mustika(self._screen, (self._mustika.position.x, self._mustika.position.y))
        if self._br_pose is not None:
            fd.draw_robot(self._screen, fd.COLOR_ROBOT_BR, (self._br_pose.pose.x, self._br_pose.pose.y))
        if self._tr_pose is not None:
            fd.draw_robot(self._screen, fd.COLOR_ROBOT_TR, (self._tr_pose.pose.x, self._tr_pose.pose.y))

        score_text = f'RED {self._score_red}  -  BLUE {self._score_blue}'
        img = self._font.render(score_text, True, fd.COLOR_TEXT)
        self._screen.blit(img, (10, 10))

        pygame.display.flip()

        if self._headless and self._screenshot_path:
            pygame.image.save(self._screen, self._screenshot_path)


def main(args=None):
    rclpy.init(args=args)
    node = BrVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
