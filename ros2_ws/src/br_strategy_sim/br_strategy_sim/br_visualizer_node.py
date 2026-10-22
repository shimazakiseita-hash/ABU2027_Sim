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

from br_msgs.msg import BlockArray, GripperCmd, MustikaPose, RobotPose, TowerArray

from . import field_drawing as fd
from .steady_clock import steady_clock

RENDER_HZ = 30.0


class BrVisualizerNode(Node):
    def __init__(self):
        super().__init__('br_visualizer_node')

        self.declare_parameter('screenshot_path', '')
        self._screenshot_path = self.get_parameter('screenshot_path').value
        # ロボットの枠をチーム色で描くため(br_referee_nodeと同じteamパラメータ)
        self.declare_parameter('team', 'red')
        self._team = self.get_parameter('team').value

        pygame.init()
        self._font = fd.load_font(12)
        self._tag_font = pygame.font.SysFont(None, 18, bold=True)
        self._header_font = fd.load_font(24)
        self._legend_font = fd.load_font(14)
        self._screen = pygame.display.set_mode((fd.WINDOW_SIZE, fd.WINDOW_SIZE))
        pygame.display.set_caption('ABU Robocon 2027 BR Strategy Sim Visualizer')
        self._headless = os.environ.get('SDL_VIDEODRIVER') == 'dummy'

        self._br_pose: RobotPose | None = None
        self._tr_pose: RobotPose | None = None
        self._blocks = BlockArray()
        self._mustika: MustikaPose | None = None
        self._score_red = 0
        self._score_blue = 0
        self._gripper_closed = {'tr': False, 'br': False}
        self._towers = TowerArray()

        self.create_subscription(RobotPose, '/true_state/br_pose', self._on_br_pose, 10)
        self.create_subscription(RobotPose, '/true_state/tr_pose', self._on_tr_pose, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_subscription(MustikaPose, '/true_state/mustika_pose', self._on_mustika, 10)
        self.create_subscription(TowerArray, '/true_state/tower_state', self._on_towers, 10)
        self.create_subscription(Int32, '/score/red', self._on_score_red, 10)
        self.create_subscription(Int32, '/score/blue', self._on_score_blue, 10)
        self.create_subscription(GripperCmd, '/tr_gripper_cmd', lambda m: self._on_gripper('tr', m), 10)
        self.create_subscription(GripperCmd, '/br_gripper_cmd', lambda m: self._on_gripper('br', m), 10)

        self.create_timer(1.0 / RENDER_HZ, self._render, clock=steady_clock())

    def _on_br_pose(self, msg: RobotPose) -> None:
        self._br_pose = msg

    def _on_tr_pose(self, msg: RobotPose) -> None:
        self._tr_pose = msg

    def _on_blocks(self, msg: BlockArray) -> None:
        self._blocks = msg

    def _on_mustika(self, msg: MustikaPose) -> None:
        self._mustika = msg

    def _on_towers(self, msg: TowerArray) -> None:
        self._towers = msg

    def _on_score_red(self, msg: Int32) -> None:
        self._score_red = msg.data

    def _on_score_blue(self, msg: Int32) -> None:
        self._score_blue = msg.data

    def _on_gripper(self, robot: str, msg: GripperCmd) -> None:
        self._gripper_closed[robot] = not msg.open

    def _render(self) -> None:
        pygame.event.pump()

        fd.draw_static_field(self._screen, self._font)

        # 塔に積まれた物(level>0)はdraw_towerで段の中身ごと描くので個別には描かない。
        # 塔と保持中の物は、ロボットに隠れないようロボットの後に描く
        for block in self._blocks.blocks:
            if block.held_by == 'none' and block.level == 0:
                fd.draw_block(self._screen, block)
        mustika_held = self._mustika is not None and self._mustika.held_by != 'none'
        if self._mustika is not None and not mustika_held:
            fd.draw_mustika(self._screen, (self._mustika.position.x, self._mustika.position.y))
        for robot, pose in (('br', self._br_pose), ('tr', self._tr_pose)):
            if pose is not None:
                fd.draw_robot(self._screen, self._tag_font, robot, self._team, (pose.pose.x, pose.pose.y),
                              pose.pose.theta, self._gripper_closed[robot])
        for tower in self._towers.towers:
            fd.draw_tower(self._screen, self._font, tower)
        for block in self._blocks.blocks:
            if block.held_by != 'none':
                fd.draw_block(self._screen, block)
        if mustika_held:
            fd.draw_mustika(self._screen, (self._mustika.position.x, self._mustika.position.y))

        fd.draw_field_labels(self._screen, self._font)
        fd.draw_header(self._screen, self._header_font, self._legend_font, self._score_red, self._score_blue)

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
