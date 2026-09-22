"""
ABU Robocon 2027 Phase 1 2Dシム: 手動操縦コンソール(br_teleop_node)。

意思決定ノード(br_decision_tr_node/br_decision_br_node)を起動しない状態
(`enable_decision:=false`)で、TR/BRを手動でキーボード操作するためのデバッグ用
ノード。br_visualizer_nodeと同じく独立したpygameウィンドウを持つ(別プロセス
として起動されるため自然に別ウィンドウになる)。

トピック契約(docs/topic_contract.md)に定義済みの/tr_cmd_vel, /br_cmd_vel,
/tr_gripper_cmd, /br_gripper_cmdへpublishするだけで、sim_bridge_node側の
変更は不要(「ロボットへの指令は単一トピックで受ける」設計のおかげで、
意思決定ノードの代わりにこのノードが同じトピックへpublishすれば手動操縦に
差し替えられる)。

操作方法:
    Tab   : 操作対象をTR<->BRで切替
    W/A/S/D: 対象ロボットをフィールド座標(u,v)で移動(押している間だけ)
    Space : 対象ロボットのグリッパー開閉をトグル
    Esc / ウィンドウを閉じる: 終了

初回実装のスコープは移動とグリッパーのみ(BuildActionによる手動建築は
未対応。要調整)。
"""

from __future__ import annotations

import pygame
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from br_msgs.msg import GripperCmd

from .decision_common import CRUISE_SPEED_M_S

CONTROL_HZ = 20.0
WINDOW_WIDTH = 420
WINDOW_HEIGHT = 260

_ROBOT_IDS = ('tr', 'br')


class BrTeleopNode(Node):
    def __init__(self):
        super().__init__('br_teleop_node')

        pygame.init()
        self._font = pygame.font.SysFont(None, 22)
        self._screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('ABU Robocon 2027 BR Teleop Console')

        self._active_robot = 'tr'
        # ロボットごとに開閉状態を独立に保持する(Tabで切り替えても
        # 相手側の状態が変わらないようにするため)
        self._gripper_open = {'tr': True, 'br': True}
        self._quit_requested = False

        self.pub_cmd_vel = {
            'tr': self.create_publisher(Twist, '/tr_cmd_vel', 10),
            'br': self.create_publisher(Twist, '/br_cmd_vel', 10),
        }
        self.pub_gripper_cmd = {
            'tr': self.create_publisher(GripperCmd, '/tr_gripper_cmd', 10),
            'br': self.create_publisher(GripperCmd, '/br_gripper_cmd', 10),
        }

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    def _tick(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._request_quit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self._request_quit()
                elif event.key == pygame.K_TAB:
                    self._active_robot = 'br' if self._active_robot == 'tr' else 'tr'
                elif event.key == pygame.K_SPACE:
                    robot = self._active_robot
                    self._gripper_open[robot] = not self._gripper_open[robot]
                    self.pub_gripper_cmd[robot].publish(GripperCmd(
                        open=self._gripper_open[robot],
                        target_force=0.0 if self._gripper_open[robot] else 1.0,
                        target_block_id='',
                    ))

        if self._quit_requested:
            return

        keys = pygame.key.get_pressed()
        # W/A/S/Dで押した方向にフィールド座標(u,v)で移動する。
        # field_drawing.mm_to_pxはvが増えるほど画面上で上に描画するため、
        # W(画面上で上)=+v, S(下)=-v, A(左)=-u, D(右)=+uとする。
        vx = 0.0
        vy = 0.0
        if keys[pygame.K_d]:
            vx += CRUISE_SPEED_M_S
        if keys[pygame.K_a]:
            vx -= CRUISE_SPEED_M_S
        if keys[pygame.K_w]:
            vy += CRUISE_SPEED_M_S
        if keys[pygame.K_s]:
            vy -= CRUISE_SPEED_M_S

        for robot in _ROBOT_IDS:
            twist = Twist()
            if robot == self._active_robot:
                twist.linear.x = vx
                twist.linear.y = vy
            self.pub_cmd_vel[robot].publish(twist)

        self._render()

    def _request_quit(self) -> None:
        # rclpy.spin(node)を使う標準的な形に合わせるため(下記main参照)、
        # ウィンドウを閉じる/Escで終了したい場合はrclpy.shutdown()を直接
        # 呼んでspin()自体を抜けさせる(rclpy.ok()がFalseになりループが終わる)。
        self._quit_requested = True
        if rclpy.ok():
            rclpy.shutdown()

    def _render(self) -> None:
        # pygame.font.SysFont(None, ...)の既定フォントは日本語グリフを
        # 持たないため、画面表示は英語のみにする(field_drawing.pyの
        # 画面表示が全て英語なのも同じ理由。文字化けの原因になっていた)。
        self._screen.fill((30, 30, 35))

        lines = [
            f'ACTIVE: {self._active_robot.upper()}   (Tab to switch)',
            '',
            'W/A/S/D: move    Space: toggle gripper    Esc: quit',
            '',
        ]
        for robot in _ROBOT_IDS:
            state = 'OPEN' if self._gripper_open[robot] else 'CLOSED'
            marker = '>> ' if robot == self._active_robot else '   '
            lines.append(f'{marker}{robot.upper()}  gripper: {state}')

        y = 20
        for line in lines:
            img = self._font.render(line, True, (230, 230, 230))
            self._screen.blit(img, (16, y))
            y += 28

        pygame.display.flip()

    def should_quit(self) -> bool:
        return self._quit_requested


def main(args=None):
    rclpy.init(args=args)
    node = BrTeleopNode()
    try:
        # 他のノード(br_visualizer_node等)と同じくrclpy.spin(node)を使う。
        # 独自のspin_onceループ(timeout_sec=0.05)を_tickのタイマー周期
        # (1/CONTROL_HZ=0.05s)と偶然一致させていた旧実装は、タイミングが
        # ぴったり噛み合わずタイマーが安定して発火しない場合があった
        # (WASD操作が反応しないという報告で発覚)。ウィンドウを閉じる/Escでの
        # 終了は_request_quitが直接rclpy.shutdown()を呼ぶことで実現する。
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
