"""
ABU Robocon 2027 3Dシムの表示(MuJoCoのビューア)。

3Dシム(sim_3d_node)と同じモデルを持ち、物理は計算せずに/sim3d/poses(ロボットと物の位置姿勢)を
当てはめて描くだけのノード。物理と表示を別プロセスに分けているのは、GPUの無い環境では描画が
CPU頼みで重く、同じプロセスで描くと描画中に物理の計算が止まってしまうため。
スカイの色(上面の色)は/true_state/blocksから取る。

パラメータ:
- show_window(既定true): ビューアのウィンドウを開く
- screenshot_path(既定空): 指定すると、俯瞰の画像をscreenshot_interval_sごとにこのパスへ保存する
"""

from __future__ import annotations

import os

import mujoco
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from br_msgs.msg import BlockArray

from . import scene_3d
from .robot_spec import declare_robot_specs
from .steady_clock import steady_clock

RENDER_HZ = 30.0
ROBOT_IDS = ('tr', 'br')


class Sim3DViewerNode(Node):
    def __init__(self):
        super().__init__('br_sim_3d_viewer_node')
        specs = declare_robot_specs(self)
        self.declare_parameter('show_window', True)
        self.declare_parameter('screenshot_path', '')
        self.declare_parameter('screenshot_interval_s', 2.0)
        self._screenshot_path = self.get_parameter('screenshot_path').value
        self._screenshot_interval = self.get_parameter('screenshot_interval_s').value

        self.model = mujoco.MjModel.from_xml_string(scene_3d.build_scene_xml(specs, 0.002))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self._mocap = [self.model.body_mocapid[self.model.body(r).id] for r in ROBOT_IDS]
        self._objects = []
        for obj_id, *_rest in scene_3d.object_specs():
            body = self.model.body(obj_id).id
            joint = self.model.body_jntadr[body]
            self._objects.append((obj_id, self.model.jnt_qposadr[joint], self.model.geom(obj_id).id))
        self._poses: PoseArray | None = None
        self._dirty = False
        self._top_colors: dict[str, str] = {}

        self._viewer = None
        if self.get_parameter('show_window').value:
            try:
                from mujoco import viewer as mujoco_viewer
                self._viewer = mujoco_viewer.launch_passive(self.model, self.data, show_left_ui=False,
                                                            show_right_ui=False)
                self._set_camera(self._viewer.cam)
            except Exception as e:  # 画面の無い環境などではウィンドウなしで続ける
                self.get_logger().warn(f'MuJoCoのビューアを開けませんでした: {e}')
        self._renderer = None
        self._screenshot_elapsed = 0.0

        self.create_subscription(PoseArray, '/sim3d/poses', self._on_poses, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_timer(1.0 / RENDER_HZ, self._render, clock=steady_clock())

    @staticmethod
    def _set_camera(cam) -> None:
        cam.lookat[:] = (5.5, 4.5, 0.3)
        cam.distance, cam.elevation, cam.azimuth = 14.0, -50.0, 90.0

    def _on_poses(self, msg: PoseArray) -> None:
        self._poses = msg
        self._dirty = True

    def _on_blocks(self, msg: BlockArray) -> None:
        for b in msg.blocks:
            if b.block_type == 'sky':
                self._top_colors[b.id] = b.top_color

    def _apply(self) -> None:
        poses = self._poses.poses
        if len(poses) != len(ROBOT_IDS) + len(self._objects):
            return
        for i, mocap in enumerate(self._mocap):
            p, q = poses[i].position, poses[i].orientation
            self.data.mocap_pos[mocap] = (p.x, p.y, p.z)
            self.data.mocap_quat[mocap] = (q.w, q.x, q.y, q.z)
        for (obj_id, adr, geom), pose in zip(self._objects, poses[len(ROBOT_IDS):]):
            p, q = pose.position, pose.orientation
            self.data.qpos[adr:adr + 7] = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
            color = self._top_colors.get(obj_id)
            if color:
                self.model.geom_rgba[geom] = scene_3d.color_rgba(f'sky_{color}')
        mujoco.mj_kinematics(self.model, self.data)

    def _render(self) -> None:
        if self._dirty:
            self._dirty = False
            if self._viewer is not None:
                with self._viewer.lock():
                    self._apply()
            else:
                self._apply()
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()
        self._maybe_screenshot()

    def _maybe_screenshot(self) -> None:
        if not self._screenshot_path:
            return
        self._screenshot_elapsed += 1.0 / RENDER_HZ
        if self._screenshot_elapsed < self._screenshot_interval:
            return
        self._screenshot_elapsed = 0.0
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 720, 960)
            self._camera = mujoco.MjvCamera()
            self._set_camera(self._camera)
        self._renderer.update_scene(self.data, camera=self._camera)
        image = self._renderer.render()
        import pygame
        tmp = self._screenshot_path + '.tmp.png'
        pygame.image.save(pygame.surfarray.make_surface(image.swapaxes(0, 1)), tmp)
        os.replace(tmp, self._screenshot_path)


def main(args=None):
    rclpy.init(args=args)
    node = Sim3DViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._viewer is not None:
            node._viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
