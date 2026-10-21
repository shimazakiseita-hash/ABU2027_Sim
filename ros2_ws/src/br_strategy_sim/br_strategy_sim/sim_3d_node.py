"""
ABU Robocon 2027 3Dシム(MuJoCo): 実機のcan_node+STM32+センサーの代わりをするノード。

ハードウェア境界(docs/topic_contract.md「ハードウェア境界」)のトピックだけを話す:
- 受ける: /<r>/base_cmd(機体座標の速度), /<r>/mechanism_cmd(機構の指令)
- 返す: /<r>/wheel_deltas(4輪のエンコーダ差分), /<r>/imu, /<r>/scan(LiDAR), /<r>/mechanism_state
(<r>はtr/br)。その上の足回り制御・推測航法・機構制御は本番と同じノード(br_robotパッケージ)が
動く。審判・2D可視化・認識の代行用に、シムだけが知る正解値/true_state/*も今まで通り出す。

箱型ロボットの簡略化(機体設計が決まったら車輪・アームのモデルに差し替える):
- 車体は機体仕様の寸法の箱。平面の動きは2Dと同じ平面モデル(robot_body.RobotBody: 加減速・旋回の
  上限、柵、通れない地形)で決め、高さはfield_geometry.floor_height_atに合わせる。ブロックは
  車体に押されるが、車体はブロックに押し返されない(車体50kg、ブロック0.6kg)
- 足回りはSTM32相当として、指令速度を機体仕様の加減速で追従する。エンコーダは仮の4輪メカナムの
  配置で、実際の車体の動きから逆算したカウントに滑りのノイズを乗せて返す
- 機構は「機体中心から届く範囲の腕」とみなす。CLOSEは目標位置の近くの物をつかんで車体の上に
  固定し、PLACEは持っている物を目標位置の真下にある物(塔・台・床)の上に置いて手を離す。
  その後は物理任せなので、ずれていれば塔から落ちる。所要時間は機体仕様
"""

from __future__ import annotations

import math
import os
import random

import mujoco
import numpy as np
import pymunk
import rclpy
from geometry_msgs.msg import Point, Pose2D, Quaternion, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int16MultiArray

from br_msgs.msg import (Block, BlockArray, MechanismCmd, MechanismState, MustikaPose, RobotPose, TowerArray,
                         TowerState)

from . import field_constants as fc
from . import field_geometry as fg
from . import scene_3d
from .physics_blocks import LEVEL_GROUND, LEVEL_L1, LEVEL_L2
from .robot_body import M_TO_MM, RobotBody, add_static_field, make_robot_body
from .robot_spec import body_axes, declare_robot_specs, mecanum_wheel_speeds

PHYSICS_HZ = 60.0
SUBSTEPS = 8  # MuJoCoの1ステップ = 1/(60*8)秒
MM = scene_3d.MM
ROBOT_IDS = ('tr', 'br')

# 機構: CLOSE/FLIPで、目標位置からこの距離以内の物を対象にする
TARGET_TOLERANCE_MM = 150.0
# 置く・はなすときに、下にある物の上面からこれだけ浮かせて置く(めり込み防止)
DROP_CLEARANCE_MM = 3.0
# 中央の柱の受けにムスティカが収まったとみなす、柱の中心からの距離
SOCKET_CAPTURE_MM = 60.0
# 場外: フィールドの外、または床よりこれだけ下に落ちた物
OUT_OF_FIELD_DROP_MM = 300.0
# 遊技から除いた物の置き場(フィールドの外)
PARKING_U_MM = -3000.0

_EVER_HELD_CONTYPE, _EVER_HELD_CONAFFINITY = 4, 5


def _yaw_quaternion(theta: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(theta / 2), w=math.cos(theta / 2))


class _Object:
    def __init__(self, model, obj_id, kind, half, color, owner_team, top_color):
        self.id = obj_id
        self.kind = kind
        self.half = half
        self.color = color
        self.owner_team = owner_team
        self.top_color = top_color
        self.body_id = model.body(obj_id).id
        self.geom_id = model.geom(obj_id).id
        joint = model.body_jntadr[self.body_id]
        self.qpos_adr = model.jnt_qposadr[joint]
        self.dof_adr = model.jnt_dofadr[joint]
        self.held_by = 'none'
        self.removed = False
        self.on_socket = False


class _Mechanism:
    def __init__(self):
        self.holding: _Object | None = None
        self.job: dict | None = None
        self.last_command = ''
        self.last_result = ''
        self.pending: MechanismCmd | None = None


class Sim3DNode(Node):
    def __init__(self):
        super().__init__('br_sim_3d_node')
        self._specs = declare_robot_specs(self)
        self.declare_parameter('show_viewer', True)
        self.declare_parameter('screenshot_path', '')
        self.declare_parameter('screenshot_interval_s', 2.0)
        self._screenshot_path = self.get_parameter('screenshot_path').value
        self._screenshot_interval = self.get_parameter('screenshot_interval_s').value

        dt = 1.0 / PHYSICS_HZ
        self.model = mujoco.MjModel.from_xml_string(scene_3d.build_scene_xml(self._specs, dt / SUBSTEPS))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.objects = [_Object(self.model, oid, kind, half, color, team, top)
                        for oid, kind, _uvz, half, color, team, top in scene_3d.object_specs()]
        self.mustika = next(o for o in self.objects if o.kind == 'mustika')
        self._hold_eq = {(r, o.id): self.model.equality(f'hold_{r}_{o.id}').id for r in ROBOT_IDS for o in self.objects}
        self._socket_eq = self.model.equality('fix_mustika').id
        self._robot_body_id = {r: self.model.body(r).id for r in ROBOT_IDS}
        self._robot_mocap = {r: self.model.body_mocapid[self._robot_body_id[r]] for r in ROBOT_IDS}

        # ロボットの平面の動き(加減速・旋回・柵・通れない地形)は2Dと同じ平面モデルで決める
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        starts = fg.start_positions(self._specs)
        self.robots: dict[str, RobotBody] = {
            r: make_robot_body(self.space, r, starts[r], LEVEL_GROUND, self._specs[r], fc.ROBOT_WEIGHT_MAX_KG)
            for r in ROBOT_IDS}
        add_static_field(self.space, self.robots.values())
        self._body_cmd = {r: (0.0, 0.0, 0.0) for r in ROBOT_IDS}
        self._prev_velocity = {r: (0.0, 0.0) for r in ROBOT_IDS}
        self._wheel_counts = {r: [0.0] * 4 for r in ROBOT_IDS}
        self._mech = {r: _Mechanism() for r in ROBOT_IDS}
        self._lidar_elapsed = {r: 0.0 for r in ROBOT_IDS}
        self._match_ended = False
        self._screenshot_elapsed = 0.0
        self._renderer = None
        self._viewer = None
        self._update_robot_mocap()

        self.pub = {}
        for r in ROBOT_IDS:
            self.create_subscription(Twist, f'/{r}/base_cmd', lambda m, r=r: self._on_base_cmd(r, m), 10)
            self.create_subscription(MechanismCmd, f'/{r}/mechanism_cmd', lambda m, r=r: self._on_mechanism_cmd(r, m), 10)
            self.pub[r] = {
                'wheel': self.create_publisher(Int16MultiArray, f'/{r}/wheel_deltas', 10),
                'imu': self.create_publisher(Imu, f'/{r}/imu', 10),
                'scan': self.create_publisher(LaserScan, f'/{r}/scan', 10),
                'mech': self.create_publisher(MechanismState, f'/{r}/mechanism_state', 10),
            }
        self.create_subscription(Bool, '/match_ended', self._on_match_ended, 10)
        self.pub_pose = {r: self.create_publisher(RobotPose, f'/true_state/{r}_pose', 10) for r in ROBOT_IDS}
        self.pub_blocks = self.create_publisher(BlockArray, '/true_state/blocks', 10)
        self.pub_mustika = self.create_publisher(MustikaPose, '/true_state/mustika_pose', 10)
        self.pub_towers = self.create_publisher(TowerArray, '/true_state/tower_state', 10)

        if self.get_parameter('show_viewer').value:
            self._open_viewer()
        self.create_timer(dt, self._step)

    # --- 受信 ---

    def _on_base_cmd(self, robot: str, msg: Twist) -> None:
        if not self._match_ended:
            self._body_cmd[robot] = (msg.linear.x * M_TO_MM, msg.linear.y * M_TO_MM, msg.angular.z)

    def _on_mechanism_cmd(self, robot: str, msg: MechanismCmd) -> None:
        if not self._match_ended:
            self._mech[robot].pending = msg

    def _on_match_ended(self, msg: Bool) -> None:
        self._match_ended = self._match_ended or msg.data

    # --- 1ステップ ---

    def _step(self) -> None:
        dt = 1.0 / PHYSICS_HZ
        for r, robot in self.robots.items():
            if self._match_ended:
                # 9.4.1: 試合終了でロボットは直ちに停止し、以後の指令は反映しない
                robot.stop_now()
                continue
            u, v = robot.position
            robot.set_level(fg.level_at(u, v))
            forward, left = body_axes(robot.body.angle)
            bx, by, bw = self._body_cmd[r]
            robot.set_cmd_vel(bx * forward[0] + by * left[0], bx * forward[1] + by * left[1], bw)
            robot.step(dt, fg.speed_scale_at(robot.spec, u, v))
        self.space.step(dt)
        self._update_robot_mocap()
        if not self._match_ended:
            for r in ROBOT_IDS:
                self._update_mechanism(r, dt)
        lock = self._viewer.lock() if self._viewer is not None else _NullLock()
        with lock:
            for _ in range(SUBSTEPS):
                mujoco.mj_step(self.model, self.data)
        self._enforce_out_of_field()
        for r in ROBOT_IDS:
            self._publish_wheels_and_imu(r, dt)
            self._lidar_elapsed[r] += dt
            if self._lidar_elapsed[r] >= 1.0 / self._specs[r].lidar_rate_hz:
                self._lidar_elapsed[r] = 0.0
                self._publish_scan(r)
            self._publish_mechanism_state(r)
        self._publish_true_state()
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()
        self._maybe_screenshot(dt)

    def _update_robot_mocap(self) -> None:
        for r, robot in self.robots.items():
            u, v = robot.position
            z = fg.floor_height_at(u, v) + robot.spec.body_height_mm / 2
            mocap = self._robot_mocap[r]
            self.data.mocap_pos[mocap] = (u * MM, v * MM, z * MM)
            theta = robot.body.angle
            self.data.mocap_quat[mocap] = (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2))

    # --- 物の操作 ---

    def _obj_pos_mm(self, obj: _Object) -> np.ndarray:
        return self.data.xpos[obj.body_id] * 1000.0

    def _teleport(self, obj: _Object, pos_mm, quat=(1.0, 0.0, 0.0, 0.0)) -> None:
        self.data.qpos[obj.qpos_adr:obj.qpos_adr + 3] = np.asarray(pos_mm) * MM
        self.data.qpos[obj.qpos_adr + 3:obj.qpos_adr + 7] = quat
        self.data.qvel[obj.dof_adr:obj.dof_adr + 6] = 0.0

    def _robot_frame_to_field(self, robot: str, target: Point) -> tuple[float, float]:
        rb = self.robots[robot]
        forward, left = body_axes(rb.body.angle)
        return (rb.position.x + target.x * forward[0] + target.y * left[0],
                rb.position.y + target.x * forward[1] + target.y * left[1])

    def _top_object_near(self, uv, kinds=('earth', 'sky', 'mustika')) -> _Object | None:
        """uvから一定距離内の、保持されていない物のうち一番高い所にある物。"""
        best = None
        for obj in self.objects:
            if obj.removed or obj.held_by != 'none' or obj.kind not in kinds:
                continue
            p = self._obj_pos_mm(obj)
            if math.dist((p[0], p[1]), uv) <= TARGET_TOLERANCE_MM and (best is None or p[2] > self._obj_pos_mm(best)[2]):
                best = obj
        return best

    def _surface_height_below(self, uv, exclude: _Object | None) -> float:
        """uvの真下にある物(塔・台・床)の上面の高さ(mm)。ロボットは対象外。"""
        group = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        geomid = np.zeros(1, dtype=np.int32)
        top = 3000.0
        dist = mujoco.mj_ray(self.model, self.data, np.array([uv[0] * MM, uv[1] * MM, top * MM]),
                             np.array([0.0, 0.0, -1.0]), group, 1,
                             exclude.body_id if exclude is not None else -1, geomid)
        return top - dist * 1000.0 if dist >= 0 else 0.0

    def _attach(self, robot: str, obj: _Object) -> None:
        rb = self.robots[robot]
        u, v = rb.position
        lift = rb.spec.body_height_mm / 2 + obj.half + DROP_CLEARANCE_MM
        # 一度でも保持された物はロボットとぶつからない(2Dと同じ)
        self.model.geom_contype[obj.geom_id] = _EVER_HELD_CONTYPE
        self.model.geom_conaffinity[obj.geom_id] = _EVER_HELD_CONAFFINITY
        if obj.on_socket:
            self.data.eq_active[self._socket_eq] = 0
            obj.on_socket = False
        eq = self._hold_eq[(robot, obj.id)]
        self.model.eq_data[eq, 3:6] = (0.0, 0.0, lift * MM)
        self.model.eq_data[eq, 6:10] = (1.0, 0.0, 0.0, 0.0)
        center_z = fg.floor_height_at(u, v) + rb.spec.body_height_mm / 2
        self._teleport(obj, (u, v, center_z + lift), self.data.mocap_quat[self._robot_mocap[robot]])
        self.data.eq_active[eq] = 1
        obj.held_by = robot
        self._mech[robot].holding = obj

    def _detach_onto(self, robot: str, uv) -> None:
        obj = self._mech[robot].holding
        self.data.eq_active[self._hold_eq[(robot, obj.id)]] = 0
        z = self._surface_height_below(uv, exclude=obj) + obj.half + DROP_CLEARANCE_MM
        self._teleport(obj, (uv[0], uv[1], z))
        obj.held_by = 'none'
        self._mech[robot].holding = None
        if obj.kind == 'mustika':
            center = (fc.FIELD_CENTER, fc.FIELD_CENTER)
            top = fc.L2_Z + fc.CENTRAL_PILLAR_HEIGHT
            if math.dist(uv, center) <= SOCKET_CAPTURE_MM and z > top:
                # 受けに収まった: 柱に固定する(受けの凹みの形は作らない簡略化)
                self._teleport(obj, (center[0], center[1], top + obj.half))
                # 溶接拘束の相対位置は「body1(ムスティカ)から見たbody2(ワールド原点)の位置」なので符号が逆になる
                self.model.eq_data[self._socket_eq, 3:6] = (-center[0] * MM, -center[1] * MM, -(top + obj.half) * MM)
                self.model.eq_data[self._socket_eq, 6:10] = (1.0, 0.0, 0.0, 0.0)
                self.data.eq_active[self._socket_eq] = 1
                obj.on_socket = True

    # --- 機構(CLOSE/OPEN/PLACE/FLIP) ---

    def _reach_mm(self, robot: str, extra: float) -> float:
        spec = self._specs[robot]
        return spec.half_width_mm + spec.grasp_reach_mm + extra

    def _update_mechanism(self, robot: str, dt: float) -> None:
        mech = self._mech[robot]
        spec = self._specs[robot]
        cmd, mech.pending = mech.pending, None
        if cmd is not None:
            self._accept(robot, cmd)
        job = mech.job
        if job is None:
            return
        if job['kind'] == 'CLOSE':
            obj = job['object']
            rb = self.robots[robot]
            p = self._obj_pos_mm(obj)
            if obj.held_by != 'none' or obj.removed or \
                    math.dist((p[0], p[1]), tuple(rb.position)) > spec.grasp_range_mm(obj.kind):
                mech.job, mech.last_result = None, 'NOTHING_THERE'
                return
        job['elapsed'] += dt
        if job['elapsed'] < job['duration']:
            return
        mech.job = None
        mech.last_result = 'OK'
        if job['kind'] == 'CLOSE':
            self._attach(robot, job['object'])
        elif job['kind'] == 'OPEN':
            self._detach_onto(robot, tuple(self.robots[robot].position))
        elif job['kind'] == 'PLACE':
            self._detach_onto(robot, job['target'])
        elif job['kind'] == 'FLIP':
            obj = self._top_object_near(job['target'], kinds=('sky',))
            if obj is None:
                mech.last_result = 'NOTHING_THERE'
                return
            obj.top_color = fc.TeamColor.BLUE if obj.top_color == fc.TeamColor.RED else fc.TeamColor.RED
            self.model.geom_rgba[obj.geom_id] = scene_3d.color_rgba(f'sky_{obj.top_color}')

    def _accept(self, robot: str, cmd: MechanismCmd) -> None:
        """指令を受け付けて動作を始める。動作中・意味のない指令は無視する(実機と同じく再送してよい)。"""
        mech = self._mech[robot]
        spec = self._specs[robot]
        kind = cmd.command
        busy = mech.job is not None
        if kind == 'OPEN':
            if busy and mech.job['kind'] in ('PLACE', 'FLIP', 'OPEN'):
                return  # 置く/裏返す/はなす動作中は開指令を無視する
            mech.job = None
            mech.last_command = kind
            if mech.holding is None:
                mech.last_result = 'OK'
                return
            mech.job = {'kind': 'OPEN', 'elapsed': 0.0, 'duration': spec.release_time_s}
            mech.last_result = ''
            return
        if busy or kind not in ('CLOSE', 'PLACE', 'FLIP'):
            return
        mech.last_command = kind
        target = self._robot_frame_to_field(robot, cmd.target)
        center = tuple(self.robots[robot].position)
        if kind == 'CLOSE':
            if mech.holding is not None:
                return
            obj = self._top_object_near(target)
            if obj is None:
                mech.last_result = 'NOTHING_THERE'
                return
            p = self._obj_pos_mm(obj)
            if math.dist((p[0], p[1]), center) > spec.grasp_range_mm(obj.kind):
                mech.last_result = 'OUT_OF_REACH'
                return
            mech.job = {'kind': 'CLOSE', 'object': obj, 'elapsed': 0.0, 'duration': spec.grasp_time_s}
        else:
            if kind == 'PLACE' and mech.holding is None:
                mech.last_result = 'NOT_HOLDING'
                return
            if math.dist(target, center) > self._reach_mm(robot, fc.BUILD_SPOT_SIZE[0] / 2):
                mech.last_result = 'OUT_OF_REACH'
                return
            duration = spec.place_time_s if kind == 'PLACE' else spec.flip_time_s
            mech.job = {'kind': kind, 'target': target, 'elapsed': 0.0, 'duration': duration}
        mech.last_result = ''

    # --- 場外 ---

    def _enforce_out_of_field(self) -> None:
        """6.4 場外: フィールドの外に出た物は遊技から除き(アース/スカイ)、ムスティカは柱に戻す。"""
        for obj in self.objects:
            if obj.removed or obj.held_by != 'none':
                continue
            p = self._obj_pos_mm(obj)
            inside = 0.0 <= p[0] <= fc.GAME_FIELD_SIZE and 0.0 <= p[1] <= fc.GAME_FIELD_SIZE
            if inside and p[2] > -OUT_OF_FIELD_DROP_MM:
                continue
            if obj.kind == 'mustika':
                pu, pv = fc.MUSTIKA_PILLAR_ORIGIN
                self._teleport(obj, (pu, pv, fc.MUSTIKA_PILLAR_HEIGHT + obj.half))
            else:
                obj.removed = True
                self._teleport(obj, (PARKING_U_MM, 500.0 + 500.0 * self.objects.index(obj), obj.half))

    # --- ハードウェア境界の出力 ---

    def _body_velocity(self, robot: str) -> tuple[float, float, float]:
        rb = self.robots[robot]
        forward, left = body_axes(rb.body.angle)
        vu, vv = rb.body.velocity
        return (vu * forward[0] + vv * forward[1], vu * left[0] + vv * left[1], rb.body.angular_velocity)

    def _publish_wheels_and_imu(self, robot: str, dt: float) -> None:
        spec = self._specs[robot]
        vx, vy, wz = self._body_velocity(robot)
        counts_per_rad = spec.encoder_counts_per_rev / (2 * math.pi)
        deltas = []
        for i, w in enumerate(mecanum_wheel_speeds(spec, vx, vy, wz)):
            self._wheel_counts[robot][i] += w * dt * counts_per_rad * (1.0 + random.gauss(0.0, spec.wheel_slip_std))
            whole = int(self._wheel_counts[robot][i])
            self._wheel_counts[robot][i] -= whole
            deltas.append(whole)
        self.pub[robot]['wheel'].publish(Int16MultiArray(data=deltas))

        rb = self.robots[robot]
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = f'{robot}/imu'
        imu.orientation = _yaw_quaternion(rb.body.angle + random.gauss(0.0, spec.imu_yaw_noise_std_rad))
        imu.angular_velocity.z = wz + random.gauss(0.0, spec.imu_gyro_noise_std_rad_s)
        pvx, pvy = self._prev_velocity[robot]
        imu.linear_acceleration.x = (vx - pvx) / 1000.0 / dt
        imu.linear_acceleration.y = (vy - pvy) / 1000.0 / dt
        imu.linear_acceleration.z = 9.81
        self._prev_velocity[robot] = (vx, vy)
        self.pub[robot]['imu'].publish(imu)

    def _publish_scan(self, robot: str) -> None:
        spec = self._specs[robot]
        rb = self.robots[robot]
        n = spec.lidar_beams
        u, v = rb.position
        origin = np.array([u * MM, v * MM, (fg.floor_height_at(u, v) + spec.lidar_mount_height_mm) * MM])
        theta = rb.body.angle
        angles = -math.pi + np.arange(n) * (2 * math.pi / n)
        # 機体座標のx(前方)からの角度。フィールドでの前方は(-sin, cos)
        heading = angles + theta + math.pi / 2
        vec = np.stack([np.cos(heading), np.sin(heading), np.zeros(n)], axis=1).reshape(-1)
        geomid = np.zeros(n, dtype=np.int32)
        dist = np.zeros(n, dtype=np.float64)
        group = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
        mujoco.mj_multiRay(self.model, self.data, origin, vec, group, 1, self._robot_body_id[robot],
                           geomid, dist, None, n, spec.lidar_range_max_mm * MM)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f'{robot}/laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / n
        scan.angle_max = scan.angle_min + scan.angle_increment * (n - 1)
        scan.range_min = 0.05
        scan.range_max = spec.lidar_range_max_mm * MM
        scan.ranges = [float(d) if d >= 0 else float('inf') for d in dist]
        self.pub[robot]['scan'].publish(scan)

    def _publish_mechanism_state(self, robot: str) -> None:
        mech = self._mech[robot]
        self.pub[robot]['mech'].publish(MechanismState(
            holding=mech.holding is not None, busy=mech.job is not None,
            last_command=mech.last_command, last_result=mech.last_result))

    # --- 正解値(/true_state/*) ---

    def _towers(self):
        """各建築スポットの上に実際に積まれている物(下から順)。落ちた物は数えない。"""
        towers = {}
        for spot_id, level, _team, origin in fc.BUILD_SPOTS:
            base = fc.L1_Z if level == 1 else fc.L2_Z
            members = []
            for obj in self.objects:
                if obj.removed or obj.held_by != 'none' or obj.kind == 'mustika':
                    continue
                p = self._obj_pos_mm(obj)
                if (origin[0] <= p[0] <= origin[0] + fc.BUILD_SPOT_SIZE[0]
                        and origin[1] <= p[1] <= origin[1] + fc.BUILD_SPOT_SIZE[1] and p[2] > base):
                    members.append((p[2], obj))
            if members:
                towers[spot_id] = (level, [o for _z, o in sorted(members, key=lambda zo: zo[0])])
        return towers

    def _publish_true_state(self) -> None:
        for r, rb in self.robots.items():
            u, v = rb.position
            self.pub_pose[r].publish(RobotPose(
                pose=Pose2D(x=float(u), y=float(v), theta=float(rb.body.angle)), level=fg.level_at(u, v)))

        towers = self._towers()
        placed_level = {o.id: level for level, members in towers.values() for o in members}
        blocks = BlockArray()
        for obj in self.objects:
            if obj.removed or obj.kind == 'mustika':
                continue
            p = self._obj_pos_mm(obj)
            blocks.blocks.append(Block(
                id=obj.id, position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                level=placed_level.get(obj.id, LEVEL_GROUND), block_type=obj.kind, top_color=obj.top_color,
                owner_team=obj.owner_team, held_by=obj.held_by, placed=obj.id in placed_level))
        self.pub_blocks.publish(blocks)

        m = self.mustika
        p = self._obj_pos_mm(m)
        self.pub_mustika.publish(MustikaPose(
            position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
            level=LEVEL_L2 if m.on_socket else LEVEL_GROUND, held_by=m.held_by))

        msg = TowerArray()
        for spot_id, (level, members) in towers.items():
            msg.towers.append(TowerState(
                build_spot_id=spot_id, level=level, block_types=[o.kind for o in members],
                owner_teams=[o.owner_team if o.kind == 'earth' else '' for o in members],
                top_colors=[o.top_color for o in members]))
        self.pub_towers.publish(msg)

    # --- 表示 ---

    def _open_viewer(self) -> None:
        try:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False,
                                                        show_right_ui=False)
            cam = self._viewer.cam
            cam.lookat[:] = (5.5, 4.5, 0.3)
            cam.distance, cam.elevation, cam.azimuth = 14.0, -50.0, 90.0
        except Exception as e:  # 画面の無い環境などでは表示なしで続ける
            self.get_logger().warn(f'MuJoCoのビューアを開けませんでした(表示なしで続行): {e}')
            self._viewer = None

    def _maybe_screenshot(self, dt: float) -> None:
        if not self._screenshot_path:
            return
        self._screenshot_elapsed += dt
        if self._screenshot_elapsed < self._screenshot_interval:
            return
        self._screenshot_elapsed = 0.0
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 720, 960)
            self._camera = mujoco.MjvCamera()
            self._camera.lookat[:] = (5.5, 4.5, 0.3)
            self._camera.distance, self._camera.elevation, self._camera.azimuth = 14.0, -50.0, 90.0
        self._renderer.update_scene(self.data, camera=self._camera)
        image = self._renderer.render()
        import pygame
        tmp = self._screenshot_path + '.tmp.png'
        pygame.image.save(pygame.surfarray.make_surface(image.swapaxes(0, 1)), tmp)
        os.replace(tmp, self._screenshot_path)


class _NullLock:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Sim3DNode()
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
