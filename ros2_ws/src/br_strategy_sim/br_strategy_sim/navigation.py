"""
ABU Robocon 2027 Phase 1 2Dシム: 意思決定ノード用の経路計画(格子A*)と経路追従。

フィールドの柵(field_geometry.WALL_SEGMENTS)を車体の半幅だけ膨らませた格子地図を
ロボットごとに作り、A*で経路を求めて、見通しの良い点まで間引いた経由点を順にたどる。
通行不可にするのは、柵のほか、機体仕様で通れない地形(階段/スロープ)と、TRについては
ルール6.2.1で入れないL1/L2。地図はフィールドの既知の形状と機体仕様だけから作る
(シムの内部状態は見ない)ので、実機でも同じものが使える。
"""

from __future__ import annotations

import heapq
import math

import numpy as np
from geometry_msgs.msg import Twist

from . import field_constants as fc
from . import field_geometry as fg

CELL_MM = 100.0
# 柵からの余裕(車体半幅に足す)
WALL_CLEARANCE_MM = 30.0
# 入れない区域(TRのL1/L2、通れない地形)からの余裕(車体半幅に足す)。格子の粗さ(100mm)と、
# 曲がるときの膨らみで車体がはみ出さないよう、柵より大きく取る
KEEP_OUT_CLEARANCE_MM = 120.0
# 経由点にこの距離まで近づいたら次へ進む
WAYPOINT_REACHED_MM = 100.0
# 曲がり角(経由点)を通過するときの速度の下限
CORNER_SPEED_M_S = 0.05
# 次の経由点からこれ以上離れていて、しかも見通せなくなったら再計画する
OFF_PATH_REPLAN_MM = 500.0
# 再計画が必要かを確かめる間隔(drive呼び出し回数)
REPLAN_CHECK_INTERVAL = 10


def _inflated_box(px, py, u0, v0, u1, v1, mu, mv):
    """軸平行の矩形[u0,u1]x[v0,v1]をU方向mu・V方向mvだけ膨らませた範囲に入る点(numpy配列)。"""
    return (px > u0 - mu) & (px < u1 + mu) & (py > v0 - mv) & (py < v1 + mv)


class GridMap:
    """ロボット1台分の通行可否の格子地図。"""

    def __init__(self, robot_id: str, spec):
        self.n = int(math.ceil(fc.GAME_FIELD_SIZE / CELL_MM))
        centers = (np.arange(self.n) + 0.5) * CELL_MM
        px, py = np.meshgrid(centers, centers, indexing='ij')  # [iu, iv]
        # ロボットは回転しない(向き0、長さ方向が+V)四角形で、柵もすべて縦横の線分なので、
        # 柵を車体の半分の大きさの四角で膨らませれば車体がちょうど触れない範囲になる
        # (円で膨らませると、柵の端点に車体の角が引っかかる)
        hu = spec.footprint_mm[1] / 2
        hv = spec.footprint_mm[0] / 2

        blocked = np.zeros((self.n, self.n), dtype=bool)
        for _kind, p0, p1 in fg.WALL_SEGMENTS:
            blocked |= _inflated_box(px, py, min(p0[0], p1[0]), min(p0[1], p1[1]), max(p0[0], p1[0]),
                                     max(p0[1], p1[1]), hu + WALL_CLEARANCE_MM, hv + WALL_CLEARANCE_MM)
        keep_out = list(fg.forbidden_terrain_rects(spec))
        if robot_id == 'tr':
            keep_out += fg.TR_KEEP_OUT_RECTS
        for (u0, v0), (w, h) in keep_out:
            # 車体の一部でも入らないよう、車体の半分+余裕だけ膨らませた矩形を通行不可にする
            blocked |= _inflated_box(px, py, u0, v0, u0 + w, v0 + h,
                                     hu + KEEP_OUT_CLEARANCE_MM, hv + KEEP_OUT_CLEARANCE_MM)
        self.blocked = blocked

    def cell_of(self, p) -> tuple[int, int]:
        return (min(max(int(p[0] // CELL_MM), 0), self.n - 1), min(max(int(p[1] // CELL_MM), 0), self.n - 1))

    @staticmethod
    def center_of(cell) -> tuple[float, float]:
        return ((cell[0] + 0.5) * CELL_MM, (cell[1] + 0.5) * CELL_MM)

    def is_free(self, cell) -> bool:
        return 0 <= cell[0] < self.n and 0 <= cell[1] < self.n and not self.blocked[cell]

    def nearest_free(self, cell) -> tuple[int, int] | None:
        if self.is_free(cell):
            return cell
        free = np.argwhere(~self.blocked)
        if len(free) == 0:
            return None
        i = int(np.argmin(np.hypot(free[:, 0] - cell[0], free[:, 1] - cell[1])))
        return (int(free[i][0]), int(free[i][1]))

    def line_free(self, a, b) -> bool:
        steps = max(1, int(math.dist(a, b) / (CELL_MM / 2)))
        for k in range(steps + 1):
            t = k / steps
            if not self.is_free(self.cell_of((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))):
                return False
        return True

    def plan(self, start, goal) -> list[tuple[float, float]] | None:
        """startからgoal(通れない場所なら最寄りの通れる場所)までの経由点列。"""
        s = self.nearest_free(self.cell_of(start))
        g = self.nearest_free(self.cell_of(goal))
        if s is None or g is None:
            return None
        open_heap = [(0.0, s)]
        came_from = {s: None}
        cost = {s: 0.0}
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == g:
                break
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    if du == 0 and dv == 0:
                        continue
                    nxt = (cur[0] + du, cur[1] + dv)
                    if not self.is_free(nxt):
                        continue
                    if du and dv and not (self.is_free((cur[0] + du, cur[1])) and self.is_free((cur[0], cur[1] + dv))):
                        continue  # 柵の角をすり抜けない
                    new_cost = cost[cur] + (1.4142 if du and dv else 1.0)
                    if new_cost < cost.get(nxt, math.inf):
                        cost[nxt] = new_cost
                        came_from[nxt] = cur
                        heapq.heappush(open_heap, (new_cost + math.dist(nxt, g), nxt))
        if g not in came_from:
            return None
        cells = []
        c = g
        while c is not None:
            cells.append(c)
            c = came_from[c]
        cells.reverse()
        points = [self.center_of(c) for c in cells]
        points[-1] = goal if self.is_free(self.cell_of(goal)) else points[-1]
        return self._shortcut(start, points)

    def _shortcut(self, start, points):
        """見通しが通る範囲で経由点を間引く。"""
        result = []
        anchor = start
        i = 0
        while i < len(points):
            j = len(points) - 1
            while j > i and not self.line_free(anchor, points[j]):
                j -= 1
            result.append(points[j])
            anchor = points[j]
            i = j + 1
        return result


class Navigator:
    """GridMapの経路をたどる速度指令を作る(加減速の上限を見込んで手前から減速する)。"""

    def __init__(self, grid: GridMap, spec):
        self.grid = grid
        self.spec = spec
        self._goal = None
        self._path: list[tuple[float, float]] = []
        self._checks_since_replan = 0

    def drive(self, current, target, arrival_threshold_mm: float) -> tuple[Twist, bool]:
        """targetへ向かう速度指令(フィールド座標, m/s)と、到着したかを返す。
        targetが通れない場所(柵際の建築スポット等)なら、その最寄りの通れる地点を目的地とする。"""
        if math.dist(current, target) <= arrival_threshold_mm:
            return Twist(), True
        if self._needs_replan(current, target):
            self._goal = target
            self._path = self.grid.plan(current, target) or [target]
            self._checks_since_replan = 0
        while len(self._path) > 1 and math.dist(current, self._path[0]) <= WAYPOINT_REACHED_MM:
            self._path.pop(0)
        end = self._path[-1]
        if math.dist(current, end) <= arrival_threshold_mm:
            return Twist(), True
        waypoint = self._path[0]
        corner_speed = 0.0 if len(self._path) == 1 else self._corner_speed(current, waypoint, self._path[1])
        return self._velocity_toward(current, waypoint, corner_speed), False

    def reset(self) -> None:
        self._goal = None
        self._path = []

    def _needs_replan(self, current, target) -> bool:
        if self._goal is None or math.dist(self._goal, target) > 300.0 or not self._path:
            return True
        # 押されるなどして次の経由点が見えなくなったら引き直す(A*は重いので毎回は確かめない)
        self._checks_since_replan += 1
        if self._checks_since_replan < REPLAN_CHECK_INTERVAL:
            return False
        self._checks_since_replan = 0
        return (math.dist(current, self._path[0]) > OFF_PATH_REPLAN_MM
                and not self.grid.line_free(current, self._path[0]))

    def _corner_speed(self, current, waypoint, after) -> float:
        """経由点を通過してよい速さ。曲がる角度が大きいほど遅くする(加減速の上限があるので、
        速いまま曲がると外側へ膨らみ、柵やL1(TRは6.2.1違反)に掛かる)。"""
        a = (waypoint[0] - current[0], waypoint[1] - current[1])
        b = (after[0] - waypoint[0], after[1] - waypoint[1])
        na, nb = math.hypot(*a), math.hypot(*b)
        if na < 1e-6 or nb < 1e-6:
            return CORNER_SPEED_M_S
        cos_turn = (a[0] * b[0] + a[1] * b[1]) / (na * nb)
        return max(CORNER_SPEED_M_S, self.spec.max_speed_m_s * max(0.0, cos_turn) ** 2)

    def _velocity_toward(self, current, waypoint, corner_speed_m_s: float) -> Twist:
        dx, dy = waypoint[0] - current[0], waypoint[1] - current[1]
        distance = math.hypot(dx, dy)
        twist = Twist()
        if distance < 1e-6:
            return twist
        # 止まる/曲がる点で、上限加速度で減速しきれる速さに抑える(v^2 = v_c^2 + 2ad)
        speed = min(self.spec.max_speed_m_s,
                    math.sqrt(corner_speed_m_s ** 2 + 2.0 * self.spec.max_accel_m_s2 * distance / 1000.0))
        twist.linear.x = dx / distance * speed
        twist.linear.y = dy / distance * speed
        return twist
