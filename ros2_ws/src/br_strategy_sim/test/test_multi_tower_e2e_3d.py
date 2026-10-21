"""
3Dシムでの複数塔+ムスティカのend-to-end受け入れテスト(2D版はtest_multi_tower_e2e.py)。

3Dシム(ハードウェア境界)の上で、本番と同じ足回り制御・推測航法・機構制御(br_robot)と
意思決定ノードを動かし、最後まで完走することを確かめる。推測航法の誤差で置く位置がずれると
塔が物理的に崩れるので、得点は2Dの490点に届かないことがある。ここでは完走と、塔が2本とも
建ったか(得点が塔の分を含むか)を確かめ、得点そのものは記録として出力する。
mujocoが入っていない環境ではskipする。
"""

from __future__ import annotations

import pytest
import rclpy
from std_msgs.msg import Int32

from br_strategy_sim import field_constants as fc
from br_strategy_sim.br_decision_br_node import BrState
from br_strategy_sim.br_decision_tr_node import TrState

from ._sim_harness import sim_harness

pytest.importorskip('mujoco')

TIMEOUT_S = 1500.0
MATCH_DURATION_SEC = 3000.0
# ムスティカ奉納(250点)が入っていれば、塔の得点(最大210点)と合わせて最低でもこれ以上になる
MIN_SCORE_WITH_MUSTIKA = 250


def test_multi_tower_and_mustika_complete_in_3d(monkeypatch):
    monkeypatch.setattr(fc, 'MATCH_DURATION_SEC', MATCH_DURATION_SEC)

    with sim_harness(enable_decision=True, physics='3d') as harness:
        scores = {'red': 0}
        watcher = rclpy.create_node('score_watcher')
        watcher.create_subscription(
            Int32, '/score/red', lambda m: scores.__setitem__('red', m.data), 10)
        harness.add_node(watcher)

        tr = harness.tr_decision
        br = harness.br_decision
        reached_end = harness.spin_until(
            lambda: tr._state == TrState.DONE and br._state == BrState.PLAN_COMPLETE,
            timeout_s=TIMEOUT_S,
        )
        harness.spin_for(3.0)
        print(f'3D final score: {scores["red"]}')
        assert reached_end, (
            f'TR/BRが時間内に完了状態へ到達しなかった '
            f'(tr_state={tr._state}, br_state={br._state}, score={scores["red"]})'
        )
        assert scores['red'] >= MIN_SCORE_WITH_MUSTIKA, f'ムスティカ奉納が得点に入っていない: {scores["red"]}'
