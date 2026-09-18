"""
複数塔+ムスティカのend-to-end受け入れテスト。

TRがストレージのアース2個・共用エリアのスカイ1個を受渡しエリア経由でBRに
届け、BRがBUILD_SPOT_PLAN(現状L1・L2に1本ずつ)に完成塔を組む一連の流れを、
秘蹟の要件(Sanctuary Mandate)を満たしたムスティカの回収・中央支柱への設置
まで含めて確認する。実時間で2分前後かかる(意思決定ロジックが実際に
ロボットを動かして最後まで通す想定のため、早送りはしない)。
"""

from __future__ import annotations

import rclpy
from std_msgs.msg import Int32

from br_strategy_sim.br_decision_br_node import BrState
from br_strategy_sim.br_decision_tr_node import TrState

from ._sim_harness import sim_harness

TIMEOUT_S = 200.0
# 塔2本(L1: 10+30+70=..最終210) + ムスティカ奉納(250) = 460点
EXPECTED_FINAL_SCORE = 460


def test_multi_tower_and_mustika_reach_expected_score():
    with sim_harness(enable_decision=True) as harness:
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

        assert reached_end, (
            f'TR/BRが時間内に完了状態へ到達しなかった '
            f'(tr_state={tr._state}, br_state={br._state}, score={scores["red"]})'
        )
        assert scores['red'] == EXPECTED_FINAL_SCORE, (
            f'最終スコアが期待値と異なる: {scores["red"]} != {EXPECTED_FINAL_SCORE}'
        )
