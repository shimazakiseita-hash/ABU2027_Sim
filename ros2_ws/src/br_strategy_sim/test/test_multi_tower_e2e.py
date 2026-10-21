"""
複数塔+ムスティカのend-to-end受け入れテスト。

TRがストレージのアース2個・共用エリアのスカイ1個を受渡しエリア経由でBRに
届け、BRがBUILD_SPOT_PLAN(現状L1・L2に1本ずつ)に完成塔を組む一連の流れを、
秘蹟の要件(Sanctuary Mandate)を満たしたムスティカの回収・中央支柱への設置
まで含めて確認する。既定の機体仕様(config/robot_spec.yamlの仮値、最高速度0.3m/s)
では実時間で11分前後かかる(意思決定ロジックが実際にロボットを動かして最後まで
通す想定のため、早送りはしない)。

既定の仮値では試合時間3分に収まらない(CLAUDE.md「既知の残課題」参照)。この
テストは意思決定ロジックが最後まで通ることの確認が目的なので、試合時間を延ばして
実行する。
"""

from __future__ import annotations

import rclpy
from std_msgs.msg import Int32

from br_strategy_sim import field_constants as fc
from br_strategy_sim.br_decision_br_node import BrState
from br_strategy_sim.br_decision_tr_node import TrState

from ._sim_harness import sim_harness

TIMEOUT_S = 1100.0
MATCH_DURATION_SEC = 2000.0
# 塔2本(210点) + 受渡し点6個x5点(30点) + ムスティカ奉納(250点) = 490点
EXPECTED_FINAL_SCORE = 490


def test_multi_tower_and_mustika_reach_expected_score(monkeypatch):
    monkeypatch.setattr(fc, 'MATCH_DURATION_SEC', MATCH_DURATION_SEC)

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
        # 完了状態はBRが設置完了を観測した時点なので、審判の得点更新が届くまで少し待つ
        harness.spin_until(lambda: scores['red'] == EXPECTED_FINAL_SCORE, timeout_s=5.0)
        assert scores['red'] == EXPECTED_FINAL_SCORE, (
            f'最終スコアが期待値と異なる: {scores["red"]} != {EXPECTED_FINAL_SCORE}'
        )
