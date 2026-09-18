"""
複数塔+ムスティカのend-to-end受け入れテスト。

TRがストレージのアース2個・共用エリアのスカイ1個を受渡しエリア経由でBRに
届け、BRがBUILD_SPOT_PLAN(現状L1・L2に1本ずつ)に完成塔を組む一連の流れを、
秘蹟の要件(Sanctuary Mandate)を満たしたムスティカの回収・中央支柱への設置
まで含めて確認する。実時間で2分前後かかる(意思決定ロジックが実際に
ロボットを動かして最後まで通す想定のため、早送りはしない)。

既知の不具合(要調整、CLAUDE.md「既知の残課題」参照): 受渡しエリア内で
BRに回収されず待機しているブロックが、TR/BRの物理的な出入りでエリア境界の
すぐ外へ押し出されると、point_in_rectの厳密な内外判定により以後
「受渡し待ち」として検知されなくなり、BRが該当レイヤーを永久に待ち続ける
ことがある。統合テストでは概ね5回に3回程度の頻度で発生し、その場合この
テストはreached_endのアサーションでタイムアウト失敗する(意思決定ロジック
自体は正常だが、現在のシムの物理挙動に起因する既知のflakinessであり、
このテストが偶発的に失敗すること自体が不具合の記録になっている)。
"""

from __future__ import annotations

import rclpy
from std_msgs.msg import Int32

from br_strategy_sim.br_decision_br_node import BrState
from br_strategy_sim.br_decision_tr_node import TrState

from ._sim_harness import sim_harness

TIMEOUT_S = 200.0
# 塔2本(210点) + 受渡し点6個x5点(30点) + ムスティカ奉納(250点) = 490点
EXPECTED_FINAL_SCORE = 490


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
