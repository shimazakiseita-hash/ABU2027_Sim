"""
9.1試合時間(3分)・9.4試合終了時ルールの受け入れテスト。

実際の3分を待つのは非現実的なため、field_constants.MATCH_DURATION_SECを
テスト用に短縮した上で、意思決定ノードを動かしたまま試合終了を迎えさせる。
9.1.3(ブザーの瞬間の状態で最終得点を確定)・9.4.1(ロボットを直ちに停止)が
守られていることを確認する。
"""

from __future__ import annotations

import math

import rclpy
from std_msgs.msg import Bool, Int32

from br_strategy_sim import field_constants as fc

from ._sim_harness import sim_harness

SHORT_MATCH_DURATION_SEC = 20.0
POST_MATCH_OBSERVE_S = 10.0
# 速度コマンドはゼロに固定されるが、停止直後に他物体と接触していると
# pymunkの衝突解決でごくわずかに位置が補正されることがある(意思決定ノードに
# 駆動されての移動ではない、無視できる規模の物理的な"落ち着き")。これを
# 「動いた」と誤判定しないための許容量。
POSITION_DRIFT_TOLERANCE_MM = 50.0


def test_match_ends_freezes_score_and_stops_robots(monkeypatch):
    monkeypatch.setattr(fc, 'MATCH_DURATION_SEC', SHORT_MATCH_DURATION_SEC)

    with sim_harness(enable_decision=True) as harness:
        bridge = harness.bridge
        scores = {'red': 0}
        match_ended_events: list[bool] = []
        watcher = rclpy.create_node('match_end_watcher')
        watcher.create_subscription(
            Int32, '/score/red', lambda m: scores.__setitem__('red', m.data), 10)
        watcher.create_subscription(
            Bool, '/match_ended', lambda m: match_ended_events.append(m.data), 10)
        harness.add_node(watcher)

        # match_ended_eventsはwatcherノード用の購読コールバックの発火を、
        # bridge._match_endedはsim_bridge_node自身の購読コールバックの発火を
        # それぞれ見ている。SingleThreadedExecutor.spin_once()は1回につき
        # 準備済みコールバックを1つしか処理しない場合があるため、両方が
        # 実際に処理されるまで待つ(片方だけで判定すると、まだbridge側の
        # コールバックが処理されておらず速度がゼロ化される前にアサーションを
        # 実行してしまうことがある)。
        ended = harness.spin_until(
            lambda: bool(match_ended_events) and bridge._match_ended,
            timeout_s=SHORT_MATCH_DURATION_SEC + 10.0,
        )
        assert ended, '/match_endedが時間内に発行されなかった'
        assert len(match_ended_events) == 1, (
            f'/match_endedは一度だけ発行されるべき: {match_ended_events}'
        )

        # bridge._match_endedがTrueになった後、実際に速度がゼロ化されるのは
        # 次の物理tick(_step_physics)なので、それが回るまで少し待つ。
        harness.spin_for(0.5)

        score_at_end = scores['red']
        tr_pos_at_end = (bridge.tr.position.x, bridge.tr.position.y)
        br_pos_at_end = (bridge.br.position.x, bridge.br.position.y)
        assert bridge.tr.body.velocity.length < 1e-6
        assert bridge.br.body.velocity.length < 1e-6

        # 試合終了後もしばらくスピンし続け、意思決定ノードが(match_endedを
        # 知らずに)cmd_vel等を送り続けても、スコア・ロボット位置が変化しない
        # ことを確認する(9.1.3/9.4.1)。
        harness.spin_for(POST_MATCH_OBSERVE_S)

        assert scores['red'] == score_at_end, (
            f'試合終了後にスコアが変化した: {score_at_end} -> {scores["red"]}'
        )
        tr_drift = math.hypot(
            bridge.tr.position.x - tr_pos_at_end[0], bridge.tr.position.y - tr_pos_at_end[1])
        br_drift = math.hypot(
            bridge.br.position.x - br_pos_at_end[0], bridge.br.position.y - br_pos_at_end[1])
        assert tr_drift < POSITION_DRIFT_TOLERANCE_MM, f'試合終了後にTRが動いた(drift={tr_drift:.1f}mm)'
        assert br_drift < POSITION_DRIFT_TOLERANCE_MM, f'試合終了後にBRが動いた(drift={br_drift:.1f}mm)'
