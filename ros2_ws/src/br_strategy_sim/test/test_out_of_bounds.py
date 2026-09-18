"""
6.4場外の受け入れテスト。

意思決定ノードは起動しない(enable_decision=False)。ブロック/ムスティカを
テストコードから直接フィールド外へ移動させ、sim_bridge_nodeが実際に
遊技から除外/開始位置へ復帰させること、br_referee_nodeがそれを検出して
`violation`(type="out_of_bounds")を記録することを確認する。
"""

from __future__ import annotations

import rclpy

from br_msgs.msg import Violation

from ._sim_harness import sim_harness


def test_earth_block_knocked_out_is_removed_and_reported():
    with sim_harness(enable_decision=False) as harness:
        bridge = harness.bridge
        violations: list[tuple[str, str, str, bool]] = []
        watcher = rclpy.create_node('violation_watcher')
        watcher.create_subscription(
            Violation, '/violation',
            lambda m: violations.append((m.type, m.robot, m.team, m.forced_retry)), 10)
        harness.add_node(watcher)

        # true_stateの初期配信を待つ
        harness.spin_for(0.5)

        target_block = bridge.blocks[0]
        block_id = target_block.id
        target_block.body.position = (-500.0, -500.0)

        removed = harness.spin_until(
            lambda: not any(b.id == block_id for b in bridge.blocks), timeout_s=5.0)
        assert removed, 'フィールド外のブロックがself.blocksから除去されなかった'

        reported = harness.spin_until(
            lambda: any(v[0] == 'out_of_bounds' for v in violations), timeout_s=5.0)
        assert reported, f'out_of_bounds違反が記録されなかった: {violations}'


def test_mustika_knocked_out_returns_to_pillar_and_is_reported():
    with sim_harness(enable_decision=False) as harness:
        import math

        from br_strategy_sim import field_constants as fc

        bridge = harness.bridge
        violations: list[tuple[str, str, str, bool]] = []
        watcher = rclpy.create_node('violation_watcher2')
        watcher.create_subscription(
            Violation, '/violation',
            lambda m: violations.append((m.type, m.robot, m.team, m.forced_retry)), 10)
        harness.add_node(watcher)

        harness.spin_for(0.5)

        bridge.mustika.body.position = (99999.0, 99999.0)

        def mustika_back_at_pillar() -> bool:
            pos = bridge.mustika.body.position
            return math.hypot(
                pos.x - fc.MUSTIKA_PILLAR_ORIGIN[0], pos.y - fc.MUSTIKA_PILLAR_ORIGIN[1]) < 1.0

        reset_done = harness.spin_until(mustika_back_at_pillar, timeout_s=5.0)
        assert reset_done, 'フィールド外のムスティカがムスティカ柱へ復帰しなかった'
        assert bridge.mustika.held_by == 'none'

        reported = harness.spin_until(
            lambda: any(v[0] == 'out_of_bounds' for v in violations), timeout_s=5.0)
        assert reported, f'out_of_bounds違反が記録されなかった: {violations}'
