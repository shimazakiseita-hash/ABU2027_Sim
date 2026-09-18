"""
統合テスト共通ヘルパー。

`ros2 launch`経由の複数プロセス起動は、この開発環境(サンドボックス)では
子プロセスが不定期にSIGKILLされ不安定だったことが分かっている(詳細はgit
履歴参照)。そのため統合テストは全ノードを1プロセス内で直接インスタンス化し、
`SingleThreadedExecutor`で駆動する方式に統一する(手動検証時から一貫して
使ってきた方式)。

各テストは実時間で数十秒〜数分かかる(意思決定ロジックが実際にロボットを
動かして塔を組む/ムスティカを運ぶ過程を検証するため、シミュレーション時間を
早送りする仕組みは持たない)。CIで毎コミット回す想定ではなく、シム全体の
end-to-endな健全性を確認する受け入れテストという位置づけ。
"""

from __future__ import annotations

import contextlib
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from br_strategy_sim.br_decision_br_node import BrDecisionBrNode
from br_strategy_sim.br_decision_tr_node import BrDecisionTrNode
from br_strategy_sim.br_observation_node import BrObservationNode
from br_strategy_sim.br_referee_node import BrRefereeNode
from br_strategy_sim.sim_bridge_node import SimBridgeNode


class SimHarness:
    """物理シム・審判・観測・(必要なら)TR/BR意思決定ノードを1プロセスで束ねる。"""

    def __init__(self, enable_decision: bool = True):
        rclpy.init()
        self.bridge = SimBridgeNode()
        self.referee = BrRefereeNode()
        self.observation = BrObservationNode()
        self.nodes = [self.bridge, self.referee, self.observation]

        self.tr_decision: BrDecisionTrNode | None = None
        self.br_decision: BrDecisionBrNode | None = None
        if enable_decision:
            self.tr_decision = BrDecisionTrNode()
            self.br_decision = BrDecisionBrNode()
            self.nodes += [self.tr_decision, self.br_decision]

        self.executor = SingleThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)

    def add_node(self, node) -> None:
        """テスト専用のwatcherノード等を追加する(shutdown時に自動的に破棄される)。"""
        self.nodes.append(node)
        self.executor.add_node(node)

    def spin_for(self, seconds: float, tick_sec: float = 0.05) -> None:
        """指定秒数(実時間)スピンし続ける。"""
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            self.executor.spin_once(timeout_sec=tick_sec)

    def spin_until(self, predicate, timeout_s: float, tick_sec: float = 0.05) -> bool:
        """predicate()がTrueになるまでスピンする。timeout_s以内に満たせなければFalseを返す。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self.executor.spin_once(timeout_sec=tick_sec)
            if predicate():
                return True
        return False

    def shutdown(self) -> None:
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()


@contextlib.contextmanager
def sim_harness(enable_decision: bool = True):
    harness = SimHarness(enable_decision=enable_decision)
    try:
        yield harness
    finally:
        harness.shutdown()
