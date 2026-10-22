"""周期処理と経過時間の計測に使う、巻き戻らない時計。

ROS2のタイマーは既定ではシステム時計を基準にするため、システム時計が巻き戻ると、追い付くまで
タイマーが鳴らなくなる(この開発環境のWSLでは、ハードウェア時計の扱いのずれで時計が9時間
跳んでは戻ることがあり、シムが途中で止まって見えた)。各ノードの周期処理と経過時間はこちらで測る。
"""

from rclpy.clock import Clock, ClockType


def steady_clock() -> Clock:
    return Clock(clock_type=ClockType.STEADY_TIME)
