"""
ABU Robocon 2027 Phase 1 2Dシムの一括起動launchファイル。

br_sim_bridge_node, br_referee_node, br_observation_node, br_visualizer_node,
br_decision_tr_node, br_decision_br_nodeをまとめて起動する。

起動方法:
    ros2 launch br_strategy_sim launch_simulator.py
    ros2 launch br_strategy_sim launch_simulator.py observation_noise:=true
    ros2 launch br_strategy_sim launch_simulator.py team:=blue
    ros2 launch br_strategy_sim launch_simulator.py enable_decision:=false
    ros2 launch br_strategy_sim launch_simulator.py enable_decision:=false enable_teleop:=true \
        match_duration_sec:=999999
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    observation_noise_arg = DeclareLaunchArgument('observation_noise', default_value='false')
    team_arg = DeclareLaunchArgument('team', default_value='red')
    # ウィンドウが出ない環境でのデバッグ用: 指定すると可視化ノードが
    # 最新フレームをこのパスへ毎ティック上書き保存する
    screenshot_path_arg = DeclareLaunchArgument('screenshot_path', default_value='')
    # false にすると br_decision_tr_node / br_decision_br_node を起動しない
    # (手動でcmd_vel等をpublishして単体動作を確認したい場合向け)
    enable_decision_arg = DeclareLaunchArgument('enable_decision', default_value='true')
    # true にすると br_teleop_node (キーボード手動操縦コンソール) を起動する。
    # enable_decision:=false と組み合わせて、意思決定ノードの代わりに
    # 手動でTR/BRを動かす用途を想定
    enable_teleop_arg = DeclareLaunchArgument('enable_teleop', default_value='false')
    # br_referee_nodeが9.1試合時間の終了(ブザー)とみなすまでの秒数。
    # 既定値はfield_constants.MATCH_DURATION_SEC(3分=180秒)と一致させて
    # あるが、手動操縦(enable_teleop:=true)で3分を超えて長時間テストしたい
    # 場合は大きな値(例: 999999)を指定する。試合終了後はsim_bridge_nodeが
    # 全てのcmd_vel/gripper_cmd等を無視するようになるため(9.4.1)、これを
    # 忘れると「操作しても反応しない」ように見える
    match_duration_sec_arg = DeclareLaunchArgument('match_duration_sec', default_value='180.0')

    return LaunchDescription([
        observation_noise_arg,
        team_arg,
        screenshot_path_arg,
        enable_decision_arg,
        enable_teleop_arg,
        match_duration_sec_arg,
        Node(package='br_strategy_sim', executable='br_sim_bridge_node', name='br_sim_bridge_node'),
        Node(
            package='br_strategy_sim',
            executable='br_referee_node',
            name='br_referee_node',
            parameters=[{
                'team': LaunchConfiguration('team'),
                'match_duration_sec': LaunchConfiguration('match_duration_sec'),
            }],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_observation_node',
            name='br_observation_node',
            parameters=[{'observation_noise': LaunchConfiguration('observation_noise')}],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_visualizer_node',
            name='br_visualizer_node',
            parameters=[{'screenshot_path': LaunchConfiguration('screenshot_path')}],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_decision_tr_node',
            name='br_decision_tr_node',
            condition=IfCondition(LaunchConfiguration('enable_decision')),
        ),
        Node(
            package='br_strategy_sim',
            executable='br_decision_br_node',
            name='br_decision_br_node',
            condition=IfCondition(LaunchConfiguration('enable_decision')),
        ),
        Node(
            package='br_strategy_sim',
            executable='br_teleop_node',
            name='br_teleop_node',
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
        ),
    ])
