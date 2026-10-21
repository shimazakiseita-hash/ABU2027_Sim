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
    ros2 launch br_strategy_sim launch_simulator.py robot_spec_file:=/path/to/robot_spec.yaml
    ros2 launch br_strategy_sim launch_simulator.py physics:=3d

physics:=3dでは、2Dの物理シムの代わりに3Dシム(br_sim_3d_node: 実機のcan_node+STM32+センサーの
代わり)を起動し、その上で本番と同じ足回り制御・推測航法・機構制御(br_robotパッケージ)を
TR/BRそれぞれに動かす。観測ノードは認識(/detected_*)の代行だけをする。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    # TR/BRの機体仕様(速度・加速度・寸法・機構の動作時間・通れる地形)。空なら
    # パッケージ同梱のconfig/robot_spec.yaml(仮値)を使う
    robot_spec_file_arg = DeclareLaunchArgument('robot_spec_file', default_value='')
    spec = {'robot_spec_file': LaunchConfiguration('robot_spec_file')}
    # 物理シムの種類: 2d(今までのpymunkのシム) | 3d(MuJoCo、ハードウェア境界の代わり)
    physics_arg = DeclareLaunchArgument('physics', default_value='2d')
    # 3Dシムのビューア(MuJoCoの3D表示)を開くか
    show_viewer_arg = DeclareLaunchArgument('show_viewer', default_value='true')
    is_3d = IfCondition(PythonExpression(["'", LaunchConfiguration('physics'), "' == '3d'"]))
    is_2d = UnlessCondition(PythonExpression(["'", LaunchConfiguration('physics'), "' == '3d'"]))
    robot_nodes = [
        Node(package='br_robot', executable=executable, name=f'{robot}_{name}',
             parameters=[{'robot': robot}, spec], condition=is_3d)
        for robot in ('tr', 'br')
        for executable, name in (('base_controller_node', 'base_controller'), ('odometry_node', 'odometry'),
                                 ('mechanism_controller_node', 'mechanism_controller'))
    ]

    return LaunchDescription([
        observation_noise_arg,
        team_arg,
        screenshot_path_arg,
        enable_decision_arg,
        enable_teleop_arg,
        match_duration_sec_arg,
        robot_spec_file_arg,
        physics_arg,
        show_viewer_arg,
        Node(package='br_strategy_sim', executable='br_sim_bridge_node', name='br_sim_bridge_node',
             parameters=[spec], condition=is_2d),
        Node(package='br_strategy_sim', executable='br_sim_3d_node', name='br_sim_3d_node',
             parameters=[{'show_viewer': LaunchConfiguration('show_viewer')}, spec], condition=is_3d),
        *robot_nodes,
        Node(
            package='br_strategy_sim',
            executable='br_referee_node',
            name='br_referee_node',
            parameters=[{
                'team': LaunchConfiguration('team'),
                'match_duration_sec': LaunchConfiguration('match_duration_sec'),
            }, spec],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_observation_node',
            name='br_observation_node',
            parameters=[{
                'observation_noise': LaunchConfiguration('observation_noise'),
                # 3Dでは推定位置は推測航法(br_robot)が出すので、観測ノードは認識の代行だけ
                'publish_pose': PythonExpression(["'", LaunchConfiguration('physics'), "' != '3d'"]),
            }],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_visualizer_node',
            name='br_visualizer_node',
            parameters=[{
                'screenshot_path': LaunchConfiguration('screenshot_path'),
                'team': LaunchConfiguration('team'),
            }],
        ),
        Node(
            package='br_strategy_sim',
            executable='br_decision_tr_node',
            name='br_decision_tr_node',
            parameters=[spec],
            condition=IfCondition(LaunchConfiguration('enable_decision')),
        ),
        Node(
            package='br_strategy_sim',
            executable='br_decision_br_node',
            name='br_decision_br_node',
            parameters=[spec],
            condition=IfCondition(LaunchConfiguration('enable_decision')),
        ),
        Node(
            package='br_strategy_sim',
            executable='br_teleop_node',
            name='br_teleop_node',
            parameters=[spec],
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
        ),
    ])
