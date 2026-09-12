#!/usr/bin/env bash
# br_msgs, br_strategy_simをビルドする。
#
# 開発機では ~/mujoco_rl のvenvがデフォルトで有効になっており、
# そのpython3にはempyが入っていないためcolcon buildの中のrosidl_adapterが
# ModuleNotFoundError: No module named 'em' で失敗する。venvを外したPATH
# (system python3)を使って回避する(詳細はCLAUDE.mdの「環境メモ」参照)。
# -u(未定義変数エラー)は/opt/ros/jazzy/setup.bashが未定義変数を参照しており
# 併用できないため付けない
set -eo pipefail
cd "$(dirname "$0")/../ros2_ws"

source /opt/ros/jazzy/setup.bash

CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "mujoco_rl" | tr '\n' ':')
unset VIRTUAL_ENV
export PATH="$CLEAN_PATH"

colcon build --packages-select br_msgs br_strategy_sim "$@"
