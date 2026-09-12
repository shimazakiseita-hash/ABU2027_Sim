#!/usr/bin/env bash
# br_msgs, br_strategy_simをビルドする。
#
# 何らかのPython venvが有効な状態でcolcon buildすると、そのvenvのpython3に
# empy(ROS2のrosidl_adapterが依存)が入っていないことがあり、
# ModuleNotFoundError: No module named 'em' で失敗することがある。
# venvを外したPATH(system python3)を使って回避する。
# -u(未定義変数エラー)は/opt/ros/jazzy/setup.bashが未定義変数を参照しており
# 併用できないため付けない
set -eo pipefail
cd "$(dirname "$0")/../ros2_ws"

source /opt/ros/jazzy/setup.bash

if [ -n "${VIRTUAL_ENV:-}" ]; then
  CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | tr '\n' ':')
  export PATH="$CLEAN_PATH"
  unset VIRTUAL_ENV
fi

colcon build --packages-select br_msgs br_strategy_sim "$@"
