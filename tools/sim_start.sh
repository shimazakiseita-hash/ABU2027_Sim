#!/usr/bin/env bash
# Phase1 2Dシムの一括起動スクリプト。
# br_sim_bridge_node, br_referee_node, br_observation_node,
# br_visualizer_node, br_decision_tr_node, br_decision_br_nodeを
# まとめて起動する。
#
# 使い方:
#   ./tools/sim_start.sh
#   ./tools/sim_start.sh observation_noise:=true
#   ./tools/sim_start.sh team:=blue
#   ./tools/sim_start.sh enable_decision:=false
#   ./tools/sim_start.sh screenshot_path:=/tmp/out.png
# -u(未定義変数エラー)は/opt/ros/jazzy/setup.bashが未定義変数を参照しており
# 併用できないため付けない
set -eo pipefail
cd "$(dirname "$0")/../ros2_ws"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 何らかのPython venvが有効だと、そこにpygame/pymunkがインストールされて
# いないためsystem python3を使えなくなることがある。ノード実行時もPATHから
# 外しておく(tools/build.shと同じ理由)。
if [ -n "${VIRTUAL_ENV:-}" ]; then
  CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | tr '\n' ':')
  export PATH="$CLEAN_PATH"
  unset VIRTUAL_ENV
fi

ros2 launch br_strategy_sim launch_simulator.py "$@"
