#!/usr/bin/env bash
# Phase1 2Dシムの一括起動(RFC-Tsudanuma方式のsim_start.sh相当)。
# br_sim_bridge_node, br_referee_node, br_observation_node,
# br_visualizer_nodeをまとめて起動する。br_decision(意思決定ノード)は
# まだ実装していないため含まない。
#
# 使い方:
#   ./tools/sim_start.sh
#   ./tools/sim_start.sh observation_noise:=true
#   ./tools/sim_start.sh team:=blue
# -u(未定義変数エラー)は/opt/ros/jazzy/setup.bashが未定義変数を参照しており
# 併用できないため付けない
set -eo pipefail
cd "$(dirname "$0")/../ros2_ws"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# ~/mujoco_rl のvenvが有効だとpygame/pymunkがインストールされていない
# system python3を使えなくなる(CLAUDE.mdの「環境メモ」参照)。ノード実行時も
# 同じ理由でPATHから外す。
CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "mujoco_rl" | tr '\n' ':')
unset VIRTUAL_ENV
export PATH="$CLEAN_PATH"

ros2 launch br_strategy_sim launch_simulator.py "$@"
