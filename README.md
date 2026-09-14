# ABU Robocon 2027 BR Phase 1 2Dシミュレータ

ABU Robocon 2027「THE PURSUIT OF MUSTIKA NUSANTARA」向け、建築ロボット(BR)
完全自律システムの意思決定ロジックを検証するための2D物理シミュレータ
（ROS2 + pymunk + pygame）。

背景・設計方針・既知の暫定事項は [CLAUDE.md](CLAUDE.md) を参照。
トピック/メッセージの仕様は [docs/topic_contract.md](docs/topic_contract.md)
を参照（実装のインターフェースの正）。

## 必要環境

- ROS2 Jazzy（`/opt/ros/jazzy`にインストール済みであること）
- Python3（system python3。後述の理由でvenv環境では動かさないこと）
- pygame, pymunk（下記手順でインストール）

## セットアップ

### 1. Python依存パッケージのインストール

ROS2ノードは system python3（`/usr/bin/python3`）で実行する前提。他の
Python venv環境を有効にしたまま実行しないこと（後述の既知の環境問題参照）。

```bash
/usr/bin/python3 -m pip install --user pygame pymunk
```

### 2. ビルド

```bash
./tools/build.sh
```

`ros2_ws/`配下の`br_msgs`（カスタムメッセージ）・`br_strategy_sim`
（本体パッケージ）をcolcon buildする。

> **既知の環境問題**：何らかのPython venvを有効にしたまま`colcon build`すると、
> そのvenvのpython3に`empy`が入っておらず
> `ModuleNotFoundError: No module named 'em'`で失敗することがある。
> `tools/build.sh`は`$VIRTUAL_ENV`を見てPATHから自動的に除外するので、
> 通常はこのスクリプト経由でビルドすれば問題ない。詳細はCLAUDE.mdの
> 「環境メモ」参照。それでも失敗する場合は `ros2_ws/build` `ros2_ws/install`
> `ros2_ws/log` を削除してから再実行すること。

## 起動

```bash
./tools/sim_start.sh
```

物理シム(`br_sim_bridge_node`)・審判(`br_referee_node`)・観測
(`br_observation_node`)・可視化(`br_visualizer_node`)・TR/BR意思決定
(`br_decision_tr_node`, `br_decision_br_node`)の6ノードを一括起動する。
起動後、TRがストレージのアースブロック2個とグラウンド共用エリアのスカイ
ブロック1個を受渡しエリア経由でBRに届け、BRが建築スポットに完成塔
（アース2段+スカイ1段）を組み上げる、という一連の動作を複数の建築スポット
（現状は`br_decision_br_node.BUILD_SPOT_PLAN`で指定したL1・L2に1本ずつ）分
繰り返す。両塔完成後は、秘蹟の要件（Sanctuary Mandate）を満たしたTRが
ムスティカを回収して同じ受渡しエリア経由でBRに渡し、BRが中央支柱へ設置する
ところまで自動的に進む（所要時間は目安110〜150秒）。得点は`/score/red`・
`/score/blue`（`std_msgs/Int32`）に発行される。

### 起動オプション

```bash
./tools/sim_start.sh observation_noise:=true   # 自己位置推定にノイズを乗せる
./tools/sim_start.sh team:=blue                # 審判ノードの自チームをblueに
./tools/sim_start.sh enable_decision:=false    # TR/BR意思決定ノードを起動しない
                                                # (手動でcmd_vel等をpublishして
                                                # 個別に動作確認したい場合)
./tools/sim_start.sh screenshot_path:=/tmp/out.png
                                                # ウィンドウが出ない環境
                                                # (SDL_VIDEODRIVER=dummy)で
                                                # 可視化ノードが最新フレームを
                                                # 毎ティック上書き保存する
```

複数指定する場合は空白区切りでそのまま並べればよい
（例: `./tools/sim_start.sh team:=blue observation_noise:=true`）。

### 動作確認だけしたい場合

```bash
ros2 topic echo /score/red
```

別ターミナルで実行し、時間経過とともに 10 → 30 → 70 →（2本目の建築スポットの
アース1段目・2段目・スカイの順でさらに加算）→ 210 →（ムスティカ奉納）→ 460
と増えていけば正常に動作している（各塔ともアース1段目・2段目・スカイの順）。

## パッケージ構成

- `br_msgs` — カスタムメッセージ定義一式
- `br_strategy_sim` — Phase 1本体
  - `sim_bridge_node` — pymunk物理シム本体（競技物・ロボットのspawn、
    `/true_state/*`の発行、把持・建築アクションの実行）
  - `br_referee_node` — 得点計算・違反判定
  - `br_observation_node` — 観測トピックの簡易モック（認識・自己位置推定）
  - `br_visualizer_node` — デバッグ用pygame可視化
  - `br_decision_tr_node` / `br_decision_br_node` — TR/BRの意思決定ロジック
  - `field_constants.py` / `physics_blocks.py` / `robot_body.py` /
    `field_drawing.py` / `decision_common.py` — 共通の定数・物理・描画ロジック

## 現在の状況

固定の建築スポット1箇所に完成塔（アース2段+スカイ1段）を組み上げるまでの
一連の流れをheadless環境でend-to-end検証済み。複数建築スポット・ムスティカ
関連の意思決定ロジックは未着手（詳細はCLAUDE.mdの「現在地」参照）。
