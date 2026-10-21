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
/usr/bin/python3 -m pip install --user --break-system-packages pygame pymunk
```

`--break-system-packages`が無いと、比較的新しいDebian/Ubuntu(PEP 668)では
`externally-managed-environment`エラーで失敗する。`/usr/bin/python3`を
絶対パスで指定しているため、他のvenvが有効になっていてもシステムの
python3に正しくインストールされる。

3Dシム（`physics:=3d`）を使う場合は、MuJoCoも同じ方法で入れる:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages mujoco
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
./tools/sim_start.sh enable_decision:=false enable_teleop:=true \
                      match_duration_sec:=999999
                                                # TR/BRの自律ロジックを止め、
                                                # 代わりにキーボード手動操縦
                                                # コンソール(br_teleop_node)を
                                                # 起動する。可視化ウィンドウとは
                                                # 別ウィンドウが開く。
                                                # match_duration_sec指定は必須
                                                # ではないが、既定の3分を過ぎると
                                                # 9.1試合終了扱いになり以後の
                                                # 操作が一切反映されなくなるため、
                                                # 長時間テストするなら大きな値を
                                                # 指定しておくとよい
```

複数指定する場合は空白区切りでそのまま並べればよい
（例: `./tools/sim_start.sh team:=blue observation_noise:=true`）。

### 3Dシム（MuJoCo）

```bash
./tools/sim_start.sh physics:=3d                        # 3Dビューア+2D可視化が開く
./tools/sim_start.sh physics:=3d show_viewer:=false     # 3Dビューアを開かない
./tools/sim_start.sh physics:=3d enable_decision:=false enable_teleop:=true \
                      match_duration_sec:=999999        # 3Dで手動操縦
```

3Dシム（`br_sim_3d_node`）は実機の**can_node＋STM32＋モーター・センサーの代わり**で、
車輪エンコーダ・IMU・LiDARの値を返し、機体座標の速度指令と機構の指令を受ける
（トピックは`docs/topic_contract.md`の「ハードウェア境界」）。その上では、本番でも同じものを
動かす足回り制御・推測航法・機構制御（`br_robot`パッケージ）がTR/BRそれぞれに起動する。
そのため、推測航法の誤差で置く位置がずれると、塔が物理的に崩れる。

- 3Dビューア: マウスの左ドラッグで回転、右ドラッグで移動、ホイールで拡大縮小
- ロボットは機体仕様の寸法の箱（車輪・アームのモデルは機体設計が決まってから）
- 認識（`/detected_*`）は本番ノードがまだ無いので、観測ノードが正解値から代行する

### 機体仕様（速度・寸法・動作時間）

TR/BRの機体の数値は `ros2_ws/src/br_strategy_sim/config/robot_spec.yaml` に
まとめてある。シム（ロボットの動き）、意思決定ノード（速度・到着判定・待ち時間・
通れる道の計画）、操縦コンソール、審判がすべてこのファイルを読む。現在の値は
**仮値**で、機体要件が決まったらここを書き換えて再ビルドすれば反映される。
ビルドせずに別の値を試すときは、ファイルをコピーして起動引数で渡す:

```bash
./tools/sim_start.sh robot_spec_file:=/path/to/my_robot_spec.yaml
```

| 項目 | 意味 |
|---|---|
| `footprint_mm` | 車体の長さ x 幅 |
| `max_speed_m_s` / `max_accel_m_s2` | 平地での最高速度 / 加減速の上限 |
| `max_yaw_rate_rad_s` / `max_yaw_accel_rad_s2` | 旋回速度 / 旋回の加減速の上限 |
| `ramp_speed_scale` / `stairs_speed_scale` | スロープ / 階段上での速度倍率 |
| `can_use_ramp` / `can_use_stairs` | スロープ / 階段を通れるか（通れない地形は経路計画が避け、シムでも入れない） |
| `grasp_reach_mm` | 車体の縁からどこまで離れた物をつかめるか |
| `grasp_time_s` / `release_time_s` | つかむ / はなすのにかかる時間 |
| `place_time_s` / `flip_time_s` | 建築スポットに置く（ムスティカを柱に置く）/ スカイを裏返すのにかかる時間（BR） |
| `body_height_mm` | 車体の高さ（3Dシムの箱） |
| `wheel_*` / `encoder_counts_per_rev` / `wheel_slip_std` | 足回り（仮: 4輪メカナム）の寸法、エンコーダ分解能、滑りのノイズ |
| `imu_*` / `lidar_*` | IMUのノイズ、LiDARの本数・最大距離・取り付け高さ・周期 |

### 手動操縦（デバッグ用）

`enable_teleop:=true`で起動した操縦コンソールのウィンドウで操作する。
日本語のボタン画面になっており、下表のキー操作は全て画面上のボタンのクリック
でも行える（各ボタンの右上に対応キーを表示）。画面上部の案内バーに「次に
すること」と操作結果（つかめた/つかめなかった等）が表示され、今押しても
意味のないボタンは灰色になり理由が表示される：

| キー | 動作 |
|---|---|
| Tab | 操作するロボット（運搬ロボTR/建築ロボBR）を切替 |
| W/A/S/D | 移動（押している間だけ。方向は俯瞰画面の上下左右） |
| Shift | 押している間だけゆっくり移動（ボタンのクリックで固定ON/OFF） |
| Q / E | つかむ対象（近い順）を前/次に切替（候補のクリックで直接選択も可） |
| Space | つかむ / はなす |
| Z / X | 建築スポット（置く場所）を前/次に切替 |
| 1 | BRが持っているアース/スカイを選択中のスポットに置く |
| 2 | 選択中のスポットの一番上のスカイを裏返す |
| 3 | BRが持っているムスティカを中央の柱に置く |
| Esc | 終了 |

TR/BRいずれも既存のトピック契約（`/tr_cmd_vel`・`/br_build_action`等）へ
publishするだけなので、sim_bridge_node側の変更は不要（自律ノードの代わりに
同じトピックへpublishすれば操縦元を差し替えられる設計）。

日本語表示にはLinux側の日本語フォント（Noto CJK / IPA等）を使い、無ければ
WSL環境のWindowsフォント（BIZ UDゴシック・メイリオ等）を使う。どちらも
無い環境では文字化けするので`sudo apt install fonts-noto-cjk`で入れること。

**注意**: 起動から既定の3分（`match_duration_sec`、9.1試合時間）を過ぎると
`sim_bridge_node`が以後の全てのcmd_vel/gripper指令を無視するようになる
（試合終了扱い）。キーを押しても反応しなくなった場合、これが原因の可能性が
高い。長時間テストするときは起動時に`match_duration_sec:=999999`等を
指定しておくこと。

### 動作確認だけしたい場合

```bash
ros2 topic echo /score/red
```

別ターミナルで実行し、値が単調に増えていき、正常に完走した場合は最終的に
490点（塔2基分210点+受渡し点30点+ムスティカ奉納250点）に到達すれば正常に
動作している。ただし複数塔構成では、受渡しエリア境界付近の判定に起因する
既知の不具合により、完走せず途中で止まることがある（現状5回中3回程度の
頻度。詳細はCLAUDE.mdの「既知の残課題」参照）。

## テスト

`br_strategy_sim/test/`に受け入れテスト（pytest）がある。`ros2 launch`は
使わず、全ノードを1プロセス内でインスタンス化して`SingleThreadedExecutor`で
駆動する方式（この開発環境では複数プロセス起動が不安定だったため。
CLAUDE.md参照）。意思決定ロジックが実際にロボットを動かして検証するため、
早送りはできず実時間で数分かかる。

```bash
./tools/build.sh
colcon test --packages-select br_strategy_sim
colcon test-result --verbose
```

個別に実行・デバッグしたい場合は直接pytestを呼んでもよい（`tools/build.sh`と
同じくvenvを無効化してから）:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 -m pytest ros2_ws/src/br_strategy_sim/test/ -v
```

3Dシムのテスト（`test_multi_tower_e2e_3d.py`）はMuJoCoが入っていないとskipされる。

## パッケージ構成

- `br_msgs` — カスタムメッセージ定義一式
- `br_strategy_sim` — Phase 1本体
  - `sim_bridge_node` — pymunk物理シム本体（競技物・ロボットのspawn、
    `/true_state/*`の発行、把持・建築アクションの実行）
  - `br_referee_node` — 得点計算・違反判定
  - `br_observation_node` — 観測トピックの簡易モック（認識・自己位置推定）
  - `br_visualizer_node` — デバッグ用pygame可視化
  - `br_decision_tr_node` / `br_decision_br_node` — TR/BRの意思決定ロジック
  - `br_teleop_node` — キーボード手動操縦コンソール（デバッグ用、`enable_teleop:=true`）
  - `robot_spec.py` — 機体仕様（`config/robot_spec.yaml`）の読み込み
  - `field_geometry.py` — 柵と地形（段差・スロープ・階段）の簡易モデル
  - `navigation.py` — 意思決定ノード用の経路計画（格子A*）と経路追従
  - `sim_3d_node` / `scene_3d.py` — 3Dシム（MuJoCo、ハードウェア境界の代わり）とそのフィールドモデル
- `br_robot` — 実機でもシムでも同じものを動かすノード（足回り制御・推測航法・機構制御）
  - `field_constants.py` / `physics_blocks.py` / `robot_body.py` /
    `field_drawing.py` / `decision_common.py` — 共通の定数・物理・描画ロジック

## 現在の状況

複数の建築スポットへの完成塔（アース2段+スカイ1段）構築、秘蹟の要件
（Sanctuary Mandate）判定を経たムスティカの回収・中央支柱への設置、
6.4場外・9.1試合時間の違反判定までを含む一連の流れをheadless環境で
end-to-end検証済み（詳細はCLAUDE.mdの「現在地」参照）。ただし複数塔構成では
既知の完走率の不安定さがあり、未着手項目（6.5落下・6.6妨害・Phase 2）と
併せてCLAUDE.mdの「既知の残課題」に記載。
