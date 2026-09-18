# ABU Robocon 2027 - BR Simulation Project

## プロジェクト概要

ABU Robocon 2027「THE PURSUIT OF MUSTIKA NUSANTARA」（インドネシア・ソロ、
2027年8月開催）向け、建築ロボット(BR)完全自律システムの意思決定ロジックを、
実機完成前にシミュレーション上で開発・検証する。

CITRobocon制御班が開発する。実機はPi5 + ROS2 Jazzy + STM32F446RE +
CAN 500kbpsの構成を予定している。

環境構築・ビルド・起動手順は [README.md](README.md) 参照。このファイルは
背景・設計方針・既知の暫定事項など、変更の「なぜ」を残すためのもの。

## 開発方針

1. **Phase 1**: 2D sim (pymunk + pygame) で意思決定ロジックを固める
2. **Phase 2**: 3D MuJoCo で物理的妥当性（把持・積み上げ）を検証
3. 意思決定ノードはPhase 1/2/実機で完全共通のコードを使う
   （ハードウェア抽象層のみ差し替え。詳細は docs/topic_contract.md 参照）

**現在地：Phase 1実装中。** `br_msgs`（カスタムメッセージ）と`br_strategy_sim`
（物理シム・審判・観測・TR/BR意思決定ノード一式）を実装済み。TRがストレージから
アース2個・共用エリアからスカイ1個を受渡しエリア経由でBRに届け、BRが複数の
建築スポット（`br_decision_br_node.BUILD_SPOT_PLAN`、現状はL1・L2に1本ずつ）に
順番に完成塔（アース2段+スカイ1段）を組む。両塔完成後は秘蹟の要件（Sanctuary
Mandate, 3.6/4.5.1）をTRが`/detected_towers`で確認した上でムスティカを回収し、
同じ受渡しエリア経由でBRに渡し、BRが中央支柱へ設置(8.5)する。ここまでの
end-to-endループを確認済み（headless実行、正常完走時`/score/red`は490点
=塔210点+受渡し点30点+ムスティカ250点に到達）。得点計算・違反判定は公式
ルールブック（後述）と突き合わせて整合済み。ただし完走率には既知の不具合が
あり、下記「既知の残課題」参照。

6.4場外（ロボットがブロックをフィールド外へ弾き出す/ムスティカが場外に
出る）も実装済み：sim_bridge_nodeがフィールド外に出た未保持ブロックを
遊技から除外（ムスティカは開始位置へ復帰）し、br_referee_nodeが
`/true_state/blocks`からのid消失・ムスティカの位置遷移から検出して
`violation`(type="out_of_bounds")を記録する。

9.1試合時間（3分）も実装済み：br_referee_nodeがノード起動から
`MATCH_DURATION_SEC`経過を監視し、経過した瞬間の状態で最終得点を確定して
`/match_ended`(std_msgs/Bool)を一度だけ発行する。sim_bridge_nodeはこれを
購読し、以後のcmd_vel/gripper/build_actionを一切反映せずTR/BRを強制停止する
（9.4.1）。保持中の物体の得点除外（9.4.2）はムスティカの奉納判定に
`held_by=="none"`チェックを追加することで対応（アース/スカイブロックは
設置済みのものしか得点計算に現れない設計のため元々対応不要だった）。

未着手・既知の残課題：
- **複数塔構成での完走率が不安定**（統合テストで5回中3回、受渡しエリア境界
  付近でBRが永久に停止する形で再現）。根本原因は、BRが塔1基目の建築で
  忙しい間、TRが先に届けた未回収ブロックが受渡しエリア(1000x1000mm)内に
  放置され、その間にTR/BRがエリアを出入りする物理的接触でブロックが
  エリア境界からわずかに(数百mm程度)押し出されることがある点。
  `br_decision_br_node._find_waiting_block`の`point_in_rect`判定は境界に
  対して許容誤差ゼロの厳密判定のため、境界のすぐ外に出た瞬間そのブロックは
  「受渡し待ち」として二度と検知されなくなり、BRが該当レイヤーを永久に
  待ち続ける。対策候補：point_in_rectに小さなマージンを持たせる、受渡し
  エリアでのTR/BR同時アクセスを減らす、BRが範囲外でも近傍のブロックを
  拾えるようにする、等（未着手）
- **上記と別に、完走はしてもスコアが490点にならないケースを1回観測**
  （450点、40点不足）。原因は未特定。再現条件も未確認
- 6.5落下：Phase1の物理モデルには「ロボットが保持中のブロックを意図せず
  落とす」という事象自体が存在しない（heldブロックは保持中に他物体と
  一切衝突しない設計のため）。実際の衝突動力学を持つPhase2(MuJoCo)向けの課題
- 6.6妨害（共用区域）：相手ロボットの物理的な存在自体をPhase1ではシミュレート
  していない（自チーム1台構成の前提）ため、対応するには先に相手ロボットの
  モデル化が必要
- Phase 2 (MuJoCo) は未着手

## 参考アーキテクチャ

既存の設計パターンを参考にした。具体的に踏襲したパターン：
- 真値(`/true_state/*`)とノイズ有り観測トピックの分離
- ロボットへの指令は単一トピックで受ける（実機/シム共通、意思決定ノードは
  どちらに繋がっているか意識しない）
- ゲームコントローラー相当（審判ノード `br_referee_node`）を意思決定ノードと分離
- 起動時オプションで観測ノイズON/OFFを切替 (`observation_noise:=true/false`)
- `tools/` 配下に起動・ビルドスクリプトを揃える構成

## 技術スタック

- ROS2 Jazzy, Python3
- pymunk (2D物理), pygame (2D可視化)
- MuJoCo (Phase 2、未着手)

## テスト

`br_strategy_sim/test/`にpytestベースの受け入れテスト一式がある
（`_sim_harness.py`が共通ヘルパー）。`ros2 launch`は使わず、全ノードを
1プロセス内で直接インスタンス化し`SingleThreadedExecutor`で駆動する方式
（下記「環境メモ」参照）。`colcon test --packages-select br_strategy_sim`で
自動的に拾われる。実行手順はREADME.md「テスト」参照。

## パッケージ構成

- `br_msgs` — カスタムメッセージ定義一式（`docs/topic_contract.md`参照）
- `br_strategy_sim` — Phase 1本体
  - `sim_bridge_node` — pymunk物理シム本体。競技物・BR/TRのspawn、
    `/true_state/*`の発行、`/*_gripper_cmd`・`/br_build_action`の実行
  - `br_referee_node` — 得点計算・違反判定（ルールブック6章・7.2・8章）
  - `br_observation_node` — 観測トピック(認識・自己位置推定)の簡易モック
  - `br_visualizer_node` — デバッグ用pygame可視化（スクリーンショット保存対応）
  - `br_decision_tr_node` / `br_decision_br_node` — TR/BRの意思決定ステート
    マシン（複数の建築スポットへのアース2段+スカイ1段の完成塔を順に構築した後、
    秘蹟の要件を満たしてムスティカを回収・中央支柱へ設置するところまで）
  - `physics_blocks.py` / `robot_body.py` / `field_constants.py` /
    `field_drawing.py` / `decision_common.py` — 共通ロジック・定数・描画

## 参照ドキュメント

- `docs/Robocon_2027_Rulebook_v1-1.pdf` — 公式ルールブック英語版(全28ページ)。
  条文の一次情報源。数値・ルール解釈で迷ったら必ずここを確認する
- `docs/field_specs.md` — フィールド寸法・競技物仕様（ルールブックからの転記、
  公式PDF入手後に重量等の誤りを修正済み）
- `docs/topic_contract.md` — ROS2トピック・メッセージ契約（インターフェースの正）

## 既知の暫定事項（要調整の可能性あり）

`field_constants.py` 内にコメントで明記済み：
- 建築スポット4隅のうちどちらが赤/青専有かは、中央分離線の対称性を根拠にした
  実装上の割り当て（ルールブックに明記なし）

ストレージエリアの向き（`STORAGE_AREA_SIZE`/`STORAGE_AREA_ORIGIN`）は
公式図面の実測値（長辺2000mmがフィールド外周の辺に平行、短辺1000mmが内側への
奥行き）に基づき確定済み。

赤/青の分割線が「フィールド中央のU軸対称（直線）」であることは、公式
ルールブック本文（"Center Divider: a fence running along the **center
line**"）とフィールド寸法図（ユーザー提供画像）の両方で確認済み（対角線
分割ではない）。

これらはPhase 1のロジック検証には影響しない精度だが、Phase 2以降で
精度を上げる際は見直すこと。

## 環境メモ

- 開発環境によっては`ros2 launch`での複数プロセス起動が不安定（子プロセスが
  不定期にSIGKILLされる等）なことがある。手動確認・自動テストとも、全ノードを
  1プロセス内で直接インスタンス化し`SingleThreadedExecutor`で駆動する方式
  （`br_strategy_sim/test/_sim_harness.py`参照）の方が確実。実際に
  `./tools/sim_start.sh`（`ros2 launch`経由）で起動して使う分には問題ない。
- 何らかのPython venvを有効にしたまま`colcon build`すると、そのvenvの
  python3に`empy`が入っておらず`rosidl_adapter`が
  `ModuleNotFoundError: No module named 'em'`で失敗することがある。
  `tools/build.sh`/`tools/sim_start.sh`は`$VIRTUAL_ENV`を見てPATHから
  自動的に除外するので、通常はこれらのスクリプト経由でビルド・起動すれば
  問題ない（詳細はREADME.md参照）。一度誤った状態でconfigureすると
  `build/`/`install/`にキャッシュが残るため、症状が出たら該当パッケージの
  `build/`/`install/`を削除してからやり直すこと。
