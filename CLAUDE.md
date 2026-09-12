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
アース2個・共用エリアからスカイ1個を受渡しエリア経由でBRに届け、BRが1つの
建築スポットに完成塔（アース2段+スカイ1段）を組んで得点計算まで通す
end-to-endループを確認済み（headless実行、`/score/red`が70点に到達）。
得点計算・違反判定は公式ルールブック（後述）と突き合わせて整合済み。

未着手・既知の残課題：
- 複数の建築スポット・複数の塔への拡張（現状は固定の建築スポット1箇所のみ）
- ムスティカ（Mustika）関連の意思決定ロジック（秘蹟の要件=Sanctuary Mandateの
  判定を含む）
- 6.4場外・6.5落下・6.6妨害の違反判定
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

## パッケージ構成

- `br_msgs` — カスタムメッセージ定義一式（`docs/topic_contract.md`参照）
- `br_strategy_sim` — Phase 1本体
  - `sim_bridge_node` — pymunk物理シム本体。競技物・BR/TRのspawn、
    `/true_state/*`の発行、`/*_gripper_cmd`・`/br_build_action`の実行
  - `br_referee_node` — 得点計算・違反判定（ルールブック6章・7.2・8章）
  - `br_observation_node` — 観測トピック(認識・自己位置推定)の簡易モック
  - `br_visualizer_node` — デバッグ用pygame可視化（スクリーンショット保存対応）
  - `br_decision_tr_node` / `br_decision_br_node` — TR/BRの意思決定ステート
    マシン（現状はアース2段+スカイ1段の単一塔完成まで）
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
- ストレージエリアの正確な向き（U/V軸のどちらに長辺を沿わせるか）は未確認
- 建築スポット4隅のうちどちらが赤/青専有かは、中央分離線の対称性を根拠にした
  実装上の割り当て（ルールブックに明記なし）

赤/青の分割線が「フィールド中央のU軸対称（直線）」であることは、公式
ルールブック本文（"Center Divider: a fence running along the **center
line**"）とフィールド寸法図（ユーザー提供画像）の両方で確認済み（対角線
分割ではない）。

これらはPhase 1のロジック検証には影響しない精度だが、Phase 2以降で
精度を上げる際は見直すこと。

## 環境メモ

- 何らかのPython venvを有効にしたまま`colcon build`すると、そのvenvの
  python3に`empy`が入っておらず`rosidl_adapter`が
  `ModuleNotFoundError: No module named 'em'`で失敗することがある。
  `tools/build.sh`/`tools/sim_start.sh`は`$VIRTUAL_ENV`を見てPATHから
  自動的に除外するので、通常はこれらのスクリプト経由でビルド・起動すれば
  問題ない（詳細はREADME.md参照）。一度誤った状態でconfigureすると
  `build/`/`install/`にキャッシュが残るため、症状が出たら該当パッケージの
  `build/`/`install/`を削除してからやり直すこと。
