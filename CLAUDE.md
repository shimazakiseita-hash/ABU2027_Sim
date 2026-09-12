# ABU Robocon 2027 - BR Simulation Project

## プロジェクト概要

ABU Robocon 2027「THE PURSUIT OF MUSTIKA NUSANTARA」（インドネシア・ソロ、
2027年8月開催）向け、建築ロボット(BR)完全自律システムの意思決定ロジックを、
実機完成前にシミュレーション上で開発・検証する。

CITRobocon制御班（島崎誠大がリーダー）が開発する。実機はPi5 + ROS2 Jazzy +
STM32F446RE + CAN 500kbpsの構成で、前年度toRobo2026のスタックを踏襲する。

## 開発方針

1. **Phase 1**: 2D sim (pymunk + pygame) で意思決定ロジックを固める
2. **Phase 2**: 3D MuJoCo で物理的妥当性（把持・積み上げ）を検証
3. 意思決定ノードはPhase 1/2/実機で完全共通のコードを使う
   （ハードウェア抽象層のみ差し替え。詳細は docs/topic_contract.md 参照）

**現在地：Phase 1着手前。** フィールド座標・トピック契約は確定済み。
`reference/draw_field_preview.py` で座標の妥当性は目視確認済み
（`reference/field_preview.png` 参照）。

## 参考アーキテクチャ

RFC-Tsudanuma（RoboCupヒューマノイドチーム）のstrategy_sim設計を参考にする。
具体的に踏襲するパターン：
- 真値(`/true_state/*`)とノイズ有り観測トピックの分離
- ロボットへの指令は単一トピックで受ける（実機/シム共通、意思決定ノードは
  どちらに繋がっているか意識しない）
- ゲームコントローラー相当（審判ノード `br_referee_node`）を意思決定ノードと分離
- 起動時オプションで観測ノイズON/OFFを切替 (`observation_noise:=true/false`)
- `tools/` 配下に `setup_*.sh`（環境構築）、`run_*.sh`（個別起動）、
  `build_pkg_select.sh`（選択ビルド）を揃える構成

## 技術スタック

- ROS2 Jazzy, Python3 (uv環境)
- pymunk (2D物理), pygame (2D可視化)
- MuJoCo (Phase 2、未着手)

## 参照ドキュメント

- `docs/Robocon_2027_Rulebook_v1-1.pdf` — 公式ルールブック英語版(全28ページ)。
  条文の一次情報源。数値・ルール解釈で迷ったら必ずここを確認する
- `docs/field_specs.md` — フィールド寸法・配置（ルールブック実測値、確定済み）
- `docs/topic_contract.md` — ROS2トピック・メッセージ契約（インターフェースの正）
- `reference/field_constants.py` — 上記specsをPython定数化したもの
- `reference/draw_field_preview.py` — 座標検証用の最小pygame描画スクリプト（動作確認済み）
- `reference/field_preview.png` — 上記スクリプトの実行結果（座標の見た目確認用）

## 既知の暫定事項（要調整の可能性あり）

`field_constants.py` 内にコメントで明記済み：
- ストレージエリアの正確な向き（U/V軸のどちらに長辺を沿わせるか）
- グラウンド共用エリア（スカイブロック格子）の中央寄せ位置
- 赤/青チームの分割線が「フィールド中央のU軸対称」という単純化の仮定で
  よいか（大会公式CAD図面が出れば要再確認）

これらはPhase 1のロジック検証には影響しない精度だが、Phase 2以降で
精度を上げる際は見直すこと。

## 既存資産（前年度toRobo2026から）

- CAN通信プロトコルは1byte離散コマンド方式（`CAN_COMMAND_SPEC.md`参照、
  別リポジトリ）。BR完全自律では連続値伝送への拡張が別途必要
- HMI/手動操縦は `/joy` トピックを境界にした抽象化パターンが確立済み
- AS5600磁気エンコーダ、VL53L0Xフットプリント基板（ToF転用実績あり）
- 北陽電機の2D LiDARを借用交渉済み（自己位置推定に活用予定）

## 環境メモ

- 開発機では `/home/seita/mujoco_rl` のvenvがデフォルトで有効になっており、
  そのpython3には`empy`が入っていないため、有効なままだと`colcon build`が
  `rosidl_adapter`の`ModuleNotFoundError: No module named 'em'`で失敗する。
  `colcon build`前にvenvを外したPATH（system python3、`/usr/bin/python3`）を
  使うこと。一度誤った状態で configure すると `build/`/`install/` にキャッシュが
  残るため、症状が出たら該当パッケージの`build/`/`install/`を削除してからやり直す。
  ノード実行時（`ros2 run`/`ros2 launch`）も、system python3にpygame/pymunkを
  インストールして使う前提のため同じPATH対策が要る。`tools/build.sh`/
  `tools/sim_start.sh`はこの対策込みなので、通常はこれらのスクリプト経由で
  ビルド・起動すればよい。
