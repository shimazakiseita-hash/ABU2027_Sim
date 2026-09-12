# HANDOFF TO CLAUDE CODE — Phase 1: 2D Simulator 実装

## ゴール

`br_strategy_sim` パッケージ（ROS2 Python）を新規作成し、以下が動作すること：

1. フィールドの静的要素（グラウンド/L1/L2、建築スポット、受渡しエリア、
   ムスティカ台、中央柱、スタートゾーン、ストレージエリア）をpygameで描画
2. BR/TRを簡易矩形として表現し、`docs/topic_contract.md` 記載のトピックで制御
3. ブロックの物理演算（pymunk）：持ち上げ/押し出し判定
4. 得点計算ノード（`br_referee_node`）：ルールブック8章の得点ロジックに基づく採点

## 参照ドキュメント（この順で読むこと）

1. `CLAUDE.md` — プロジェクト全体の方針
2. `docs/field_specs.md` — フィールド寸法・配置の確定値
3. `reference/field_constants.py` — 上記をPython定数化したもの。**そのまま使ってよい**
4. `docs/topic_contract.md` — ROS2トピック・メッセージ契約。**この契約に従うこと**
5. `reference/draw_field_preview.py` — 座標検証用スクリプト（動作確認済み）。
   フィールド描画の出発点として流用してよい
6. `reference/field_preview.png` — 上記の実行結果。この見た目が「正解」の基準

## 実装順序

1. **`br_msgs` パッケージ**：`docs/topic_contract.md` の「6. カスタムメッセージ定義」
   に従って .msg ファイルを作成
2. **`field_constants.py` の移設**：`reference/field_constants.py` を
   `br_strategy_sim/br_strategy_sim/field_constants.py` に配置
3. **`br_strategy_sim/scripts/main.py`**：`reference/draw_field_preview.py` を
   ベースに、pygame描画ループの骨格を作る（ROS2ノード化はまだ不要、まず描画のみ）
4. **物理ブロックオブジェクト**：pymunk body/shape定義
   - アースブロック（350mm立方体、350×350mmの2D矩形として近似）
   - スカイブロック（200mm立方体、赤/青の面色プロパティを持つ）
   - ムスティカ（200mm直径の円）
5. **ROS2ブリッジノード**：`/br_cmd_vel`, `/tr_cmd_vel` を購読し、
   pymunkのbody速度に反映
6. **`br_referee_node`**：
   - `/true_state/*` を購読
   - 区域侵犯・押出し禁止・受渡し違反の判定（ルールブック6章）
   - 得点計算（ルールブック8章）を `/score/red`, `/score/blue` に発行
7. **観測ノード（認識・自己位置推定の簡易モック）**：
   - Phase 1では「視野内のブロックをそのまま返す」簡易実装でよい
   - `observation_noise:=true` オプションでガウスノイズを乗せる分岐だけ用意

## 完了条件（Phase 1終了の目安）

- [ ] 手動で `/br_cmd_vel` にTwistをpublishしてBRが移動できる
- [ ] ブロックを「持ち上げ」相当の操作（`/br_gripper_cmd`）で運搬でき、
      押し出し動作は `br_referee_node` が違反判定する
- [ ] `/br_build_action` で建築スポットに塔を置くと得点が計算される
- [ ] ムスティカを中央柱に置くと250点が加算される
- [ ] `reference/field_preview.png` と同等の見た目でフィールドが描画される

## 起動スクリプト（RFC-Tsudanuma方式に倣う）

```bash
./run_br_sim.sh                           # 2Dシム起動
./run_br_sim.sh observation_noise:=true   # ノイズON
./run_br_decision.sh                      # 意思決定ノード起動（Phase 1後半で実装）
./run_br_visualizer.sh                    # デバッグ用可視化（得点表示等）
```

## 注意事項

- 意思決定ノード（`br_decision`）はこの契約に定義されたトピックだけを見て
  動くこと。pymunkの内部実装に直接依存するコードを書かない
- `/true_state/*` は審判ノード専用。意思決定ノードから購読しない
- 座標系は `field_constants.py` のコメントに記載の通り（原点はランプ側の角、
  U/V軸、赤/青はU軸対称の鏡映）。この前提を崩す変更をする場合は
  `docs/field_specs.md` と `CLAUDE.md` の「既知の暫定事項」を先に確認すること

## Phase 2（3D MuJoCo）への申し送り事項（今は着手不要）

- 意思決定ノード（`br_decision`）はPhase 1で固めたインターフェースのまま流用
- Phase 2ではpymunkの代わりにMuJoCo物理エンジンに差し替えるのみ
- `field_constants.py` の寸法をMJCF（MuJoCo XML）に変換する変換スクリプトが
  別途必要になる見込み
