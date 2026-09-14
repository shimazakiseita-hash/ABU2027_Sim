# ABU Robocon 2027 BR/TR トピック契約仕様

以下の設計パターンを踏襲する：
- 真値トピックとノイズ有り観測トピックを分離する
- ロボットへの指令は単一トピックで受ける（実機/シムで意思決定ノード側は変更不要）
- 得点計算・違反判定（審判相当）は意思決定ノードと分離した独立ノードにする

この文書がインターフェースの正。実装（br_decision, br_strategy_sim, br_hw_bridge,
br_perception）はこの契約に従う。契約を変える場合はこのファイルを先に直す。

条文の引用は `docs/Robocon_2027_Rulebook_v1-1.pdf`（公式ルールブック英語版、
全28ページ）を一次情報源とする。

## 命名規則

- ロボット種別のプレフィックスを付ける: `/tr_*`, `/br_*`
- チームは実行時に1台構成（自チームのみシミュレートする想定。相手チームの動きは
  Phase 1では審判ノードが管理する静的/簡易モデルとする）

---

## 1. 真値トピック（シムのみが発行。意思決定ノードは購読禁止）

デバッグ・審判ノード専用。意思決定ロジックがこれを直接見ると「チート」になるので、
br_decision パッケージからは購読しない規約とする。

| トピック名 | 型 | 内容 |
|---|---|---|
| `/true_state/tr_pose` | `geometry_msgs/Pose2D` + level(int) | TRの正確な位置・階層 |
| `/true_state/br_pose` | `geometry_msgs/Pose2D` + level(int) | BRの正確な位置・階層 |
| `/true_state/blocks` | カスタム msg `BlockArray` | 全ブロックの位置・色・所有チーム・状態 |
| `/true_state/mustika_pose` | カスタム msg `MustikaPose` | ムスティカの正確な位置・held_by |
| `/true_state/tower_state` | カスタム msg `TowerArray` | 各建築スポットの構成状況（得点計算の元データ） |

## 2. 観測トピック（認識・自己位置推定ノードが発行。意思決定ノードはこちらを購読）

| トピック名 | 型 | 内容 | ノイズ |
|---|---|---|---|
| `/br_pose_estimated` | `geometry_msgs/PoseWithCovarianceStamped` | 自己位置推定結果 | Phase 1は誤差なしでも可（起動フラグで切替） |
| `/tr_pose_estimated` | 同上 | TRの自己位置推定 | 同上 |
| `/detected_blocks` | カスタム msg `DetectedBlockArray` | カメラ視野内で認識したブロック（位置・推定色） | 視野外は含まれない |
| `/detected_mustika` | カスタム msg `DetectedMustika` | 検出したムスティカ位置・held_by（視野内のみ） | 視野外は発行しない |
| `/detected_towers` | カスタム msg `TowerArray` | 完成塔の一覧（秘蹟の要件=Sanctuary Mandate判定用） | 視野判定なし（塔は静的な大型構造物のため。要調整） |

起動時オプション：

```
ros2 launch br_strategy_sim launch_simulator.py observation_noise:=true
```

## 3. 指令トピック（意思決定ノード → シム or 実機）

実機/シムのどちらに繋がっているかを意思決定ノードは意識しない。

| トピック名 | 型 | 内容 |
|---|---|---|
| `/br_cmd_vel` | `geometry_msgs/Twist` | BR本体の移動指令 |
| `/tr_cmd_vel` | `geometry_msgs/Twist` | TR本体の移動指令（手動操縦時は現行の`/joy`経路から変換） |
| `/br_gripper_cmd` | カスタム msg `GripperCmd` | BRの開閉・目標把持力 |
| `/tr_gripper_cmd` | カスタム msg `GripperCmd` | TRの開閉・目標把持力（br_decision実装時に追加。契約当初`/br_gripper_cmd`のみ定義していたが、TRも自前のグリッパーで把持・受渡しを行うため追加） |
| `/br_build_action` | カスタム msg `BuildAction` | 積み上げの高レベル指令（PLACE_EARTH_BLOCK, PLACE_SKY_BLOCK, PLACE_MUSTIKA, FLIP_SKY_BLOCK） |

`/br_cmd_vel`, `/tr_cmd_vel` の単位・座標系：
- `linear.x`/`linear.y` は m/s、`angular.z` は rad/s（ROS標準単位のまま）
- フィールド座標系(u, v)での速度指令としてそのまま扱う（ロボット自身の向きに対する相対速度への変換はしない）
- シム側(`br_strategy_sim`)は受信時にmm/sへ変換して物理演算(pymunk)に渡す。実機の足回り座標変換はbr_hw_bridge側の責務

## 4. 状態フィードバック（ハードウェア抽象層 → 意思決定ノード）

実機ではCAN経由でSTM32から、シムではpymunk/MuJoCoの内部状態から、それぞれ
このトピックに変換して発行する。意思決定ノード側のインターフェースは共通。

| トピック名 | 型 | 内容 |
|---|---|---|
| `/br_status` | カスタム msg `RobotStatus` | level(ground/L1/L2), holding_blocks, current_action |
| `/tr_status` | 同上 | 同上 |

## 5. 審判ノード（br_referee_node）

意思決定ノードとは独立したノード。得点計算・違反判定を一元管理する。

購読：`/true_state/*` 全て、および `/br_build_action`（受渡しと建築設置を
区別するため。建築行為であることを示す直近のヒントとして使う。詳細は
下記「受渡し違反と建築設置の区別」参照）
発行：

| トピック名 | 型 | 内容 |
|---|---|---|
| `/score/red` | `std_msgs/Int32` | 赤チーム得点（リアルタイム） |
| `/score/blue` | `std_msgs/Int32` | 青チーム得点 |
| `/violation` | カスタム msg `Violation` | 違反種別・対象ロボット・強制リトライ要否 |

判定ロジック（6章・7.2・8章に対応。条文はルールブックより）：
- 6.1 押出し禁止：ロボットはブロックを持ち上げるか運搬しなければならない。
  床面上を押す・引きずる・滑らせて移動させる行為は禁止 → `violation`(forced_retry=true)
- 6.2.1 区域違反(TR)：TRのいずれかの部分が受渡しエリアを越えてL1/L2の
  鉛直境界内へ入ること → `violation`(forced_retry=true)
- 6.2.2 区域違反(相手専用区域)：ロボットのいずれかの部分が相手の専用区域
  （スタートゾーン、ストレージエリア、非共用のL1建築スポット/受渡しエリア）へ
  入ること → `violation`(forced_retry=true)。実装上は相手側スタートゾーン・
  ストレージエリア・建築スポットの侵入のみ判定しており、L1の非共用エリア全体の
  正確な境界（共用エリアとの厳密な切り分け線）は未確定のため、それ以外の
  「非共用L1」への侵入は判定していない（要確認）
- 6.3 受渡し違反：以下のいずれか → `violation`(forced_retry=true)
  1. TR-BR間の受渡しが受渡しエリアの鉛直境界外で解放/落下した場合
  2. BRが、受渡しエリアの鉛直境界内に完全には収まっていない物体に触れた場合
     （＝BRが把持した瞬間のブロック位置がエリア外）
  4.4.3補足：受渡しはエリア境界内で完全に行う必要があり、エリア境界内にいる限り
  TR-BR間の物理接触は許容される
- 6.4 場外：ロボットがアース/スカイブロックをフィールド外(0〜GAME_FIELD_SIZE
  mm四方の外)へ弾き出した場合 → sim_bridge_node側でそのブロックを
  "permanently removed from play"としてpymunk空間・`self.blocks`から完全に
  除去する（`sim_bridge_node._enforce_out_of_bounds`）。ムスティカの場合は
  ムスティカ柱(`MUSTIKA_PILLAR_ORIGIN`)へ即時リセットする。br_referee_node
  はこの除去を`/true_state/blocks`から該当idが消えたことで検出し、
  `violation`(type="out_of_bounds", forced_retry=true)を記録する
  （held中=ロボットが自らの制御下で運んでいる間は対象外）
- 6.5 落下：未実装。heldブロックは保持中に他物体と一切衝突しない設計
  （physics_blocks.py参照。解放時の分離爆発を避けるための恒久措置）のため、
  Phase1の物理モデルには「ロボットが意図せず落とす」という事象自体が
  存在しない。実際の衝突動力学を持つPhase2(MuJoCo)で対応すべき項目
- 6.6 妨害（共用区域）：未実装。相手ロボットの物理的な存在自体をPhase1では
  シミュレートしていない（本文書冒頭の「チームは実行時に1台構成」参照）ため、
  そもそも「相手の作業を妨害する」という状況が発生しない。加えて5秒間の
  意図的な進路妨害の判定にはタイマー管理も別途必要（要調整）
- 7.2 設置済みアースブロックの移動（失格）：相手が正しく設置した**アース
  ブロック**を意図的に除去・移動・妨害する行為 → `violation`(type="disqualification",
  forced_retryは意味を持たないため常にfalse。強制リトライではなく即座に失格＝敗戦扱い)。
  スカイブロックは対象外（3.5.8で「ひっくり返す/位置を変える」ことが正規の
  "stealing"手段として認められているため、動かしても失格にはならない）
- 得点計算（8章、SCORE_*定数）：塔単位ではなくブロック単位で計算する
  - 受渡し点：TRが受渡しエリアへ届けたブロック1個につき5点（8.1）
  - アースブロック：設置した個人チームに得点が固定される（8.3.1）。上に他
    チームのブロックが積まれても変わらない
  - スカイブロック：得点は所有チームではなく**その時点の上面色**で決まる
    （8.3.2）。ひっくり返す(`FLIP_SKY_BLOCK`)たびに得点の帰属が変わる
  - 1つの建築スポットに赤アース+青アース+赤スカイのような**混成タワー**も
    正規（8.4の実例通り）
  - ムスティカ奉納点：250点（8.5）。br_referee_node._mustika_on_central_pillar()が
    中央支柱(FIELD_CENTER, レベルL2)への設置を検出して加算する
  - 秘蹟の要件（Sanctuary Mandate, 3.6/4.5.1）：共有エリアの塔を1つ以上含む
    完成塔2つを満たすまでTRはムスティカを回収できない。
    `decision_common.sanctuary_mandate_satisfied()`で判定し、TR側の意思決定
    ステートマシン（WAIT_SANCTUARY_MANDATE状態）が`/detected_towers`を見て
    判定・強制する。共有エリア＝L2（ルールブック上「全体が共用エリア」）を
    含む塔があるかで判定する簡易実装（L1内の一部共有エリアは
    field_constants.BUILD_SPOTSの割り当てにまだ反映していないため対象外。
    要調整）

### 受渡し違反と建築設置の区別

`held_by`が"none"以外→"none"に変化した瞬間だけでは、「TR-BR間の受渡し」なのか
「BRによる建築スポットへの正当な設置」なのか区別できない。区別のため、
br_referee_nodeは直近に受信した`/br_build_action`を「保留中の建築行為」として
覚えておき、ブロックが解放されたときに保留中の建築行為があれば「これは建築設置」
とみなして受渡し判定をスキップする（保留は1回消費したらクリアする簡易実装。
複数ブロックを連続して建築する場合や、`/br_build_action`と実際の解放の
タイミングがずれる場合の取り違えは未対応、要調整）。

## 6. カスタムメッセージ定義（概要）

Phase 1実装時に `br_msgs` パッケージとして .msg ファイルを作成する。

```
# BlockArray.msg
Block[] blocks

# Block.msg
string id
geometry_msgs/Point position
int32 level          # 0=ground, 1=L1, 2=L2
string block_type    # "earth" | "sky" | "mustika"
string top_color     # "red" | "blue" | "" (アースブロックは無関係)
string owner_team     # "red" | "blue" | "" (未確定)
string held_by        # "none" | "tr" | "br" (どちらのロボットが保持しているか。
                       # br_decision実装時にbool held_by_robotから変更。取り違え防止のため
                       # 「保持しているか否か」だけでなく「誰が」まで持たせる)
bool placed           # 建築スポットに正しく設置され得点対象になっているか
                       # (7.2の「設置済みブロック」判定に使う。br_referee_node実装時に追加)

# RobotStatus.msg
int32 level
string[] holding_block_ids
string current_action

# GripperCmd.msg
bool open
float32 target_force
string target_block_id  # open=falseで閉じる際に狙う対象(/detected_blocksのid)。
                         # 空文字なら把持レンジ内最近傍にフォールバックする

# BuildAction.msg
string action_type   # "PLACE_EARTH_BLOCK" | "PLACE_SKY_BLOCK" | "PLACE_MUSTIKA"
                      # | "FLIP_SKY_BLOCK" (3.5.8、他チームの塔から奪う"stealing"用に追加)
string target_build_spot_id

# Violation.msg
string type           # "zone" | "push" | "transfer" | "disqualification"(7.2、追加)
                      # | "out_of_bounds"(6.4、追加)
string robot          # "tr" | "br"
string team           # "red" | "blue"
bool forced_retry      # type=="disqualification"の場合は意味を持たず常にfalse
                        # (強制リトライではなく即座に失格＝敗戦扱いのため)

# RobotPose.msg (/true_state/tr_pose, /true_state/br_pose用。br_referee_node実装時に追加)
geometry_msgs/Pose2D pose
int32 level          # 0=ground, 1=L1, 2=L2

# MustikaPose.msg (/true_state/mustika_pose用。br_referee_node実装時に追加。
# held_byはムスティカ回収・受渡し・設置の意思決定ロジック実装時に追加)
geometry_msgs/Point position
int32 level
string held_by  # "none" | "tr" | "br"

# TowerState.msg (建築スポット1個分の積み上げ状態。br_referee_node実装時に追加、
# ルールブック8.3/8.4対応のため塔単位のteamからブロック単位のowner_teamsに変更)
string build_spot_id
int32 level             # 1=L1, 2=L2
string[] block_types    # 積み上げ順(下から上)。"earth" | "sky"
string[] owner_teams    # block_typesに対応。アースは設置した個人チームに固定
                         # (8.3.1)。スカイは常に""(得点はowner_teamではなく
                         # top_colorで決まるため)
string[] top_colors     # block_typesに対応。skyの現在の上面色(8.3.2、
                         # ひっくり返すたびに変わる)。earthは ""

# TowerArray.msg (/true_state/tower_state用。br_referee_node実装時に追加)
TowerState[] towers

# DetectedBlock.msg (/detected_blocks用。br_observation_node実装時に追加)
string id
geometry_msgs/Point position
int32 level
string block_type
string top_color
string owner_team
string held_by         # "none" | "tr" | "br" (Block.msgと同様。br_decision実装時に変更)

# DetectedBlockArray.msg (/detected_blocks用。br_observation_node実装時に追加)
DetectedBlock[] blocks

# DetectedMustika.msg (/detected_mustika用。当初はgeometry_msgs/PointStampedの
# ままだったが、ムスティカ回収の意思決定ロジック実装時にheld_byが必要になり
# 専用msgへ変更)
geometry_msgs/Point position
string held_by  # "none" | "tr" | "br"
```

## Phase 1 → Phase 2 → 実機での差し替え箇所

意思決定ノード（br_decision）はこの契約だけを見て動く。差し替わるのは
ハードウェア抽象層のみ：

| Phase | 差し替え対象 |
|---|---|
| Phase 1 (2D sim) | pymunk物理演算 → 本契約トピックへの変換ノード |
| Phase 2 (3D MuJoCo) | MuJoCo物理演算 → 本契約トピックへの変換ノード |
| 実機 | STM32/CAN → 本契約トピックへの変換ノード（br_hw_bridge） |
