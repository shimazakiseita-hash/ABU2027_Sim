"""
ABU Robocon 2027 Phase 1 2Dシム: 審判ノード(br_referee_node)。

/true_state/* と /br_build_action を購読し、違反判定(ルールブック6章・7.2)と
得点計算(8章、field_constants.pyのSCORE_*定数を使用)を行う。意思決定ノード
(br_decision)はこのノードの出力(/score/*, /violation)だけを見て、
/true_state/*を直接購読してはいけない(topic_contract.mdの規約)。

判定ロジックの根拠と既知の未対応点は topic_contract.md「5. 審判ノード」参照。
得点計算(8章)はSCORE_*定数が確定値として与えられているため信頼度が高い。
違反判定(6章・7.2)は条文通り実装したが、以下は未対応(要調整):
- 6.2.2「非共用のL1」全体の正確な境界(共用エリアとの切り分け線)が未確定のため、
  相手側スタートゾーン・ストレージエリア・建築スポットの侵入のみ判定する
- 6.6 妨害(共用区域): 相手ロボットの物理的な存在自体をPhase1ではシミュレート
  していない(topic_contract.md「チームは実行時に1台構成」の前提)ため未実装。
  5秒間の意図的な進路妨害の判定にはタイマー管理も別途必要
- 6.5 落下ブロック: heldブロックは保持中に他物体と一切衝突しない設計
  (physics_blocks.py参照)のため、Phase1の物理モデルには「robotが意図せず
  落とす」という事象自体が存在しない。実際の衝突動力学を持つPhase2(MuJoCo)
  で対応すべき項目のため、Phase1では未実装のまま据え置く
- 6.3/7.2の判定は/br_build_actionの「直近受信」を建築行為のヒントとして使う
  簡易実装で、複数ブロックを連続建築する場合の取り違えは未対応

6.4場外は、sim_bridge_node側で実際にブロックを遊技から除外(または
ムスティカを開始位置へ復帰)する物理的な処理を行う。このノードはそれを
/true_state/blocksから該当ブロックのidが消えたことで検出し、Violationの
記録のみを行う(_check_disappeared_blocks参照)。

9.1試合時間(3分がデフォルト。match_duration_secパラメータ、既定値は
field_constants.MATCH_DURATION_SEC)：ノード起動time基準で経過時間を監視し、
経過したら/match_ended(std_msgs/Bool)を一度だけ発行して以後の得点再計算・
違反判定を停止する(9.1.3/9.4.1「ブザーが鳴った瞬間の状態で
最終得点を確定する」「ロボットは直ちに停止」に対応。ロボット停止自体は
sim_bridge_nodeが/match_endedを購読して行う)。9.4.2「保持中の物体はその
タスクの得点を除外」は、ムスティカについては_mustika_on_central_pillarに
held_by=="none"チェックを追加することで対応済み(アース/スカイブロックは
設置=released済みのものしかtower_stateに現れないため、保持中のものは
そもそも得点計算に含まれず追加対応不要)。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from br_msgs.msg import BlockArray, BuildAction, MustikaPose, RobotPose, TowerArray, Violation

from . import field_constants as fc
from .physics_blocks import LEVEL_GROUND, LEVEL_L1, LEVEL_L2

# 6.1押出し・7.2設置済み移動の判定: /true_state/blocks受信間隔での位置変化(mm)の閾値。
# 発行周期が固定でない前提のため「速度」ではなく「1メッセージあたりの変位」として扱う(要調整)。
PUSH_DISPLACEMENT_THRESHOLD_MM = 30.0
# 違反したロボットを推定する際、ブロックとロボット中心の距離がこれ以内なら「関与した」とみなす
NEAREST_ROBOT_MAX_DIST_MM = 800.0


def _point_in_rect(x: float, y: float, origin: tuple[float, float], size: tuple[float, float]) -> bool:
    return origin[0] <= x <= origin[0] + size[0] and origin[1] <= y <= origin[1] + size[1]


class BrRefereeNode(Node):
    def __init__(self):
        super().__init__('br_referee_node')

        # Phase1は自チーム1台構成のため、TR/BRの区域侵犯をどちらのteamとして
        # 報告するか、また相手チームがどちらかをパラメータで持たせる
        self.declare_parameter('team', 'red')
        self._own_team = self.get_parameter('team').value

        self._tr_pose: RobotPose | None = None
        self._br_pose: RobotPose | None = None
        self._mustika_pose: MustikaPose | None = None
        self._tower_state = TowerArray()
        self._prev_block_position: dict[str, Point] = {}
        self._prev_block_held: dict[str, str] = {}
        self._prev_block_placed: dict[str, bool] = {}
        self._prev_block_owner_team: dict[str, str] = {}
        self._known_block_ids: set[str] = set()
        # 直近に受信した/br_build_actionを「保留中の建築行為」として覚えておき、
        # 次のブロック解放が建築設置か受渡しかの区別に使う(1回消費でクリア)
        self._pending_build_action: BuildAction | None = None

        # 得点は「tower_state/mustikaから毎回全体を再計算する分」と
        # 「受渡し成功イベントで積み増す分」を別に持ち、発行時に合算する
        # (両方を1つのキャッシュに混ぜると、片方の更新でもう片方を消してしまうため)
        self._tower_score = {fc.TeamColor.RED: 0, fc.TeamColor.BLUE: 0}
        self._transfer_score = {fc.TeamColor.RED: 0, fc.TeamColor.BLUE: 0}

        # 9.1試合時間: ノード起動時刻を試合開始とみなし、match_duration_sec
        # 経過で試合終了(ブザー)とする。以後は得点再計算・違反判定を行わない
        # (9.1.3: 最終得点はブザーが鳴った瞬間の状態で確定するため)。
        # デフォルトはfield_constants.MATCH_DURATION_SEC(3分)だが、
        # br_teleop_nodeでの長時間の手動テスト時にcmd_vel等が無視される
        # ようになるのを避けたい場合のため、パラメータで上書きできるように
        # しておく(例: match_duration_sec:=999999)。
        self.declare_parameter('match_duration_sec', float(fc.MATCH_DURATION_SEC))
        self._match_duration_sec = self.get_parameter('match_duration_sec').value
        self._match_start_time = self.get_clock().now()
        self._match_ended = False

        self.pub_score_red = self.create_publisher(Int32, '/score/red', 10)
        self.pub_score_blue = self.create_publisher(Int32, '/score/blue', 10)
        self.pub_violation = self.create_publisher(Violation, '/violation', 10)
        self.pub_match_ended = self.create_publisher(Bool, '/match_ended', 10)

        self.create_subscription(RobotPose, '/true_state/tr_pose', self._on_tr_pose, 10)
        self.create_subscription(RobotPose, '/true_state/br_pose', self._on_br_pose, 10)
        self.create_subscription(BlockArray, '/true_state/blocks', self._on_blocks, 10)
        self.create_subscription(MustikaPose, '/true_state/mustika_pose', self._on_mustika_pose, 10)
        self.create_subscription(TowerArray, '/true_state/tower_state', self._on_tower_state, 10)
        self.create_subscription(BuildAction, '/br_build_action', self._on_build_action, 10)

        self.create_timer(0.1, self._check_match_timer)

    # --- 試合時間 (9.1) ---

    def _check_match_timer(self) -> None:
        if self._match_ended:
            return
        elapsed_sec = (self.get_clock().now() - self._match_start_time).nanoseconds / 1e9
        if elapsed_sec < self._match_duration_sec:
            return
        # 9.1.3: ブザーが鳴った瞬間の状態で最終得点を確定する。以後は
        # /true_state/*が変化しても得点・違反判定に反映しない(_on_blocks等の
        # ガード参照)。ロボットの物理的な停止はsim_bridge_node側の責務
        # (/match_endedを購読して行う。9.4.1)。
        self._match_ended = True
        self._recompute_tower_score()
        self.pub_match_ended.publish(Bool(data=True))

    # --- 区域侵犯 (6.2) ---

    def _on_tr_pose(self, msg: RobotPose) -> None:
        self._tr_pose = msg
        if self._match_ended:
            return
        if msg.level in (LEVEL_L1, LEVEL_L2):
            # 6.2.1: TRは受渡しエリアを越えてL1/L2の鉛直境界内へ入ってはならない
            self._publish_violation('zone', 'tr', self._own_team, forced_retry=True)
            return
        if self._in_opponent_zone(msg.pose.x, msg.pose.y, msg.level):
            self._publish_violation('zone', 'tr', self._own_team, forced_retry=True)

    def _on_br_pose(self, msg: RobotPose) -> None:
        # BRは建築のためL1/L2に登る前提なので、レベル侵入自体は違反にしない(6.2.1はTR限定)
        self._br_pose = msg
        if self._match_ended:
            return
        if self._in_opponent_zone(msg.pose.x, msg.pose.y, msg.level):
            self._publish_violation('zone', 'br', self._own_team, forced_retry=True)

    def _in_opponent_zone(self, x: float, y: float, level: int) -> bool:
        """6.2.2: 相手専用区域(スタートゾーン/ストレージエリア/建築スポット)への侵入。"""
        for origin, size in self._opponent_ground_zones():
            if _point_in_rect(x, y, origin, size):
                return True
        opponent = self._opponent_team()
        for _spot_id, spot_level, spot_team, spot_origin in fc.BUILD_SPOTS:
            if spot_level == level and spot_team == opponent:
                if _point_in_rect(x, y, spot_origin, fc.BUILD_SPOT_SIZE):
                    return True
        return False

    def _opponent_team(self) -> str:
        return fc.TeamColor.BLUE if self._own_team == fc.TeamColor.RED else fc.TeamColor.RED

    def _opponent_ground_zones(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        # field_constants.pyのSTART_ZONE_*/STORAGE_AREA_*は赤チーム側の座標として
        # 定義されているので、自チームが赤なら相手(青)側は鏡映、青ならそのまま使う
        red_side_zones = [
            (fc.START_ZONE_1_ORIGIN, fc.START_ZONE_SIZE),
            (fc.START_ZONE_2_ORIGIN, fc.START_ZONE_SIZE),
            (fc.STORAGE_AREA_ORIGIN, fc.STORAGE_AREA_SIZE),
        ]
        if self._own_team == fc.TeamColor.RED:
            return [(fc.mirror_rect_origin(o, s), s) for o, s in red_side_zones]
        return red_side_zones

    # --- ブロックの状態遷移: 6.1押出し / 7.2設置済み移動(失格) / 6.3受渡し ---

    def _on_blocks(self, msg: BlockArray) -> None:
        if self._match_ended:
            return
        current_ids = set()
        for block in msg.blocks:
            current_ids.add(block.id)
            self._check_block_movement(block)
            self._check_release(block)
            self._check_grasp(block)
            self._prev_block_position[block.id] = block.position
            self._prev_block_held[block.id] = block.held_by
            self._prev_block_placed[block.id] = block.placed
            self._prev_block_owner_team[block.id] = block.owner_team
        self._check_disappeared_blocks(current_ids)
        self._known_block_ids = current_ids

    def _check_disappeared_blocks(self, current_ids: set[str]) -> None:
        """
        6.4場外: sim_bridge_nodeはフィールド外に出た未保持ブロックを
        "permanently removed from play"として物理的に取り除く
        (sim_bridge_node._enforce_out_of_bounds参照)ため、このノードからは
        /true_state/blocksから該当idが突然消えたように見える。直前まで存在
        していたidが消えたら、その最後の既知位置に最も近かったロボットを
        違反対象として記録する(設置済み=placedのブロックが消えることは
        現状の実装では起きない想定だが、念のため対象外にする)。
        """
        for block_id in self._known_block_ids - current_ids:
            if self._prev_block_placed.get(block_id, False):
                continue
            prev = self._prev_block_position.get(block_id)
            if prev is None:
                continue
            robot = self._nearest_robot(prev.x, prev.y)
            owner_team = self._prev_block_owner_team.get(block_id, '')
            self._publish_violation('out_of_bounds', robot or 'br', owner_team, forced_retry=True)
            self._prev_block_position.pop(block_id, None)
            self._prev_block_held.pop(block_id, None)
            self._prev_block_placed.pop(block_id, None)
            self._prev_block_owner_team.pop(block_id, None)

    def _check_block_movement(self, block) -> None:
        """held中(正当な運搬)以外でブロックが動いた場合の判定(6.1押出し禁止 / 7.2失格)。"""
        if block.held_by != "none":
            return
        prev = self._prev_block_position.get(block.id)
        if prev is None:
            return
        displacement = math.hypot(block.position.x - prev.x, block.position.y - prev.y)
        if displacement <= PUSH_DISPLACEMENT_THRESHOLD_MM:
            return

        was_placed = self._prev_block_placed.get(block.id, False)
        robot = self._nearest_robot(block.position.x, block.position.y)
        if was_placed:
            # 7.2: 設置済みブロックが動いた。対象はアースブロックのみ(ルールブック
            # 7.2/3.5.5とも"Earth Block"限定。スカイブロックは3.5.8で意図的に
            # ひっくり返す/動かすことが正規の"stealing"手段として認められている
            # ため、スカイブロックの移動は失格対象にしない)。
            # 相手の設置物を動かした場合のみ即失格(Phase1で物理的に動かせるのは
            # 自チームのロボットだけなので、「自チームが相手の設置物を動かした」
            # 場合のみ判定できる)
            if block.block_type == 'earth' and block.owner_team and block.owner_team != self._own_team:
                self._publish_violation(
                    'disqualification', robot or 'br', self._own_team, forced_retry=False)
            return
        # 6.1: 未設置・未保持のブロックが動いた -> 押出し禁止違反
        if robot is not None:
            self._publish_violation('push', robot, block.owner_team, forced_retry=True)

    def _check_release(self, block) -> None:
        """
        6.3受渡し違反 / 8.1受渡し点。held=True->Falseの遷移のうち、建築設置
        ではないものが対象。

        以前は「block.level != LEVEL_L1ならreturn」という判定があったが、
        これは誤り: block.levelはブロックが何段目に積まれているか(0=ground,
        1=L1, 2=L2)を表すスタック高さであり、未設置のブロックは受渡しエリア
        内にあっても常にlevel=0(ground)のまま(sim_bridge_node参照。levelは
        _execute_pending_build_actionで実際に設置されたときにしか変わらない)。
        そのためこの判定は「未設置(=これから受渡し点を判定すべき対象)」を
        逆に毎回弾いてしまい、受渡し点(8.1)が一度も加算されない不具合の
        原因になっていた。「建築設置かどうか」は直前のpending_build_actionの
        有無で既に判別済みなので、level判定自体が不要だった。

        もう1点、pending_build_actionの消費対象もTR/BRを区別せず「直近に
        BuildActionがあれば、次に見つかった保持->解放イベント全てを建築設置
        とみなす」実装になっていたため、TRの受渡し解放とBRの建築設置が
        同一の/true_state/blocksメッセージ内で同時に起きた場合、どちらが
        先にイテレートされるかによってTR側の受渡しが誤って「建築設置」と
        判定され、受渡し点が消費されるだけで加算されない不具合があった
        (統合テストで発覚。BuildActionを実際に発行できるのはBRだけなので、
        「直前の保持者がbrだった場合」に限定することで解消する)。
        """
        was_held_by = self._prev_block_held.get(block.id, "none")
        was_held = was_held_by != "none"
        if not was_held or block.held_by != "none":
            return  # 「保持中->解放」の遷移でなければ対象外
        if self._pending_build_action is not None and was_held_by == "br":
            # 直近にBuildActionがあった、かつ直前の保持者がBRだった解放は
            # 建築設置とみなし、受渡し判定はしない
            self._pending_build_action = None
            return
        if block.block_type not in ('earth', 'sky'):
            # 8.1「アースブロックまたはスカイブロック1個につき5点」に明記の
            # 対象のみ。ムスティカは受渡し点の対象外(8.5で別途250点を計上)。
            # (現実装ではmustikaはBlockArrayに含まれずこの関数に渡ってくる
            # ことはないが、条文の対象を明示するため型でも防御しておく)
            return
        # 受渡し点の帰属チームはblock.owner_teamではなくself._own_teamを使う。
        # owner_teamは8.3.1のタワー得点計算用のフィールドで、スカイブロックは
        # 所有権の概念が無いため常に""(_recompute_tower_score参照)であり、
        # これをそのまま渡すと_add_scoreがteam未確定として黙って無視してしまい
        # スカイブロックの受渡し点だけが一度も加算されない不具合があった。
        # ここでTR-BR間の受渡しを行えるのはPhase1では自チーム(=self._own_team)
        # のTRだけなので、block_typeによらず常にself._own_teamに帰属させれば
        # よい(アースブロックの場合もowner_teamは元々自チーム固定なので結果は
        # 同じになる)。
        if _point_in_rect(block.position.x, block.position.y, fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE):
            self._add_score(self._own_team, fc.SCORE_TRANSFER_PER_BLOCK)
        else:
            self._publish_violation('transfer', 'br', self._own_team, forced_retry=True)

    def _check_grasp(self, block) -> None:
        """
        6.3受渡し違反のもう一方の条件: 「BRが受渡しエリアに完全に入っていない
        物体に触れた」場合。TRの把持(ストレージ/共用エリアでの通常の収集)は
        対象外(ルールブック6.3はBR限定)。
        """
        was_held_by_br = self._prev_block_held.get(block.id, "none") == "br"
        if was_held_by_br or block.held_by != "br":
            return  # 「(brでない)->br」への遷移でなければ対象外
        if not _point_in_rect(
                block.position.x, block.position.y, fc.TRANSFER_AREA_ORIGIN, fc.TRANSFER_AREA_SIZE):
            self._publish_violation('transfer', 'br', block.owner_team, forced_retry=True)

    def _on_build_action(self, msg: BuildAction) -> None:
        self._pending_build_action = msg

    def _nearest_robot(self, x: float, y: float) -> str | None:
        candidates = []
        if self._tr_pose is not None:
            candidates.append(('tr', self._tr_pose.pose.x, self._tr_pose.pose.y))
        if self._br_pose is not None:
            candidates.append(('br', self._br_pose.pose.x, self._br_pose.pose.y))
        best_robot = None
        best_dist = NEAREST_ROBOT_MAX_DIST_MM
        for robot, rx, ry in candidates:
            dist = math.hypot(x - rx, y - ry)
            if dist <= best_dist:
                best_robot = robot
                best_dist = dist
        return best_robot

    def _publish_violation(self, vtype: str, robot: str, team: str, forced_retry: bool) -> None:
        msg = Violation()
        msg.type = vtype
        msg.robot = robot
        msg.team = team
        msg.forced_retry = forced_retry
        self.pub_violation.publish(msg)

    # --- 得点計算 (8章) ---

    def _on_mustika_pose(self, msg: MustikaPose) -> None:
        if self._match_ended:
            return
        self._check_mustika_out_of_bounds_reset(msg)
        self._mustika_pose = msg
        self._recompute_tower_score()

    def _check_mustika_out_of_bounds_reset(self, msg: MustikaPose) -> None:
        """
        6.4場外(ムスティカ): sim_bridge_nodeはフィールド外に出た未保持の
        ムスティカを検出した次のtickでムスティカ柱(MUSTIKA_PILLAR_ORIGIN)へ
        リセットする(_publish_true_stateの後に_enforce_out_of_boundsを呼ぶ
        順序のため、場外に出た瞬間の位置が一度は/true_state/mustika_poseとして
        発行されてから是正される。sim_bridge_node._enforce_out_of_bounds参照)。
        このノードからは「大きく位置が動き、直後にムスティカ柱ぴったりの
        位置になっている」という遷移としてしか観測できない(ブロックの
        _check_disappeared_blocksと同じ考え方)。
        """
        prev = self._mustika_pose
        if prev is None or msg.held_by != 'none' or prev.held_by != 'none':
            return
        at_pillar_now = math.hypot(
            msg.position.x - fc.MUSTIKA_PILLAR_ORIGIN[0], msg.position.y - fc.MUSTIKA_PILLAR_ORIGIN[1]) < 1.0
        was_at_pillar = math.hypot(
            prev.position.x - fc.MUSTIKA_PILLAR_ORIGIN[0], prev.position.y - fc.MUSTIKA_PILLAR_ORIGIN[1]) < 1.0
        if not at_pillar_now or was_at_pillar:
            return
        displacement = math.hypot(msg.position.x - prev.position.x, msg.position.y - prev.position.y)
        if displacement <= PUSH_DISPLACEMENT_THRESHOLD_MM:
            return
        robot = self._nearest_robot(prev.position.x, prev.position.y)
        self._publish_violation('out_of_bounds', robot or 'br', self._own_team, forced_retry=True)

    def _on_tower_state(self, msg: TowerArray) -> None:
        if self._match_ended:
            return
        self._tower_state = msg
        self._recompute_tower_score()

    def _recompute_tower_score(self) -> None:
        """
        8.3/8.4: ブロックごとに得点を計算する(塔単位ではない)。
        - アースブロックは設置した個人チームに固定(owner_teams[i], 8.3.1)。
          上に他チームのブロックが積まれても変わらない。
        - スカイブロックは常にその時点の上面色(top_colors[i], 8.3.2)で決まり、
          ひっくり返す(FLIP_SKY_BLOCK)たびに帰属が変わる。設置したチームとは
          無関係(3.5.7: 相手が置いた土台の上でも、自分の色を上にすれば
          そのチームの塔になる)。
        1つの建築スポットに赤アース+青アース+赤スカイのような混成タワー
        (8.4)も自然に成立する。
        """
        red = 0
        blue = 0
        for tower in self._tower_state.towers:
            for i, block_type in enumerate(tower.block_types):
                layer = i + 1
                if block_type == 'earth':
                    team = tower.owner_teams[i]
                else:  # sky
                    team = tower.top_colors[i]
                points = self._block_score(block_type, tower.level, layer)
                if team == fc.TeamColor.RED:
                    red += points
                elif team == fc.TeamColor.BLUE:
                    blue += points
                # team未確定("")のブロックは得点計上しない

        if self._mustika_on_central_pillar():
            # ムスティカ奉納点の帰属チームを判別する情報が契約上無いため、
            # 暫定的に自チームに加算する(要確認)
            if self._own_team == fc.TeamColor.RED:
                red += fc.SCORE_MUSTIKA
            else:
                blue += fc.SCORE_MUSTIKA

        self._tower_score = {fc.TeamColor.RED: red, fc.TeamColor.BLUE: blue}
        self._publish_scores()

    def _mustika_on_central_pillar(self) -> bool:
        """
        8.5奉納の成立条件。9.4.2「保持中の物体はそのタスクの得点を除外」に
        対応するため、held_by!="none"(ロボットが保持中)なら常にFalseとする
        (現状のBR実装ではPLACE_MUSTIKA成立時にheld_by="none"かつlevel=L2へ
        同時に切り替わるため実際には起こらないが、条文の明文規定でもあり
        安全側の保証として明示的にチェックする)。
        """
        if self._mustika_pose is None or self._mustika_pose.held_by != 'none':
            return False
        if self._mustika_pose.level != LEVEL_L2:
            return False
        dx = self._mustika_pose.position.x - fc.FIELD_CENTER
        dy = self._mustika_pose.position.y - fc.FIELD_CENTER
        return math.hypot(dx, dy) <= fc.CENTRAL_PILLAR_SOCKET_DIAMETER / 2

    def _block_score(self, block_type: str, level: int, layer: int) -> int:
        if block_type == 'earth':
            if level == LEVEL_L1:
                return fc.SCORE_EARTH_BLOCK_L1_LAYER1 if layer == 1 else fc.SCORE_EARTH_BLOCK_L1_LAYER2
            if level == LEVEL_L2:
                return fc.SCORE_EARTH_BLOCK_L2_LAYER1 if layer == 1 else fc.SCORE_EARTH_BLOCK_L2_LAYER2
        elif block_type == 'sky':
            return fc.SCORE_SKY_BLOCK_L1 if level == LEVEL_L1 else fc.SCORE_SKY_BLOCK_L2
        return 0

    def _add_score(self, team: str, points: int) -> None:
        if team not in self._transfer_score:
            return  # owner_team未確定("")のブロックは受渡し点を計上しない
        self._transfer_score[team] += points
        self._publish_scores()

    def _publish_scores(self) -> None:
        red = self._tower_score[fc.TeamColor.RED] + self._transfer_score[fc.TeamColor.RED]
        blue = self._tower_score[fc.TeamColor.BLUE] + self._transfer_score[fc.TeamColor.BLUE]
        self.pub_score_red.publish(Int32(data=red))
        self.pub_score_blue.publish(Int32(data=blue))


def main(args=None):
    rclpy.init(args=args)
    node = BrRefereeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
