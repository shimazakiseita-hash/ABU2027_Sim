"""
ABU Robocon 2027 Phase 1 2Dシム: TR側の意思決定ノード(br_decision_tr_node)。

最小限のステートマシン:
    IDLE -> APPROACH_STORAGE -> GRASP_EARTH_BLOCK
         -> APPROACH_TRANSFER_AREA -> RELEASE_IN_TRANSFER -> (ループ)
         -> WAIT_SANCTUARY_MANDATE -> APPROACH_MUSTIKA -> GRASP_MUSTIKA
         -> APPROACH_TRANSFER_AREA -> RELEASE_IN_TRANSFER -> DONE

/tr_pose_estimated, /detected_blocks, /detected_mustika, /detected_towers
(いずれも観測トピック)だけを見て動く。/true_state/*は購読しない
(topic_contract.mdの規約)。BR側(br_decision_br_node)とは完全に独立した
ノード/ステートマシンで、通信プロトコルは持たない。受渡しは「受渡しエリアに
置く/受渡しエリアで拾う」という物理的な状態(held_by, 位置)だけで成立させる
(HANDOFFの方針通りの最小実装)。

複数の建築スポットに完成塔(アース2段+スカイ1段)を作るのに必要な分だけ
ブロックを届け続ける。TRはBRの状態を直接知らない(通信プロトコルを持たない
設計)ため、自分の配送回数(_delivery_index)だけを頼りに「今何個目を
届けているか」を数え、DELIVERY_SEQUENCEの順序で配送する種別を切り替える。
BR側(br_decision_br_node.BUILD_SEQUENCE, BUILD_SPOT_PLAN)と同じ順序・
同じ塔数を独立に前提として動く。両者が受渡しエリアを介して1対1で同期する
(次を取りに行くのは前回分をBRが受け取った後)前提のため、この前提が崩れる
状況には対応していない。状態名はEARTH限定の最初のゴール時のまま
(GRASP_EARTH_BLOCK等)だが、実際に対象とする種別はDELIVERY_SEQUENCEに従う。

全ブロックを届け終えたら、秘蹟の要件(Sanctuary Mandate, 3.6/4.5.1)が
満たされるまで待ち(/detected_towersで完成塔を確認する。BUILD_SPOT_PLANに
共有エリア=L2の塔を含めてあるので、両塔完成時点で自然に満たされる設計)、
満たされたらムスティカを取りに行き、同じ受渡しエリア経由でBRに渡す。
"""

from __future__ import annotations

import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

from br_msgs.msg import DetectedBlockArray, DetectedMustika, GripperCmd, TowerArray

from . import field_constants as fc
from .decision_common import (
    TRANSFER_POINT_ARRIVAL_THRESHOLD_MM,
    block_arrival_threshold_mm,
    drive_toward,
    point_in_rect,
    sanctuary_mandate_satisfied,
    transfer_area_rect,
    tr_transfer_release_point,
)

_EARTH_BLOCK_ARRIVAL_THRESHOLD_MM = block_arrival_threshold_mm(fc.EARTH_BLOCK_SIZE / 2)
_SKY_BLOCK_ARRIVAL_THRESHOLD_MM = block_arrival_threshold_mm(fc.SKY_BLOCK_SIZE / 2)
_MUSTIKA_ARRIVAL_THRESHOLD_MM = block_arrival_threshold_mm(fc.MUSTIKA_DIAMETER / 2)
_ARRIVAL_THRESHOLD_BY_TYPE = {'earth': _EARTH_BLOCK_ARRIVAL_THRESHOLD_MM, 'sky': _SKY_BLOCK_ARRIVAL_THRESHOLD_MM}

# BUILD_SEQUENCE(br_decision_br_node)の段数と一致させること
# (秘蹟の要件の「完成塔」判定=/detected_towersのblock_typesが何個揃えば
# 完成とみなすか、に使う)。
TOWER_REQUIRED_LAYERS = 3

_STORAGE_AREA_CENTER = (
    fc.STORAGE_AREA_ORIGIN[0] + fc.STORAGE_AREA_SIZE[0] / 2,
    fc.STORAGE_AREA_ORIGIN[1] + fc.STORAGE_AREA_SIZE[1] / 2,
)
_SHARED_AREA_CENTER = (
    fc.GROUND_SHARED_AREA_ORIGIN[0] + fc.GROUND_SHARED_AREA_SIZE[0] / 2,
    fc.GROUND_SHARED_AREA_ORIGIN[1] + fc.GROUND_SHARED_AREA_SIZE[1] / 2,
)
_SOURCE_AREA_CENTER_BY_TYPE = {'earth': _STORAGE_AREA_CENTER, 'sky': _SHARED_AREA_CENTER}

# 塔の構成順(アース2段->スカイ1段)を、建てる塔の数だけ繰り返す。
# br_decision_br_node.BUILD_SEQUENCE * len(BUILD_SPOT_PLAN)と対応させること
# (要素数・種別の順序を一致させる)。
NUM_TOWERS = 2
DELIVERY_SEQUENCE = ('earth', 'earth', 'sky') * NUM_TOWERS

# このノードは(BUILD_SPOT_IDと同様に)赤チーム固定の暫定実装。スカイブロックは
# 得点(8.3.2)がowner_teamではなく現在の上面色で決まるため、自チームの色が
# 上を向いている個体を選んで届ける(そうしないと届けたスカイブロックがBRの
# 得点にならない)。ひっくり返す(FLIP_SKY_BLOCK)処理はまだ実装していない。
OWN_TEAM = fc.TeamColor.RED

CONTROL_HZ = 10.0
GRASP_TIMEOUT_TICKS = int(CONTROL_HZ * 5)  # 5秒粘って掴めなければ諦める
RELEASE_SETTLE_TICKS = int(CONTROL_HZ * 0.5)  # 解放コマンドが反映されるまでの待ち


class TrState(Enum):
    IDLE = auto()
    APPROACH_STORAGE = auto()
    GRASP_EARTH_BLOCK = auto()
    APPROACH_TRANSFER_AREA = auto()
    RELEASE_IN_TRANSFER = auto()
    WAIT_SANCTUARY_MANDATE = auto()
    APPROACH_MUSTIKA = auto()
    GRASP_MUSTIKA = auto()
    DONE = auto()


class BrDecisionTrNode(Node):
    def __init__(self):
        super().__init__('br_decision_tr_node')

        self._pose: PoseWithCovarianceStamped | None = None
        self._blocks = DetectedBlockArray()
        self._mustika: DetectedMustika | None = None
        self._towers = TowerArray()
        self._state = TrState.IDLE
        self._target_block_id: str | None = None
        self._timeout_counter = 0
        self._delivery_index = 0
        self._delivering_mustika = False

        self.pub_cmd_vel = self.create_publisher(Twist, '/tr_cmd_vel', 10)
        self.pub_gripper = self.create_publisher(GripperCmd, '/tr_gripper_cmd', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/tr_pose_estimated', self._on_pose, 10)
        self.create_subscription(DetectedBlockArray, '/detected_blocks', self._on_blocks, 10)
        self.create_subscription(DetectedMustika, '/detected_mustika', self._on_mustika, 10)
        self.create_subscription(TowerArray, '/detected_towers', self._on_towers, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose = msg

    def _on_blocks(self, msg: DetectedBlockArray) -> None:
        self._blocks = msg

    def _on_mustika(self, msg: DetectedMustika) -> None:
        self._mustika = msg

    def _on_towers(self, msg: TowerArray) -> None:
        self._towers = msg

    def _current_uv(self) -> tuple[float, float]:
        p = self._pose.pose.pose.position
        return (p.x, p.y)

    def _find_block(self, block_id: str):
        return next((b for b in self._blocks.blocks if b.id == block_id), None)

    def _current_delivery_type(self) -> str:
        return DELIVERY_SEQUENCE[self._delivery_index]

    def _nearest_free_block(self, block_type: str):
        cur = self._current_uv()
        candidates = [b for b in self._blocks.blocks if b.block_type == block_type and b.held_by == 'none']
        # level==0(ground)のブロックのみを対象にする。DetectedBlockには
        # (観測トピックの制約上)「設置済みか」を示すplacedフィールドが無いが、
        # 建築スポットへ設置されたブロックは必ずlevelが1(L1)/2(L2)へ切り替わる
        # (sim_bridge_node._execute_pending_build_action参照)ため、これを
        # 代わりに使える。これが無いと、l1_red_1建築スポットの座標がたまたま
        # ストレージ寄りのuフィルタの範囲内に収まる場合、TRが既に設置済みの
        # ブロックを「未使用のストレージ在庫」と誤認して再度持ち去り、受渡し
        # エリアへ運び直してしまう(統合テストで発覚。得点が本来の460から
        # 465になる/受渡し待ちが成立せず永久に停止する、の2パターンで再現)。
        candidates = [b for b in candidates if b.level == 0]
        # 受渡しエリア内(またはその付近)にあるブロックは対象から除外する。
        # 制限が無いと、自分がさっき届けたもののBRがまだ受け取っていない
        # ブロックが「一番近い候補」になり、それを再度拾って(実質同じものを
        # 運び直すだけで)しまう。TRはBRの受け取り状況を知らずに一定時間で
        # 次の配送に進む設計(受渡し後にBRの受領を待たない)なので、複数個
        # 届ける場合はこの取り違えが起こり得る。当初はearthのみ
        # storage_u_max(ストレージ寄りのuに限定)で対処していたが、skyには
        # 対応する制限が無く、塔を2つ以上作る場合に「2個目のスカイ配送で
        # 1個目のスカイを再度拾ってしまい、2個目の塔にスカイが永久に
        # 届かない」不具合として顕在化した(統合テストで発覚。1塔構成では
        # スカイの配送が1回きりだったため露見しなかった)。
        transfer_origin, transfer_size = transfer_area_rect()
        candidates = [
            b for b in candidates
            if not point_in_rect(b.position.x, b.position.y, transfer_origin, transfer_size)
        ]
        if block_type == 'earth':
            # ストレージ付近(u座標がこの範囲内)のブロックだけを対象にする。
            # 受渡しエリア除外だけでは、受渡しエリアに向かう/そこから離れる
            # 途中の座標(まだエリア外)にあるブロックまでは弾けないため、
            # earthについては引き続きこちらも併用する。
            storage_u_max = fc.TRANSFER_AREA_ORIGIN[0] - 200.0
            candidates = [b for b in candidates if b.position.x <= storage_u_max]
        elif block_type == 'sky':
            # 自チームの色が上を向いている個体だけを対象にする(得点はowner_team
            # ではなく上面色で決まるため。8.3.2/OWN_TEAM参照)
            candidates = [b for b in candidates if b.top_color == OWN_TEAM]
        if not candidates:
            return None
        return min(candidates, key=lambda b: math.hypot(b.position.x - cur[0], b.position.y - cur[1]))

    def _tick(self) -> None:
        if self._pose is None:
            return  # 自己位置推定がまだ来ていない

        if self._state == TrState.IDLE:
            self._state = TrState.APPROACH_STORAGE
            return

        if self._state == TrState.APPROACH_STORAGE:
            delivery_type = self._current_delivery_type()
            threshold = _ARRIVAL_THRESHOLD_BY_TYPE[delivery_type]
            target = self._find_block(self._target_block_id) if self._target_block_id else None
            if target is None:
                target = self._nearest_free_block(delivery_type)
                if target is None:
                    # 視野内(2000mm圏)に対象種別のブロックが無い場合は、
                    # 具体的な目標が見えるまで供給元エリア全体の方向へ
                    # 大まかに寄る(何もせず待つと視野に入らず永久に停止してしまう)
                    twist, _arrived = drive_toward(self._current_uv(), _SOURCE_AREA_CENTER_BY_TYPE[delivery_type])
                    self.pub_cmd_vel.publish(twist)
                    return
                self._target_block_id = target.id
            twist, arrived = drive_toward(
                self._current_uv(), (target.position.x, target.position.y),
                arrival_threshold_mm=threshold,
            )
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = TrState.GRASP_EARTH_BLOCK
                self._timeout_counter = 0

        elif self._state == TrState.GRASP_EARTH_BLOCK:
            threshold = _ARRIVAL_THRESHOLD_BY_TYPE[self._current_delivery_type()]
            target = self._find_block(self._target_block_id)
            self._timeout_counter += 1
            if target is not None:
                # 直前の接近で微妙に足りない/押されて離れた場合に備え、
                # 待つだけでなく引き続き目標へ寄せ続ける
                twist, _arrived = drive_toward(
                    self._current_uv(), (target.position.x, target.position.y),
                    arrival_threshold_mm=threshold,
                )
                self.pub_cmd_vel.publish(twist)
            else:
                self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(
                open=False, target_force=1.0, target_block_id=self._target_block_id or ''))
            if target is not None and target.held_by == 'tr':
                self._state = TrState.APPROACH_TRANSFER_AREA
            elif self._timeout_counter > GRASP_TIMEOUT_TICKS:
                self._target_block_id = None
                self._state = TrState.APPROACH_STORAGE

        elif self._state == TrState.APPROACH_TRANSFER_AREA:
            self.pub_gripper.publish(GripperCmd(
                open=False, target_force=1.0, target_block_id=self._target_block_id or ''))
            twist, arrived = drive_toward(
                self._current_uv(), tr_transfer_release_point(),
                arrival_threshold_mm=TRANSFER_POINT_ARRIVAL_THRESHOLD_MM,
            )
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = TrState.RELEASE_IN_TRANSFER
                self._timeout_counter = 0

        elif self._state == TrState.RELEASE_IN_TRANSFER:
            self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            self._timeout_counter += 1
            if self._timeout_counter > RELEASE_SETTLE_TICKS:
                self._target_block_id = None
                if self._delivering_mustika:
                    self._state = TrState.DONE
                else:
                    self._delivery_index += 1
                    if self._delivery_index >= len(DELIVERY_SEQUENCE):
                        self._state = TrState.WAIT_SANCTUARY_MANDATE
                    else:
                        self._state = TrState.APPROACH_STORAGE

        elif self._state == TrState.WAIT_SANCTUARY_MANDATE:
            # 3.6/4.5.1: 完成塔2つ(うち1つは共有エリア)が揃うまでムスティカは
            # 回収できない。/detected_towersで完成塔を確認できるまでここで待つ
            self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(open=True, target_force=0.0))
            if sanctuary_mandate_satisfied(self._towers.towers, TOWER_REQUIRED_LAYERS):
                self._state = TrState.APPROACH_MUSTIKA

        elif self._state == TrState.APPROACH_MUSTIKA:
            if self._mustika is None or self._mustika.held_by != 'none':
                # まだ視野(2000mm圏)に入っていない場合は、開始位置(中央支柱脇の
                # ムスティカ柱)付近へ大まかに寄る(何もせず待つと視野に入らず
                # 永久に停止してしまう。GRASP_EARTH_BLOCK系と同じ考え方)
                twist, _arrived = drive_toward(self._current_uv(), fc.MUSTIKA_PILLAR_ORIGIN)
                self.pub_cmd_vel.publish(twist)
                return
            twist, arrived = drive_toward(
                self._current_uv(), (self._mustika.position.x, self._mustika.position.y),
                arrival_threshold_mm=_MUSTIKA_ARRIVAL_THRESHOLD_MM,
            )
            self.pub_cmd_vel.publish(twist)
            if arrived:
                self._state = TrState.GRASP_MUSTIKA
                self._timeout_counter = 0

        elif self._state == TrState.GRASP_MUSTIKA:
            self._timeout_counter += 1
            if self._mustika is not None:
                twist, _arrived = drive_toward(
                    self._current_uv(), (self._mustika.position.x, self._mustika.position.y),
                    arrival_threshold_mm=_MUSTIKA_ARRIVAL_THRESHOLD_MM,
                )
                self.pub_cmd_vel.publish(twist)
            else:
                self.pub_cmd_vel.publish(Twist())
            self.pub_gripper.publish(GripperCmd(open=False, target_force=1.0, target_block_id='mustika'))
            if self._mustika is not None and self._mustika.held_by == 'tr':
                self._delivering_mustika = True
                self._target_block_id = 'mustika'
                self._state = TrState.APPROACH_TRANSFER_AREA
            elif self._timeout_counter > GRASP_TIMEOUT_TICKS:
                self._state = TrState.APPROACH_MUSTIKA

        elif self._state == TrState.DONE:
            # 全ブロック+ムスティカを届け終えた。以降は何もせず停止する。
            self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BrDecisionTrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
