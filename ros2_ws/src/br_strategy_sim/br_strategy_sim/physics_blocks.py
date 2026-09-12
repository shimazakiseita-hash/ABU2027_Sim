"""
ABU Robocon 2027 Phase 1 2Dシム: ブロック(アース/スカイ/ムスティカ)のpymunk表現。

このモジュールが持つ各ブロックオブジェクトのフィールドは br_msgs/msg/Block.msg
に1対1で対応させてある(id, position, level, block_type, top_color, owner_team,
held_by, placed)。br_referee_node が /true_state/blocks に変換する処理は
このオブジェクトのフィールドをそのまま詰め替えるだけで済む想定。

held_byは"none"|"tr"|"br"の文字列(TR/BRどちらが保持しているか)。単なるbool
ではなく保持者を持たせているのは、受渡しエリアでTR/BRが近づいたときに
どちらのgripper_pointで把持判定したかを区別し、他方のロボットが誤って
奪い取ってしまう取り違えを防ぐため(update_holdingのholder_id引数参照)。

座標系はfield_constants.pyのu,v(mm)をそのままpymunk座標として使う。
L1/L2は物理的に独立した面ではなく同一平面上に重ねて表現するため、
異なる階層のブロック同士が誤って衝突しないよう衝突フィルタで階層ごとに分離する
(ロボット側のshapeにも同じフィルタを適用する必要がある。step5で対応)。

このモジュールは space.gravity を設定しない(呼び出し側の責務)。俯瞰2Dの
「重力」はテーブル上の摩擦に相当するため、space.damping側で表現する想定。
"""

from __future__ import annotations

import pymunk

from . import field_constants as fc

LEVEL_GROUND = 0
LEVEL_L1 = 1
LEVEL_L2 = 2

_LEVEL_CATEGORY = {
    LEVEL_GROUND: 0b001,
    LEVEL_L1: 0b010,
    LEVEL_L2: 0b100,
}

# 把持判定の距離レンジ = ロボット半幅 + ブロックの半径相当 + 余裕。
# gripper_pointにロボットの中心位置(robot_body.py参照、アーム等のオフセットは
# 持たない簡略化)を使っているため、ロボットの700mm角footprintがブロックに
# 接触した時点でのロボット中心-ブロック中心間距離(=ロボット半幅+ブロック半径)より
# 把持レンジが小さいと、衝突で押し合うだけで永遠に把持判定に入れない
# (br_decisionのTR/BR統合テストで発覚した不具合。修正済み)。
# decision_common._BLOCK_APPROACH_MARGIN_MMと同じ値に保つこと(ロボットが
# "到着"とみなす距離と、実際に把持が成立する距離を一致させるため)。
# 統合テストで、密集したブロックの山に阻まれて数十mm足りず把持レンジに
# 入れないケースが頻発したため、余裕を大きめに取っている。
GRASP_MARGIN_MM = 150
_ROBOT_HALF_WIDTH_MM = fc.ROBOT_INITIAL_MAX[0] / 2

# 保持されていない(held_by=="none")ブロックに適用する速度減衰(1ステップごとの
# 乗数)。space.gravity=(0,0)かつspace.dampingも既定値(=減衰なし)のままだと、
# 衝突で弾かれたブロックが摩擦なしで永久に滑り続けてしまうため、卓上の摩擦相当
# としてブロック単体にだけ強めの減衰をかける(ロボットのcmd_vel速度制御には
# 影響させない。space.damping(全body共通)ではなくbody.velocity_funcで個別に
# 設定する理由もここにある)。
FREE_BLOCK_VELOCITY_DAMPING_PER_STEP = 0.85


# 解放直後、ブロックは直前の保持者ロボットの中心と完全に同じ位置にいる
# (follow_gripperが常にロボット中心へ位置を上書きするため)。その状態で
# 通常の衝突を有効化すると、同一座標での完全な重なりからpymunkが分離しようと
# して巨大な速度を生む(統合テストで発覚)。さらに、受渡しエリアはTR/BR両方が
# 出入りするため、解放した瞬間に居合わせたのが元の保持者と限らない(もう一方の
# ロボットがちょうどそこにいて再度爆発するケースも実際に発生した)。
# そのため「一度でも保持されたブロックは、以後TR/BRどちらとも物理衝突しない」
# 恒久的な措置とする(collision_typeベースのpre_solveハンドラで実現。
# sim_bridge_node.py参照)。ブロック同士の衝突には影響しない。
ROBOT_COLLISION_TYPE = 1
EVER_HELD_BLOCK_COLLISION_TYPE = 2


def level_filter(level: int) -> pymunk.ShapeFilter:
    """指定階層のshapeとだけ衝突するフィルタ(異なる階層のブロックとは衝突しない)。"""
    category = _LEVEL_CATEGORY[level]
    return pymunk.ShapeFilter(categories=category, mask=category)


def _mass_kg(weight_range_g: tuple[int, int]) -> float:
    return sum(weight_range_g) / 2 / 1000


def _block_half_size_mm(block_type: str) -> float:
    if block_type == "earth":
        return fc.EARTH_BLOCK_SIZE / 2
    if block_type == "sky":
        return fc.SKY_BLOCK_SIZE / 2
    if block_type == "mustika":
        return fc.MUSTIKA_DIAMETER / 2
    raise ValueError(f"unknown block_type: {block_type}")


def _grasp_range_mm(block_type: str) -> float:
    return _ROBOT_HALF_WIDTH_MM + GRASP_MARGIN_MM + _block_half_size_mm(block_type)


def _apply_free_block_damping(body: pymunk.Body) -> None:
    """held_by=="none"の間だけ効く速度減衰をbodyに設定する(KINEMATIC中は無効)。"""

    def velocity_func(b: pymunk.Body, gravity, _damping, dt) -> None:
        pymunk.Body.update_velocity(b, gravity, FREE_BLOCK_VELOCITY_DAMPING_PER_STEP, dt)

    body.velocity_func = velocity_func


class PhysicsBlock:
    """pymunk body/shapeとbr_msgs.Block相当のゲーム状態をまとめたラッパー。"""

    def __init__(
        self,
        id_: str,
        body: pymunk.Body,
        shape: pymunk.Shape,
        block_type: str,
        level: int,
        top_color: str = "",
        owner_team: str = "",
    ):
        self.id = id_
        self.body = body
        self.shape = shape
        self.block_type = block_type
        self.level = level
        self.top_color = top_color
        self.owner_team = owner_team
        self.held_by = "none"
        self.placed = False
        # pymunkはbody_type変更でKINEMATIC<->DYNAMICを行き来すると質量・慣性を
        # 自動で元に戻さない(KINEMATIC化でinfになったmass/momentが、DYNAMIC復帰
        # 時に0.0のままになる)。0質量はその後の物理計算でNaNを生む(統合テストで
        # 発覚した「解放直後に必ずNaN化する」不具合の真因)。生成時の値を保持して
        # おき、保持解除のたびに明示的に復元する(set_held_by参照)。
        self._mass = body.mass
        self._moment = body.moment

    @property
    def position(self) -> pymunk.Vec2d:
        return self.body.position

    def set_level(self, level: int) -> None:
        """建築等でブロックが別階層に移動したときに呼ぶ(衝突フィルタも張り替える)。"""
        self.level = level
        self.shape.filter = level_filter(level)

    def set_held_by(self, holder: str) -> None:
        """
        held_byを更新し、"none"以外ならKINEMATICにして物理エンジンの重力/摩擦を
        無効化する。あわせて衝突も切り替える:
        - 保持開始時: follow_gripperで毎tickロボット自身の中心へ位置を上書きする
          ため、通常の階層フィルタのままだと自分を運んでいるロボット本体のshapeと
          常時重なり続けてしまい、pymunkの分離補正が毎tick効いてロボットが
          弾き飛ばされる(統合テストで発覚)。保持中は無衝突にする。
        - 解放時: 解放した瞬間、ブロックは直前の保持者ロボットの中心と完全に
          同じ位置にいるため通常の衝突フィルタへ戻すと同一座標での重なりから
          同様に分離爆発が起きる。さらに受渡しエリアはTR/BR両方が出入りするため
          「元の保持者とだけ衝突しない」だけでは不十分(たまたま居合わせた
          もう一方のロボットとも爆発しうることが統合テストで発覚)。そのため
          collision_typeをEVER_HELD_BLOCK_COLLISION_TYPEに変更し、以後TR/BR
          どちらとも物理衝突しないようにする(sim_bridge_nodeに登録した
          pre_solveハンドラ参照)。ブロック同士の衝突には影響しない。
        """
        if holder == self.held_by:
            return
        previous_holder = self.held_by
        was_held = previous_holder != "none"
        now_held = holder != "none"
        self.held_by = holder
        if now_held != was_held:
            self.body.body_type = pymunk.Body.KINEMATIC if now_held else pymunk.Body.DYNAMIC
            if not now_held:
                # KINEMATIC化でinfになったmass/momentは自動で戻らずDYNAMIC復帰後
                # 0.0のままになる(pymunkの仕様)。0質量は以後の物理計算でNaNを
                # 生むため明示的に復元する
                self.body.mass = self._mass
                self.body.moment = self._moment
            # 直前が押出し等で異常な(あるいはNaNの)速度を持ったまま切り替わると
            # KINEMATIC中の積分やDYNAMIC復帰後に汚染が残ることがあるため、
            # 保持開始・解放のどちらのタイミングでも速度を明示的にリセットする
            self.body.velocity = (0, 0)
            self.body.angular_velocity = 0.0
            if now_held:
                self.shape.filter = pymunk.ShapeFilter(categories=0b000, mask=0b000)
            else:
                self.shape.filter = level_filter(self.level)
                self.shape.collision_type = EVER_HELD_BLOCK_COLLISION_TYPE

    def follow_gripper(self, gripper_point: tuple[float, float]) -> None:
        """保持中のみ、グリッパー位置にブロックを追従させる(tickごとに呼ぶ想定)。"""
        if self.held_by != "none":
            self.body.position = gripper_point

    def update_holding(self, gripper_point: tuple[float, float], gripper_open: bool, holder_id: str) -> None:
        """
        holder_id("tr"|"br")側のグリッパーcmd(open状態)と、そのgripper_pointからの
        距離でheld_byを判定する。他方のロボットが既に保持しているブロックは、
        既存の保持者が明示的に離す("none"にする)までholder_idが奪えないようにして、
        受渡しエリアで両者が近づいたときの取り違えを防ぐ。
        """
        if gripper_open:
            if self.held_by == holder_id:
                self.set_held_by("none")
            return
        if self.held_by not in ("none", holder_id):
            return  # 他方のロボットが保持中なので奪えない
        distance = (self.body.position - gripper_point).length
        if distance <= _grasp_range_mm(self.block_type):
            self.set_held_by(holder_id)


def make_earth_block(
    space: pymunk.Space,
    id_: str,
    position: tuple[float, float],
    level: int,
    owner_team: str,
) -> PhysicsBlock:
    size = fc.EARTH_BLOCK_SIZE
    mass = _mass_kg(fc.EARTH_BLOCK_WEIGHT_RANGE_G)
    body = pymunk.Body(mass, pymunk.moment_for_box(mass, (size, size)))
    body.position = position
    shape = pymunk.Poly.create_box(body, (size, size))
    shape.friction = 0.8
    shape.filter = level_filter(level)
    _apply_free_block_damping(body)
    space.add(body, shape)
    return PhysicsBlock(id_, body, shape, "earth", level, top_color="", owner_team=owner_team)


def make_sky_block(
    space: pymunk.Space,
    id_: str,
    position: tuple[float, float],
    level: int,
    top_color: str,
) -> PhysicsBlock:
    size = fc.SKY_BLOCK_SIZE
    mass = _mass_kg(fc.SKY_BLOCK_WEIGHT_RANGE_G)
    body = pymunk.Body(mass, pymunk.moment_for_box(mass, (size, size)))
    body.position = position
    shape = pymunk.Poly.create_box(body, (size, size))
    shape.friction = 0.8
    shape.filter = level_filter(level)
    _apply_free_block_damping(body)
    space.add(body, shape)
    return PhysicsBlock(id_, body, shape, "sky", level, top_color=top_color, owner_team="")


def make_mustika(
    space: pymunk.Space,
    id_: str,
    position: tuple[float, float],
    level: int,
) -> PhysicsBlock:
    diameter = fc.MUSTIKA_DIAMETER
    mass = _mass_kg(fc.MUSTIKA_WEIGHT_RANGE_G)
    body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, diameter / 2))
    body.position = position
    shape = pymunk.Circle(body, diameter / 2)
    shape.friction = 0.4  # 暫定値。中央柱ソケットへの最終投入判定を実装する際に見直すこと
    shape.filter = level_filter(level)
    _apply_free_block_damping(body)
    space.add(body, shape)
    return PhysicsBlock(id_, body, shape, "mustika", level, top_color="", owner_team="")
