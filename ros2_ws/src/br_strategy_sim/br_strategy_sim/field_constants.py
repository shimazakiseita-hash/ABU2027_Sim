"""
ABU Robocon 2027 フィールド定数定義
出典: field_specs.md (docs/Robocon_2027_Rulebook_v1-1.pdf 公式ルールブック v1.0 +
公式フィールド図の実測値)

単位は全て mm。座標系:
  原点(0,0)  : 赤チーム側、ランプ/スタートゾーンに近い角
  U軸        : ストレージエリア沿いの辺 (0 -> 11000)
  V軸        : スタートゾーン沿いの辺 (0 -> 11000)
  Z軸        : グラウンド面を0とした高さ方向

赤チームの座標をここで定義し、青チームは中央分離線に対する鏡映変換
(mirror_to_blue関数)で求める。分割線の厳密な向きは未確定だが、
「U軸方向の中央(5500mm)で対称」という単純化した仮定を置いている。
2Dシムの初期実装ではこの精度で十分。要調整の場合はここだけ直せばよい。
"""

from dataclasses import dataclass


# ============================================================
# 全体寸法
# ============================================================
GAME_FIELD_SIZE = 11000  # mm, 正方形フィールドの一辺
MATCH_DURATION_SEC = 180  # 3分

FIELD_CENTER = GAME_FIELD_SIZE / 2  # 5500mm、対称変換の基準線


# ============================================================
# 高さ方向（階層）
# ============================================================
GROUND_Z = 0
L1_Z = 600
L2_Z = L1_Z + 300  # 900

L1_SIZE = 6000  # mm, 一辺
L2_SIZE = 3000  # mm, 一辺

# L1/L2はフィールド中央に入れ子で配置する(draw_field_preview.py由来の簡略化)。
# L1/L2を基準にした座標(建築スポット・受渡しエリア・ムスティカ台等)は
# すべてここを起点に計算し、同じ式を他所で重複して書かないこと。
L1_ORIGIN = (FIELD_CENTER - L1_SIZE / 2, FIELD_CENTER - L1_SIZE / 2)
L2_ORIGIN = (FIELD_CENTER - L2_SIZE / 2, FIELD_CENTER - L2_SIZE / 2)


# ============================================================
# ロボット寸法制約
# ============================================================
ROBOT_INITIAL_MAX = (700, 700, 700)       # 長さ x 幅 x 高さ
ROBOT_DEPLOYED_MAX = (1000, 1400, 1200)   # 幅 x 長さ x 高さ
ROBOT_WEIGHT_MAX_KG = 50

TR_MAX_BLOCKS_CARRIED = 3
BR_MAX_BLOCKS_CARRIED = 2


# ============================================================
# グラウンドエリア（赤チーム側の座標。青は mirror_to_blue で変換）
# ============================================================

# スタートゾーン: 原点の角に700x700mmが2個並ぶ (TR用/BR用はチーム選択)
START_ZONE_SIZE = (700, 700)
START_ZONE_1_ORIGIN = (0, 0)
START_ZONE_2_ORIGIN = (700, 0)

# ストレージエリア: 長辺(2000mm)がフィールド外周の辺に平行、短辺(1000mm)が
# 内側への奥行き(公式図面の実測値に基づき確定。U軸=ストレージエリア沿いの辺
# という座標系定義と整合させた)
STORAGE_AREA_SIZE = (2000, 1000)
STORAGE_AREA_ORIGIN = (1000, 0)

STORAGE_AREA_EARTH_BLOCK_COUNT = 20
STORAGE_AREA_MAX_STACK = 2

# グラウンド共用エリア（中央、スカイブロック5x5格子）
GROUND_SHARED_AREA_SIZE = (1200, 1200)
GROUND_SHARED_AREA_ORIGIN = (FIELD_CENTER - 600, FIELD_CENTER - 600)  # 中央寄せ(暫定)

SKY_BLOCK_GRID_ROWS = 5
SKY_BLOCK_GRID_COLS = 5
SKY_BLOCK_COUNT = 12  # 中央1マスを除く24マス中12個使用(市松状)

# ムスティカ台周囲の共用エリア
MUSTIKA_SHARED_AREA_SIZE = (1000, 1000)

# ムスティカ台（円柱）
MUSTIKA_PILLAR_HEIGHT = 500
MUSTIKA_PILLAR_DIAMETER = 270
MUSTIKA_PILLAR_SOCKET_DIAMETER = 180
MUSTIKA_PILLAR_SOCKET_DEPTH = 100
# L1手前の暫定配置(draw_field_preview.py由来。正確な位置は未確定、要調整)
MUSTIKA_PILLAR_ORIGIN = (L1_ORIGIN[0] - 500, L1_ORIGIN[1] - 500)


# ============================================================
# グラウンド -> L1 遷移構造
# ============================================================
RAMP_LENGTH = 3500
RAMP_TO_STAIRS_LANDING_WIDTH = 600
STAIRS_SECTION_DEPTH = 1900

STAIR_STEP_LENGTH = 300   # 踏み込み
STAIR_STEP_WIDTH = 1000
STAIR_STEP_HEIGHT = 150


# ============================================================
# L1の区画
# ============================================================
TRANSFER_AREA_SIZE = (1000, 1000)
TRANSFER_AREA_L1_CORNER_OFFSET = 750  # L1角からのオフセット(実測値)
# 受渡しエリアの原点。複数箇所(br_referee_node, decision_common,
# field_drawing)で同じ式を重複させないよう、ここに一本化する。
TRANSFER_AREA_ORIGIN = (
    L1_ORIGIN[0] + TRANSFER_AREA_L1_CORNER_OFFSET,
    L1_ORIGIN[1] + TRANSFER_AREA_L1_CORNER_OFFSET,
)

BUILD_SPOT_SIZE = (500, 500)
BUILD_SPOTS_PER_LEVEL = 4  # 四隅に1個ずつ (赤2・青2)

L1_RETRY_ZONE_SIZE = (700, 700)

L1_OUTER_BARRIER_WIDTH = 650  # L1外周からフィールド外縁までの帯幅


# ============================================================
# L2の区画
# ============================================================
CENTRAL_PILLAR_HEIGHT = 800
CENTRAL_PILLAR_DIAMETER = 270
CENTRAL_PILLAR_SOCKET_DIAMETER = 180
CENTRAL_PILLAR_SOCKET_DEPTH = 100


# ============================================================
# 競技物
# ============================================================
EARTH_BLOCK_SIZE = 350  # 立方体一辺
# 公式ルールブック: "Approx. weight: 600 grams per unit (±20%)"
EARTH_BLOCK_WEIGHT_RANGE_G = (480, 720)
EARTH_BLOCK_COUNT_PER_TEAM = 20

SKY_BLOCK_SIZE = 200
# 公式ルールブック: "Approx. weight: 220 grams per unit (±20%)"
SKY_BLOCK_WEIGHT_RANGE_G = (176, 264)

# 公式ルールブック: Mikasa V300W(Size 5)を使用、直径約210mm
MUSTIKA_DIAMETER = 210
# 公式ルールブック: "Approx. weight: 260-280 grams"
MUSTIKA_WEIGHT_RANGE_G = (260, 280)


# ============================================================
# 得点(8章より)
# ============================================================
SCORE_TRANSFER_PER_BLOCK = 5

SCORE_EARTH_BLOCK_L1_LAYER1 = 10
SCORE_EARTH_BLOCK_L1_LAYER2 = 20
SCORE_EARTH_BLOCK_L2_LAYER1 = 20
SCORE_EARTH_BLOCK_L2_LAYER2 = 40

SCORE_SKY_BLOCK_L1 = 40
SCORE_SKY_BLOCK_L2 = 80

SCORE_MUSTIKA = 250


# ============================================================
# チーム / 座標変換
# ============================================================
@dataclass(frozen=True)
class TeamColor:
    RED = "red"
    BLUE = "blue"


def mirror_to_blue(u: float, v: float) -> tuple[float, float]:
    """
    赤チーム座標(u, v)を青チーム側の座標に変換する(暫定: U軸中央で線対称)。
    分割線の正確な向きが確定したら、この関数だけ直せばよい。
    """
    return (GAME_FIELD_SIZE - u, v)


def mirror_rect_origin(origin: tuple[float, float], size: tuple[float, float]) -> tuple[float, float]:
    """原点(左下)+サイズで表す矩形を青チーム側へ鏡映したときの新しい原点を返す。"""
    mx, my = mirror_to_blue(origin[0], origin[1])
    return (mx - size[0], my)


def get_team_origin_offset(team: str) -> tuple[float, float]:
    """チームごとの原点シフト量(暫定実装)。"""
    if team == TeamColor.RED:
        return (0, 0)
    elif team == TeamColor.BLUE:
        return (GAME_FIELD_SIZE, 0)
    raise ValueError(f"unknown team: {team}")


# ============================================================
# L1/L2建築スポットの専有(6.2.2 相手専用区域の判定に使う)
# ============================================================
# 四隅のうちu軸が低い側(原点/赤チーム側)の2箇所を赤専有、高い側(青チーム側)の
# 2箇所を青専有とする。これは「赤/青分割線をU軸中央対称とする」という
# 既知の暫定事項(CLAUDE.md参照)と同じ前提に基づく仮定。公式図面で分割線の
# 向きが確定したら見直すこと。
# (build_spot_id, level, team, origin)
BUILD_SPOTS = [
    ("l1_red_1", 1, TeamColor.RED, (L1_ORIGIN[0], L1_ORIGIN[1])),
    ("l1_red_2", 1, TeamColor.RED, (L1_ORIGIN[0], L1_ORIGIN[1] + L1_SIZE - BUILD_SPOT_SIZE[1])),
    ("l1_blue_1", 1, TeamColor.BLUE, (L1_ORIGIN[0] + L1_SIZE - BUILD_SPOT_SIZE[0], L1_ORIGIN[1])),
    ("l1_blue_2", 1, TeamColor.BLUE,
     (L1_ORIGIN[0] + L1_SIZE - BUILD_SPOT_SIZE[0], L1_ORIGIN[1] + L1_SIZE - BUILD_SPOT_SIZE[1])),
    ("l2_red_1", 2, TeamColor.RED, (L2_ORIGIN[0], L2_ORIGIN[1])),
    ("l2_red_2", 2, TeamColor.RED, (L2_ORIGIN[0], L2_ORIGIN[1] + L2_SIZE - BUILD_SPOT_SIZE[1])),
    ("l2_blue_1", 2, TeamColor.BLUE, (L2_ORIGIN[0] + L2_SIZE - BUILD_SPOT_SIZE[0], L2_ORIGIN[1])),
    ("l2_blue_2", 2, TeamColor.BLUE,
     (L2_ORIGIN[0] + L2_SIZE - BUILD_SPOT_SIZE[0], L2_ORIGIN[1] + L2_SIZE - BUILD_SPOT_SIZE[1])),
]
