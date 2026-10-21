# ハードウェア境界(実機: can_node+STM32 / シム: sim_3d_node)への機構の指令。
# 実機のグリッパーはブロックIDや建築スポットIDを知らないので、機体座標の目標位置で指示する。
string command              # "OPEN"(はなす) | "CLOSE"(つかむ) | "PLACE"(置く) | "FLIP"(スカイを裏返す)
geometry_msgs/Point target  # 機体座標の目標位置(mm)。x=前方, y=左。zは使わない(置く高さは下にある物で決まる)
