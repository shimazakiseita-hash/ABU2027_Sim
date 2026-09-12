string id
geometry_msgs/Point position
int32 level          # 0=ground, 1=L1, 2=L2
string block_type    # "earth" | "sky" | "mustika"
string top_color     # "red" | "blue" | "" (アースブロックは無関係)
string owner_team    # "red" | "blue" | "" (未確定)
string held_by        # "none" | "tr" | "br"
bool placed           # 建築スポットに正しく設置され得点対象になっているか(7.2判定用)
