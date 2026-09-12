string build_spot_id
int32 level           # 1=L1, 2=L2
string team           # "red" | "blue" | "" (未確定/共用)
string[] block_types  # 積み上げ順(下から上)。"earth" | "sky" | "mustika"
string[] top_colors   # block_typesに対応。skyの上面色。earth/mustikaは ""
