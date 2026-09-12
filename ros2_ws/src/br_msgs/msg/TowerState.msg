string build_spot_id
int32 level             # 1=L1, 2=L2
string[] block_types    # 積み上げ順(下から上)。"earth" | "sky"
string[] owner_teams    # block_typesに対応。アースは設置した個人チームに固定
                         # (ルールブック8.3.1)。スカイは常に空文字("")
                         # (得点判定は所有チームではなくtop_colorで行うため)
string[] top_colors     # block_typesに対応。skyの現在の上面色(8.3.2、
                         # ひっくり返すたびに変わる)。earthは ""
