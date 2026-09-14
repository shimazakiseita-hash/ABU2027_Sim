bool open
float32 target_force
string target_block_id  # open=falseで閉じる際に狙う対象(/detected_blocksのid)。
                         # 空文字なら「レンジ内で最も近い1個」にフォールバックする
