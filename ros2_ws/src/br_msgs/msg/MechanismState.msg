# ハードウェア境界から返る機構の状態。
bool holding        # 何かを保持しているか
bool busy           # つかむ/はなす/置く/裏返すの動作中か
string last_command # 直前に受け付けた指令(MechanismCmd.command)
string last_result  # "OK" | "OUT_OF_REACH"(届かない) | "NOTHING_THERE"(目標位置に物が無い)
                    # | "NOT_HOLDING"(置く物を持っていない) | "" (動作中/未実行)
