; codestart.asm - 부트 ROM 이 점프하는 자리. C 초기화가 길어도 워치독에 걸리지 않게 먼저 끈다
        .ref    _c_int00
        .global code_start

        .sect   "codestart"
code_start:
        LB      wd_off

        .text
wd_off:
        SETC    OBJMODE
        EALLOW
        MOVZ    DP, #0x7029 >> 6
        MOV     @0x7029 & 0x3F, #0x0068
        EDIS
        LB      _c_int00
        .end
