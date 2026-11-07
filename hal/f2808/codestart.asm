; codestart.asm - 부트 ROM 이 점프하는 자리
; 워치독을 끄고 RAM 을 0 으로 지운 뒤 C 초기화로 간다.
; 이 빌드의 .ebss·.bss 는 부트 코드가 지우지 않는다. .cinit 은 _c_int00 안에서 적용되므로
; 반드시 그 앞에서 지운다. 전원 투입과 리셋 직후의 쓰레기 값이 앰프 출력으로 나가는 것을 막는다.
; shortcut: OTA 빌드로 옮길 때 0x7FA~0x7FF(부트 카운터, 부트플래그)는 지우면 안 된다.
        .ref    _c_int00
        .global code_start

M0_START  .set  0x000000        ; 스택
M0_WORDS  .set  0x0400
M1_START  .set  0x000480        ; .bss. 0x400~0x47F 는 부트 ROM 예약
M1_WORDS  .set  0x0380
L1_START  .set  0x009000        ; .ebss
L1_WORDS  .set  0x1000

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

        MOVL    XAR4, #M0_START
        MOVL    XAR5, #M0_WORDS - 1
clear_m0:
        MOV     *XAR4++, #0
        BANZ    clear_m0, AR5--

        MOVL    XAR4, #M1_START
        MOVL    XAR5, #M1_WORDS - 1
clear_m1:
        MOV     *XAR4++, #0
        BANZ    clear_m1, AR5--

        MOVL    XAR4, #L1_START
        MOVL    XAR5, #L1_WORDS - 1
clear_l1:
        MOV     *XAR4++, #0
        BANZ    clear_l1, AR5--

        LB      _c_int00
        .end
