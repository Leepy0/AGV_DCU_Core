/* 앱 링커 예시 — 기존 앱 cmd 에서 FLASH 쪽만 이렇게 바꾼다.
   앱 = 섹터 D·C·B (0x3E8000~0x3F3FFF). 섹터 A(부트로더)·BEGIN·CSM 은 할당하지 않는다.
   부트 카운터 0x0007FA~0x0007FB 와 부트플래그 0x0007FC~0x0007FF 는 RAMM1 에서 제외 (부트로더 cmd 와 동일) */
MEMORY
{
PAGE 0:
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* 부트 ROM 리셋 벡터 (DSECT) */
   RAML0       : origin = 0x008000, length = 0x001000
   APPHDR      : origin = 0x3E8000, length = 0x000040     /* 앱 헤더 (패커가 채움) */
   APPBEGIN    : origin = 0x3E8040, length = 0x000002     /* codestart: 부트로더가 여기로 점프 */
   FLASHAPP    : origin = 0x3E8042, length = 0x00BFBE     /* 섹터 D 나머지 + C + B */
   /* 사용 금지: FLASHA 0x3F4000, CSM_RSVD 0x3F7F80, BEGIN 0x3F7FF6, CSM_PWL 0x3F7FF8 */

PAGE 1:
   RAMM0       : origin = 0x000000, length = 0x000400
   RAMM1       : origin = 0x000400, length = 0x0003FA     /* 0x7FA~0x7FB 부트 카운터, 0x7FC~0x7FF 부트플래그 */
   RAML1       : origin = 0x009000, length = 0x001000
   RAMH0       : origin = 0x3FA000, length = 0x002000     /* 0x3F8000~0x3F9FFF 는 L0·L1 거울 */
}

SECTIONS
{
   .reset      : > RESET,      PAGE = 0, TYPE = DSECT
   .apphdr     : > APPHDR,     PAGE = 0
   codestart   : > APPBEGIN,   PAGE = 0      /* DSP280x_CodeStartBranch.asm 의 "codestart" 섹션 */
   .text       : > FLASHAPP,   PAGE = 0
   .cinit      : > FLASHAPP,   PAGE = 0
   .pinit      : > FLASHAPP,   PAGE = 0
   .econst     : > FLASHAPP,   PAGE = 0
   .switch     : > FLASHAPP,   PAGE = 0
   ramfuncs    : LOAD = FLASHAPP, RUN = RAML0, PAGE = 0,
                 LOAD_START(_RamfuncsLoadStart), LOAD_END(_RamfuncsLoadEnd), RUN_START(_RamfuncsRunStart)

   .stack      : > RAMM0,      PAGE = 1
   .ebss       : > RAML1,      PAGE = 1
   .esysmem    : > RAMH0,      PAGE = 1
   .bss        : > RAMM1,      PAGE = 1
   .const      : > FLASHAPP,   PAGE = 0
   .sysmem     : > RAMH0,      PAGE = 1
   test_keep   : { *(test_keep) . = 0xA40; *(test_keep_add) *(test_keep_rpm) *(test_keep_bld) *(curvelog_ram) *(gscan_ram) } > RAMH0, PAGE = 1   /* 시험 결과. codestart 가 지우지 않아 리셋에 남는다.
                                                                       뒤에 붙인 결과(test_keep_add, test_keep_rpm, test_keep_bld)는 앞 주소를 옮기지 않는다.
                                                                       curvelog_ram(곡선 기록기)은 .ebss 에 안 들어가 맨 뒤에 둔다. 부팅 때 curvelog_reset 이 지운다.
                                                                       gscan_ram(t3.8 스캔 주기 시험 결과·시각 버퍼)은 그 뒤. 보존하지 않고 부팅 때 gresp_reset 이 지운다.
                                                                       . = 0xA40: agv_stopt 를 앞 이미지와 같은 0x3FAA40 에 둔다. 앞이 넘치면 링크가 실패한다 */

   /* PIE 벡터 등 기존 DSP280x_Headers_nonBIOS.cmd 의 레지스터 섹션은 그대로 둔다 */
}
