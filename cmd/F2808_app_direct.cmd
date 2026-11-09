/* 직접 굽기 링커 파일: 부트로더 없이 JTAG 로 .out 을 플래시에 올린다.
   앱이 섹터 D·C·B·A 전체를 쓴다. 굽는 순간 장비에 있던 부트로더는 지워진다.
   CSM_RSVD·CSM_PWL 은 할당하지 않는다. 0xFFFF 유지. 잘못 쓰면 칩이 잠긴다. */
MEMORY
{
PAGE 0:
   RAML0       : origin = 0x008000, length = 0x001000
   FLASH       : origin = 0x3E8000, length = 0x00FF80     /* 섹터 D, C, B, A 중 CSM_RSVD 앞까지 */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* 부트 ROM 이 점프하는 자리 */
   RESET       : origin = 0x3FFFC0, length = 0x000002

PAGE 1:
   RAMM0       : origin = 0x000000, length = 0x000400
   RAMM1       : origin = 0x000480, length = 0x000380     /* 0x400~0x47F 부트 ROM 예약 */
   RAML1       : origin = 0x009000, length = 0x001000
   RAMH0       : origin = 0x3FA000, length = 0x002000
}

SECTIONS
{
   .reset      : > RESET,      PAGE = 0, TYPE = DSECT
   codestart   : > BEGIN,      PAGE = 0
   .text       : > FLASH,      PAGE = 0
   .cinit      : > FLASH,      PAGE = 0
   .pinit      : > FLASH,      PAGE = 0
   .econst     : > FLASH,      PAGE = 0
   .const      : > FLASH,      PAGE = 0
   .switch     : > FLASH,      PAGE = 0
   ramfuncs    : LOAD = FLASH, RUN = RAML0, PAGE = 0,
                 LOAD_START(_RamfuncsLoadStart), LOAD_END(_RamfuncsLoadEnd), RUN_START(_RamfuncsRunStart)

   .stack      : > RAMM0,      PAGE = 1
   .bss        : > RAMM1,      PAGE = 1
   .ebss       : > RAML1,      PAGE = 1
   .sysmem     : > RAMH0,      PAGE = 1
   .esysmem    : > RAMH0,      PAGE = 1
   test_keep   : { *(test_keep) . = 0xA40; *(test_keep_add) *(test_keep_rpm) *(test_keep_bld) } > RAMH0, PAGE = 1   /* 시험 결과. codestart 가 지우지 않아 리셋에 남는다.
                                                                       뒤에 붙인 결과(test_keep_add, test_keep_rpm, test_keep_bld)는 앞 주소를 옮기지 않는다.
                                                                       . = 0xA40: agv_stopt 를 앞 이미지와 같은 0x3FAA40 에 둔다. 앞이 넘치면 링크가 실패한다 */
}
