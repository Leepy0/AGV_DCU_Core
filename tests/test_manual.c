/* test_manual.c - 수동 조작 표(사슬의 첫 맞는 줄, else, 쓰지 않은 출력 유지, BELT 마스크, 방향 전환 입력, 추가 장애물)와
 * 프로파일 적재 조건. 입력은 보드 인터페이스 입력 1~5(확장 보드 없이 읽힌다)만 쓴다 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/drive.h"
#include "agv/ioport.h"
#include "agv/seq_dsl.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/site.h"
#include "hal_host.h"

#define BELT1_CW     OUTS( OUT_EXT_1 )
#define BELT1_CCW    OUTS( OUT_EXT_1, OUT_EXT_2 )
#define BELT2_CW     OUTS( OUT_EXT_3 )
#define BELT2_CCW    NO_OUT

MANUAL_BEGIN( s_tab )
    MAN( ID(4, 5), JOG, IN( ON(IN_IF_1), OFF(IN_IF_2) ), BELT( 1, CW ) )
    MAN( ID(4, 5), JOG, IN( ON(IN_IF_2) ),               BELT( 1, CCW ) )
    MAN( ID(4, 5), JOG, IN( ON(IN_IF_3) ),               ON( OUT_IF_1 ) )        /* 다른 출력은 그대로 */
    MAN( ID(4, 5), JOG, KEY( UP ),                       ON( OUT_IF_2 ) )        /* 키 없음: 거짓 */
    MAN_ELSE( ID(4, 5), JOG,                             BELT( 1, OFF ), OFF( OUT_IF_1 ) )
    MAN( ID(5), JOG, IN( ON(IN_IF_4) ),                  BELT( 2, CW ) )         /* 같은 호기 둘째 사슬 */
    MAN_ELSE( ID(5), JOG,                                NO_OUT )
    MAN( ID(4, 5), IDLE, NO_IN,                          ON( OUT_EXT_9 ) )       /* 대기 아님: 안 쓴다 */
    MAN_ELSE( ID(4, 5), IDLE,                            ON( OUT_EXT_10 ) )
    MAN_FB_SW( ID(5), IN_IF_3 )
    MAN_OBS_B( ID(4), IN_IF_4 )
MANUAL_END

LOAD_IF( s_lif_4, IN( ON(IN_IF_1) ), IN( OFF(IN_IF_2), ON(IN_IF_3) ) )
LOAD_IF( s_lif_6, IN( ON(IN_IF_4) ) )
PROFILE_TABLE_BEGIN( s_prof )
    PROFILE_LOAD_IF( 1, 0, 4, 4, s_lif_4 )
    PROFILE_LOAD_IF( 2, 0, 6, 6, s_lif_6 )
    PROFILE_LOAD( 3, 0, 5, 5, LOAD_SENSORS(IN_IF_1) )
PROFILE_TABLE_END

/* 컴파일 오류 시험(tests/run_host.ps1 이 하나씩 본다) */
#ifdef NEG_MAN_SETCLR
MANUAL_BEGIN( s_neg )  MAN( ID(1), JOG, NO_IN, ON( OUT_EXT_1 ), OFF( OUT_EXT_1 ) )  MANUAL_END   /* 켜면서 끔 */
#endif
#ifdef NEG_MAN_BELT_NODIR
MANUAL_BEGIN( s_neg )  MAN( ID(1), JOG, NO_IN, BELT( 2, CCW ) )  MANUAL_END                      /* 없는 방향 */
#endif
#ifdef NEG_MAN_ID_ORDER
MANUAL_BEGIN( s_neg )  MAN_ELSE( ID(5, 4), JOG, NO_OUT )  MANUAL_END
#endif
#ifdef NEG_MAN_LOAD_EMPTY
LOAD_IF( s_neg_if, NO_IN )                                                                        /* 늘 참인 적재 조건 */
#endif
#ifdef NEG_MAN_LOAD_NAME
static const uint32_t s_neg_if[2] = { 1, 0 };
PROFILE_TABLE_BEGIN( s_neg )  PROFILE_LOAD_IF( 1, 0, 1, 1, s_neg_if )  PROFILE_TABLE_END          /* LOAD_IF 배열이 아님 */
#endif

void site_register(void) {}
void site_manual(void) {}

#define OUTB(o) (1UL << (o))

/* 입력 인터페이스 1~5 (비트 0~4), 논리 1 = 활성. 핀은 active-low */
static uint32_t run_tick(uint16_t ifs)
{
    host.bus_in[BUS_IN_C] = (uint16_t)(0x38u | ((ifs & 1u) ? 0u : 0x80u) | ((ifs & 16u) ? 0u : 0x40u));
    host.bus_in[BUS_IN_D] = (uint16_t)(0xF8u | ((ifs & 2u) ? 0u : 1u) | ((ifs & 4u) ? 0u : 2u) | ((ifs & 8u) ? 0u : 4u));
    (void)io_isr_read();
    io_latch();
    manual_tick();
    return io_out_mask();
}

int main(void)
{
    static const io_map_t map = { 0, 0, 0, 0 };
    uint32_t o;

    host_reset();
    io_reset();
    io_bind(&map);
    memset(&agv_drive, 0, sizeof(agv_drive));
    manual_bind(s_tab, (uint16_t)(sizeof(s_tab) / sizeof(s_tab[0])), s_prof, (uint16_t)(sizeof(s_prof) / sizeof(s_prof[0])));
    agv_drive.ui_mode = UI_JOG;

    /* 줄 크기: C28x 10워드 */
    assert(sizeof(man_row_t) == 2u * sizeof(uint16_t) + 4u * sizeof(uint32_t));
    assert(s_tab[0].set == OUTB(OUT_EXT_1) && s_tab[0].clr == OUTB(OUT_EXT_2));
    assert(s_tab[1].set == (OUTB(OUT_EXT_1) | OUTB(OUT_EXT_2)) && s_tab[1].clr == 0UL);
    assert(s_tab[4].set == 0UL && s_tab[4].clr == (OUTB(OUT_EXT_1) | OUTB(OUT_EXT_2) | OUTB(OUT_IF_1)));
    assert(s_tab[3].kind == (MAN_K_JOG | (KEY_UP << 8)) && (s_tab[4].kind & MAN_F_ELSE));
    assert(s_tab[0].id == (4u | (5u << 8)) && s_tab[5].id == (5u | (5u << 8)));

    (void)param_set_ram(P_AGV_ID, 4);
    o = run_tick(1u);                                       /* 첫 줄 */
    assert((o & 0xFFFFUL) == OUTB(OUT_EXT_1));
    o = run_tick(1u | 2u);                                  /* 첫 줄 거짓(IF_2 켜짐), 둘째 */
    assert((o & 0xFFFFUL) == (OUTB(OUT_EXT_1) | OUTB(OUT_EXT_2)));
    o = run_tick(4u);                                       /* 셋째: I/F 1 만 켜고 벨트는 그대로 */
    assert((o & 0xFFFFUL) == (OUTB(OUT_EXT_1) | OUTB(OUT_EXT_2)) && (o & OUTB(OUT_IF_1)) && !(o & OUTB(OUT_IF_2)));
    o = run_tick(0u);                                       /* else: 벨트·I/F 1 끔. IDLE 사슬은 안 돈다 */
    assert(o == 0UL);
    assert(agv_drive.obs_b_ext == 0);
    o = run_tick(8u);                                       /* 호기 4: 둘째 사슬 없음, 추가 장애물 = IF_4 */
    assert(o == 0UL && agv_drive.obs_b_ext == 1);

    (void)param_set_ram(P_AGV_ID, 5);
    o = run_tick(8u);                                       /* 호기 5: 둘째 사슬 */
    assert(o == OUTB(OUT_EXT_3) && agv_drive.obs_b_ext == 1);  /* 추가 장애물은 호기 4 줄만 쓴다 */
    o = run_tick(0u);                                       /* 둘째 사슬 else 는 NO_OUT: 벨트 2 그대로 */
    assert(o == OUTB(OUT_EXT_3));
    agv_drive.stop_sw = 1;                                  /* STOP: JOG 아님 */
    o = run_tick(1u);
    assert(o == OUTB(OUT_EXT_3));
    agv_drive.stop_sw = 0;

    assert(s_tab[9].kind == MAN_K_FB_SW && s_tab[9].set == IN_IF_3);   /* 방향 전환 입력은 앱 대조 시험이 본다 */

    /* 적재: 호기 4 조건 둘, 6 조건 하나, 5 는 센서 묶음(-1), 그 밖 없음(-1) */
    (void)param_set_ram(P_AGV_ID, 4);
    (void)run_tick(0u);  assert(manual_load() == 0);
    (void)run_tick(1u);  assert(manual_load() == 1);
    (void)run_tick(4u);  assert(manual_load() == 1);
    (void)run_tick(6u);  assert(manual_load() == 0);
    (void)run_tick(2u);  assert(manual_load() == 0);
    (void)param_set_ram(P_AGV_ID, 6);
    (void)run_tick(8u);  assert(manual_load() == 1);
    (void)run_tick(7u);  assert(manual_load() == 0);
    (void)param_set_ram(P_AGV_ID, 5);
    assert(manual_load() == -1);
    (void)param_set_ram(P_AGV_ID, 7);
    assert(manual_load() == -1);

    printf("test_manual ok\n");
    return 0;
}
