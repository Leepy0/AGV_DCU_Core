/* test_sys.c - 2·3단계 통합 시험. hal/host 위에서 부팅부터 10 ms 스캔까지 돌린다 */
#include "agv/tune.h"
#include "agv/steer.h"
#include "agv/wtest.h"
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "agv/acs.h"
#include "agv/comm.h"
#include "agv/curvelog.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/guide.h"
#include "agv/io.h"
#include "agv/ioport.h"
#include "agv/link.h"
#include "agv/log.h"
#include "agv/motor.h"
#include "agv/ident.h"
#include "agv/param.h"
#include "agv/rpmtest.h"
#include "agv/run.h"
#include "agv/tag_dsl.h"
#include "agv/site.h"
#include "agv/stop.h"
#include "agv/storage.h"
#include "agv/sys.h"
#include "agv/tagact.h"
#include "agv/ui.h"
#include "hal_host.h"

#define D agv_drive
#define P param_dv

/* 입력 원시 비트 */
#define A_AMP_L  0x01
#define A_AMP_R  0x02
#define RAW_START 0x04
#define RAW_STOP  0x08
#define A_BUMP1  0x10
#define B_ESTOP  0x01
#define AGV_ID   7

#define O_X    OUT_EXT_1
#define O_ACT  OUT_EXT_2
enum { ACT_HOLD = 1 };
SEQ_LABELS( L_NONE_ )

SEQ_BEGIN( s_load )
    OUT_ON( O_X )
    DELAY( 500 )
    OUT_OFF( O_X )
    END_STOP
SEQ_END

SEQ_BEGIN( a_hold )
    OUT_ON( O_ACT )
    DELAY( 5000 )
    OUT_OFF( O_ACT )
    END_STOP
SEQ_END

/* 시퀀스 회전 속도: 15 m/min 지정, 생략(자동). 이 파일은 tag_dsl.h 를 먼저 포함해 AGV_TURN 이 태그 동작이라
 * 시퀀스 AGV_TURN( CW, 90, 0, 0, 15 )·AGV_TURN( CW, 90, 0, 0 ) 이 펼쳐지는 내부 매크로로 쓴다 */
SEQ_BEGIN( s_turn15 )
    DSL_STURN4( TURN_CW, 90, 0, 0, 15 )
    END_STOP
SEQ_END
SEQ_BEGIN( s_turn0 )
    DSL_STURN3( TURN_CW, 90, 0, 0 )
    END_STOP
SEQ_END

static const seq_def_t s_seqs[] = {
    { 0, SEQ_LOAD, s_load_steps, sizeof(s_load_steps) / sizeof(s_load_steps[0]) },
    { 0, SEQ_UNLOAD, s_turn15_steps, sizeof(s_turn15_steps) / sizeof(s_turn15_steps[0]) },
    { 0, SEQ_BOX_LOAD, s_turn0_steps, sizeof(s_turn0_steps) / sizeof(s_turn0_steps[0]) }
};
static const seq_def_t s_acts[] = { { 0, ACT_HOLD, a_hold_steps, sizeof(a_hold_steps) / sizeof(a_hold_steps[0]) } };
static int stop_len_call(seq_ctx_t *c) { (void)c; tagact_stop_length(); return 1; }
static int alias_call(seq_ctx_t *c) { (void)c; (void)tagact_run(47); return 1; }   /* 번호 변환 별칭처럼 tagact_run 을 다시 부른다 */
/* 견줌용 참조: 사이트 C 로 쓰던 센서 정지(입력 EXT_4 가 1 s 이어 켜지면 처음 켜진 자리 + FL, 시퀀스 LOAD, 전후진 전환, 5 m 오류 270).
 * site_manual 에서 돈다. AGV_STOP( FL, FROM_SENSOR( IN_EXT_4, 100 ) ) 가 노이즈 없는 입력에서 같은 틱·거리여야 한다 */
static struct { uint16_t on, armed, seen, ticks; uint32_t arm_p, on_p; } s_ref;
static uint32_t ref_pulses(void) { return run_pulses(D.backward ? 1u : 0u); }
static int32_t  ref_mm(uint32_t from) { return (int32_t)((float)(ref_pulses() - from) / P.one_length_pulse); }
static int ref_arm(seq_ctx_t *c)
{
    (void)c;
    s_ref.armed = 1;
    s_ref.seen = 0;
    s_ref.ticks = 0;
    s_ref.arm_p = ref_pulses();
    return 1;
}
static void ref_tick(void)
{
    if (!s_ref.armed) { return; }
    if (D.stop_sw || D.system_err || D.stop_err || (D.ui_mode != UI_AUTO)) { s_ref.armed = 0; return; }
    if (!D.flag_run || D.flag_stopping) { return; }
    if (io_in(IN_EXT_4)) {
        if (!s_ref.seen) { s_ref.seen = 1; s_ref.ticks = 0; s_ref.on_p = ref_pulses(); }
        else if (++s_ref.ticks >= 100u) {
            int32_t left = (int32_t)param_get(P_STOP_LEN_FWD_MM) * 10L - ref_mm(s_ref.on_p);
            s_ref.armed = 0;
            tagact_stop_after_mm((left > 0) ? left : 1L);
            seq_request(SEQ_LOAD);
            tagact_forback(DIR_TOGGLE);
        }
        return;
    }
    s_ref.seen = 0;
    if (ref_mm(s_ref.arm_p) > 5000L) { s_ref.armed = 0; D.stop_err = 270; }
}
static const seq_call_fn s_calls[] = { stop_len_call, alias_call, ref_arm };
/* 호기 77 만 적재 센서가 있다(시험용) */
PROFILE_TABLE_BEGIN( s_profiles )
    PROFILE_LOAD( 1, 0, 77, 77, LOAD_SENSORS(IN_EXT_1, IN_EXT_2) )
PROFILE_TABLE_END
/* 현장 적재 판정 시험용. 음수 = 판정 없음(센서 묶음을 쓴다) */
static int16_t s_load_site = -1;
static int16_t site_load(void) { return s_load_site; }
static const seq_site_t s_site = { s_seqs, sizeof(s_seqs) / sizeof(s_seqs[0]), s_acts, 1, s_calls, 3, s_profiles, 1, site_load };

/* 태그 표 */
static const tag_rule_t s_rules[] = {
    { 1, 0, 0, 0, 0, { TA_STOP, 0, 0, 0 } },
    { 2, 0, 0, 0, 0, { TA_SPEED | 2, 0, 0, 0 } },
    { 4, 0, 0, 0, 0, { TA_SEQ | SEQ_LOAD, 0, 0, 0 } },
    { 5, 0, 0, 0, 0, { TA_ACT | ACT_HOLD, 0, 0, 0 } },
    { 8, 0, 0, 0, 0, { TA_CALL | 0, 0, 0, 0 } },
    { 9, 0, TAG_COND_BACKWARD, 0, 0, { TA_STOP, 0, 0, 0 } },
    /* 확장 조건 둘 + 입력. 시험은 tagact_dry_run 으로 맞은 규칙만 본다 */
    TAG_IF( 20, 0, IF_X2_IN( X_ID(AGV_ID, AGV_ID), X_STATION(1, 42, 42), IN( ON(IN_START_SW) ) ), AGV_SPEED( LOW ) )
    TAG_IF( 20, 0, IF_X2( X_ID(AGV_ID, AGV_ID), X_STATION(1, 42, 42) ), AGV_SPEED( MIN ) )
    TAG_IF( 20, 0, IF_ID( 8, 9 ), AGV_SPEED( HIGH ) )
    TAG_IF( 21, 0, IF_TAG_NUM( STATION(0) + 10 ), OBS_ON )
    TAG_IF( 21, 0, IF_TAG_NUM( STATION(0) ), OBS_OFF )
    TAG_IF( 22, 0, IF_STATION_MOD( 0, 10, 3 ), OBS_ON )
    TAG_IF( 22, 0, IF_STATION_DIV( 0, 10, 4 ), OBS_OFF )
    TAG_IF( 23, 0, IF_IN( ANY_ON(IN_EXT_16, IN_START_SW) ), OBS_ON )
    TAG_IF( 23, 0, IF_X( X_NOT(X_CROSS_IN) ), OBS_OFF )
    /* 회전 속도 지정(m/min): 15, 최저 아래 1, 한계 위 31, 생략(자동) */
    TAG( 30, 0, AGV_TURN( CW, 90, 0, 0, 15 ) )
    TAG( 31, 0, AGV_TURN( CW, 90, 0, 0, 1 ) )
    TAG( 32, 0, AGV_TURN( CW, 90, 0, 0, 31 ) )
    TAG( 33, 0, AGV_TURN( CW, 90, 0, 0 ) )
    /* 대기·도착 보고 (tag_dsl.h) */
    TAG( 34, 0, AGV_STOP, ACS_SEND( STANDBY, AGV_ID_MINUS(2) ), STATION_RESET )
    TAG( 35, 0, ACS_SEND( STANDBY_RESET, 14 ) )
    TAG( 36, 0, ACS_SEND( ARRIVE, STATION(1) ) )
    /* 정지 4분류: 분류 2·3·4 가 같은 규칙의 SL 정지를 이긴다(순서와 무관) */
    TAG( 37, 0, ACS_SEND( STANDBY, 7 ), SEQ_START( 30, LOAD ) )      /* 30 cm */
    TAG( 38, 0, AGV_STOP, AGV_STOP( 0 ) )
    TAG( 39, 0, AGV_AUTOSTART( 5 ), AGV_STOP( TAPE ) )
    TAG( 40, 0, SEQ_START( LOAD ), ACS_SEND( STANDBY, 7 ), AGV_STOP( TAPE_GD ) )
    TAG( 41, 0, SEQ_START( TAPE_GD, LOAD ) )
    TAG( 42, 0, AGV_STOP( 20 ), SEQ_START( TAPE, LOAD ), AGV_STOP )
    TAG( 43, 0, AGV_TURN( CW, 90, 0, 1, 0, TAPE ) )
    TAG( 48, 0, AGV_TURN( CW, 180, DIR_BACK_AFTER_TURN, 0, 0, TAPE ) )   /* 전진 중이면 회전을 걸지 못하는 조합 */
    TAG( 46, 0, CALL( 1 ) )                                   /* 별칭: 47 을 다시 부른다 */
    TAG( 47, 0, AGV_STOP( 30 ) )
    TAG( 44, 0, AGV_AUTOSTART( 1 ), SEQ_START( TAPE, LOAD ) )
    TAG( 45, 0, AGV_STOP( TAPE ), ACS_SEND( STANDBY, 7 ), AGV_AUTOSTART( 1 ) )
    /* v0.53.0: 범위 규칙(큐[0] + 100 인 태그에서 서고 보고, 아니면 보고만), 태그 번호 식, 보고 번호 식, 파라미터 조건, 출발 대기 */
    TAG_RANGE_IF( 101, 120, 0, IF_TAG_NUM( STATION(0) + 100 ), ACS_SEND( ARRIVE, STATION(0) + 100 ), AGV_STOP, AGV_AUTOSTART_PARAM )
    TAG_RANGE( 101, 120, 0, ACS_SEND( ARRIVE, STATION(0) + 100 ) )
    TAG( 110, 0, OBS_ON )                                     /* 앞 범위 규칙이 먼저 맞는다 */
    TAG_IF( 121, 0, IF_PARAM( TFB_1, 2 ), AGV_WAIT_START )
    TAG( 121, 0, AGV_WAIT_CANCEL )
    TAG( 122, 0, ACS_SEND( ARRIVE, TAG_NUM - 100 ), ACS_SEND( LOADING, STATION(1) - 2 ) )
    TAG( 130, 0, OBS_OFF )                                    /* 뒤 범위 규칙보다 먼저 */
    TAG_RANGE( 125, 135, 0, OBS_ON )
    TAG_IF( 140, 0, IF_TAG_NUM( 100 + MOD( STATION(1), 100 ) ), OBS_ON )
    /* 센서 정지 + 범위 규칙: 뒤 동작은 확정 때(펼친 표·압축 표 같게) */
    TAG_RANGE( 150, 155, 0, AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), SEQ_START( LOAD ), AGV_FORBACK( TOGGLE ) )
    TAG_RANGE( 160, 165, 0, AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), ACS_SEND( ARRIVE, TAG_NUM ) )   /* 확정 때도 읽은 태그 번호 */
    /* 센서 정지: 센서가 켜진 자리부터 거리만큼. 뒤 동작은 확정 때 */
    TAG( 49, 0, AGV_STOP( FL, FROM_SENSOR( IN_EXT_4, 100 ) ), SEQ_START( LOAD ), AGV_FORBACK( TOGGLE ) )
    TAG( 50, 0, AGV_STOP( 1, FROM_SENSOR( IN_EXT_4, 20 ) ) )             /* 10 mm: 확정 때 이미 넘었다 */
    TAG( 51, 0, AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), AGV_STOP( TAPE ) )   /* 교차선 요청과 같이: 표 오류 */
    TAG( 52, 0, CALL( 2 ) )                                   /* 견줌용 참조(ref_arm) */
    TAG( 53, 0, OBS_OFF, AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), AGV_FORBACK( TOGGLE ) )   /* 앞 동작은 태그 때 */
};

/* 위 20~23 과 같은 규칙의 압축 표. 앞 줄은 다른 프로파일이라 건너뛴다 */
TAG_X_BEGIN( s_px )
    TAG_X( X_ID(AGV_ID, AGV_ID) )            /* 1 */
    TAG_X( X_STATION(1, 42, 42) )            /* 2 */
    TAG_X( X_ID(8, 9) )                      /* 3 */
    TAG_X( X_TAG_NUM( STATION(0) + 10 ) )    /* 4 */
    TAG_X( X_TAG_NUM( STATION(0) ) )         /* 5 */
    TAG_X( X_STATION_MOD(0, 10, 3) )         /* 6 */
    TAG_X( X_STATION_DIV(0, 10, 4) )         /* 7 */
    TAG_X( X_NOT(X_CROSS_IN) )               /* 8 */
    TAG_X( X_TAG_NUM( STATION(0) + 100 ) )   /* 9 */
    TAG_X( X_PARAM( TFB_1, 2 ) )             /* 10 */
    TAG_X( X_TAG_NUM( 100 + MOD( STATION(1), 100 ) ) )   /* 11 */
TAG_POOL_END
TAG_IN_BEGIN( s_pin )
    TAG_IN( IN( ON(IN_START_SW) ) )                  /* 1 */
    TAG_IN( IN( ANY_ON(IN_EXT_16, IN_START_SW) ) )   /* 2 */
TAG_POOL_END
TAG_ACT_BEGIN( s_pact )
    TAG_ACTS( AGV_SPEED( LOW ) )    /* 1 */
    TAG_ACTS( AGV_SPEED( MIN ) )    /* 2 */
    TAG_ACTS( AGV_SPEED( HIGH ) )   /* 3 */
    TAG_ACTS( OBS_ON )       /* 4 */
    TAG_ACTS( OBS_OFF )      /* 5 */
    TAG_ACTS( ACS_SEND( ARRIVE, STATION(0) + 100 ), AGV_STOP, AGV_AUTOSTART_PARAM )   /* 6 */
    TAG_ACTS( ACS_SEND( ARRIVE, STATION(0) + 100 ) )                                   /* 7 */
    TAG_ACTS( AGV_WAIT_START )                                                         /* 8 */
    TAG_ACTS( AGV_WAIT_CANCEL )                                                        /* 9 */
    TAG_ACTS( ACS_SEND( ARRIVE, TAG_NUM - 100 ), ACS_SEND( LOADING, STATION(1) - 2 ) )   /* 10 */
    TAG_ACTS( AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), SEQ_START( LOAD ), AGV_FORBACK( TOGGLE ) )   /* 11 */
    TAG_ACTS( AGV_STOP( 30, FROM_SENSOR( IN_EXT_4, 10 ) ), ACS_SEND( ARRIVE, TAG_NUM ) )              /* 12 */
TAG_POOL_END
TAGP_BEGIN( s_prules )
    TAGP( 20, 63, 0, 0, 0, 3 )
    TAGP( 20, 0, 1, 2, 1, 1 )
    TAGP( 20, 0, 1, 2, 0, 2 )
    TAGP( 20, 0, 3, 0, 0, 3 )
    TAGP( 21, 0, 4, 0, 0, 4 )
    TAGP( 21, 0, 5, 0, 0, 5 )
    TAGP( 22, 0, 6, 0, 0, 4 )
    TAGP( 22, 0, 7, 0, 0, 5 )
    TAGP( 23, 0, 0, 0, 2, 4 )
    TAGP( 23, 0, 8, 0, 0, 5 )
    TAGP( 101, 0, 9, 0, 0, 6 )     /* 10: 101~120 */
    TAGP( 101, 0, 0, 0, 0, 7 )     /* 11: 101~120 */
    TAGP( 110, 0, 0, 0, 0, 4 )
    TAGP( 121, 0, 10, 0, 0, 8 )
    TAGP( 121, 0, 0, 0, 0, 9 )
    TAGP( 122, 0, 0, 0, 0, 10 )
    TAGP( 130, 0, 0, 0, 0, 5 )
    TAGP( 125, 0, 0, 0, 0, 4 )     /* 17: 125~135 */
    TAGP( 140, 0, 11, 0, 0, 4 )
    TAGP( 150, 0, 0, 0, 0, 11 )    /* 19: 150~155 */
    TAGP( 160, 0, 0, 0, 0, 12 )    /* 20: 160~165 */
TAG_POOL_END
TAG_RANGES_BEGIN( s_prange )
    TAG_RANGE_P( 10, 120 )
    TAG_RANGE_P( 11, 120 )
    TAG_RANGE_P( 17, 135 )
    TAG_RANGE_P( 19, 155 )
    TAG_RANGE_P( 20, 165 )
TAG_POOL_END
TAG_PACKED_R( s_tag_pk, s_prules, s_px, s_pin, s_pact, s_prange )

static int s_manual_count;
void site_register(void)
{
    param_default_type(0);
    (void)seq_bind(&s_site);
    tagact_bind(s_rules, sizeof(s_rules) / sizeof(s_rules[0]), 0);
}
void site_manual(void) { s_manual_count++; if (s_ref.on) { ref_tick(); } }

/* ---- 도구 ---- */

static void tick(void)
{
    int i;
    for (i = 0; i < 5; i++) { sys_tick_2ms(); }
    sys_scan();
}
static void ticks(int n) { while (n-- > 0) { tick(); } }

static int s_ext_board;           /* 1 = 확장 입력 보드(입력 B b3 점퍼), 입력은 모두 꺼짐 */

static void boot(void)
{
    host_reset();
    if (s_ext_board) {
        host.bus_in[BUS_IN_B] |= 0x08u;
        host.bus_in[BUS_EXT_IN_A] = host.bus_in[BUS_EXT_IN_B] = 0xFFu;
    }
    memset(&D, 0, sizeof(D));
    /* 호기 번호가 있는 EEPROM. 체크섬은 첫 부팅이 만든다 */
    sys_boot();
    if (param_get(P_AGV_ID) != AGV_ID) {
        param_store_set(&storage_queued);
        (void)param_set(P_AGV_ID, AGV_ID);
    }
    (void)param_set(P_ACS_RETRY_SEC, 1);
    host.adc = 300u << 4;            /* 24.4 V 부근 */
    ticks(60);                       /* 전원 안정 */
    assert(sys_power_on());
    /* 이 시험의 모의 센서는 'a' 로 답한다. 기본 'h' 전용은 test_guide_h_only 가 본다 */
    (void)param_set_ram(P_LEGACY_QUIRKS, QUIRK_SRC_A);
}

static void raw_a(uint16_t clear_bits, uint16_t set_bits)
{
    host.bus_in[BUS_IN_A] = (uint16_t)((host.bus_in[BUS_IN_A] & ~clear_bits) | set_bits);
}

static void press_start(void)
{
    raw_a(RAW_START, 0);
    tick();
    raw_a(0, RAW_START);
    tick();
}

static void guide_reply(int pos)
{
    host_sci_t *b = &host.sci[HAL_SCI_B];
    uint16_t f[7];
    uint16_t sum;
    int v = (pos < 0) ? -pos : pos;
    int i;

    f[0] = 2;
    f[1] = 'a';
    f[2] = (pos < 0) ? '-' : '+';
    f[3] = (uint16_t)('0' + v / 10);
    f[4] = (uint16_t)('0' + v % 10);
    sum = (uint16_t)((f[1] + f[2] + f[3] + f[4]) & 0xF);
    f[5] = (uint16_t)((sum < 10) ? ('0' + sum) : ('a' + sum - 10));
    f[6] = 3;
    for (i = 0; i < 7; i++) { b->rx[b->rx_len++] = f[i]; }
}

/* 주행 상태까지 간다 */
static void drive_up(void)
{
    press_start();
    assert(D.flag_ready == 1);
    guide_reply(0);
    tick();
    assert(D.flag_run == 1);
    assert(D.control == RUN_DRIVE);
}

/* 주행 중 스캔: 응답과 엔코더 흉내 */
static void run_scan(int n)
{
    while (n-- > 0) {
        host_sci_t *b = &host.sci[HAL_SCI_B];
        if (b->rx_pos == b->rx_len) { b->rx_len = b->rx_pos = 0; }   /* 다 읽었으면 버퍼를 비운다(긴 주행) */
        guide_reply(0);
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
    }
}

static void send_frame(const char *body, int corrupt)
{
    /* body = 종류부터 BCC 앞까지 */
    uint16_t f[24];
    int i, n = (int)strlen(body);

    f[0] = 2;
    for (i = 0; i < n; i++) { f[1 + i] = (uint16_t)body[i]; }
    f[1 + n] = frame_checksum(&f[1], (uint16_t)n);
    if (corrupt) { f[1 + n] = (f[1 + n] == '0') ? '1' : '0'; }
    f[2 + n] = 3;
    for (i = 0; i < n + 3; i++) { sys_scia_rx(f[i]); }
}

static void tag_frame(int num, char msg, int corrupt)
{
    char b[32];
    sprintf(b, "E%c%c%02dT0%02d00000000", '0' + num / 100, msg, AGV_ID, num % 100);
    send_frame(b, corrupt);
}

static int tx_has(uint16_t type, uint16_t info)
{
    host_sci_t *a = &host.sci[HAL_SCI_A];
    int i;
    for (i = 0; i + 19 < a->tx_len; i++) {
        if ((a->tx[i] == 2) && (a->tx[i + 1] == type) && (a->tx[i + 6] == info)) { return 1; }
    }
    return 0;
}

static void key(uint16_t code)
{
    host.key = code;
    ticks(10);
    host.key = 0;
    ticks(10);
}
static void on_abort(int sig) { (void)sig; _exit(3); }

/* ---- 시험 ---- */

static void test_link(void)
{
    uint16_t ccr, brr;
    link_config(UI_AUTO, 0, LINK_ZIGBEE, &ccr, &brr);
    assert((ccr == 0x07) && (brr == 0x51));
    link_config(UI_AUTO, 5, LINK_ZIGBEE, &ccr, &brr);
    assert((ccr == 0x07) && (brr == 0x144));
    link_config(UI_AUTO, 8, LINK_ZIGBEE, &ccr, &brr);
    assert((ccr == 0x87) && (brr == 0x1A));
    link_config(UI_AUTO, 9, LINK_ZIGBEE, &ccr, &brr);
    assert(brr == 0x51);
    link_config(UI_PARAM, 0, LINK_ZIGBEE, &ccr, &brr);
    assert(brr == 0x1A);
    link_config(UI_AUTO, 5, LINK_WIFI, &ccr, &brr);
    assert((ccr == 0x07) && (brr == 0x1A));
}

static void test_boot(void)
{
    boot();
    assert(host.started && host.bus_enabled);
    assert(host.ee[PARAM_MIGRATE_ADDR] == PARAM_MIGRATE_MARK);
    assert(param_get(P_SPEED_HIGH) == 30);     /* 빈 EEPROM = 기본값 1 */
    assert(D.stop_err == 0);
    assert((host.sci[HAL_SCI_A].brr == 0x51) && (host.sci[HAL_SCI_B].brr == 0x1A));
    assert(host.bus_out[BUS_AMP] == (0x22 | 0x80));    /* 런 꺼짐, 전진 방향 */
    assert(host.bus_out[BUS_LAMP] & 0x08);             /* 전진 릴레이 */
    assert(!(host.bus_out[BUS_LAMP] & 0x80));          /* BRAKE_1 고정 0 */
    assert(host.bus_out[BUS_SEG] & 0x80);              /* BRAKE_2 고정 1 */
    assert(s_manual_count > 0);
    printf("  boot ok\n");
}

static void test_scheduler(void)
{
    uint32_t kicks;
    uint16_t over;

    boot();
    kicks = host.wdog_kicks;
    sys_scan();
    assert(host.wdog_kicks == kicks);          /* 틱이 없으면 차지 않는다 */
    tick();
    assert(host.wdog_kicks == kicks + 1);
    over = sys_overrun();
    { int i; for (i = 0; i < 10; i++) { sys_tick_2ms(); } }
    sys_scan();
    assert(sys_overrun() == over + 1);
    assert(log_count() > 0);
    assert(host.adc_starts > 0);
    printf("  scheduler ok\n");
}

static void test_storage(void)
{
    uint32_t w;

    boot();
    ticks(120);
    assert(storage_pending() == 0);
    w = host.ee_writes;
    storage_request(110, 1234);
    storage_request(110, 4321);                 /* 합쳐진다 */
    tick();
    assert(host.ee[110] != 4321);               /* 쓰기 시간 동안 끝나지 않는다 */
    ticks(3);
    assert(host.ee[110] == 4321);
    assert(host.ee_writes == w + 1);

    /* 읽기 검증 실패: 3회 뒤 포기하고 250 */
    host.ee_bad_addr = 111;
    storage_request(111, 7);
    ticks(30);
    assert(storage_pending() == 0);
    assert(D.stop_err == ERR_PARAM_LIMIT);
    assert(host.ee_writes == w + 1 + STORAGE_RETRY);
    printf("  storage ok\n");
}

static void test_keep_restore(void)
{
    uint16_t sum = 0;
    int i;

    boot();
    ticks(120);
    host.ee[KEEP_BACKWARD] = 1;
    host.ee[KEEP_STATION0] = 123;
    host.ee[KEEP_STATION_CNT] = 1;
    host.ee[KEEP_PAN] = 3;
    /* 다시 부팅 */
    {
        host_hw_t saved = host;
        host_reset();
        memcpy(host.ee, saved.ee, sizeof(host.ee));
        for (i = 1; i <= P_SAVED_LAST; i++) { sum = (uint16_t)(sum + host.ee[i]); }
        assert(sum == host.ee[0]);
    }
    memset(&D, 0, sizeof(D));
    sys_boot();
    assert(D.backward == 1);
    assert(acs_station_get(0) == 123);
    assert(acs_station_count() == 1);
    assert(acs_pan() == '3');
    printf("  keep restore ok\n");
}

static void test_start_drive(void)
{
    boot();
    ticks(120);
    drive_up();
    assert(host.sci[HAL_SCI_B].tx_len >= 4);
    assert(host.sci[HAL_SCI_B].tx[1] == 'A');
    assert(host.sci[HAL_SCI_B].tx[2] == '1');         /* 'A' 체크섬 */
    run_scan(20);
    assert(D.step == DRIVE_CRUISE || D.step == DRIVE_CHANGE_SPEED);
    assert(host.bus_out[BUS_AMP] & 0x44);             /* 런 */
    assert(D.vin_out > 0);
    assert(host.dac[0] > 0);
    printf("  start ok\n");
}

static void test_ready_timeout(void)
{
    boot();
    press_start();
    assert(D.flag_ready == 1);
    ticks(RUN_READY_TIMEOUT + 2);
    assert(D.stop_err == ERR_GUIDE_RANGE);
    ticks(30);
    assert(D.flag_ready == 0);
    printf("  257 ready ok\n");
}

static void test_guide_dropout(void)
{
    boot();
    drive_up();
    run_scan(5);
    ticks(5);                                         /* 응답 없음 */
    assert(D.stop_err == ERR_GUIDE_COMM);
    printf("  255 ok\n");
}

static void test_guide_lost_distance(void)
{
    int i;
    boot();
    drive_up();
    run_scan(5);
    for (i = 0; (i < 2000) && (D.stop_err == 0); i++) {
        /* 전부 꺼짐: 응답은 오고 거리만 는다 */
        host_sci_t *b = &host.sci[HAL_SCI_B];
        static const char f[] = "\2a+17";
        int k, s = 0;
        for (k = 0; k < 5; k++) { b->rx[b->rx_len++] = (uint16_t)f[k]; if (k) { s += f[k]; } }
        s &= 0xF;
        b->rx[b->rx_len++] = (uint16_t)((s < 10) ? ('0' + s) : ('a' + s - 10));
        b->rx[b->rx_len++] = 3;
        if (b->rx_len > 400) { b->rx_len = b->rx_pos = 0; }
        for (k = 0; k < 20; k++) { run_encoder(0, 20000); }
        tick();
    }
    assert(D.stop_err == ERR_GUIDE_RANGE);
    assert(i > 3);
    printf("  257 distance ok\n");
}

static void test_sensor_switch(void)
{
    guide_rx_t rx;
    int fed = 0, i;
    static const uint16_t part[3] = { 2, 'a', '+' };

    boot();
    for (i = 0; i < 3; i++) { fed += guide_feed(part[i], &rx); }
    assert(fed == 0);
    run_relay(1, 1);
    tick();
    /* 전환 뒤 파서가 비워졌으면 ETX 만으로 프레임이 끝나지 않는다 */
    assert(guide_feed(3, &rx) == 0);
    assert(host.bus_out[BUS_LAMP] & 0x08 ? 0 : 1);    /* 릴레이 저활성 */
    ticks(2);
    assert(keep_get(KEEP_BACKWARD) == 1);
    printf("  sensor switch ok\n");
}

static void test_estop(void)
{
    boot();
    drive_up();
    run_scan(20);
    host.bus_in[BUS_IN_B] |= B_ESTOP;
    tick();
    assert(D.system_err == ERR_ESTOP);
    assert(D.estop == 1);
    assert((host.bus_out[BUS_AMP] & 0x44) == 0);      /* 같은 틱에 런 꺼짐 */
    assert((host.dac[0] == 0) && (host.dac[1] == 0));
    assert(D.flag_run == 0);
    /* 누르고 있는 동안 STOP 리셋으로 풀리지 않는다 */
    raw_a(RAW_STOP, 0);
    ticks(5);
    assert(D.system_err == ERR_ESTOP);
    assert(D.estop == 1);
    raw_a(0, RAW_STOP);
    host.bus_in[BUS_IN_B] &= (uint16_t)~B_ESTOP;
    tick();
    raw_a(RAW_STOP, 0);
    ticks(3);
    raw_a(0, RAW_STOP);
    ticks(3);
    assert(D.system_err == 0);
    assert(D.estop == 0);
    printf("  e-stop ok\n");
}

static void test_bumper(void)
{
    boot();
    drive_up();
    run_scan(20);
    raw_a(0, A_BUMP1);
    tick();
    assert(D.system_err == ERR_BUMPER);
    assert((host.dac[0] == 0) && !(host.bus_out[BUS_AMP] & 0x44));
    printf("  bumper ok\n");
}

static void test_amp_alarm(void)
{
    uint32_t i;
    int pulsed = 0;

    boot();
    drive_up();
    run_scan(20);
    raw_a(A_AMP_L, 0);
    tick();
    assert(D.system_err == ERR_AMP_L_OVERLOAD);
    assert(!(host.bus_out[BUS_AMP] & 0x44));
    /* 알람이 풀린 뒤 STOP: RESETZ 한 틱 펄스 */
    raw_a(0, A_AMP_L);
    raw_a(RAW_STOP, 0);
    tick();
    raw_a(0, RAW_STOP);
    for (i = 0; i < 4; i++) {
        tick();
        if (host.bus_out[BUS_AMP] & 0x11) { pulsed++; }
    }
    assert(pulsed == 1);
    assert(D.system_err == 0);
    printf("  amp alarm ok\n");
}

static void test_stop_switch_aborts(void)
{
    boot();
    (void)act_start(ACT_HOLD);
    tick();
    assert(io_out_mask() & (1UL << O_ACT));
    seq_request(SEQ_LOAD);
    ticks(2);
    assert(seq_is_active());
    assert(io_out_mask() & (1UL << O_X));
    raw_a(RAW_STOP, 0);
    tick();
    assert(!seq_is_active());
    assert(!act_is_busy(ACT_HOLD));
    assert((io_out_mask() & ((1UL << O_X) | (1UL << O_ACT))) == 0);
    raw_a(0, RAW_STOP);
    printf("  stop aborts seq+act ok\n");
}

/* 원본처럼 ETX 로 끝난 프레임이면 버려져도 수신 점을 켠다. 리더 보드의 MsgID 'Z' 도 받는다 */
static void test_rx_tag_reader(void)
{
    boot();
    while (comm_rx_flash()) { }
    send_frame("W1199C00000000000", 0);
    assert(comm_rx_flash());
    while (comm_rx_flash()) { }
    tag_frame(2, '1', 1);
    assert(comm_rx_flash());
    while (comm_rx_flash()) { }
    drive_up();
    run_scan(30);
    tag_frame(2, 'Z', 0);
    run_scan(1);
    assert(tx_has('A', 'E'));
    assert(D.speed_cmd == P.llow_speed);
}

static void test_tag(void)
{
    boot();
    drive_up();
    run_scan(30);
    tag_frame(2, '1', 0);
    run_scan(1);
    assert(tx_has('A', 'E'));
    assert(D.speed_cmd == P.llow_speed);
    /* 같은 태그 반복은 무시 */
    D.speed_cmd = P.low_speed;
    tag_frame(2, '2', 0);
    run_scan(1);
    assert(D.speed_cmd == P.low_speed);

    /* 다른 호기의 ACS 명령은 버린다 */
    send_frame("W1199C00000000000", 0);
    run_scan(1);
    assert(!D.flag_stopping);

    /* 회전 중 무시: ACK 없음 */
    D.turn_pulses = 10;
    host.sci[HAL_SCI_A].tx_len = 0;
    tag_frame(1, '3', 0);
    run_scan(1);
    assert(!tx_has('A', 'E'));
    D.turn_pulses = 0;

    printf("  tag ok\n");
}

/* 태그 규칙의 확장 조건·입력 조건 */
static void tag_cond_checks(void)
{
    uint16_t a[4];

    acs_station_set(1, 42, 2);
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == AGV_SPEED( MIN )));
    raw_a(RAW_START, 0);                        /* START 누름 */
    tick();
    assert(io_in(IN_START_SW));
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == AGV_SPEED( LOW )));
    assert((tagact_dry_run(23, a, 4) == 1) && (a[0] == OBS_ON));
    raw_a(0, RAW_START);
    tick();
    assert((tagact_dry_run(23, a, 4) == 1) && (a[0] == OBS_OFF));
    D.cross_in_flag = 1;
    assert(tagact_dry_run(23, a, 4) == 0);
    D.cross_in_flag = 0;
    acs_station_set(1, 43, 2);
    assert(tagact_dry_run(20, a, 4) == 0);
    acs_station_set(1, 42, 2);
    (void)param_set(P_AGV_ID, 8);
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == AGV_SPEED( HIGH )));
    (void)param_set(P_AGV_ID, AGV_ID);

    acs_station_set(0, 11, 2);                  /* 21 - 10 == 11 */
    assert((tagact_dry_run(21, a, 4) == 1) && (a[0] == OBS_ON));
    acs_station_set(0, 21, 2);
    assert((tagact_dry_run(21, a, 4) == 1) && (a[0] == OBS_OFF));
    acs_station_set(0, 43, 2);
    assert((tagact_dry_run(22, a, 4) == 1) && (a[0] == OBS_ON));
    acs_station_set(0, 45, 2);
    assert((tagact_dry_run(22, a, 4) == 1) && (a[0] == OBS_OFF));
    acs_station_set(0, 55, 2);
    assert(tagact_dry_run(22, a, 4) == 0);
    acs_station_reset();
}

/* v0.53.0 태그 표 문법: 범위 규칙(표 순서 그대로), 태그 번호 식, 파라미터 조건(자리) */
static void tag_expr_checks(void)
{
    uint16_t a[4];
    int16_t tfb = param_get(P_TURN_FORBACK);

    acs_station_set(0, 5, 2);
    acs_station_set(1, 240, 2);
    assert((tagact_dry_run(105, a, 4) == 4) && (a[0] == (TA_ACS_NUM | 100)) && (a[2] == TA_STOP) && (a[3] == TA_STOP_AUTOSTART));
    assert((tagact_dry_run(106, a, 4) == 2) && (a[0] == (TA_ACS_NUM | 100)) && (a[1] == ('A' | TA_ACS_STATION)));
    assert((tagact_dry_run(101, a, 4) == 2) && (tagact_dry_run(120, a, 4) == 2));
    assert((tagact_dry_run(110, a, 4) == 2) && (a[0] == (TA_ACS_NUM | 100)));   /* 범위가 뒤 110 규칙보다 앞 */
    assert((tagact_dry_run(100, a, 4) == 0) && (tagact_dry_run(0, a, 4) == 0));
    acs_station_set(0, 10, 2);
    assert((tagact_dry_run(110, a, 4) == 4) && (tagact_dry_run(105, a, 4) == 2));
    assert((tagact_dry_run(125, a, 4) == 1) && (a[0] == OBS_ON) && (tagact_dry_run(135, a, 4) == 1) && (a[0] == OBS_ON));
    assert((tagact_dry_run(130, a, 4) == 1) && (a[0] == OBS_OFF));
    assert((tagact_dry_run(136, a, 4) == 0) && (tagact_dry_run(124, a, 4) == 0));
    assert((tagact_dry_run(140, a, 4) == 1) && (a[0] == OBS_ON));
    acs_station_set(1, 241, 2);
    assert(tagact_dry_run(140, a, 4) == 0);
    assert(param_set(P_TURN_FORBACK, 12) == 0);
    assert((tagact_dry_run(121, a, 4) == 1) && (a[0] == AGV_WAIT_START));
    assert(param_set(P_TURN_FORBACK, 11) == 0);
    assert((tagact_dry_run(121, a, 4) == 1) && (a[0] == AGV_WAIT_CANCEL));
    (void)param_set(P_TURN_FORBACK, tfb);
    acs_station_reset();
}

/* 보낸 프레임에서 종류·Info 가 맞는 마지막 것의 번호(3자리). 없으면 -1 */
static int tx_num(uint16_t type, uint16_t info)
{
    host_sci_t *a = &host.sci[HAL_SCI_A];
    int i, n = -1;
    for (i = 0; i + 19 < a->tx_len; i++) {
        if ((a->tx[i] == 2) && (a->tx[i + 1] == type) && (a->tx[i + 6] == info)) {
            n = (a->tx[i + 7] - '0') * 100 + (a->tx[i + 8] - '0') * 10 + (a->tx[i + 9] - '0');
        }
    }
    return n;
}

static void test_tag_cond(void)
{
    int16_t tfb;

    boot();
    tag_cond_checks();
    tag_expr_checks();
    tagact_bind_packed(&s_tag_pk, 0);         /* 압축 표도 같은 답 */
    tag_cond_checks();
    tag_expr_checks();
    /* 실행(압축 표): 범위 규칙의 정지·보고 번호(큐[0] + 100), 태그 번호·큐[1] 식 보고, 출발 대기 걸기·풀기 */
    acs_station_set(0, 5, 2);
    acs_station_set(1, 240, 2);
    D.vin_out = 50;
    host.sci[HAL_SCI_A].tx_len = 0;
    (void)tagact_run(105);
    assert(D.flag_stopping);
    ticks(2);
    assert(tx_num('E', 'A') == 105);
    D.flag_stopping = 0;
    host.sci[HAL_SCI_A].tx_len = 0;
    (void)tagact_run(122);
    ticks(4);
    assert((tx_num('E', 'A') == 22) && (tx_num('E', 'L') == 238));
    tfb = param_get(P_TURN_FORBACK);
    assert(param_set(P_TURN_FORBACK, 12) == 0);
    (void)tagact_run(121);
    assert(D.wait_start == 1);
    assert(param_set(P_TURN_FORBACK, 11) == 0);
    (void)tagact_run(121);
    assert(D.wait_start == 0);
    (void)param_set(P_TURN_FORBACK, tfb);
    acs_station_reset();
    boot();
    printf("  tag cond ok\n");
}

static void test_compensate(void)
{
    uint16_t len;

    boot();
    drive_up();
    run_scan(20);
    D.flag_stopping = 1;
    D.stop_pulses = 100;
    tagact_compensate(30);
    assert(D.stop_pulses == 70);
    tagact_compensate(100);
    assert((D.stop_pulses == 0) && (D.immediate_stop == 1));

    /* 수신 뒤 처리 전에 달린 만큼 정지 거리에서 뺀다 */
    boot();
    (void)param_set(P_STOP_LEN_MM, 100);
    drive_up();
    run_scan(20);
    tag_frame(8, '6', 0);          /* 프레임 큐에 들어가며 거리 스냅샷 */
    run_encoder(0, 20000);
    run_encoder(0, 20000);
    run_encoder(0, 20000);
    guide_reply(0);
    tick();
    len = (uint16_t)P.stop_len;
    assert(D.stop_pulses == (int16_t)(len - 3));

    /* 정지 중 다른 태그는 남은 거리를 바꾸지 않는다. 속도 태그든 정지 태그든 같다 */
    run_encoder(0, 20000);
    tag_frame(2, '7', 0);
    run_encoder(0, 20000);
    guide_reply(0);
    tick();
    assert(D.stop_pulses == (int16_t)(len - 5));
    tag_frame(8, '8', 0);
    run_encoder(0, 20000);
    guide_reply(0);
    tick();
    assert(D.stop_pulses == (int16_t)(len - 6));

    /* 일시정지 감속 중에는 실제 이동만큼만 줄고, 태그는 거리를 바꾸지 않는다 */
    D.pause_stop = 1;
    run_scan(5);                         /* 스캔마다 왼쪽 펄스 1개 */
    assert(D.stop_pulses == (int16_t)(len - 11));
    assert(stop_is_pause(stop_reason()));
    tag_frame(2, '9', 0);
    guide_reply(0);
    tick();
    assert(D.stop_pulses == (int16_t)(len - 11));
    assert(D.flag_stopping == 1);

    /* 재출발 대기 뒤 끊긴 위치 정지를 남은 거리로 이어 간다 */
    {
        int i;
        for (i = 0; i < 40; i++) { guide_reply(0); tick(); }   /* 멈춰서 기다림. 센서는 계속 응답 */
    }
    assert(D.stop_err == 0);
    D.pause_stop = 0;
    guide_reply(0);
    tick();
    guide_reply(0);
    tick();
    assert(stop_is_position(stop_reason()));
    assert(D.stop_pulses == (int16_t)(len - 11));
    printf("  compensate ok\n");
}
static void test_rfid_retry(void)
{
    boot();
    (void)param_set(P_STOP_LEN_MM, 100);
    drive_up();
    run_scan(20);
    tag_frame(8, '4', 1);          /* 체크섬이 깨진 첫 수신 */
    run_scan(5);
    tag_frame(8, '5', 0);          /* 재전송: 첫 수신 뒤 달린 거리를 뺀다 */
    run_scan(1);
    assert(D.stop_pulses == (int16_t)(P.stop_len - 6));

    /* 번호가 다른 재전송은 보상하지 않는다 */
    boot();
    (void)param_set(P_STOP_LEN_MM, 100);
    drive_up();
    run_scan(20);
    tag_frame(9, '4', 1);
    run_scan(5);
    tag_frame(8, '5', 0);
    run_scan(1);
    assert(D.stop_pulses == (int16_t)(P.stop_len - 1));
    printf("  rfid retry ok\n");
}

static void test_cross(void)
{
    char b[32];

    boot();
    (void)param_set(P_CROSS_MODE, 1);
    drive_up();
    run_scan(5);
    sprintf(b, "E1A%02dI01200000000", AGV_ID);
    send_frame(b, 0);
    run_scan(1);
    assert(D.cross_in_flag == 1);
    /* 대기 거리 CID cm. 'T' 처럼 수신 뒤 달린 1 펄스를 뺀다 */
    assert(D.cross_pulses == (int16_t)((float)param_get(P_CROSS_IN_CM) * 10.0f * P.one_length_pulse + 0.5f) - 1);
    run_scan(1);
    assert(tx_has('E', 'I'));
    /* ACS 정지: 교차로 대기 */
    sprintf(b, "W1B%02dC00000000000", AGV_ID);
    b[8] = '0';
    send_frame(b, 0);
    run_scan(1);
    assert(D.cross_in_on == 1);
    assert(tx_has('A', 'W'));
    /* ACS 출발: 해제 */
    sprintf(b, "W1C%02dC00000000000", AGV_ID);
    b[8] = '1';
    send_frame(b, 0);
    run_scan(1);
    assert((D.cross_in_flag == 0) && (D.cross_in_on == 0));
    printf("  cross ok\n");
}
/* 마지막으로 보낸 (type, info) 프레임의 MsgID. 없으면 0 */
static uint16_t tx_last_msgid(uint16_t type, uint16_t info)
{
    host_sci_t *a = &host.sci[HAL_SCI_A];
    uint16_t id = 0;
    int i;
    for (i = 0; i + 19 < a->tx_len; i++) {
        if ((a->tx[i] == 2) && (a->tx[i + 1] == type) && (a->tx[i + 6] == info)) { id = a->tx[i + 3]; }
    }
    return id;
}

/* OUT 태그: 서지 않고 보고만 한다. E-STOP·리셋 뒤에도 ACK 까지 보내고, 1 s 넘게 미확인이면 EEPROM 에 두어 부팅 뒤 잇는다 */
static void test_cross_out(void)
{
    char b[32];
    uint16_t id;

    boot();
    (void)param_set(P_CROSS_MODE, 1);
    drive_up();
    run_scan(5);
    sprintf(b, "E1A%02dO00700000000", AGV_ID);
    send_frame(b, 0);
    run_scan(1);
    assert(acs_cross_out_count() == 1 && acs_cross_num(ACS_CROSS_OUT) == 7);
    run_scan(50);
    assert(tx_has('E', 'O') && D.flag_run == 1 && !D.cross_out_flag && !D.cross_out_on && D.action_err == 0);
    /* E-STOP 중에도, 풀고 STOP 리셋(run_seq_init) 뒤에도 보낸다 */
    host.bus_in[BUS_IN_B] |= B_ESTOP;
    tick();
    assert(D.system_err == ERR_ESTOP && D.flag_run == 0);
    host.sci[HAL_SCI_A].tx_len = 0;
    ticks(30);
    assert(tx_has('E', 'O') && acs_cross_out_count() == 1);
    host.bus_in[BUS_IN_B] &= (uint16_t)~B_ESTOP;
    tick();
    raw_a(RAW_STOP, 0);
    ticks(3);
    raw_a(0, RAW_STOP);
    ticks(3);
    assert(D.system_err == 0);
    host.sci[HAL_SCI_A].tx_len = 0;
    ticks(30);
    assert(tx_has('E', 'O') && acs_cross_out_count() == 1);
    /* 1 s 를 넘겼다: EEPROM 기록 */
    ticks(20);
    assert(host.ee[ACS_OUT_EE_FIRST] == 7 && host.ee[ACS_OUT_EE_HEAD] == 0x4F01);
    /* 전원 차단 뒤 부팅: 읽어서 다시 보낸다 */
    {
        host_hw_t saved = host;
        host_reset();
        memcpy(host.ee, saved.ee, sizeof(host.ee));
    }
    memset(&D, 0, sizeof(D));
    sys_boot();
    assert(acs_cross_out_count() == 1 && acs_cross_num(ACS_CROSS_OUT) == 7);
    host.adc = 300u << 4;
    ticks(60);
    host.sci[HAL_SCI_A].tx_len = 0;
    ticks(30);
    id = tx_last_msgid('E', 'O');
    assert(id != 0);
    /* ACS ACK: MsgID 를 되돌린다. 기록을 지운다 */
    sprintf(b, "A1%c%02dEOPA00000000", (char)id, AGV_ID);
    send_frame(b, 0);
    ticks(5);
    assert(acs_cross_out_count() == 0 && host.ee[ACS_OUT_EE_HEAD] == 0xFFFF);
    host.sci[HAL_SCI_A].tx_len = 0;
    ticks(30);
    assert(!tx_has('E', 'O'));
    printf("  cross out ok\n");
}
static void test_status_report(void)
{
    boot();
    D.stop_err = ERR_BRANCH_TIMEOUT;
    ticks(5900);
    host.sci[HAL_SCI_A].tx_len = 0;
    ticks(120);
    assert(tx_has('E', 'P'));
    printf("  'P' 60 s ok\n");
}

static void test_ui(void)
{
    uint16_t d[5];

    boot();
    key(KEY_MODE);
    assert(D.ui_mode == UI_JOG);
    assert(D.flag_jog_on == 1);
    ticks(5);
    ui_digits(d);
    assert(d[0] == 20);                               /* J */
    key(KEY_MODE);
    ticks(40);
    assert(D.ui_mode == UI_PARAM);
    assert(host.sci[HAL_SCI_A].brr == 0x1A);          /* 파라미터 모드 115200 */
    ui_digits(d);
    assert(d[0] == 26);                               /* P */
    key(KEY_SET);                                     /* P1SPD */
    key(KEY_SET);                                     /* 첫 번호 */
#if AGV_DEV_TESTS
    assert(ui_para_number() == P_DEV_TEST);           /* TST 가 DRS 앞 */
    key(KEY_UP);
#endif
    assert(ui_para_number() == P_SPEED_HIGH);
    key(KEY_UP);
    assert(ui_para_number() == P_SPEED_LOW);
    key(KEY_DOWN);
    assert(ui_para_number() == P_SPEED_HIGH);
    key(KEY_SET);                                     /* 편집 */
    key(KEY_UP);
    key(KEY_SET);                                     /* 저장 */
    assert(param_get(P_SPEED_HIGH) == 31);
    ticks(10);
    assert(host.ee[P_SPEED_HIGH] == 31);
    assert(host.ee[0] != 0xFFFF);
    key(KEY_RESET);
    key(KEY_RESET);
    key(KEY_MODE);
    assert(D.ui_mode == UI_AUTO);
    assert(host.sci[HAL_SCI_A].brr == 0x51);
    D.flag_run = 1;                                   /* 자동 표시: 뺀 8 은 건너뛰고, 저장된 8 은 역 번호로 */
    param_dv.display_mode = DISP_STATION;
    key(KEY_UP);
    assert(param_dv.display_mode == DISP_EXT_IO);
    key(KEY_DOWN);
    assert(param_dv.display_mode == DISP_STATION);
    param_dv.display_mode = DISP_GONE_8;
    ticks(2);
    assert(param_dv.display_mode == DISP_STATION);
    D.flag_run = 0;
    printf("  ui ok\n");
}

static int shows(const char *t);

/* 파라미터 메뉴 5묶음: 상하키로 묶음, SET 들어가기, UP 항목, RESET 나오기. 5번 묶음 SHIFT 2 s 숨김 */
static void test_menu5(void)
{
#define TM AGV_DEV_TESTS              /* TST 칸(개발 이미지) */
    static const uint16_t want[] = {
#if AGV_DEV_TESTS
        P_DEV_TEST,
#endif
        P_SPEED_HIGH, P_SPEED_LOW, P_SPEED_TURN, P_EMPTY_SPEED, P_EMPTY_TURN, P_ACC, P_DCC, P_SPEED_JOG,
        P_EMPTY_ACC, P_EMPTY_DCC, P_SPEED_MIN,
        P_STOP_LEN_MM, P_CROSS_IN_CM, P_AUTO_START_SEC, P_OBS_USE, P_BUMPER_USE, P_STOP_LEN_FWD_MM, P_LOAD_DETECT,
        P_GUIDE_DIST_MM, P_TURN_FORBACK, P_BRANCH_OFFSET, P_ENC_STALL_TICKS, P_GUIDE_TIMEOUT,
        P_AGV_ID, P_WHEEL_DIA_MM, P_WHEEL_BASE_MM, P_GEAR_RATIO, P_AGV_TYPE,
        P_STEER_KP, P_STEER_KI, P_SPEED_FF,
        P_ACS_RETRY_LEGACY, P_CROSS_RETRY_100MS, P_ACS_STANDBY, P_ACS_RETRY_SEC, P_COMM_LINK, P_INTERFACE_MODE,
        P_ACS_VERSION, P_ERR_REPORT_MIN, P_SOUND, P_DISPLAY_MODE, P_BAT_OFFSET_DV, P_LAMP_EXT_USE, UI_PARA_LAMP_FIRST, UI_PARA_LAMP_FIRST + 1, UI_PARA_LAMP_LAST,
        P_BAT_DV, P_BAT_PERCENT, P_VERSION, P_PARA_INIT,
        P_AUTO_DRIVE_MODE, P_STEER_RPM_OFFSET, P_LEGACY_QUIRKS
    };
    static const int ends[5] = { 11 + TM, 23 + TM, 31 + TM, 46 + TM, 53 + TM };
    static const struct { uint16_t idx; char c[4]; } names[] = {
        { P_SPEED_MIN, "SPN" }, { P_GUIDE_TIMEOUT, "GTO" }, { P_COMM_LINK, "CML" }, { P_STEER_RPM_OFFSET, "SR " },
        { P_GUIDE_CURVE_DETECT, "GSR" }, { P_LEGACY_QUIRKS, "L  " }, { P_EMPTY_SPEED, "EDS" }, { P_EMPTY_TURN, "ETN" },
        { P_EMPTY_ACC, "EAC" }, { P_EMPTY_DCC, "EDC" }, { P_LOAD_DETECT, "LDS" }, { P_AUTO_DRIVE_MODE, "ATD" }
    };
    uint16_t d[5];
    char l[3];
    int g, i, k = 0, n;

    assert(sizeof(want) / sizeof(want[0]) == 53u + TM);
    assert(ends[2] - ends[1] == 8);     /* P3CAR 8 항목 */
    for (i = 0; i < 53 + TM; i++) {     /* 빠진 여덟. TAG·GSR·BRM·OPT 는 미사용 */
        assert(want[i] != P_STATION_NO && want[i] != P_SWITCH_START_NUM && want[i] != P_TAG_TYPE && want[i] != P_CROSS_MODE);
        assert(want[i] != P_TAG_DIGITS && want[i] != P_GUIDE_CURVE_DETECT && want[i] != P_BRANCH_MODE && want[i] != P_OPT_FLAGS);
    }
    for (i = 0; i < (int)(sizeof(names) / sizeof(names[0])); i++) {
        (void)ui_param_label(names[i].idx, l);
        assert(memcmp(l, names[i].c, 3) == 0);
    }
    boot();
    (void)param_set(P_LAMP_EXT_USE, 1);
    key(KEY_MODE);
    key(KEY_MODE);
    ticks(40);
    assert(D.ui_mode == UI_PARAM);
    key(KEY_SET);
    for (g = 0; g < 5; g++) {
        ui_digits(d);
        assert(d[0] == 26 && d[1] == (uint16_t)(g + 1));       /* P1 ~ P5 */
        key(KEY_MODE);                                        /* MODE 는 묶음을 안 바꾼다 */
        ui_digits(d);
        assert(d[1] == (uint16_t)(g + 1));
        key(KEY_SET);
        n = (g == 4) ? 4 : ends[g] - k;
        for (i = 0; i < n; i++) {
            assert(ui_para_number() == (int16_t)want[k + i]);
            key(KEY_UP);
        }
        assert(ui_para_number() == (int16_t)want[k]);           /* 숨김 없이 돈다 */
        if (g == 4) {
            host.key = KEY_SHIFT;                             /* 1.5 s 는 모자란다 */
            ticks(150);
            host.key = 0;
            ticks(10);
            for (i = 0; i < 4; i++) { key(KEY_UP); }
            assert(ui_para_number() == P_BAT_DV);
            host.key = KEY_SHIFT;
            ticks(150);
            assert(!shows("OPEN "));                          /* 2 s 전에는 메시지 없음 */
            ticks(80);
            assert(shows("OPEN "));                           /* 열리면 1 s */
            ticks(120);
            assert(!shows("OPEN ") && ui_para_number() == P_BAT_DV);   /* 목록으로 돌아온다 */
            ticks(300);
            assert(!shows("OPEN "));                          /* 계속 눌러도 다시 안 뜬다 */
            host.key = 0;
            ticks(10);
            for (i = 0; i < 7; i++) {           /* 숨김 3개 */
                assert(ui_para_number() == (int16_t)want[k + i]);
                key(KEY_UP);
            }
            assert(ui_para_number() == P_BAT_DV);
        }
        key(KEY_RESET);
        k = ends[g];
        key(KEY_UP);                                          /* 다음 묶음, 5 다음은 1 */
    }
    ui_digits(d);
    assert(d[1] == 1 && d[2] == 29 && d[3] == 26 && d[4] == 14);   /* P1SPD */
    key(KEY_DOWN);                                            /* 1 이전은 5 */
    ui_digits(d);
    assert(d[1] == 5);
    key(KEY_DOWN);
    ui_digits(d);
    assert(d[1] == 4);
    key(KEY_UP);
    key(KEY_UP);
    ui_digits(d);
    assert(d[1] == 1);
    /* 27번 0 이면 램프 셋이 없다 */
    (void)param_set(P_LAMP_EXT_USE, 0);
    for (i = 0; i < 3; i++) { key(KEY_UP); }
    key(KEY_SET);
    for (i = 0; i < 12; i++) { key(KEY_UP); }
    assert(ui_para_number() == P_ACS_RETRY_LEGACY);
    key(KEY_RESET);
    key(KEY_RESET);
    key(KEY_MODE);
    assert(D.ui_mode == UI_AUTO);
    printf("  menu 5 ok\n");
}

#if AGV_DEV_TESTS
static void boot_keys(uint16_t keys, int keep);
/* TST 임시 시험: 0·2, EEPROM 82 에 남아 부팅 뒤 되살아난다. K_P·K_I·FF 원래 값은 그대로 둔다.
 * 0 기본(param_tst_old 0: 속도 루프·끝 처리 A+B·속도 비례, K_P·K_I 0 = 60·5), 1 비었다,
 * 2 예전(param_tst_old 1: 예전 속도 루프·끝 감쇠, 속도 비례 없음, K_P·K_I 0 = 자동 이득) */
static void test_tst(void)
{
    char c[5];
    int16_t lg, t, k;
    boot();
    lg = param_get(P_LEGACY_QUIRKS);
    assert(!(lg & QUIRK_STEER_STEP) && param_get(P_DEV_TEST) == 0 && !param_tst_old && param_get(P_STEER_KP) == 0);
    assert(param_get(P_STEER_KI) == 0);
    assert(D.p_kp == 60.0f && D.p_ki == 0.5f && motor_speed_ff() == 0.999f);          /* 부팅: K_P·K_I 0 = 권장값 */
    ui_param_text(P_DEV_TEST, 5, c);
    assert(memcmp(c, "TST 5", 5) == 0);
    assert(param_set(P_DEV_TEST, 1) != 0 && param_set(P_DEV_TEST, 3) != 0 && param_set(P_DEV_TEST, 9) != 0);
    assert(param_get(P_DEV_TEST) == 0);
    assert(param_set_ram(P_SPEED_FF, 300) == 0);
    for (k = 0; k < 2; k++) {                         /* K_P·K_I 파라미터 45·3, 0·0 */
        assert((param_set_ram(P_STEER_KP, (int16_t)(k ? 0 : 45)) == 0) && (param_set_ram(P_STEER_KI, (int16_t)(k ? 0 : 3)) == 0));
        for (t = 2; t >= 0; t -= 2) {
            assert(param_set(P_DEV_TEST, t) == 0);
            assert(param_tst_old == (t == 2) && param_get(P_OPT_FLAGS) == 0);
            assert(param_get(P_STEER_KP) == (k ? 0 : 45) && param_get(P_STEER_KI) == (k ? 0 : 3));   /* 파라미터 칸은 그대로 */
            assert(param_gain(P_STEER_KP) == (k ? (t ? 0 : 60) : 45) && D.p_kp == (float)param_gain(P_STEER_KP));
            assert(param_gain(P_STEER_KI) == (k ? (t ? 0 : 5) : 3) && D.p_ki == (float)param_gain(P_STEER_KI) * 0.1f);
            assert(motor_speed_ff() == (t ? 0.3f : 0.999f) && param_get(P_SPEED_FF) == 300);   /* FF 파라미터는 예전 루프만 */
            assert(!param_quirk(QUIRK_STEER_STEP) && param_get(P_LEGACY_QUIRKS) == lg);
        }
    }
    (void)param_set_ram(P_SPEED_FF, 0);
    assert(param_set(P_DEV_TEST, 2) == 0);
    ticks(100);
    assert(host.ee[82] == 2);
    boot_keys(0, 1);                                  /* 전원을 다시 켜도 2 */
    assert(param_get(P_DEV_TEST) == 2 && param_tst_old);
    for (k = 0; k < 2; k++) {                         /* 53차까지의 1·8 이 EEPROM 에 남아 있으면 부팅 때 0 */
        host.ee[82] = (uint16_t)(k ? 8 : 1);
        boot_keys(0, 1);
        assert(param_get(P_DEV_TEST) == 0 && !param_tst_old);
    }
    boot();
    assert(param_set(P_DEV_TEST, 2) == 0 && param_tst_old);
    param_tst_old = 0;                                /* 주행 시작 때 다시 건다(guide_poll, TST 0 이 아니면) */
    ticks(120);
    drive_up();
    run_scan(20);
    assert(D.flag_run && param_tst_old);
    (void)param_set(P_DEV_TEST, 0);
    printf("  TST ok\n");
}

/* 시퀀스 AGV_MOVE 이동 중 태그: ACK 는 보내고 동작(정지·속도)은 하지 않는다. 거리·정지 위치는 test_curve move 표(바퀴 모형) */
static void test_move_tag_ignore(void)
{
    int16_t cmd;
    int n0;
    boot();
    drive_up();
    run_scan(50);
    D.move_on = 3;                                    /* 이동 중으로 둔다 */
    cmd = D.speed_cmd;
    n0 = host.sci[HAL_SCI_A].tx_len;
    tag_frame(1, '1', 0);                             /* 정지 태그 */
    run_scan(2);
    tag_frame(2, '2', 0);                             /* 속도 태그(LLOW) */
    run_scan(2);
    assert(!D.flag_stopping && (D.speed_cmd == cmd) && (host.sci[HAL_SCI_A].tx_len > n0));   /* 무시, ACK 는 나감 */
    D.move_on = 0;
    tag_frame(1, '3', 0);
    run_scan(2);
    assert(D.flag_stopping);                          /* 이동이 끝나면 다시 태그대로 */
    printf("  MOVE_MM tag ignore ok\n");
}

/* 속도 조정 태그 'S'(실제 수신 경로 'E' 프레임). [2] 0 = 임시(RAM, 전원 재투입 때 저장된 LOS), 1 = LOS 저장,
 * 0S00 = 저장된 LOS 로 복원, 1S00 = 무시. 묶음: 0 은 SPN~DRS(rpm), 1 은 LOS 하한~DRS(m/min) */
static void s_tag(char sub, int num)
{
    char b[32];
    sprintf(b, "E%cA%02dS%03d00000000", sub, AGV_ID, num);
    send_frame(b, 0);
    run_scan(1);
}

static void test_s_tag(void)
{
    int16_t los0, lo, up;
    boot();
    drive_up();
    run_scan(50);
    assert(D.start_align == 0);                       /* 출발 정렬이 끝난 뒤(정렬 전에는 LLOW 상한이 지령을 누른다) */
    los0 = param_get(P_SPEED_LOW);
    param_limit(P_SPEED_LOW, &lo, &up);
    s_tag('0', 20);                                   /* 임시 */
    assert(P.low_speed == param_rpm(20) && D.speed_cmd == P.low_speed && param_get(P_SPEED_LOW) == los0);
    s_tag('0', 99);                                   /* DRS 로 묶임 */
    assert(P.low_speed == P.target_speed);
    s_tag('0', 1);                                    /* SPN 으로 묶임 */
    assert(P.low_speed == P.min_speed);
    s_tag('0', 0);                                    /* 0S00 복원 */
    assert(P.low_speed == param_rpm(los0) && param_get(P_SPEED_LOW) == los0);
    s_tag('1', 22);                                   /* 저장 */
    assert(param_get(P_SPEED_LOW) == 22 && P.low_speed == param_rpm(22) && D.speed_cmd == P.low_speed);
    s_tag('1', 0);                                    /* 1S00 무시 */
    assert(param_get(P_SPEED_LOW) == 22);
    s_tag('1', 50);                                   /* DRS 로 묶임 */
    assert(param_get(P_SPEED_LOW) == param_get(P_SPEED_HIGH));
    s_tag('1', 1);                                    /* LOS 하한으로 묶임 */
    assert(param_get(P_SPEED_LOW) == lo);
    s_tag('1', 21);
    s_tag('0', 15);                                   /* 임시로 바꾼 채 전원 재투입 */
    assert(P.low_speed == param_rpm(15));
    ticks(100);
    assert(host.ee[P_SPEED_LOW] == 21);
    boot_keys(0, 1);
    assert(param_get(P_SPEED_LOW) == 21 && P.low_speed == param_rpm(21));   /* 저장한 LOS 로 돌아온다 */
    printf("  S tag ok\n");
}

/* t3.9 WT FF 추천: 측정 직전 정속 0.5 s 의 두 바퀴 토크 지령 합 ÷ 지령 rpm 합 × 1000. 호스트 엔코더는 고정 주기라 토크 값은
 * 실차와 다르다(PI 적분이 쌓인 값). 창(마지막 WT_STEADY 50틱)과 식만 본다 */
static void test_wt_ff(void)
{
    static float tq[50], w[50];
    float st = 0.0f, sw = 0.0f;
    int i, k = 0;
    boot();
    ticks(120);
    drive_up();
    run_scan(20);
    wtest_start();
    for (i = 0; (i < 900) && (agv_wt.phase == 1); i++) {
        run_scan(1);
        tq[k] = (float)(D.tq_cmd[0] + D.tq_cmd[1]);
        w[k] = (float)(D.w_cmd[0] + D.w_cmd[1]);
        k = (k + 1) % 50;
        (void)wtest_run();
    }
    assert(agv_wt.phase == 2);
    for (i = 0; i < 50; i++) { st += tq[i]; sw += w[i]; }
    printf("  WT FF 추천 %d (토크 합 %.0f, rpm 합 %.0f)\n", agv_wt.ff, st, sw);
    assert((sw > 0.0f) && (agv_wt.ff == (int16_t)(st / sw * 1000.0f + 0.5f)));
    steer_test_rpm = 0;
    /* 자동 저장: 선 뒤(WT_STOP → 끝) 정상이고 0~999 면 FF 에 쓴다. 범위 밖·실패는 그대로 */
    D.flag_run = D.flag_ready = D.flag_stopping = 0;
    agv_wt.phase = 4; agv_wt.state = 2; agv_wt.ff = 1000;
    assert(wtest_run() == 0 && param_get(P_SPEED_FF) == 0 && agv_wt.ff_old == 0);
    agv_wt.phase = 4; agv_wt.state = 3; agv_wt.ff = 123;
    assert(wtest_run() == 0 && param_get(P_SPEED_FF) == 0);
    assert(param_get(P_DEV_TEST) == 0);              /* 기본 속도 루프(고정 FF)는 저장하지 않는다 */
    agv_wt.phase = 4; agv_wt.state = 2;
    assert(wtest_run() == 0 && param_get(P_SPEED_FF) == 0);
    assert(param_set_ram(P_DEV_TEST, 2) == 0);       /* 예전 루프(FF 파라미터)만 */
    agv_wt.phase = 4; agv_wt.state = 2;
    assert(wtest_run() == 0 && param_get(P_SPEED_FF) == 123 && agv_wt.ff_old == 0);
    ticks(50);
    assert(host.ee[117] == 123);
    (void)param_set(P_SPEED_FF, 0);
    (void)param_set_ram(P_DEV_TEST, 0);
    wtest_reset();
    printf("  WT FF ok\n");
}
#endif

static void ext_in(uint16_t bit, int on)
{
    if (on) { host.bus_in[BUS_EXT_IN_A] &= (uint16_t)~bit; }
    else    { host.bus_in[BUS_EXT_IN_A] |= bit; }
}

/* 적재·공차: LDS 0 이면 그대로, 센서가 켜지면 바로 적재, 서서 5 s 꺼져야 공차, 달리는 중에는 공차로 가지 않는다 */
static void test_load_profile(void)
{
    int16_t load_t, load_dcc;

    s_ext_board = 1;
    boot();
    (void)param_set(P_AGV_ID, 77);
    seq_pick_profile();
    assert(seq_load_mask() == ((1UL << IN_EXT_1) | (1UL << IN_EXT_2)));
    assert(!io_in(IN_EXT_1) && !io_in(IN_EXT_2));
    (void)param_set(P_EMPTY_SPEED, 45);
    (void)param_set(P_EMPTY_DCC, 30);
    load_t = P.target_speed;
    load_dcc = P.dcc;
    ticks(600);
    assert(!param_profile_empty() && P.target_speed == load_t && P.dcc == load_dcc);   /* LDS 0 */

    (void)param_set(P_LOAD_DETECT, 1);
    ticks(490);
    assert(!param_profile_empty());
    ext_in(0x02u, 1);                                 /* 4.9 s 에 센서: 다시 센다 */
    ticks(2);
    ext_in(0x02u, 0);
    ticks(490);
    assert(!param_profile_empty());
    ticks(15);
    assert(param_profile_empty() && P.target_speed == param_rpm(45) && P.dcc == 30 && P.acc == param_get(P_ACC));
    (void)param_set(P_EMPTY_SPEED, 0);                /* 0 = 적재 값 */
    assert(P.target_speed == load_t);
    (void)param_set(P_EMPTY_SPEED, 45);
    ext_in(0x01u, 1);                                 /* 바로 적재 */
    ticks(2);
    assert(!param_profile_empty() && P.target_speed == load_t && P.dcc == load_dcc);
    ext_in(0x01u, 0);

    /* 달리는 중에는 공차로 가지 않는다 */
    ticks(10);
    drive_up();
    run_scan(700);
    assert(D.flag_run && (D.vin_out > 0) && !param_profile_empty());
    run_command_stop();
    ticks(300);

    /* 현장 판정이 있으면 센서 묶음보다 앞선다 */
    ext_in(0x03u, 1);
    s_load_site = 0;
    ticks(510);
    assert(param_profile_empty());
    s_load_site = 1;
    ticks(2);
    assert(!param_profile_empty());
    s_load_site = -1;
    ext_in(0x03u, 0);

    /* 거리 정지: 지금 자리에서 mm 만큼. 서는 중에는 거리를 바꾸지 않는다 */
    boot();
    ticks(10);
    drive_up();
    run_scan(700);
    assert(D.flag_run && (D.vin_out > 0) && !D.flag_stopping);
    tagact_stop_after_mm(500);
    assert(D.flag_stopping && D.stop_pulses == (int16_t)(500.0f * P.one_length_pulse));
    tagact_stop_after_mm(100);
    assert(D.stop_pulses == (int16_t)(500.0f * P.one_length_pulse));
    run_command_stop();
    ticks(300);
    s_ext_board = 0;
    boot();
    printf("  load profile ok\n");
}

/* 달리는 중 적재로 바뀌면 지령을 적재 최고 속도로 낮춘다 */
static void test_load_while_running(void)
{
    int16_t load_t;

    s_ext_board = 1;
    boot();
    (void)param_set(P_AGV_ID, 77);
    seq_pick_profile();
    (void)param_set(P_EMPTY_SPEED, 45);
    (void)param_set(P_LOAD_DETECT, 1);
    load_t = (int16_t)param_rpm(param_get(P_SPEED_HIGH));
    ticks(520);
    assert(param_profile_empty());
    drive_up();
    run_scan(20);
    assert(param_profile_empty());
    D.speed_cmd = P.target_speed;
    D.step = DRIVE_CRUISE;
    D.vin_out = P.target_speed;
    ext_in(0x01u, 1);
    run_scan(1);
    assert(!param_profile_empty() && P.target_speed == load_t && D.speed_cmd == load_t);
    assert(D.step == DRIVE_CHANGE_SPEED && D.vin_out > load_t);
    ext_in(0x01u, 0);
    s_ext_board = 0;
    boot();
    printf("  load while running ok\n");
}

static void test_para_init(void)
{
    int i;

    boot();
    (void)param_set(P_SPEED_HIGH, 44);
    key(KEY_MODE);
    key(KEY_MODE);
    ticks(40);
    assert(D.ui_mode == UI_PARAM);
    key(KEY_SET);
    for (i = 0; i < 4; i++) { key(KEY_UP); }          /* P5INF */
    key(KEY_SET);
    for (i = 0; (i < 40) && (ui_para_number() != P_PARA_INIT); i++) { key(KEY_UP); }
    assert(ui_para_number() == P_PARA_INIT);
    key(KEY_SET);
    /* 값 88, 99 는 아무 일도 하지 않는다 */
    key(KEY_SHIFT);
    for (i = 0; i < 8; i++) { key(KEY_UP); }
    key(KEY_SHIFT);
    for (i = 0; i < 8; i++) { key(KEY_UP); }
    key(KEY_SET);
    ticks(10);
    assert(param_get(P_SPEED_HIGH) == 44 && !shows("INIT"));
    for (i = 0; i < 11; i++) { key(KEY_UP); }         /* 88 → 99 */
    key(KEY_SET);
    ticks(10);
    assert(param_get(P_SPEED_HIGH) == 44 && !shows("INIT"));
    /* 값 77 = 기본값 한 벌 */
    key(KEY_SHIFT);
    for (i = 0; i < 2; i++) { key(KEY_DOWN); }
    key(KEY_SHIFT);
    for (i = 0; i < 2; i++) { key(KEY_DOWN); }
    key(KEY_SET);
    assert(param_get(P_SPEED_HIGH) == 30);
    ticks(100);
    assert(host.ee[P_SPEED_HIGH] == 30);
    printf("  77 ok\n");
}

static void test_lamp(void)
{
    boot();
    ticks(60);
    assert(!(host.bus_out[BUS_LAMP] & 0x01));         /* 초록 켜짐 (저활성) */
    D.stop_err = ERR_BRANCH_TIMEOUT;
    ticks(60);
    assert(!(host.bus_out[BUS_LAMP] & 0x02));         /* 빨강 */
    assert(host.bus_out[BUS_LAMP] & 0x01);
    printf("  lamp ok\n");
}

static void test_boot_garbage(void)
{
    /* 리셋 직후 쓰레기 값으로 차가 움직이면 안 된다 */
    host_reset();
    memset((void *)&D, 0xA5, sizeof(D));
    sys_boot();
    ticks(200);
    assert(!D.flag_run && !D.flag_ready && !D.flag_jog_on && !D.flag_turn);
    assert(host.dac[0] == 0 && host.dac[1] == 0);
    printf("  boot garbage ok\n");
}

/* 시험 결과는 리셋에 남는다. 표시가 틀리면(전원 투입) 지운다. 진행 중이던 결과는 끊김으로 둔다 */
static void test_keep_results(void)
{
    test_keep_mark = 0;
    agv_ident.state = 2;
    agv_ident.track_mm = 5000.0f;
    agv_coast.state = 2;
    agv_stopt.state = 2;
    boot();
    assert(agv_stopt.state == 0);
    assert(agv_ident.state == 0 && agv_ident.track_mm == 0.0f && test_keep_mark != 0);
    assert(agv_coast.state == 0);
    agv_ident.state = 2;
    agv_ident.track_mm = 5000.0f;
    agv_top.state = 1;
    agv_all.state = 1;
    agv_all.fail = 2034;
    agv_coast.state = 1;
    agv_rpmt.state = 1;
    agv_stopt.state = 1;
    agv_stopt.p[5].overshoot_mm = 150.0f;
    boot();
    assert(agv_ident.state == 2 && agv_ident.track_mm == 5000.0f && agv_ident.fail == 0);
    assert(agv_top.state == 3 && agv_top.fail == -1);
    assert(agv_all.state == 3 && agv_all.fail == 2034);
    assert(agv_coast.state == 3 && agv_coast.fail == -1);
    assert(agv_rpmt.state == 3 && agv_rpmt.fail == -1);
    assert(agv_stopt.state == 3 && agv_stopt.fail == -1 && agv_stopt.p[5].overshoot_mm == 150.0f);
    /* 끝난 바닥 rpm 결과는 그대로 */
    agv_rpmt.state = 2;
    agv_rpmt.fail = 0;
    agv_rpmt.amp = 'S';
    agv_rpmt.brk.move_dac[1] = 120;
    agv_rpmt.stp[RPMT_STOPS - 1].dist_mm = 12.5f;
    agv_rpmt.stp[RPMT_STOPS - 1].trace[RPMT_TRACE - 1] = 61;
    boot();
    assert(agv_rpmt.state == 2 && agv_rpmt.fail == 0 && agv_rpmt.amp == 'S' && agv_rpmt.brk.move_dac[1] == 120);
    assert(agv_rpmt.stp[RPMT_STOPS - 1].dist_mm == 12.5f && agv_rpmt.stp[RPMT_STOPS - 1].trace[RPMT_TRACE - 1] == 61);
    test_keep_mark = 0;                    /* 전원 투입: 지운다 */
    boot();
    assert(agv_rpmt.state == 0 && agv_rpmt.stp[RPMT_STOPS - 1].dist_mm == 0.0f);
    printf("  keep results ok\n");
}

static void test_acs_off(void)
{
    boot();
    (void)param_set(P_ACS_RETRY_LEGACY, 0);
    host.sci[HAL_SCI_A].tx_len = 0;
    acs_report('L', 3);
    ticks(50);
    assert(host.sci[HAL_SCI_A].tx_len == 0);
    tag_frame(2, '1', 0);                 /* 태그 ACK 는 계속 나가고 태그 동작도 한다 */
    run_scan(1);
    assert(tx_has('A', 'E'));
    printf("  acs off ok\n");
}

/* 'h' 응답 하나 */
static void guide_led_reply(uint16_t leds)
{
    static const char hex[] = "0123456789abcdef";
    host_sci_t *b = &host.sci[HAL_SCI_B];
    uint16_t f[8], sum = 0;
    int i;

    f[0] = 2;
    f[1] = 'h';
    for (i = 0; i < 4; i++) { f[2 + i] = (uint16_t)hex[(leds >> (i * 4)) & 0xFu]; }
    for (i = 1; i <= 5; i++) { sum = (uint16_t)(sum + f[i]); }
    f[6] = (uint16_t)hex[sum & 0xFu];
    f[7] = 3;
    for (i = 0; i < 8; i++) { b->rx[b->rx_len++] = f[i]; }
}

static int guide_tx_has(uint16_t c)
{
    host_sci_t *b = &host.sci[HAL_SCI_B];
    int i;
    for (i = 0; i + 3 < b->tx_len; i++) {
        if ((b->tx[i] == 2) && (b->tx[i + 1] == c)) { return 1; }
    }
    return 0;
}

static void turn_scan(uint16_t leds, int32_t pulses)
{
    int32_t i;
    for (i = 0; i < pulses; i++) { run_encoder(0, 20000); }
    guide_led_reply(leds);
    tick();
}

#define LED_CENTER 0x0180u     /* 8·9번 칸: 위치 0 */
#define LED_EDGE   0x0003u     /* 1·2번 칸: 위치 -14 */
#define LED_NEAR   0x0030u     /* 5·6번 칸: 위치 -6 */
#define LED_OVER2  0x0300u     /* 9·10번 칸: 위치 +2 */

/* 가장자리 고정은 달린 거리 GUIDE_EDGE_LIMIT_MM 까지 유지되고 그 뒤 풀린다. 속도(vin_out)는 모터 rpm 이다 */
static void test_edge_release(void)
{
    float mm = 0.0f;
    int k, n = 0;

    boot();
    assert(param_set_ram(P_LEGACY_QUIRKS, 0) == 0);
    drive_up();
    for (k = 0; k < 400; k++) {
        host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;
        guide_led_reply((k == 399) ? (LED_CENTER | 0x3000u) : LED_CENTER);     /* 마지막에 떨어진 둘째 덩어리 */
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
    }
    assert(D.guide_track == GUIDE_TRK_NORMAL && D.guide_pos == 0 && D.vin_out > 1000);
    for (k = 0; k < 300; k++) {
        host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;
        guide_led_reply(0x1F80u);                                              /* 8~13번 칸: 주행선에 붙었다 */
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
        if (D.guide_track != GUIDE_TRK_EDGE) { break; }
        assert(D.guide_pos == 0);
        mm += (float)D.vin_out * P.one_rpm_meter / 6.0f;
        n++;
    }
    printf("  가장자리 고정 %d 틱, %.0f mm\n", n, mm);
    assert((n > 20) && (mm > (float)GUIDE_EDGE_LIMIT_MM - 10.0f) && (mm < (float)GUIDE_EDGE_LIMIT_MM + 10.0f));
    assert(D.guide_track == GUIDE_TRK_NORMAL && D.guide_pos == 4);              /* 풀리면 덩어리 가운데 */
}

static void align_start(uint16_t next)
{
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    D.vin_out = 50;
    D.flag_run = 0;
    tagact_turn_align(TURN_CW, 90, next);
    assert(D.tape_align == 1);
    stop_reset();                   /* 회전 전 정지는 끝난 것으로 본다 */
    D.flag_stopping = 0;
    D.stop_pulses = 0;
    D.vin_out = 0;
    D.flag_turn = 1;
    D.control = RUN_TURN;
    D.guide_all_off = 1;
}

/* 태그 회전을 걸고 회전 상태로 넘긴다(정지는 끝난 것으로 본다) */
static void tag_turn_start(uint16_t tag)
{
    D.vin_out = 50;
    D.flag_run = 0;
    (void)tagact_run(tag);
    assert(D.turn_pulses > 0);
    stop_reset();
    D.flag_stopping = 0;
    D.stop_pulses = 0;
    D.vin_out = 0;
    D.flag_turn = 1;
    D.control = RUN_TURN;
}

/* 회전 틱을 돌려 가장 큰 지령을 돌려준다. stop_at = 1 이면 중간에 STOP 명령 */
static int16_t tag_turn_run(int stop_mid)
{
    int16_t vmax = 0;
    int i;
    for (i = 0; (i < 5000) && (D.flag_turn || D.turn_motion); i++) {
        if (stop_mid && (i == 100)) { run_command_stop(); }
        turn_scan(LED_CENTER, (D.turn_pulses > 0) ? 4 : 0);   /* 다 돌면 바퀴가 선다 */
        if (D.vin_out > vmax) { vmax = D.vin_out; }
    }
    assert(!D.flag_turn && (D.turn_motion == 0));
    return vmax;
}

/* 태그 회전 속도: 지정값이 회전 한 번의 상한, 공차·최저·한계, 끝·STOP 뒤 자동 */
static void test_turn_speed(void)
{
    int16_t v15, v;

    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    v15 = param_rpm(15);
    assert(P.turn_speed != v15);
    tag_turn_start(30);
    assert(D.turn_rpm == v15 && stop_turn_vmax() == v15);
    (void)param_set(P_EMPTY_TURN, 10);               /* 공차 ETN 10 으로 바뀌어도 지정값 */
    param_profile(1);
    printf("  공차 회전 속도 %d rpm (ETN 10 = %d)\n", P.turn_speed, param_rpm(10));
    assert(P.turn_speed == param_rpm(10) && stop_turn_vmax() == v15);
    v = tag_turn_run(0);
    printf("  태그 회전 15 m/min: 상한 %d rpm, 지령 최대 %d rpm\n", v15, v);
    assert((v <= v15) && (v > v15 - 30));
    assert(D.turn_rpm == 0);                         /* 끝나면 자동 */
    param_profile(0);
    tag_turn_start(33);                              /* 다음 자동 회전은 파라미터 */
    assert(D.turn_rpm == 0 && stop_turn_vmax() == P.turn_speed);
    (void)tag_turn_run(0);

    tag_turn_start(31);                              /* 1 m/min: 최저 속도로 */
    assert(D.turn_rpm == P.turn_min);
    (void)tag_turn_run(0);
    tag_turn_start(32);                              /* 31 m/min: TNS 한계(25)로 */
    assert(D.turn_rpm == param_rpm(25));
    (void)tag_turn_run(0);

    tag_turn_start(30);                              /* 회전 중 STOP: 다음 회전에 남지 않는다 */
    (void)tag_turn_run(1);
    assert(D.turn_rpm == 0);
    tag_turn_start(33);
    assert(stop_turn_vmax() == P.turn_speed);
    printf("  turn speed ok\n");
}

/* 시퀀스 회전을 걸고 회전 상태로 넘긴다(서 있다 시퀀스가 회전을 건다) */
static void seq_turn_start(uint16_t id)
{
    int i;
    D.vin_out = 0;
    D.flag_run = 0;
    seq_request(id);
    for (i = 0; (i < 5) && (D.turn_pulses <= 0); i++) { seq_run(); }
    assert(D.turn_pulses > 0 && seq_is_active());
    D.flag_turn = 1;
    D.control = RUN_TURN;
}

/* 시퀀스 회전 속도(v0.50.0): 태그와 같은 사양. 지정 15 → 상한 15 m/min, 생략 → 자동, 다음 회전에 남지 않는다 */
static void test_seq_turn_speed(void)
{
    int16_t v15 = param_rpm(15), v;
    int i;

    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    assert(P.turn_speed != v15);
    seq_turn_start(SEQ_UNLOAD);
    assert(D.turn_rpm == v15 && stop_turn_vmax() == v15);
    v = tag_turn_run(0);
    printf("  시퀀스 회전 15 m/min: 상한 %d rpm, 지령 최대 %d rpm\n", v15, v);
    assert((v <= v15) && (v > v15 - 30));
    assert(D.turn_rpm == 0);
    for (i = 0; (i < 5) && seq_is_active(); i++) { seq_run(); }   /* 회전이 끝나면 다음 스텝(END_STOP) */
    assert(!seq_is_active());
    seq_turn_start(SEQ_BOX_LOAD);                     /* 생략: 자동 */
    assert(D.turn_rpm == 0 && stop_turn_vmax() == P.turn_speed);
    (void)tag_turn_run(0);
    for (i = 0; (i < 5) && seq_is_active(); i++) { seq_run(); }
    assert(!seq_is_active());
    printf("  seq turn speed ok\n");
}

/* 태그 대기 보고: 번호(호기 - n), 스테이션 큐 지움(뒤·앞), 정지. 도착 보고 번호 = 스테이션 큐[1] */
static void test_tag_standby(void)
{
    boot();
    acs_station_set(0, 101, 2);
    acs_station_set(1, 202, 2);
    D.vin_out = 50;
    host.sci[HAL_SCI_A].tx_len = 0;
    (void)tagact_run(36);
    ticks(2);
    assert(tx_has('E', 'A'));
    (void)tagact_run(34);
    assert(run_standby_num() == AGV_ID - 2 && D.flag_stopping && acs_station_count() == 0);
    run_standby_clear(0);
    acs_station_set(0, 101, 1);
    (void)tagact_run(35);
    assert(run_standby_num() == 14 && acs_station_count() == 0);
    ticks(2);
    assert(tx_has('E', 'B'));
    run_standby_clear(0);
    /* 자동 재전송(1분)은 번호가 0 보다 클 때만: 34 번 = 호기 - 2 가 0 이면 다시 보내지 않는다 */
    for (int k = 0; k < 2; k++) {
        int resent = 0;
        (void)param_set_ram(P_AGV_ID, (int16_t)(2 + k));
        (void)tagact_run(34);
        assert(run_standby_num() == k);
        acs_slot_clear(ACS_SLOT_STANDBY);                /* 첫 보고는 받은 셈 */
        for (int t = 0; t < 6100; t++) {
            host.sci[HAL_SCI_A].tx_len = 0;
            tick();
            resent |= tx_has('E', 'B');
        }
        assert(resent == k);
        acs_slot_clear(ACS_SLOT_STANDBY);
        run_standby_clear(0);
    }
    (void)param_set_ram(P_AGV_ID, AGV_ID);
    printf("  tag standby ok\n");
}

/* 달리는 중 태그 규칙 하나를 실행한다 */
static void tag_stop_rule(uint16_t tag)
{
    boot();
    drive_up();
    run_scan(20);
    (void)tagact_run(tag);
}

/* 태그 정지 4분류. 분류 3·4 의 찾기·정지는 run.c 몫이라 여기서는 태그 해석기가 넣는 값만 본다 */
static void test_tag_stop_class(void)
{
    /* 분류 1 대기 보고는 급정지(위치 정지 아님) */
    tag_stop_rule(34);
    guide_reply(0);
    tick();
    assert(D.flag_stopping && !stop_is_position(stop_reason()));
    run_standby_clear(0);

    /* 분류 2: 태그 자리에서 30 cm(SL 과 같은 cm × 10 mm 환산). 대기 보고는 보고만, 시퀀스는 바로 요청 */
    tag_stop_rule(37);
    assert(D.flag_stopping && (D.stop_pulses == (int16_t)(30.0f * P.one_length_pulse * 10.0f)));
    assert((run_standby_num() == 7) && seq_request_flag[SEQ_LOAD] && (D.tape_stop_kind == 0));
    guide_reply(0);
    tick();
    assert(stop_is_position(stop_reason()));
    run_standby_clear(0);
    run_seq_init();                              /* 요청한 시퀀스를 지운다(부팅은 남긴다) */

    /* 별칭(사용자 함수가 tagact_run 을 다시 부름): 안쪽 규칙의 분류 2 거리(30 cm)를 바깥 호출이 SL 로 덮지 않는다 */
    tag_stop_rule(46);
    assert(D.flag_stopping && (D.stop_pulses == (int16_t)(30.0f * P.one_length_pulse * 10.0f)) && (P.stop_len != D.stop_pulses));

    /* 분류 2, 0 cm = 바로 정지. 앞의 AGV_STOP(SL) 을 이긴다 */
    tag_stop_rule(38);
    assert(D.flag_stopping && (D.stop_pulses == 0));

    /* 분류 3: 태그 자리에서 서지 않는다. 자동 출발은 태그 때 걸지 않고 보관한다(5 초 + 1. 교차선 정지를 건 때 run.c 가 건다) */
    tag_stop_rule(39);
    assert(!D.flag_stopping && (D.tape_stop_kind == TA_STOP_KIND_TAPE) && (D.tape_stop_id == 0) && !D.wait_start);
    assert(D.tape_autostart == 6u);

    /* 분류 4: 같은 규칙의 SEQ_START( 시퀀스 ) 는 교차선 정지 뒤 시퀀스, 대기 보고는 정지 없이 */
    tag_stop_rule(40);
    assert(!D.flag_stopping && (D.tape_stop_kind == TA_STOP_KIND_TAPE_GD) && (D.tape_stop_id == SEQ_LOAD));
    assert(!seq_request_flag[SEQ_LOAD] && (run_standby_num() == 7));
    run_standby_clear(0);

    tag_stop_rule(41);
    assert(!D.flag_stopping && (D.tape_stop_kind == TA_STOP_KIND_TAPE_GD) && (D.tape_stop_id == SEQ_LOAD));
    {   /* 끝에서 끝까지: 교차선을 보면 GD 정지, 서면 시퀀스, 기록 run_tape_last(분류 4) */
        int i;
        uint16_t n0 = run_tape_last.n;
        for (i = 0; (i < 20) && (D.tape_stop_kind != 0); i++) { guide_led_reply(0x03FFu); run_encoder(0, 20000); tick(); }
        assert((D.tape_stop_kind == 0) && D.flag_stopping && (D.stop_pulses > 0) && !D.system_err);
        for (i = 0; (i < 20000) && D.flag_run; i++) {   /* 지령이 0 이면 바퀴도 선다(지나침 60 mm 넘으면 258) */
            if (D.vin_out > 0) { run_scan(1); }
            else { guide_reply(0); tick(); }
        }
        for (i = 0; (i < 50) && !seq_is_active(); i++) { guide_reply(0); tick(); }
        assert(!D.flag_run && seq_is_active() && (run_tape_last.n == (uint16_t)(n0 + 1u)) && (run_tape_last.kind == 4));
        run_seq_init();
    }

    /* 끝에서 끝까지: SEQ_START( 30, LOAD ) 는 태그 자리에서 30 cm 가서 서고 시퀀스 */
    tag_stop_rule(37);
    {
        int i;
        uint32_t p0 = run_pulses(0);
        int32_t want = (int32_t)(30.0f * P.one_length_pulse * 10.0f);
        for (i = 0; (i < 2000) && (D.stop_pulses > 0); i++) { run_scan(1); }   /* 정지 거리를 다 갈 때까지(스캔당 1 펄스) */
        {
            int32_t d = (int32_t)(run_pulses(0) - p0) - want;
            assert((d >= -3) && (d <= 3));
        }
        for (i = 0; (i < 2000) && D.flag_run; i++) { guide_reply(0); tick(); }   /* 바퀴가 서고 정지가 끝난다 */
        for (i = 0; (i < 50) && !seq_is_active(); i++) { guide_reply(0); tick(); }
        assert(!D.flag_run && seq_is_active());
        run_standby_clear(0);
        run_seq_init();
    }

    /* 분류 3 이 2 를 이긴다(앞뒤 순서와 무관), AGV_STOP 도 서지 않는다 */
    tag_stop_rule(42);
    assert(!D.flag_stopping && (D.tape_stop_kind == TA_STOP_KIND_TAPE) && (D.tape_stop_id == SEQ_LOAD));

    /* 진행 중인 정지가 있어도 교차선 정지 요청은 AGV_STOP 처럼 받는다(정지 거리는 그대로. 보류는 run.c) */
    boot();
    drive_up();
    run_scan(20);
    tag_frame(1, '1', 0);                        /* AGV_STOP */
    run_scan(1);
    assert(D.flag_stopping && (D.stop_pulses > 0));
    {
        int16_t len = D.stop_pulses;
        (void)tagact_run(41);
        assert((D.tape_stop_kind == TA_STOP_KIND_TAPE_GD) && (D.tape_stop_id == SEQ_LOAD) && (D.stop_pulses == len));
    }
    D.tape_stop_kind = 0;
    D.tape_stop_id = 0;

    /* 교차선 정지 + 자동 출발: 태그 자리에서 서지 않고 교차선에서 선다. 자동 출발은 보관(1 초 + 1)했다가 교차선 정지를 건 때
     * 건다(주행 쪽 2차 패치). 선 뒤에는 분류 1 의 SEQ_START( x ) + 자동 출발과 같다(재출발 대기 뒤 시퀀스, 재출발은 시퀀스 뒤) */
    tag_stop_rule(44);
    assert(!D.flag_stopping && !D.wait_start && (D.tape_autostart == 2u));
    assert((D.tape_stop_kind == TA_STOP_KIND_TAPE) && (D.tape_stop_id == SEQ_LOAD));
    {
        int i;
        for (i = 0; (i < 10) && !D.flag_stopping; i++) { guide_led_reply(0x03FFu); tick(); }
        assert(D.flag_stopping && (D.stop_pulses == 0) && seq_request_flag[SEQ_LOAD] && D.wait_start && !D.tape_autostart);
        for (i = 0; (i < 400) && !D.run_restart && !(D.flag_run && !D.flag_stopping); i++) { guide_reply(0); tick(); }
        assert((i < 400) && !D.flag_run && seq_request_flag[SEQ_LOAD]);
        for (i = 0; (i < 50) && !seq_is_active(); i++) { guide_reply(0); tick(); }
        assert(seq_is_active() && D.run_restart && !D.flag_run);   /* 시퀀스가 돌고, 재출발은 시퀀스 뒤로 미뤄진다 */
        run_seq_init();
    }

    /* 분류 3 + 자동 출발(시퀀스 없음): 교차선에서 서고 AS(5 초) 뒤 실제로 다시 떠난다 */
    tag_stop_rule(39);
    {
        int i, stopped = 0;
        for (i = 0; (i < 10) && !D.flag_stopping; i++) { guide_led_reply(0x03FFu); tick(); }
        assert(D.flag_stopping && D.wait_start && !D.tape_autostart);
        for (i = 0; (i < 1000) && !(stopped && D.flag_run); i++) {
            guide_reply(0);
            tick();
            stopped |= !D.flag_run;
        }
        assert(stopped && (i > 400) && (i < 1000));
    }

    /* 교차선 정지 + 대기 보고 + 자동 출발: 보고는 태그 자리, 정지는 교차선, 자동 출발은 보관 */
    tag_stop_rule(45);
    assert(!D.flag_stopping && !D.wait_start && (D.tape_autostart == 2u) && (run_standby_num() == 7) && (D.tape_stop_kind == TA_STOP_KIND_TAPE));
    run_standby_clear(0);

    D.tape_stop_kind = 0;
    boot();
    printf("  tag stop class ok\n");
}

/* 교차선 회전 요청: 일반 회전을 걸지 않는 상황(거리 정지 진행 중, 서 있음, 회전 모드 0)이면 아무 일도 없다.
 * 거리 없는 감속(명령 정지) 중에는 건다. 요청이 걸려 교차선을 찾는 동안 읽은 태그는 회전 중과 같이 ACK 없이 버린다 */
static void test_tape_turn_request(void)
{
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    drive_up();
    run_scan(20);
    D.stop_pulses = 100;
    D.flag_stopping = 1;                         /* 거리 정지 진행 중 */
    (void)tagact_run(43);
    assert(D.tape_turn_dir == 0);
    D.stop_pulses = 0;                           /* 거리 없는 감속(명령 정지) 중: 건다(레거시는 SL 정지 뒤 회전) */
    (void)tagact_run(43);
    assert((D.tape_turn_dir == TURN_CW) && (D.stop_pulses == 0));
    D.tape_turn_dir = 0;

    boot();                                      /* 서 있음 */
    (void)param_set(P_TURN_FORBACK, 10);
    (void)tagact_run(43);
    assert(D.tape_turn_dir == 0);

    boot();                                      /* 회전 모드 0 */
    (void)param_set(P_TURN_FORBACK, 0);
    drive_up();
    run_scan(20);
    (void)tagact_run(43);
    assert(D.tape_turn_dir == 0);

    /* 달리는 중이면 건다. 찾는 동안 정지 태그는 ACK 없이 버린다 */
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    drive_up();
    run_scan(20);
    (void)tagact_run(43);
    assert((D.tape_turn_dir == TURN_CW) && (D.tape_turn_deg == 90) && !D.flag_stopping);
    host.sci[HAL_SCI_A].tx_len = 0;
    tag_frame(1, '2', 0);
    run_scan(1);
    assert(!tx_has('A', 'E') && !D.flag_stopping && (D.tape_turn_dir == TURN_CW));

    /* ACS 교통 정지로 서 있는 동안: 요청은 남고(보류), 같은 자리 태그를 다시 읽어도 버린다. 다시 출발해도 요청은 남는다 */
    {
        char b[32];
        int i;
        sprintf(b, "W1B%02dC00000000000", AGV_ID);
        b[8] = '0';                                   /* 'C' '0': 교통 정지(교차 대기 없음 = 명령 정지) */
        send_frame(b, 0);
        for (i = 0; (i < 400) && D.flag_run; i++) { guide_reply(0); tick(); }
        assert(!D.flag_run && (D.tape_turn_dir == TURN_CW));
        host.sci[HAL_SCI_A].tx_len = 0;
        tag_frame(43, '4', 0);
        guide_reply(0);
        tick();
        assert(!tx_has('A', 'E') && (D.tape_turn_dir == TURN_CW));
        b[8] = '1';                                   /* 'C' '1': 출발 */
        send_frame(b, 0);
        for (i = 0; (i < 400) && !D.flag_ready && !D.flag_run; i++) { guide_reply(0); tick(); }
        assert((D.flag_ready || D.flag_run) && (D.tape_turn_dir == TURN_CW));
    }

    /* 180도 + 회전 뒤 방향이 지금(전진)과 반대: 회전을 걸 수 없는 조합이라 교차선 요청도 걸지 않는다 */
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    drive_up();
    run_scan(20);
    (void)tagact_run(48);
    assert((D.tape_turn_dir == 0) && !D.backward);

    /* 조그로 움직이며 읽은 교차선 태그(회전·분류 4)와 서 있는 자동 차가 다시 읽은 태그는 요청이 되지 않는다 */
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    D.control = RUN_JOG;
    D.flag_run = 1;
    D.vin_out = 50;
    (void)tagact_run(43);
    (void)tagact_run(41);
    assert((D.tape_turn_dir == 0) && (D.tape_stop_kind == 0) && (D.tape_stop_id == 0));
    boot();
    drive_up();
    run_scan(20);
    D.flag_run = 0;                                  /* 섰다 */
    D.vin_out = 0;
    (void)tagact_run(41);
    (void)tagact_run(39);
    assert((D.tape_stop_kind == 0) && (D.tape_stop_id == 0));
    boot();
    printf("  tape turn request ok\n");
}

/* 시퀀스 이동·교차선 회전 앞 찾기 중 교차로 IN·OUT 태그: 버리지 않고 서고 오류 290. 다른 태그는 버린다 */
static void test_move_cross_tag(void)
{
    char b[32];
    int k;

    for (k = 0; k < 3; k++) {
        boot();
        (void)param_set(P_TURN_FORBACK, 10);
        drive_up();
        run_scan(20);
        if (k < 2) { D.move_on = 3; }                 /* 시퀀스 이동 중 */
        else       { (void)tagact_run(43); assert(D.tape_turn_dir != 0); }   /* 교차선 회전 앞 찾는 중 */
        host.sci[HAL_SCI_A].tx_len = 0;
        tag_frame(2, '1', 0);                          /* 속도 태그는 버린다 */
        run_scan(1);
        assert((D.stop_err == 0) && !D.flag_stopping && !D.cross_in_flag);
        assert((k < 2) == (tx_has('A', 'E') != 0));   /* 이동 중은 ACK, 찾는 중은 회전 중처럼 ACK 없음 */
        sprintf(b, (k == 1) ? "E1B%02dO00700000000" : "E1B%02dI01200000000", AGV_ID);
        send_frame(b, 0);
        run_scan(1);
        assert((D.stop_err == SEQ_ERR_CROSS) && D.flag_stopping && (D.stop_pulses == 0));
        assert((D.tape_turn_dir == 0) && tx_has('A', 'E'));
        /* 보고는 한다: IN 은 ACS 로 나가고, OUT 은 ACK 까지 큐에 남는다 */
        if (k == 1) { assert(acs_cross_out_count() == 1); }
        else {
            int i, sent = 0;
            for (i = 0; (i < 5) && !sent; i++) { sent = tx_has('E', 'I'); guide_reply(0); tick(); }
            assert(sent && D.cross_in_flag);
            run_reset_request();                       /* 리셋 키: 오류를 풀어도 허가 대기는 남아 START 뒤 교차로 대기 거리에서 선다 */
            for (i = 0; i < 3; i++) { guide_reply(0); tick(); }
            assert((D.stop_err == 0) && D.cross_in_flag);
        }
        D.move_on = 0;
    }
    boot();
    printf("  move cross tag ok\n");
}

/* 정렬 회전: 'h' 로 슬립을 재고, 목표각 근방에서 라인을 찾아 중앙에서 끝낸다 */
static void test_align_turn(void)
{
    int32_t w;
    int i, total;

    align_start(0x10);
    total = D.tape_total_pulse;
    w = D.tape_max;
    assert(total > 0);
    host.sci[HAL_SCI_B].tx_len = 0;
    turn_scan(LED_CENTER, 0);
    assert(guide_tx_has('H'));
    assert(D.guide_all_off == 0);
    assert(D.turn_pulses == total);                    /* 펄스는 따로 센다 */
    turn_scan(LED_EDGE, total / 4);
    assert(D.tape_turned == total / 4);
    assert(D.turn_pulses == total);
    assert(D.tape_step == 0);
    for (i = 0; i < 7; i++) { turn_scan(0, 0); }       /* 출발 라인 벗어남 */
    assert(D.tape_step == 1);
    assert(D.stop_err == 0);                           /* 사라져도 오류가 아니다 */

    /* 목표각 밖의 라인은 무시한다 */
    D.tape_target_cnt = (float)total;
    turn_scan(LED_CENTER, 0);
    assert(D.tape_step == 1);
    turn_scan(0, total - D.tape_turned - w + 1);
    turn_scan(LED_EDGE, 0);
    assert(D.tape_step == 1);                          /* 위치 14 는 아직 멀다 */
    turn_scan(LED_NEAR, 0);
    assert(D.tape_step == 2);
    turn_scan(LED_NEAR, 0);
    assert(D.tape_align == 1);
    turn_scan(LED_CENTER, 0);                          /* 중앙: 그 틱에 지령 0 으로 선다. 회전은 아직 안 끝났다 */
    assert(D.tape_step == TAPE_STEP_SETTLE && D.vin_out == 0 && D.w_cmd[0] == 0 && D.w_cmd[1] == 0);
    assert(D.tape_align == 1 && D.flag_turn == 1 && D.turn_pulses == total && D.turn_motion == MOTION_CW);
    assert(D.tape_side == -1);                         /* 새 라인은 - 쪽에서 왔다 */
    {   /* 선 뒤 +2 로 지나쳤다: 정착 0.2 s 뒤 반대 방향(CCW)으로 최저 속도 보정, 중앙에서 다시 선다 */
        int32_t t0 = D.tape_turned;
        for (i = 0; i < 19; i++) { turn_scan(LED_OVER2, 0); }
        assert(D.tape_step == TAPE_STEP_SETTLE && D.vin_out == 0);
        turn_scan(LED_OVER2, 0);
        assert(D.tape_step == TAPE_STEP_FIX && D.tape_fix_rev == 1 && D.tape_fix_cnt == 1 && D.tape_stop_pos == 2);
        turn_scan(LED_OVER2, 30);
        assert(D.motion == MOTION_CCW && D.vin_out > 0 && D.vin_out <= P.turn_speed);
        assert(D.tape_turned == t0 && D.tape_fix_turned == 30);   /* 보정은 회전 보정비 측정에 안 들어간다 */
        assert(D.flag_turn == 1 && D.turn_pulses == total);
        turn_scan(LED_CENTER, 0);
        assert(D.tape_step == TAPE_STEP_SETTLE && D.vin_out == 0 && D.tape_align == 1);
        for (i = 0; i < 19; i++) { turn_scan(LED_CENTER, 0); }
        assert(D.tape_align == 1);
        turn_scan(LED_CENTER, 0);
        assert((D.tape_align == 0) && (D.turn_pulses == 0));
        assert(D.tape_fix_end == 0 && D.tape_fix_cnt == 1 && D.tape_final_pos == 0);
    }
    assert(D.system_err == 0);
    for (i = 0; i < 40; i++) { turn_scan(LED_CENTER, 0); }   /* 정지 처리가 회전을 끝낸다 */
    assert(D.flag_turn == 0 && D.turn_motion == 0);

    /* 라인을 못 찾으면 오류로 끝낸다 */
    align_start(0x10);
    total = D.tape_total_pulse;
    w = D.tape_max;
    D.tape_step = 1;
    turn_scan(0, total + w + 1);
    assert((D.tape_align == 0) && (D.turn_pulses == 0));
    assert(D.system_err == ERR_GUIDE_SENSOR);

    /* 라인이 안 사라져도 찾기로 넘어간다 */
    align_start(0x10);
    total = D.tape_total_pulse;
    w = D.tape_max;
    turn_scan(LED_EDGE, 0);
    turn_scan(LED_EDGE, total - w);
    assert(D.tape_step == 1);

    /* 방향이 바뀌는 회전은 슬립 측정을 건너뛴다 */
    align_start(0x13);
    assert(D.tape_step == 1);
    printf("  align turn ok\n");
}

/* ---- 튜닝 모드 ---- */

static uint16_t glyph(char c)
{
    if ((c >= '0') && (c <= '9')) { return (uint16_t)(c - '0'); }
    if ((c >= 'A') && (c <= 'Z')) { return (uint16_t)(11 + (c - 'A')); }
    return (c == '-') ? 37u : 39u;
}

/* FND 5자리가 t 와 같다 */
static int shows(const char *t)
{
    uint16_t d[5];
    int i;
    ui_digits(d);
    for (i = 0; i < 5; i++) {
        if (d[i] != glyph(t[i])) { return 0; }
    }
    return 1;
}

/* keys 를 누른 채 전원을 켠다. EEPROM 은 keep 이면 그대로 둔다 */
static void boot_keys(uint16_t keys, int keep)
{
    uint16_t ee[256];
    memcpy(ee, host.ee, sizeof(ee));
    host_reset();
    if (keep) { memcpy(host.ee, ee, sizeof(ee)); }
    memset(&D, 0, sizeof(D));
    host.key = keys;
    sys_boot();
    host.adc = 300u << 4;
    ticks(60);
    assert(sys_power_on());
    host.key = 0;
    ticks(10);
}

/* 전원 투입: MODE + SET = 튜닝(t1), MODE + DOWN 은 더 이상 튜닝이 아니다 */
static void test_tune_entry(void)
{
    boot_keys(KEY_MODE + KEY_DOWN, 0);
    assert(!tune_active());
    boot_keys(KEY_MODE + KEY_SET, 0);
    assert(tune_active() && (tune_state() == TUNE_WAIT) && (tune_menu() == 1000));
    assert(shows("EE256") && (D.ui_mode == UI_AUTO));   /* 켠 뒤 1 s 칩 크기 */
    ticks(100);
    assert(shows("T1   "));
    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_UP);                   /* 키를 누르면 바로 메뉴 */
    assert(shows("T2   "));
    printf("  tune entry ok\n");
}

/* 칩 크기: 128~255 를 읽기만 해 0~127 과 견준다. 128 워드 칩은 주소 7비트를 무시한다 */
static void test_ee_probe(void)
{
    int a, same = 1;

    boot();
    assert(ee_probe.ee_words == 256 && ee_probe.diff_n > 0 && ee_probe.first_addr >= 128);
    assert(ee_probe.first_hi == 0xFFFFu && ee_probe.first_lo != 0xFFFFu);   /* 위는 빈 칸, 아래는 부팅 때 읽은 값 */
    for (a = 128; a < 256; a++) { same = same && (host.ee[a] == 0xFFFFu); }
    assert(same);                    /* 위 128 워드는 쓰지 않았다 */
    host_reset();
    host.ee_128 = 1;
    memset(&D, 0, sizeof(D));
    sys_boot();
    assert(ee_probe.ee_words == 128 && ee_probe.diff_n == 0 && ee_probe.first_addr == 0);
    printf("  ee probe ok\n");
}

/* 메뉴: MODE 같은 단계 다음, SET 아래로, RESET 위로. MODE 는 파라미터·조그 화면으로 가지 않는다 */
static void test_tune_menu(void)
{
    uint16_t d[5];
    int k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_UP);
    assert(tune_menu() == 2000 && shows("T2   "));
    key(KEY_UP);
#if AGV_DEV_TESTS
    assert(tune_menu() == 3000 && shows("T3   "));
    key(KEY_UP);
#endif
    assert(tune_menu() == 1000);
#if !AGV_DEV_TESTS
    key(KEY_DOWN);                                    /* t3 없음: t1 앞은 t2, t2 다음은 t1 */
    assert(tune_menu() == 2000 && shows("T2   "));
    key(KEY_UP);
    assert(tune_menu() == 1000);
#endif
    key(KEY_SET);                                     /* t1 입력: 시험 전에 확정할 값 14칸 */
    {   /* 파라미터 칸은 파라미터 화면과 같은 이름·자릿수·글자. 0 = t1 전용(AU, L) */
        static const uint16_t want[14] = { P_SPEED_HIGH, P_SPEED_LOW, P_SPEED_TURN, 0, P_ACC, P_OBS_USE, P_BUMPER_USE,
                                           P_GUIDE_DIST_MM, P_TURN_FORBACK, P_AGV_ID, P_WHEEL_DIA_MM, P_WHEEL_BASE_MM,
                                           P_GEAR_RATIO, 0 };
        char t[5], u[5];
        for (k = 0; k < 14; k++) {
            assert(tune_menu() == (uint16_t)(1000 + (k + 1) * 10));
            (void)tune_text(t);
            if (want[k] != 0u) {
                ui_param_text(want[k], param_get(want[k]), u);
                assert(memcmp(t, u, 5) == 0);
            }
            if (k == 0)  { assert(shows("DRS30")); }
            if (k == 7)  { assert(shows("GD250")); }
            if (k == 10) { assert(shows("WS202")); }
            if (k == 13) { assert(shows("L0000")); }
            if (k == 3) {                             /* ACC 상한. 빈 EEPROM 은 ACC 지금 값 */
                char want[6] = "AU0  ";
                want[3] = (char)('0' + param_get(P_ACC) / 10);
                want[4] = (char)('0' + param_get(P_ACC) % 10);
                assert(shows(want));
            }
            key(KEY_UP);
        }
    }
    assert(tune_menu() == 1010);
    for (k = 0; k < 11; k++) { key(KEY_UP); }       /* WW 를 고치면 그 파라미터가 바로 바뀐다 */
    assert(tune_menu() == 1120);
    {
        int16_t ww = param_get(P_WHEEL_BASE_MM);
        key(KEY_SET);
        key(KEY_UP);
        key(KEY_SET);
        assert(param_get(P_WHEEL_BASE_MM) == ww + 1);
        (void)param_set(P_WHEEL_BASE_MM, ww);
    }
    for (k = 0; k < 3; k++) { key(KEY_UP); }
    assert(tune_menu() == 1010);
    key(KEY_RESET);
    assert(tune_menu() == 1000);
    key(KEY_UP);
    key(KEY_SET);
    assert(tune_menu() == 2010 && shows("T21CH"));
    ui_digits(d);
    for (k = 0; k < 3; k++) { key(KEY_UP); }
    assert(tune_menu() == 2040 && shows("T24AP"));
    key(KEY_SET);                                     /* L 입력 */
    assert(tune_menu() == 2041 && shows("L0000"));
    key(KEY_UP);
    assert(tune_menu() == 2041);                       /* 한 칸뿐 */
    key(KEY_RESET);
    key(KEY_UP);
    assert(tune_menu() == 2010);
    key(KEY_UP);
    key(KEY_UP);
    key(KEY_SET);                                     /* t2.3 은 L 이 없다 */
    assert(tune_menu() == 2030 && shows("T23GS"));
    key(KEY_RESET);
    key(KEY_RESET);
    assert(tune_menu() == 2000);
#if AGV_DEV_TESTS
    key(KEY_UP);
    key(KEY_SET);
    assert(tune_menu() == 3010 && tune_ident_mode() && shows("T31ID"));
    for (k = 0; k < 4; k++) { key(KEY_UP); }
    assert(tune_temp_mode() && !tune_all_mode() && shows("T35TE"));
    key(KEY_UP);
    assert(tune_hs_mode() && shows("T36HS"));
    key(KEY_SET);                                     /* 속도 입력. 기본 1번(30), 1번 위로는 못 간다 */
    assert(tune_menu() == 3061 && shows("SP030"));
    key(KEY_SET);
    key(KEY_UP);
    key(KEY_SET);
    assert(shows("SP030") && tune_hs_mpm() == 30);
    key(KEY_SET);
    key(KEY_DOWN);
    key(KEY_SET);
    assert(shows("SP029") && tune_hs_mpm() == 29 && host.ee[TUNE_EE_CAP] != 29);   /* RAM 에만 */
    key(KEY_RESET);
    assert(tune_menu() == 3060);
    key(KEY_UP);
    assert(tune_menu() == 3070 && shows("T37GR"));
    key(KEY_UP);
    assert(tune_menu() == 3080 && shows("T38GS"));
    key(KEY_UP);
    assert(tune_menu() == 3090 && shows("T39WT"));
    key(KEY_UP);
    assert(tune_ident_mode());
#endif
    assert(D.ui_mode == UI_AUTO && tune_active() && !D.flag_jog_on);
    printf("  tune menu ok\n");
}

/* 튜닝 메뉴 이동은 UP 다음·DOWN 이전, MODE 는 넘기지 않는다 */
static void test_tune_nav(void)
{
    int k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_MODE);                                    /* 단계 선택에서도 MODE 는 그대로 */
    assert(tune_menu() == 1000);
    key(KEY_DOWN);
#if AGV_DEV_TESTS
    assert(tune_menu() == 3000);                      /* 처음 → 끝 */
    key(KEY_DOWN);
#endif
    assert(tune_menu() == 2000);
    key(KEY_UP);
#if AGV_DEV_TESTS
    key(KEY_UP);
#endif
    assert(tune_menu() == 1000);
    key(KEY_SET);                                     /* t1 입력 14칸 */
    key(KEY_DOWN);
    assert(tune_menu() == 1140);                      /* 처음 → 끝 */
    key(KEY_UP);
    assert(tune_menu() == 1010);                      /* 끝 → 처음 */
    key(KEY_MODE);
    assert(tune_menu() == 1010);                      /* MODE 는 칸을 안 넘긴다 */
    for (k = 0; k < 13; k++) { key(KEY_UP); }
    assert(tune_menu() == 1140);
    for (k = 0; k < 13; k++) { key(KEY_DOWN); }
    assert(tune_menu() == 1010);
    key(KEY_SET);                                     /* 수정 중 UP·DOWN 은 값 증감 */
    key(KEY_UP);
    key(KEY_DOWN);
    key(KEY_DOWN);
    assert(tune_menu() == 1010 && tune_editing());
    key(KEY_RESET);
    assert(!tune_editing() && tune_menu() == 1010);
    printf("  tune nav ok\n");
}

/* t1 입력 칸 번호(1~14)로 MODE 를 눌러 간다. 1 DRS, 4 ACC 상한(AU), 11 지름(WS), 14 L */
static void t1_to(int item)
{
    int k;
    for (k = 0; (k < 16) && (tune_menu() != (uint16_t)(1000 + item * 10)); k++) { key(KEY_UP); }
    assert(tune_menu() == (uint16_t)(1000 + item * 10));
}

/* 입력: SET 수정, SHIFT 자리, UP·DOWN 값, SET 확정 즉시 저장, RESET 취소. L·ACC 상한은 다시 켜도 남는다 */
static void test_tune_inputs(void)
{
    int k;
    uint16_t cap_ee;

    boot_keys(KEY_MODE + KEY_SET, 0);
    cap_ee = host.ee[TUNE_EE_CAP];
    key(KEY_SET);                                     /* t1 */
    t1_to(11);                                        /* 지름 WS */
    key(KEY_SET);
    assert(tune_editing() && tune_flick() == 0x10u);
    key(KEY_UP);
    key(KEY_SHIFT);
    assert(tune_flick() == 0x08u);
    key(KEY_UP);
    assert(shows("WS213") && param_get(P_WHEEL_DIA_MM) == 202);   /* 확정 전 */
    key(KEY_SET);
    assert(!tune_editing() && param_get(P_WHEEL_DIA_MM) == 213);
    ticks(10);
    assert(host.ee[P_WHEEL_DIA_MM] == 213);
    key(KEY_SET);
    key(KEY_UP);
    key(KEY_RESET);                                   /* 취소 */
    assert(!tune_editing() && param_get(P_WHEEL_DIA_MM) == 213 && tune_menu() == 1110);
    t1_to(1);                                         /* DRS: 최고 속도 상한 = DRS 파라미터 */
    assert(param_get(P_SPEED_HIGH) == 30);
    (void)param_set(P_SPEED_LOW, 28);                 /* 확정할 DRS 25 보다 큼 */
    key(KEY_SET);
    key(KEY_SHIFT);
    key(KEY_DOWN);                                    /* 20 */
    key(KEY_SHIFT);                                   /* 두 자리 칸은 1 → 2 → 1 */
    for (k = 0; k < 5; k++) { key(KEY_UP); }
    key(KEY_SET);
    assert(shows("SAVE "));                           /* 저장 알림 1 s */
    assert(param_get(P_SPEED_HIGH) == 25);            /* 확정하는 순간 DRS */
    ticks(100);
    assert(shows("DRS25") && host.ee[P_SPEED_HIGH] == 25);
    assert(param_get(P_SPEED_LOW) == 25 && host.ee[P_SPEED_LOW] == 25);   /* LOS 도 DRS 이하로 */
    t1_to(14);                                        /* L 네 자리 */
    key(KEY_SET);
    for (k = 0; k < 3; k++) { key(KEY_SHIFT); }
    assert(tune_flick() == 0x02u);
    for (k = 0; k < 7; k++) { key(KEY_UP); }
    key(KEY_SHIFT);                                   /* 넷째 자리 다음은 일의 자리 */
    for (k = 0; k < 5; k++) { key(KEY_UP); }
    key(KEY_SET);
    ticks(100);
    assert(shows("L7005"));
    ticks(20);
    assert(host.ee[TUNE_EE_TRACK] == 7005 && host.ee[TUNE_EE_CAP] == cap_ee);   /* DRS 는 상한 칸을 안 쓴다 */
    t1_to(4);                                         /* ACC 상한 12 → 위 바이트 */
    key(KEY_SET);
    for (k = 0; (k < 10) && !shows("AU012"); k++) { key((param_get(P_ACC) < 12) ? KEY_UP : KEY_DOWN); }
    key(KEY_SET);
    ticks(120);
    assert(shows("AU012") && host.ee[TUNE_EE_CAP] == (uint16_t)(12u << 8));
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_SET);
    t1_to(1);
    assert(shows("DRS25"));
    t1_to(14);
    assert(shows("L7005"));
    t1_to(4);
    assert(shows("AU012"));
    printf("  tune inputs ok\n");
}

/* 5자리 FND: 파라미터 화면 이름 + 자릿수가 다 들어가고, 결과 값은 넘치면 자르지 않고 OVF·OVER 로 보인다 */
static void test_fnd_fit(void)
{
    char c[6] = "     ", l[3];
    uint16_t i, k, n;

    for (i = 1; i < P_COUNT; i++) {   /* 값 자리(오른쪽 n 칸) 앞까지만 이름 글자 */
        n = ui_param_label(i, l);
        if (n == 0u) { n = 2; }      /* ON/OFF */
        for (k = (uint16_t)(5u - n); k < 3u; k++) { assert(l[k] == ' '); }
    }
    ui_param_text(P_CROSS_IN_CM, 60, c);
    assert(memcmp(c, "CD060", 5) == 0);
    ui_param_text(P_SPEED_MIN, 7, c);
    assert(memcmp(c, "SPN07", 5) == 0);
    ui_param_text(P_LEGACY_QUIRKS, 2047, c);   /* LG 는 b10 까지라 네 자리: L + 값 */
    assert(memcmp(c, "L2047", 5) == 0);
    ui_param_text(P_LOAD_DETECT, 1, c);
    assert(memcmp(c, "LDSON", 5) == 0);
    tune_value_text(c, 'D', 'E', 5414, 1);
    assert(memcmp(c, "D5414", 5) == 0);
    tune_value_text(c, 'D', 'E', 414, 1);
    assert(memcmp(c, "D0414", 5) == 0);
    tune_value_text(c, 'D', 'E', 12000, 1);
    assert(memcmp(c, "DOVER", 5) == 0);
    tune_value_text(c, 'D', 'E', -42, 1);
    assert(memcmp(c, "D-042", 5) == 0);
    tune_value_text(c, 'S', 'E', 1200, 0);
    assert(memcmp(c, "SEOVF", 5) == 0);
    tune_value_text(c, 'L', 'R', -15, 0);
    assert(memcmp(c, "L-015", 5) == 0);
    tune_value_text(c, 'S', 'E', TUNE_NO_DATA, 0);
    assert(memcmp(c, "SE---", 5) == 0);
    printf("  fnd fit ok\n");
}

/* START 한 뒤 연속 시험이 끝난 것처럼 만들고 세운다. mask 와 시작 값은 ident_start_build 가 넣었다 */
static void fake_build_done(void)
{
    agv_build.state = 2;
    agv_ident.state = 2;             /* 지금 단계 칸. ident_run 이 1 을 낸다 */
    tick();
    assert(tune_state() == TUNE_BLD_DONE);
    raw_a(RAW_STOP, 0);
    ticks(2);
    raw_a(0, RAW_STOP);
    ticks(300);
    assert(!D.flag_run && !D.flag_stopping);
}

/* 결과 쪽 SAVE 칸: 채택 수 "SV  n" 또는 "NONE " */
static int on_save(void)
{
    char c[5];
    (void)tune_text(c);
    return ((c[0] == 'S') && (c[1] == 'V')) || ((c[0] == 'N') && (c[1] == 'O') && (c[2] == 'N'));
}

/* 결과 쪽을 SAVE 까지 MODE 로 넘긴다(UP 과 같다) */
static void to_save(void)
{
    int k;
    for (k = 0; (k < 30) && !on_save(); k++) { key(KEY_MODE); }
    assert(on_save());
}

/* t1 결과: 채택 저장 1·13·35 는 SET 채택, RESET 건너뛰기, SAVE 에서만 쓴다. 51번 추천은 없다. 기준값을 쓰고 RBOT */
static void test_tune_t1_save(void)
{
    uint16_t before[128];
    int a, k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    (void)param_set(P_SPEED_LOW, 28);   /* 채택 DRS 25 보다 크다: 저장 때 LOS 도 25 로 낮아진다 */
    drive_up();                      /* t1 에서 START */
    run_scan(3);
    assert(tune_state() == TUNE_BLD && ident_build_mode() && agv_build.mask != 0);
    assert(agv_build.cap_mpm == param_get(P_SPEED_HIGH));   /* 최고 속도 상한 = DRS */
    assert(shows("B1008"));          /* 단계 1, 0회, 선 찾기 */
    agv_build.high_rec = 25;
    agv_build.dcc_rec = 35;
    agv_build.cid_rec = 40;
    agv_build.se_x10 = 43;           /* 편차 단계 4.3 mm */
    agv_build.min_rec = 7;
    agv_build.turn_x1000 = 1110;
    agv_build.top_rpm = 3000;
    agv_build.dcc_max = 35;
    agv_build.lr_permille = -4;
    agv_build.amp = 'S';
    fake_build_done();
    ticks(200);
    memcpy(before, host.ee, sizeof(before));
    /* 1번: 파라미터 화면 이름(DRS)으로 지금 30 과 새 25 를 1 s 씩. 새 값이면 둘째 자리 점. 추천은 처음부터 채택(첫 자리 점) */
    for (k = 0; (k < 250) && !shows("DRS25"); k++) { tick(); }
    assert(shows("DRS25"));
    for (k = 0; (k < 250) && !shows("DRS30"); k++) { tick(); }
    assert(shows("DRS30"));
    {
        char c[5];
        assert(tune_text(c) & 1u);
    }
    key(KEY_MODE);                   /* MODE 는 다음 쪽(UP 과 같다). 채택은 그대로 */
    assert(tune_page() == 1);
    key(KEY_RESET);                  /* 13번 건너뛰기 → 다음 */
    assert(tune_page() == 2);
    {
        char c[5];
        assert(tune_text(c) & 1u);   /* 35번(CD)도 처음부터 채택 */
        for (k = 0; (k < 250) && !shows("CD040"); k++) { tick(); }
        assert(shows("CD040"));
    }
    key(KEY_DOWN);
    {
        char c[5];
        assert(tune_page() == 1 && !(tune_text(c) & 1u));   /* 건너뛴 13번은 점이 없다 */
    }
    key(KEY_DOWN);
    assert(tune_page() == 0);
    {   /* 값 쪽 이름: 회전 보정비·최고 rpm 은 이름 한 글자 + 네 자리. 가감속 찾기는 AM·DM */
        for (k = 0; (k < 30) && !shows("R1110"); k++) { key(KEY_UP); }
        assert(shows("R1110"));
        for (k = 0; (k < 30) && !shows("T3000"); k++) { key(KEY_UP); }
        assert(shows("T3000"));
        key(KEY_UP);
        assert(shows("AM---"));
        key(KEY_UP);
        assert(shows("DM035"));
    }
    /* 저장 전에는 아무것도 쓰지 않았다 */
    assert(param_get(P_SPEED_HIGH) == 30 && param_get(P_CROSS_IN_CM) == 60 && param_get(P_OPT_FLAGS) == 0);
    assert(!tune_base_valid());
    to_save();
    assert(shows("SV  2"));          /* 1·35 채택, 13 건너뜀 */
    key(KEY_SET);
    assert(tune_state() == TUNE_SAVED && (storage_pending() != 0u) && shows("BUSY "));   /* 쓰기 끝까지 */
    for (k = 0; (k < 300) && (storage_pending() != 0u); k++) { tick(); }
    assert(storage_pending() == 0u);
    ticks(5);
    assert(shows("RBOT "));
    assert(param_get(P_SPEED_HIGH) == 25 && param_get(P_DCC) == 40 && param_get(P_CROSS_IN_CM) == 40);
    assert(param_get(P_OPT_FLAGS) == 0 && host.ee[116] == before[116]);
    assert(host.ee[P_SPEED_HIGH] == 25 && host.ee[P_DCC] == 40 && host.ee[P_CROSS_IN_CM] == 40);
    assert(param_get(P_SPEED_LOW) == 25 && host.ee[P_SPEED_LOW] == 25);   /* DRS 채택과 함께 LOS 도 낮아짐 */
    assert(tune_base_valid() && tune_base(TUNE_BASE_TOP) == 3000 && tune_base(TUNE_BASE_UNUSED) == 0);
    assert(tune_base(TUNE_BASE_LR) == -4 && ((uint16_t)tune_base(TUNE_BASE_AMP_SE) >> 8) == 'S');
    assert(((uint16_t)tune_base(TUNE_BASE_AMP_SE) & 0xFFu) == 43u);
    /* 쓴 칸: 체크섬 0, 채택한 1·35, 기준값 110~114·119·125 뿐 */
    for (a = 0; a < 128; a++) {
        int base = (a == TUNE_EE_CHECK) || (a == 111) || (a == 112) || (a == 113) || (a == 114) || (a == 119) || (a == 125);
        int adopt = (a == 0) || (a == P_SPEED_HIGH) || (a == P_SPEED_LOW) || (a == P_CROSS_IN_CM);
        if (!base && !adopt) { assert(host.ee[a] == before[a]); }
    }
    key(KEY_MODE);                   /* 저장 뒤 키는 듣지 않는다 */
    key(KEY_RESET);
    assert(tune_state() == TUNE_SAVED && D.ui_mode == UI_AUTO);
    printf("  tune t1 save ok\n");
}

/* EEPROM 자리: 128 워드 안, 파라미터·KEEP·체크섬 밖 파라미터·OUT 큐와 안 겹치고 서로 다르다 */
static void test_tune_ee_layout(void)
{
    uint16_t a[TUNE_BASE_N + 3];
    int i, j;

    for (i = 0; i < TUNE_BASE_N; i++) { a[i] = tune_base_addr[i]; }
    a[TUNE_BASE_N] = TUNE_EE_CHECK;
    a[TUNE_BASE_N + 1] = TUNE_EE_TRACK;
    a[TUNE_BASE_N + 2] = TUNE_EE_CAP;
    for (i = 0; i < TUNE_BASE_N + 3; i++) {
        assert(a[i] < 128u && a[i] > KEEP_LAST);
        assert(!((a[i] >= PARAM_GUIDE_DIST_ADDR) && (a[i] <= 118u)));
        assert(!((a[i] >= ACS_OUT_EE_FIRST) && (a[i] <= ACS_OUT_EE_HEAD)));
        for (j = 0; j < i; j++) { assert(a[i] != a[j]); }
    }
    /* 공차 파라미터 77~81 은 KEEP 의 빈 칸(태그 이력 뒤, 스테이션 개수 앞) */
    assert(PARAM_EMPTY_ADDR0 >= KEEP_TAG_NUM0 + 5 && PARAM_EMPTY_ADDR0 + 4 < KEEP_STATION_CNT);
    printf("  tune ee layout ok\n");
}

/* t2.1 빠른 점검: 기준 대비 %, NG 는 이름이 깜박인다. 추천은 채택하지 않으면 안 쓴다. SAVE 에서 RESET 은 메뉴로 */
static void test_tune_t2_compare(void)
{
    int k;

    test_tune_t1_save();             /* 기준값 TOP 3000 */
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_UP);
    key(KEY_SET);                    /* t2.1 */
    assert(tune_base_valid());
    drive_up();
    run_scan(3);
    assert(tune_state() == TUNE_BLD && shows("C1008"));
    agv_build.high_rec = 22;
    agv_build.top_rpm = 2700;        /* -10% NG */
    agv_build.over_mm = 25;          /* NG */
    agv_build.lr_permille = 15;      /* NG */
    agv_build.se_x10 = 70;           /* 기준 43 의 +63% NG */
    fake_build_done();
    assert(tune_page() == 0 && tune_flick() == 0);
    key(KEY_RESET);                  /* 1번 건너뛰기 */
    for (k = 0; (k < 20) && !shows("T-010"); k++) { key(KEY_UP); }
    assert(shows("T-010") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("AM---"));          /* 가감속 찾기가 없었다 */
    key(KEY_UP);
    assert(shows("DM---"));
    key(KEY_UP);
    assert(shows("FR---"));          /* 정지가 없었다. 이름은 파라미터 화면 FL(32번)과 다르다 */
    key(KEY_UP);
    assert(shows("OS025") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("SE070") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("LR015") && tune_flick() == 0x03u);
    to_save();
    assert(shows("NONE "));          /* 1번을 건너뛰어 채택이 없다 */
    key(KEY_RESET);                  /* 저장 없이 메뉴로 */
    assert(tune_state() == TUNE_WAIT && tune_menu() == 2010);
    ticks(50);
    assert(param_get(P_SPEED_HIGH) == 25 && host.ee[P_SPEED_HIGH] == 25);
    assert(tune_base(TUNE_BASE_TOP) == 3000);
    /* 판 1(편차 1단위를 6.89 mm 로 환산하던 이미지) 기준: 다른 칸은 견주고 SE 는 견주지 않는다 */
    host.ee[TUNE_EE_CHECK] = (uint16_t)(host.ee[TUNE_EE_CHECK] ^ TUNE_BASE_MARK ^ TUNE_BASE_MARK_V1);
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_UP);
    key(KEY_SET);
    assert(tune_base_valid());
    drive_up();
    run_scan(3);
    agv_build.top_rpm = 2700;
    agv_build.se_x10 = 70;
    fake_build_done();
    key(KEY_RESET);
    for (k = 0; (k < 20) && !shows("T-010"); k++) { key(KEY_UP); }
    assert(shows("T-010") && tune_flick() == 0x03u);
    for (k = 0; (k < 20) && !shows("SE070"); k++) { key(KEY_UP); }
    assert(shows("SE070") && tune_flick() == 0u);
    printf("  tune t2 compare ok\n");
}

/* t2.4 앰프 쪽: 기준 앰프가 0(모름. t1 은 1회를 안 한다)이면 견주지 않고, 기준이 있으면 다를 때만 깜박인다 */
static void t24_amp(uint16_t base_amp, int16_t now, uint16_t flick)
{
    uint16_t a = tune_base_addr[TUNE_BASE_AMP_SE], old;
    char want[6] = "A    ";
    int k;

    test_tune_t1_save();             /* 기준 앰프 'S' */
    old = host.ee[a];
    host.ee[a] = (uint16_t)((base_amp << 8) | (old & 0xFFu));
    host.ee[TUNE_EE_CHECK] = (uint16_t)(TUNE_BASE_MARK ^ (uint16_t)(((uint16_t)(host.ee[TUNE_EE_CHECK] ^ TUNE_BASE_MARK)) - old + host.ee[a]));
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_UP);
    key(KEY_SET);
    for (k = 0; k < 3; k++) { key(KEY_UP); }   /* t2.4 */
    assert(tune_base_valid() && tune_menu() == 2040);
    drive_up();
    run_scan(3);
    assert(tune_state() == TUNE_BLD);
    agv_build.amp = now;
    fake_build_done();
    want[4] = now ? (char)now : '-';
    for (k = 0; (k < 30) && !shows(want); k++) { key(KEY_UP); }
    assert(shows(want) && tune_flick() == flick);
}

static void test_tune_t24_amp(void)
{
    t24_amp(0, 'S', 0);
    t24_amp(0, 'T', 0);
    t24_amp('S', 'S', 0);
    t24_amp('S', 'T', 0x03u);
    printf("  tune t2.4 amp ok\n");
}

/* 연속 시험 중 가이드 측정: 'h' 부호가 반대이고 배율이 2 이면 저장 뒤 되살아난다. 튜닝 중 저전압은 보이지 않는다 */
#if AGV_DEV_TESTS
/* ---- 가이드 센서 응답 시험(t3.7 GR) ----
 * 모의 센서: 'H' 요청 송신이 끝난 뒤 지연 + 흔들림(0~jitter) 뒤에 'h' 응답 8바이트를 115200 bps 바이트 간격으로 낸다.
 * 앞 응답을 다 내기 전에 온 요청은 무시한다(센서가 바쁨). bad_at 번째 응답(또는 bad_every 마다)은 체크섬을 깬다 */
#define GM_BYTE 8681UL               /* 115200 bps 10비트 = 86.8 us (100 MHz 카운트) */
static struct { uint32_t delay, jitter, busy, seed; int on, line, bad_at, bad_every, count; } s_gm;

static void gm_send(uint16_t port, const uint16_t *buf, uint16_t n)
{
    host_sci_t *b = &host.sci[HAL_SCI_B];
    uint16_t f[8], sum, leds = s_gm.line ? 0x0180u : 0u;
    uint32_t t;
    int i, k;

    if ((port != HAL_SCI_B) || !s_gm.on || (n < 2) || (buf[1] != 'H')) { return; }
    t = host.cycles + n * GM_BYTE;
    if ((int32_t)(t - s_gm.busy) < 0) { return; }
    s_gm.seed = s_gm.seed * 1103515245UL + 12345UL;
    t += s_gm.delay + ((s_gm.jitter > 0) ? ((s_gm.seed >> 8) % (s_gm.jitter + 1UL)) : 0UL);
    s_gm.count++;
    f[0] = 2;
    f[1] = 'h';
    for (i = 0; i < 4; i++) {
        uint16_t v = (uint16_t)((leds >> (4 * i)) & 0xFu);
        f[2 + i] = (uint16_t)((v < 10) ? ('0' + v) : ('a' + v - 10));
    }
    sum = (uint16_t)((f[1] + f[2] + f[3] + f[4] + f[5]) & 0xF);
    if ((s_gm.count == s_gm.bad_at) || (s_gm.bad_every && (s_gm.count % s_gm.bad_every == 0))) { sum = (uint16_t)((sum + 1) & 0xF); }
    f[6] = (uint16_t)((sum < 10) ? ('0' + sum) : ('a' + sum - 10));
    f[7] = 3;
    k = b->rx_len - b->rx_pos;           /* 남은 바이트를 앞으로 당긴다 */
    memmove(b->rx, &b->rx[b->rx_pos], (size_t)k * sizeof(b->rx[0]));
    memmove(b->rx_at, &b->rx_at[b->rx_pos], (size_t)k * sizeof(b->rx_at[0]));
    b->rx_len = (uint16_t)k;
    b->rx_pos = 0;
    for (i = 0; i < 8; i++) {
        t += GM_BYTE;
        b->rx[b->rx_len] = f[i];
        b->rx_at[b->rx_len++] = t;
    }
    s_gm.busy = t;
}

/* 10 us 마다 메인 루프, 2 ms 마다 틱. 결과가 나올 때까지 걸린 ms */
static uint32_t gr_run(void)
{
    uint32_t t0 = host.cycles, i = 0;
    host.sci[HAL_SCI_B].tx_len = 0;
    while (tune_state() != TUNE_GR_DONE) {
        host.cycles += 1000u;
        if ((++i % 200u) == 0u) { sys_tick_2ms(); }
        sys_scan();
        assert(D.vin_out == 0);                       /* 달리지 않는다 */
        assert(i < 2000000u);                         /* 20 s */
    }
    return (host.cycles - t0) / 100000u;
}

/* 튜닝으로 켜서 t3.7 로 간다 */
static void gr_menu(void)
{
    int i;
    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_DOWN);                                    /* t1 → t3 */
    key(KEY_SET);
    for (i = 0; i < 6; i++) { key(KEY_UP); }
    assert(tune_menu() == 3070 && shows("T37GR"));
}

/* START 한 번. 서 있는 그대로 끝나고 달리지 않는다 */
static uint32_t gr_go(void)
{
    uint32_t ms;
    int i;

    host.on_send = gm_send;
    press_start();                                    /* 출발 대기를 바로 세운다 */
    assert(tune_state() == TUNE_GR);
    ms = gr_run();
    for (i = 0; i < 20; i++) {
        assert(D.vin_out == 0);
        tick();
    }
    assert(!D.flag_ready && !D.flag_run && !D.flag_stopping && (D.stop_err == 0) && (D.system_err == 0));
    return ms;
}

static void gm_set(uint32_t delay_us, uint32_t jitter_us, int line, int on)
{
    memset(&s_gm, 0, sizeof(s_gm));
    s_gm.delay = delay_us * 100u;
    s_gm.jitter = jitter_us * 100u;
    s_gm.line = line;
    s_gm.on = on;
    s_gm.seed = 7;
}

static void test_tune_gr(void)
{
    const gresp_result_t *g = &agv_all.gr;
    uint32_t ms;
    uint16_t i, tx;

    /* 1 지연 1.0 ms + 흔들림 0~0.3 ms: 응답 = 요청 4바이트 0.35 + 지연 + 응답 8바이트 0.69 = 2.04~2.34 ms.
     *   2 ms 간격은 앞 응답이 끝나기 전이라 무응답 → 안정 3 ms */
    gr_menu();
    gm_set(1000, 300, 1, 1);
    ms = gr_go();
    printf("  GR delay 1.0+0~0.3: GRL %d GRS %d GRH %d n %d err %d step %d, %u ms\n", g->grl, g->grs, g->grh, g->n, g->err, g->step, (unsigned)ms);
    assert(g->state == 2 && g->n == GR_N);
    assert(g->grl >= 20 && g->grl <= 21 && g->grh >= 22 && g->grh <= 24 && g->grs == 30);
    assert(g->err > 0 && g->step == 7);
    assert(ms < 4000u);
    assert(shows("GRL20") || shows("GRL21"));
    key(KEY_UP);
    assert(shows("GRS30"));
    key(KEY_UP);
    assert(shows("GRH22") || shows("GRH23") || shows("GRH24"));
    key(KEY_UP);
    assert(shows("GRL20") || shows("GRL21"));         /* 처음으로 */
    key(KEY_DOWN);
    assert(shows("GRH22") || shows("GRH23") || shows("GRH24"));
    for (i = 0; i < host.sci[HAL_SCI_B].tx_len; i += 4) { assert(host.sci[HAL_SCI_B].tx[i + 1] == 'H'); }   /* 시험 요청만 */
    key(KEY_RESET);
    assert(tune_state() == TUNE_WAIT && shows("T37GR"));

    /* 2 빠른 센서(지연 0.2 ms, 흔들림 없음): 2 ms 까지 통과, 바로 재요청 주기 = 응답 1.24 ms(+ 메인 루프 10 us) */
    gr_menu();
    gm_set(200, 0, 1, 1);
    ms = gr_go();
    printf("  GR delay 0.2: GRL %d GRS %d GRH %d err %d step %d, %u ms\n", g->grl, g->grs, g->grh, g->err, g->step, (unsigned)ms);
    assert(g->state == 2 && g->grl == 12 && g->grh == 12 && g->err == 0 && g->step == 8);
    assert(g->grs >= 12 && g->grs <= 13);
    assert(shows("GRL12"));

    /* 3 잡음 한 번(20번째 응답 체크섬): 그 간격을 다시 해서 결과는 같고 오류만 센다 */
    gr_menu();
    gm_set(200, 0, 1, 1);
    s_gm.bad_at = 20;
    ms = gr_go();
    printf("  GR noise once: GRL %d GRS %d GRH %d err %d, %u ms\n", g->grl, g->grs, g->grh, g->err, (unsigned)ms);
    assert(g->state == 2 && g->err == 1 && g->grl == 12 && g->grs >= 12 && g->grs <= 13);

    /* 4 잡음 잦음(30번째마다): 안정 주기는 없음(--), 응답 시간과 오류 수는 남긴다(시험을 버리지 않는다) */
    gr_menu();
    gm_set(200, 0, 1, 1);
    s_gm.bad_every = 30;
    ms = gr_go();
    printf("  GR noise 1/30: GRL %d GRS %d GRH %d n %d err %d, %u ms\n", g->grl, g->grs, g->grh, g->n, g->err, (unsigned)ms);
    assert(g->state == 2 && g->grs == TUNE_NO_DATA && g->grl == 12 && g->err >= 3 && g->n >= 90);
    key(KEY_UP);
    assert(shows("GRS--"));

    /* 5 무응답: 없음, 오류 아님, 곧 끝난다 */
    gr_menu();
    gm_set(1000, 0, 1, 0);
    ms = gr_go();
    printf("  GR no reply: state %d, %u ms\n", g->state, (unsigned)ms);
    assert(g->state == 3 && g->grl == TUNE_NO_DATA && g->grs == TUNE_NO_DATA && g->grh == TUNE_NO_DATA && ms < 200u);
    assert(shows("GRL--"));

    /* 6 라인 없음(LED 모두 꺼짐): 없음 */
    gr_menu();
    gm_set(1000, 0, 0, 1);
    ms = gr_go();
    printf("  GR no line: state %d, %u ms\n", g->state, (unsigned)ms);
    assert(g->state == 3 && g->grh == TUNE_NO_DATA);

    /* 7 시험 뒤 평상시 조회: 10 ms 마다 요청 하나, 응답 처리, 오류 없음 */
    host.on_send = 0;
    tune_reset();
    (void)param_set_ram(P_LEGACY_QUIRKS, QUIRK_SRC_A);
    host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;
    drive_up();
    tx = host.sci[HAL_SCI_B].tx_len;
    run_scan(10);
    assert(host.sci[HAL_SCI_B].tx_len == tx + 40u);
    assert(host.sci[HAL_SCI_B].tx[tx + 1] == 'A' && D.flag_run && (D.stop_err == 0));
    printf("  tune GR ok\n");
}
#endif

static void test_tune_guide(void)
{
    guide_rx_t rx;
    guide_cfg_t cfg;
    int i;

    boot_keys(KEY_MODE + KEY_SET, 0);
    drive_up();
    run_scan(5);
    assert(tune_state() == TUNE_BLD);
    memset(&rx, 0, sizeof(rx));
    rx.kind = GUIDE_RX_LED;
    for (i = 0; i < 40; i++) {
        int pos = (i % 7) - 3;                    /* 'a' 위치 */
        int cell = 8 - pos;                       /* 'h' 두 칸 c, c+1: 위치 = 2c - 16 = -2pos */
        tune_on_a(GUIDE_NORMAL, (int16_t)(2 * pos));
        rx.led_lo = 3UL << (cell - 1);            /* 부호 반대, 배율 1 */
        tune_on_h(&rx, 0);
    }
    assert(tune_guide_k100() == -100);
    assert(tune_guide_save() == 1);
    guide_cfg_get(&cfg);
    assert(cfg.invert_fwd == 1);
    /* 저장값을 부팅 때 되살린다 */
    keep_set(KEEP_GUIDE_INV, 1);
    keep_set(KEEP_GUIDE_SCALE, 1250);
    keep_set(KEEP_GUIDE_MARK, 0x7E57);
    tune_load();
    guide_cfg_get(&cfg);
    assert((cfg.invert_fwd == 1) && (cfg.invert_back == 0) && (cfg.h_scale == 1.25f));
    {   /* 튜닝 중 저전압 280 은 표시·부저에 나오지 않는다 */
        uint16_t d[5];
        boot_keys(KEY_MODE + KEY_SET, 0);
        host.adc = 100u << 4;
        ticks(1100);
        assert(D.action_err == ERR_BAT_LOW);
        ui_digits(d);
        assert(d[0] != 15u);                      /* 'E' 가 아니다 */
        assert(host.bus_out[BUS_LAMP] & 0x40u);   /* 부저 꺼짐 (저활성) */
    }
    printf("  tune guide ok\n");
}
/* 곡선 기록기: 합성 궤적. 휠 간격 600 mm, GD 250, 틱마다 5 mm. 선 잃음 진단 입력은 s_cl_in 으로 준다 */
static struct { int16_t lost, track, reacq; uint32_t led; } s_cl_in;

static void cl_feed(float ds, float r_mm, int16_t pos, int16_t active, int16_t back)
{
    curvelog_in_t in;
    float k = (r_mm != 0.0f) ? (600.0f * 0.5f / r_mm) : 0.0f;   /* r < 0 = 오른쪽 */
    float dl = ds * (1.0f - k), dr = ds * (1.0f + k);
    if (back) { float t = dl; dl = dr; dr = t; }                  /* 후진: 진행 방향 기준 왼쪽 = 차체 오른쪽 바퀴 */
    in.dl_mm = dl;
    in.dr_mm = dr;
    in.backward = back;
    in.pos = pos;
    in.lost = s_cl_in.lost;
    in.led = s_cl_in.led;
    in.track = s_cl_in.track;
    in.reacq = s_cl_in.reacq;
    in.active = active;
    in.wb_mm = 600.0f;
    in.gd_mm = 250.0f;
    curvelog_feed(&in);
}

/* 직선 1 m, 반경 r 원호 len, 직선 1 m */
static void cl_route(float r_mm, float len, int16_t pos, int16_t back)
{
    float s;
    for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, pos, 1, back); }
    for (s = 0.0f; s < len; s += 5.0f)     { cl_feed(5.0f, r_mm, pos, 1, back); }
    for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, pos, 1, back); }
}

static int16_t cl_in(int16_t v, int16_t lo, int16_t hi) { return (int16_t)((v >= lo) && (v <= hi)); }

/* 바퀴 펄스로 달린다(curvelog_tick 경로). 반경 r 원호 len mm */
static void cl_drive(float r_mm, float len, int turn_mid)
{
    float wb = (float)param_get(P_WHEEL_BASE_MM) * 10.0f, s, acc[2] = { 0.0f, 0.0f };
    int i;
    for (s = 0.0f; s < len + 2000.0f; s += 5.0f) {
        float k = ((s >= 1000.0f) && (s < 1000.0f + len)) ? (wb * 0.5f / r_mm) : 0.0f;
        acc[0] += 5.0f * (1.0f - k) * param_dv.one_length_pulse;
        acc[1] += 5.0f * (1.0f + k) * param_dv.one_length_pulse;
        for (i = 0; i < 2; i++) { while (acc[i] >= 1.0f) { run_encoder((uint16_t)i, 20000); acc[i] -= 1.0f; } }
        D.flag_turn = (int16_t)(turn_mid && (s > 1000.0f + 0.5f * len) && (s < 1100.0f + 0.5f * len));
        curvelog_tick();
    }
    D.flag_turn = 0;
}

static void test_curvelog(void)
{
    const float rb = 968.246f;          /* sqrt(1000² - 250²): 센서 점이 반경 1000 원 위 */
    const curve_rec_t *c;
    int16_t lg0;

    boot();
    curvelog_reset();
    assert(agv_curvelog.group == 1 && agv_curvelog.g[0].n == 0);
    cl_route(rb, 1521.0f, 0, 0);        /* 왼쪽 90도 */
    c = &agv_curvelog.g[0].c[0];
    printf("  곡선 1: 길이 %d 속도 %d 테이프 %d 차체 %d 이론 %d 최대 %d rms %d 넘침 %d 지연 %d 방향 %d\n", c->len_mm, c->v_x10,
           c->r_tape_mm, c->r_body_mm, c->r_theory_mm, c->dev_max_x10, c->dev_rms_x10, c->exit_over_x10, c->entry_lag_mm, c->dir);
    assert(agv_curvelog.g[0].n == 1 && c->seq == 1 && c->dir == 1 && c->lost == 0);
    assert(cl_in(c->r_tape_mm, 980, 1020) && cl_in(c->r_body_mm, 948, 988) && cl_in(c->r_theory_mm, 948, 988));
    assert(cl_in(c->len_mm, 1300, 1521) && c->v_x10 == 300);     /* 5 mm/10 ms = 30 m/min */
    assert(c->dev_max_x10 == 0 && c->dev_rms_x10 == 0 && c->exit_over_x10 == 0);
    cl_feed(5.0f, 0.0f, 0, 1, 0);
    for (int i = 0; i < 1000; i++) { cl_feed(5.0f, 0.0f, 0, 1, 0); }   /* 직선 5 m: 기록 없음 */
    assert(agv_curvelog.g[0].n == 1);
    cl_route(-rb, 1521.0f, 4, 0);       /* 오른쪽, 편차 4단위 일정: 13.78 mm */
    c = &agv_curvelog.g[0].c[1];
    assert(agv_curvelog.g[0].n == 2 && c->seq == 2 && c->dir == -1);
    assert(c->dev_max_x10 == 138 && c->dev_rms_x10 == 138 && c->exit_over_x10 == 0);
    cl_route(rb, 1521.0f, 0, 1);        /* 후진도 진행 방향 기준 */
    c = &agv_curvelog.g[0].c[2];
    assert(agv_curvelog.g[0].n == 3 && c->dir == 1 && cl_in(c->r_tape_mm, 980, 1020) && cl_in(c->r_body_mm, 948, 988));
    cl_route(rb, 250.0f, 0, 0);         /* 0.3 m 미만은 버린다 */
    assert(agv_curvelog.g[0].n == 3);
    /* 회전 중(비활성)이 끼면 그 구간을 버린다 */
    { float s;
      for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, 0, 1, 0); }
      for (s = 0.0f; s < 1500.0f; s += 5.0f) { cl_feed(5.0f, rb, 0, (int16_t)((s < 700.0f) || (s > 720.0f)), 0); }
      for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, 0, 1, 0); } }
    assert(agv_curvelog.g[0].n == 4 && cl_in(agv_curvelog.g[0].c[3].len_mm, 300, 800));   /* 회전 뒤 남은 부분만 */
    /* 묶음 넘김: 다음 묶음을 비우고 그때 LG·OPT 를 적는다. 4 다음은 1 */
    lg0 = param_get(P_LEGACY_QUIRKS);
    assert(agv_curvelog.g[0].lg == lg0);
    assert(param_set_ram(P_LEGACY_QUIRKS, 368) == 0 && param_set_ram(P_OPT_FLAGS, 8) == 0);
    curvelog_next_group();
    assert(agv_curvelog.group == 2 && agv_curvelog.g[1].n == 0 && agv_curvelog.g[1].lg == 368 && agv_curvelog.g[1].opt == 8);
    cl_route(rb, 1521.0f, 0, 0);
    assert(agv_curvelog.g[1].n == 1 && agv_curvelog.g[0].n == 4);
    curvelog_next_group();
    curvelog_next_group();
    curvelog_next_group();
    assert(agv_curvelog.group == 1 && agv_curvelog.g[0].n == 0 && agv_curvelog.g[1].n == 1);
    assert(param_set_ram(P_LEGACY_QUIRKS, lg0) == 0 && param_set_ram(P_OPT_FLAGS, 0) == 0);
    /* curvelog_tick: 자동 주행에서만 기록. 파라미터 모드 → 자동이면 LG·OPT 가 바뀌었을 때만 다음 묶음. 튜닝 모드·회전 중은 기록 없음 */
    boot();
    curvelog_reset();
    D.flag_run = 1;
    D.control = RUN_DRIVE;
    cl_drive(rb, 1521.0f, 0);
    assert(agv_curvelog.g[0].n == 1);
    cl_drive(rb, 1521.0f, 1);           /* 곡선 가운데 회전 표시: 앞 반은 버리고 회전 뒤 남은 부분만 */
    assert(agv_curvelog.g[0].n == 2 && agv_curvelog.g[0].c[1].len_mm < 800);
    D.ui_mode = UI_PARAM;               /* 값 확인만: 묶음을 잇는다 */
    curvelog_tick();
    D.ui_mode = UI_AUTO;
    curvelog_tick();
    assert(agv_curvelog.group == 1 && agv_curvelog.g[0].n == 2);
    cl_drive(rb, 1521.0f, 0);
    assert(agv_curvelog.group == 1 && agv_curvelog.g[0].n == 3);
    D.ui_mode = UI_PARAM;               /* 기록 있는 묶음에서 값을 바꿨다: 다음 묶음 */
    curvelog_tick();
    assert(param_set_ram(P_LEGACY_QUIRKS, 368) == 0);
    D.ui_mode = UI_AUTO;
    curvelog_tick();
    assert(agv_curvelog.group == 2 && agv_curvelog.g[1].n == 0 && agv_curvelog.g[1].lg == 368 && agv_curvelog.g[0].n == 3);
    D.ui_mode = UI_PARAM;               /* 빈 묶음에서 값을 바꿨다: 같은 번호에 새 값 */
    curvelog_tick();
    assert(param_set_ram(P_LEGACY_QUIRKS, 880) == 0 && param_set_ram(P_OPT_FLAGS, 8) == 0);
    D.ui_mode = UI_AUTO;
    curvelog_tick();
    assert(agv_curvelog.group == 2 && agv_curvelog.g[1].lg == 880 && agv_curvelog.g[1].opt == 8);
    cl_drive(rb, 1521.0f, 0);
    assert(agv_curvelog.g[1].n == 1);
    {   /* 기록하고 값을 바꾸며 넘기면 3, 4, 그다음 1 */
        static const int16_t lgv[3] = { 0, 512, 368 };
        for (int k = 0; k < 3; k++) {
            D.ui_mode = UI_PARAM;
            curvelog_tick();
            assert(param_set_ram(P_LEGACY_QUIRKS, lgv[k]) == 0);
            D.ui_mode = UI_AUTO;
            curvelog_tick();
            assert(agv_curvelog.group == ((k < 2) ? k + 3 : 1) && agv_curvelog.g[agv_curvelog.group - 1].lg == lgv[k]);
            cl_drive(rb, 1521.0f, 0);
        }
        assert(agv_curvelog.g[0].n == 1 && agv_curvelog.g[1].n == 1);
    }
    assert(param_set_ram(P_LEGACY_QUIRKS, lg0) == 0 && param_set_ram(P_OPT_FLAGS, 0) == 0);
    D.flag_run = 0;
    boot_keys(KEY_MODE + KEY_SET, 0);   /* 튜닝 모드 */
    assert(tune_active());
    curvelog_reset();
    D.flag_run = 1;
    D.control = RUN_DRIVE;
    D.ui_mode = UI_AUTO;
    cl_drive(rb, 1521.0f, 0);
    assert(agv_curvelog.g[0].n == 0);
    D.flag_run = 0;
    /* 오래 달린 뒤에도 같은 곡선은 같은 값이다(원점 옮김, 표본 수 32비트). 20 km 를 완만하게(반경 30 m, 5 m 마다 좌우) 달린다 */
    curvelog_reset();
    cl_route(rb, 1521.0f, 4, 0);
    assert(agv_curvelog.g[0].n == 1);
    { long k;
      for (k = 0; k < 4000000L; k++) { cl_feed(5.0f, ((k / 1000L) & 1L) ? 30000.0f : -30000.0f, 0, 1, 0); } }
    assert(agv_curvelog.g[0].n == 1);
    cl_route(rb, 1521.0f, 4, 0);
    { const curve_rec_t *a = &agv_curvelog.g[0].c[0], *b = &agv_curvelog.g[0].c[1];
      printf("  20 km 뒤 곡선: 차체 %d → %d, 테이프 %d → %d, 길이 %d → %d\n", a->r_body_mm, b->r_body_mm, a->r_tape_mm, b->r_tape_mm, a->len_mm, b->len_mm);
      assert(agv_curvelog.g[0].n == 2);
      assert(cl_in(b->r_body_mm, (int16_t)(a->r_body_mm * 0.97f), (int16_t)(a->r_body_mm * 1.03f)));
      assert(cl_in(b->r_tape_mm, (int16_t)(a->r_tape_mm * 0.97f), (int16_t)(a->r_tape_mm * 1.03f)));
      assert(b->dev_max_x10 == a->dev_max_x10 && cl_in(b->len_mm, (int16_t)(a->len_mm - 30), (int16_t)(a->len_mm + 30))); }
    /* 곡선 안 첫 선 잃음 진단: 곡선 시작부터 거리, 직전 정상 응답 비트·위치·추적 상태, 잃은 응답 비트, 지속 확인 */
    curvelog_reset();
    { float s;
      s_cl_in.led = 0x00018000UL;
      for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, 0, 1, 0); }
      for (s = 0.0f; s < 1521.0f; s += 5.0f) {
          int in_lost = (s >= 700.0f) && (s < 730.0f);                     /* 6 틱 */
          s_cl_in.lost = (int16_t)in_lost;
          s_cl_in.led = in_lost ? ((s < 715.0f) ? 0UL : 0x00600000UL) : 0x00030000UL;
          s_cl_in.reacq = (int16_t)(in_lost && (s >= 715.0f));             /* 덩어리가 다시 보였지만 확인 중 */
          s_cl_in.track = (s >= 690.0f) ? GUIDE_TRK_EDGE : GUIDE_TRK_NORMAL;
          cl_feed(5.0f, rb, (int16_t)(in_lost ? 0 : 3), 1, 0);
          if ((s >= 900.0f) && (s < 905.0f)) {                            /* 같은 곡선 둘째 선 잃음은 진단을 덮지 않는다 */
              s_cl_in.lost = 1;
              s_cl_in.led = 0x1UL;
              cl_feed(5.0f, rb, 0, 1, 0);
          }
      }
      memset(&s_cl_in, 0, sizeof(s_cl_in));
      for (s = 0.0f; s < 1000.0f; s += 5.0f) { cl_feed(5.0f, 0.0f, 0, 1, 0); } }
    c = &agv_curvelog.g[0].c[0];
    printf("  선 잃음 진단: 잃음 %d 거리 %d 직전 위치 %d 비트 %04X%04X 잃은 비트 %04X%04X 추적 %d\n", c->lost, c->lost_at_mm,
           c->lost_prev_pos, c->lost_prev_hi, c->lost_prev_lo, c->lost_led_hi, c->lost_led_lo, c->lost_trk);
    assert(agv_curvelog.g[0].n == 1 && c->lost == 2);
    assert(cl_in(c->lost_at_mm, 450, 710) && c->lost_prev_pos == 3);
    assert(c->lost_prev_hi == 0x0003 && c->lost_prev_lo == 0 && c->lost_led_hi == 0 && c->lost_led_lo == 0);
    assert(c->lost_trk == (GUIDE_TRK_EDGE | 4));
    cl_route(rb, 1521.0f, 0, 0);        /* 선 잃음 없는 곡선은 0 */
    c = &agv_curvelog.g[0].c[1];
    assert(c->lost == 0 && c->lost_at_mm == 0 && c->lost_trk == 0 && c->lost_prev_hi == 0);
    printf("  curvelog ok\n");
}


/* 위치 소스 'h' 만(기본. GSR 미사용): 연속 시험 선 찾기 중에는 'a' 소스처럼 'A'·'H' 를 번갈아 보내 배율 짝을 모은다.
 * 'a' 응답은 짝에만 쓰고 위치는 'h' 그대로다. 위치는 LED 덩어리에 h_scale 을 곱해 'a' 단위로 쓰고,
 * 덩어리 폭이 커브 판정 입력이다. 측정 구간이 아니면 매 틱 'H' 만 보낸다 */
static void test_guide_h_only(void)
{
    guide_cfg_t cfg;
    int k, a = 0, h = 0;

    boot_keys(KEY_MODE + KEY_SET, 0);
    drive_up();
    run_scan(3);
    assert(tune_state() == TUNE_BLD && ident_want_h());   /* 선 찾기: 'a' 소스면 번갈아 보낸다 */
    for (k = 0; k < 6; k++) {
        host.sci[HAL_SCI_B].tx_len = 0;
        run_scan(1);
        assert(host.sci[HAL_SCI_B].tx_len == 4);
        if (host.sci[HAL_SCI_B].tx[1] == 'A') { a++; } else { h++; }
    }
    assert(a == 3 && h == 3);
    assert(param_set_ram(P_GUIDE_CURVE_DETECT, 0) == 0);   /* GSR 값은 무시한다 */
    assert(tune_guide_k100() == TUNE_NO_DATA);
    a = h = 0;
    for (k = 0; k < 40; k++) {                      /* 'h' 위치 0(가운데), 'a' 는 +5: 짝은 쌓이되 위치는 'h' */
        host.sci[HAL_SCI_B].tx_len = 0;
        host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;   /* 다 읽었다 */
        if ((k & 1) == 0) { guide_led_reply(LED_CENTER); } else { guide_reply(5); }
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
        assert(host.sci[HAL_SCI_B].tx_len == 4);
        if (host.sci[HAL_SCI_B].tx[1] == 'A') { a++; } else { h++; }
        assert(D.guide_pos == 0);
    }
    assert(a == 20 && h == 20);
    for (k = 0; k < 40; k++) {                      /* 'a' -6 다음 'h' -6: 배율 1 */
        host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;   /* 다 읽었다 */
        if ((k & 1) == 0) { guide_reply(-6); } else { guide_led_reply(LED_NEAR); }
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
    }
    printf("  'h' 전용 배율 k100 %d\n", tune_guide_k100());
    assert(tune_guide_k100() == 100);
    for (k = 0; k < 6; k++) {                       /* 'a' 가 와도 위치는 'h' 의 -6 그대로 */
        host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;   /* 다 읽었다 */
        guide_reply(9);
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
        assert(D.guide_pos == -6 && D.guide_track == GUIDE_TRK_NORMAL);
    }
    guide_cfg_get(&cfg);
    cfg.h_scale = 1.5f;
    guide_configure(&cfg);
    guide_led_reply(LED_NEAR);                      /* 'h' 위치 -6 → 'a' 단위 -9 */
    tick();
    assert(D.guide_pos == -9);
    cfg.h_scale = 1.0f;
    guide_configure(&cfg);
    /* 튜닝이 아닌 주행: 'H' 만 */
    boot();
    assert(param_set_ram(P_LEGACY_QUIRKS, 0) == 0);
    press_start();
    for (k = 0; k < 6; k++) {
        host.sci[HAL_SCI_B].tx_len = 0;
        guide_led_reply(LED_CENTER);
        tick();
        assert(host.sci[HAL_SCI_B].tx_len == 4 && host.sci[HAL_SCI_B].tx[1] == 'H');
    }
    assert(D.flag_run && D.guide_pos == 0 && D.guide_span == 2);
    /* LG 512(QUIRK_SRC_A): 예전 'a' 소스. 분기·테이프가 없으면 'A' 만 */
    boot();
    assert(param_set_ram(P_LEGACY_QUIRKS, QUIRK_SRC_A) == 0);
    press_start();
    for (k = 0; k < 6; k++) {
        host.sci[HAL_SCI_B].tx_len = 0;
        guide_reply(4);
        tick();
        assert(host.sci[HAL_SCI_B].tx_len == 4 && host.sci[HAL_SCI_B].tx[1] == 'A');
    }
    assert(D.flag_run && D.guide_pos == 4);
    assert(param_set_ram(P_LEGACY_QUIRKS, 0) == 0);
    printf("  guide h only ok\n");
}

/* 회전 보정비: KEEP 54 에 × 1000. 다시 켜도 남고, 일반·정렬 회전 목표 펄스에 건다. 범위 밖이면 보정 없음 */
static void test_turn_scale(void)
{
    int16_t q;

    boot();
    assert(tune_turn_scale() == 1.0f);           /* 빈 칸 */
    (void)param_set(P_TURN_FORBACK, 10);
    tagact_turn_align_stopped(TURN_CW, 180u, 0u);
    assert(D.turn_90 == P.turn_90 && D.turn_pulses == P.turn_180 && D.tape_total_pulse == P.turn_180);
    tune_turn_scale_set(1110);
    ticks(20);
    assert(host.ee[KEEP_TURN_SCALE] == 1110u);
    boot_keys(0, 1);
    assert(tune_turn_scale() > 1.1099f && tune_turn_scale() < 1.1101f);
    (void)param_set(P_TURN_FORBACK, 10);
    tagact_turn_align_stopped(TURN_CW, 180u, 0u);
    q = (int16_t)((float)P.turn_90 * 1.11f + 0.5f);
    assert(D.turn_90 == q && D.turn_pulses == q * 2 && D.tape_total_pulse == q * 2);
    assert(D.tape_target_cnt == (float)(q * 2) && D.tape_end_cnt == D.tape_target_cnt);
    assert(D.tape_max == (int16_t)((float)q / 90.0f * 20.0f));
    tune_turn_scale_set(2000);                   /* 범위 끝으로 */
    assert(keep_get(KEEP_TURN_SCALE) == TUNE_TURN_UP);
    keep_set(KEEP_TURN_SCALE, 1400);             /* 손댄 값이 범위 밖이면 보정 없음 */
    assert(tune_turn_scale() == 1.0f);
    printf("  turn scale ok\n");
}

/* 교차 테이프를 두 번 보면 정렬 회전을 건다 */
static void test_tape_turn(void)
{
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    drive_up();
    run_scan(30);
    D.tape_turn_dir = TURN_CW;
    D.tape_turn_deg = 90;
    D.tape_turn_next = 0x10;
    host.sci[HAL_SCI_B].tx_len = 0;
    guide_led_reply(0x03FFu);
    tick();
    assert(guide_tx_has('H'));
    assert((D.speed_cmd == run_seek_rpm()) && (D.speed_cmd >= P.min_speed));   /* 정렬 회전 앞 찾기 속도 */
    assert(D.tape_align == 0);
    guide_led_reply(0x03FFu);
    tick();
    assert(D.tape_turn_dir == 0);
    assert(D.tape_align == 1);
    assert(D.turn_pulses == P.turn_90);
    assert(D.flag_stopping == 1);
    /* 교차선 뒤 GD 구간: GD + 기본 반폭(10 mm) - 응답 늦음. 조향 보정 0 */
    assert((D.stop_pulses >= (int16_t)(P.guide_dist_mm * P.one_length_pulse))
           && (D.stop_pulses <= (int16_t)((P.guide_dist_mm + 10.0f) * P.one_length_pulse + 0.5f)));
    assert(D.steer_hold == 1);
    printf("  tape turn ok\n");
}

/* 'h' 응답 하나와 엔코더 흉내로 주행 스캔 */
static void led_scan(uint16_t leds, int n)
{
    while (n-- > 0) {
        host_sci_t *b = &host.sci[HAL_SCI_B];
        if (b->rx_pos == b->rx_len) { b->rx_len = b->rx_pos = 0; }
        guide_led_reply(leds);
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
    }
}

/* 교차선 인정: 새 응답이 이어서 두 번(아닌 응답에 0, 응답 없는 틱은 세지 않음), 주행 테이프가 켠 칸은 세지 않는다('h' 소스) */
static void test_tape_rule(void)
{
    boot();
    (void)param_set_ram(P_LEGACY_QUIRKS, 0);
    drive_up();
    led_scan(0x0180u, 20);                      /* 주행 테이프 칸 8·9 */
    D.tape_stop_kind = 3;
    led_scan(0x01FFu, 1);                       /* 왼쪽 칸 1~7 이 다 켜짐: 교차선 위 한 번 */
    assert(D.tape_stop_kind == 3);
    led_scan(0x0180u, 1);                       /* 아닌 응답: 0 으로 */
    led_scan(0x01FFu, 1);
    assert(D.tape_stop_kind == 3);
    run_encoder(0, 20000);                      /* 새 응답 없는 틱은 세지 않는다 */
    tick();
    assert(D.tape_stop_kind == 3);
    led_scan(0x01FFu, 1);                       /* 이어서 두 번째: 인정, 바로 정지(분류 3) */
    assert((D.tape_stop_kind == 0) && D.flag_stopping && (D.stop_pulses == 0));

    boot();                                     /* 넓은 주행 테이프(6칸): 예전 규칙(합 6 이상)은 바로 인정했다 */
    (void)param_set_ram(P_LEGACY_QUIRKS, 0);
    drive_up();
    led_scan(0x07E0u, 20);
    D.tape_stop_kind = 3;
    led_scan(0x07E0u, 20);
    assert((D.tape_stop_kind == 3) && !D.flag_stopping);
    led_scan(0x0FE0u, 5);                       /* 한 칸 넓어져도(끝까지 안 닿음) 교차선이 아니다 */
    assert((D.tape_stop_kind == 3) && !D.flag_stopping);
    led_scan(0xFFE0u, 2);                       /* 오른쪽 칸 12~16 이 끝까지 다 켜짐 */
    assert((D.tape_stop_kind == 0) && D.flag_stopping);
    printf("  tape rule ok\n");
}

/* 다른 정지가 진행 중이거나 주행 중이 아니면 교차선 요청은 보류: 요청 칸은 두고 정지 거리·속도 지령을 건드리지 않는다 */
static void tape_guard_run(int req, int16_t *sp, int16_t *cmd, int n)
{
    int i;
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    drive_up();
    run_scan(30);
    tagact_stop_length();                       /* 일반 태그 정지 */
    if (req == 1) { D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0; }
    if (req == 2) { D.tape_stop_kind = 4; D.tape_stop_id = 3; }
    for (i = 0; i < n; i++) {
        run_scan(1);
        sp[i] = D.stop_pulses;
        cmd[i] = D.speed_cmd;
    }
    assert((D.tape_turn_dir == ((req == 1) ? TURN_CW : 0)) && (D.tape_stop_kind == ((req == 2) ? 4 : 0)) && D.flag_stopping);
}

static void test_tape_guard(void)
{
    int16_t sp[3][40], cmd[3][40], v;
    int i, k;
    for (k = 0; k < 3; k++) { tape_guard_run(k, sp[k], cmd[k], 40); }
    for (k = 1; k < 3; k++) {
        for (i = 0; i < 40; i++) { assert((sp[k][i] == sp[0][i]) && (cmd[k][i] == cmd[0][i])); }
    }
    boot();                                     /* 서 있을 때: 보류, 지령 그대로 */
    (void)param_set(P_TURN_FORBACK, 10);
    v = D.speed_cmd;
    D.tape_stop_kind = 3;
    ticks(3);
    assert((D.tape_stop_kind == 3) && (D.speed_cmd == v));
    D.tape_stop_kind = 0;
    D.tape_turn_dir = TURN_CW;
    ticks(3);
    assert((D.tape_turn_dir == TURN_CW) && (D.speed_cmd == v));
    printf("  tape guard ok\n");
}

/* 'h' 응답 n 개, 응답마다 엔코더 펄스 p 개(두 바퀴) */
static void led_run(uint16_t leds, int n, int p)
{
    while (n-- > 0) {
        host_sci_t *b = &host.sci[HAL_SCI_B];
        int k;
        if (b->rx_pos == b->rx_len) { b->rx_len = b->rx_pos = 0; }
        guide_led_reply(leds);
        for (k = 0; k < p; k++) { run_encoder(0, 20000); run_encoder(1, 20000); }
        tick();
    }
}

/* 'h' 소스로 주행 상태까지, 주행 테이프 칸 8·9 */
static void h_drive_up(int turn)
{
    boot();
    (void)param_set_ram(P_LEGACY_QUIRKS, 0);
    if (turn) { (void)param_set(P_TURN_FORBACK, 10); }
    drive_up();
    led_run(0x0180u, 40, 1);
}

/* 인정 기준(2차): 그 쪽이 마지막으로 2/3 넘게 비어 있던 때부터 20 mm 안에 차야 한다. 끝 칸이 한 응답 꺼져도 기준이 새로 잡히지
 * 않는다(합류선을 인정하던 결함). 옆에 오래 켜진 선이 있어도 교차선은 인정한다. 빈 칸은 허용하지 않는다(모의에서 합류선 오인정) */
static void test_tape_rule2(void)
{
    h_drive_up(0);
    D.tape_stop_kind = 3;
    led_run(0x0181u, 10, 4);                    /* 합류선: 끝 칸부터 천천히 찬다(응답마다 4 펄스) */
    led_run(0x0183u, 10, 4);
    led_run(0x018Fu, 10, 4);
    led_run(0x01FFu, 6, 4);
    assert((D.tape_stop_kind == 3) && !D.flag_stopping);
    led_run(0x01FEu, 1, 4);                     /* 끝 칸 한 응답 꺼짐 */
    led_run(0x01FFu, 3, 4);
    assert((D.tape_stop_kind == 3) && !D.flag_stopping && !D.system_err);

    h_drive_up(0);
    led_run(0x0183u, 60, 1);                    /* 옆(칸 1·2)에 오래 켜진 선 */
    D.tape_stop_kind = 3;
    led_run(0x0183u, 5, 1);
    led_run(0x01FFu, 2, 1);                     /* 교차선 */
    assert((D.tape_stop_kind == 0) && D.flag_stopping);

    h_drive_up(0);
    D.tape_stop_kind = 3;
    led_run(0x01FBu, 2, 1);                     /* 칸 3 이 빈 응답은 교차선이 아니다 */
    assert((D.tape_stop_kind == 3) && !D.flag_stopping);
    printf("  tape rule 2 ok\n");
}

/* 교차선 회전: 찾는 중 일시정지로 교차선 위에 서서 인정하면 회전이 거부된다(속도 0). 맡은 요청은 남고 다시 움직이면 그 교차선에서 돈다 */
static void test_tape_turn_paused(void)
{
    int i;
    h_drive_up(1);
    D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0x10;
    led_run(0x0180u, 5, 1);
    D.pause_stop = 1;
    for (i = 0; (i < 300) && (D.vin_out != 0); i++) { led_run(0x0180u, 1, 1); }
    led_run(0x0180u, 30, 0);
    led_run(0x01FFu, 3, 0);                     /* 서 있는 자리가 교차선 위 */
    assert(!D.tape_align && !D.flag_stopping && !D.system_err);
    D.pause_stop = 0;
    for (i = 0; (i < 400) && !D.tape_align; i++) { led_run(0x01FFu, 1, 1); }
    assert(D.tape_align && (D.turn_pulses > 0) && D.flag_stopping && D.steer_hold && (D.tape_turn_dir == 0) && !D.system_err);
    printf("  tape turn paused ok\n");
}

/* GD 구간 중 명령 정지(ACS 교통 정지): 회전·분류 4 를 버리지 않고 다시 출발하면 남은 거리를 가서 서고 돈다.
 * 분류 4 + 시퀀스는 GD 에 선 뒤에 시퀀스 */
static void gd_cmd_stop(int turn)
{
    int i;
    h_drive_up(turn);
    if (turn) { D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0x10; }
    else      { D.tape_stop_kind = 4; D.tape_stop_id = SEQ_LOAD; }
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 3, 1);                     /* 인정: GD 정지 */
    assert(D.steer_hold && D.flag_stopping && (D.stop_pulses > 0));
    led_run(0x0180u, 5, 1);
    stop_request_rapid(STOP_REASON_COMMAND);    /* 'W' 'C' '0' */
    D.flag_stopping = 1;
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    led_run(0x0180u, 5, 0);
    assert(!D.flag_run && !D.steer_hold && !D.flag_turn && !seq_request_flag[SEQ_LOAD] && !D.system_err);
    run_wait_start();                           /* 'W' 'C' '1' 과 같은 다시 출발 */
    for (i = 0; i < 20; i++) { guide_led_reply(0x0180u); tick(); }
    for (i = 0; (i < 3000) && (D.flag_run || D.flag_ready); i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    for (i = 0; i < 5; i++) { guide_reply(0); tick(); }
    if (turn) { assert(D.flag_turn || D.turn_motion); }
    else      { assert(seq_request_flag[SEQ_LOAD] || seq_is_active()); }
    assert(!D.system_err && (run_tape_last.kind == (turn ? -1 : 4)));
    run_seq_init();
}

static void test_tape_gd_cmd_stop(void)
{
    gd_cmd_stop(1);
    gd_cmd_stop(0);
    printf("  tape gd cmd stop ok\n");
}

/* AGV_MOVE( 한계, TAPE_GD ): GD 구간 중 명령 정지는 도착이 아니다(MOVE_FAIL). 찾는 중 명령 정지 감속에 교차선을 봐도 GD 정지로
 * 바꾸지 않는다 */
static void move_gd_cmd(int early)
{
    int i;
    boot();
    (void)param_set_ram(P_LEGACY_QUIRKS, 0);
    assert(tagact_move_tape(3000, 4) == 1);
    ticks(3);
    guide_led_reply(0x0180u);
    tick();
    led_run(0x0180u, 60, 1);
    if (early) {
        stop_request_rapid(STOP_REASON_COMMAND);
        D.flag_stopping = 1;
        led_run(0x0180u, 2, 1);
    }
    led_run(0x01FFu, 2, 1);
    if (early) { assert((stop_reason() == STOP_REASON_COMMAND) && !D.steer_hold); }
    else       { assert((D.move_on == MOVE_FOUND) && D.steer_hold); }
    led_run(0x0180u, 3, 1);
    if (!early) {
        stop_request_rapid(STOP_REASON_COMMAND);
        D.flag_stopping = 1;
    }
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    assert(D.move_on == MOVE_FAIL);
}

static void test_move_gd_cmd(void)
{
    move_gd_cmd(0);
    move_gd_cmd(1);
    printf("  move gd cmd ok\n");
}

/* GD 구간 중 ACS 작업 취소('W' 'C' '2'): 요청은 지워지고 조향 0·선 없음 막기는 설 때까지 둔다 */
static void test_gd_traffic_reset(void)
{
    int i;
    h_drive_up(0);
    D.tape_stop_kind = 4;
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 3, 1);
    assert(D.steer_hold && D.flag_stopping);
    run_seq_init();                             /* traffic_reset 이 부른다 */
    led_run(0x0000u, 1, 1);
    assert(D.steer_hold && D.flag_stopping);
    for (i = 0; (i < 600) && D.flag_run; i++) { led_run(0x0000u, 1, (D.vin_out > 0) ? 1 : 0); }
    led_run(0x0000u, 2, 0);
    assert(!D.flag_run && !D.steer_hold && !D.system_err && !D.stop_err);
    printf("  gd traffic reset ok\n");
}

/* 찾기 한계는 찾는 동안 간 거리만 센다: 보류된 사이 바퀴가 돈 펄스(제자리 회전·조그)는 넣지 않는다 */
static void test_tape_seek_len(void)
{
    int i;
    h_drive_up(0);
    D.tape_stop_kind = 4;
    led_run(0x0180u, 5, 1);
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    for (i = 0; i < 4 * P.stop_len; i++) { run_encoder(0, 20000); }   /* 서 있는 동안 바퀴만 돈다 */
    tick();
    run_wait_start();
    for (i = 0; i < 20; i++) { guide_led_reply(0x0180u); tick(); }
    led_run(0x0180u, 10, 1);
    assert(!D.system_err && (D.tape_stop_kind == 4));
    printf("  tape seek len ok\n");
}

/* 교차선 회전(turn) 또는 분류 4(+ 자동 출발 as) 요청을 걸고 인정한 뒤 GD 구간 중 명령 정지로 선다 */
static void gd_to_cmd_stop(int turn, uint16_t as)
{
    int i;
    h_drive_up(turn);
    if (turn) { D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0x10; }
    else      { D.tape_stop_kind = 4; D.tape_autostart = as; }
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 3, 1);
    led_run(0x0180u, 5, 1);
    stop_request_rapid(STOP_REASON_COMMAND);    /* 'W' 'C' '0' */
    D.flag_stopping = 1;
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    led_run(0x0180u, 5, 0);
}

/* 다시 출발('W' 'C' '1')해 설 때까지 */
static void restart_to_stop(void)
{
    int i;
    run_wait_start();
    for (i = 0; i < 20; i++) { guide_led_reply(0x0180u); tick(); }
    for (i = 0; (i < 3000) && (D.flag_run || D.flag_ready); i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    for (i = 0; i < 5; i++) { guide_reply(0); tick(); }
}

/* 처음 GD 정지에서 목표를 60 mm 넘게 지나치면(서지 못하는 차) 돌지 않고 258, 지나침을 기록. 분류 3 은 기록만 */
static void test_gd_overrun(void)
{
    int i, coast = 0;
    h_drive_up(1);
    D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0x10;
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 2, 1);
    assert(D.steer_hold && D.tape_align);
    for (i = 0; (i < 900) && D.flag_run; i++) {   /* 지령이 0 이 된 뒤에도 80 틱 굴러간다 */
        led_run(0x0180u, 1, ((D.vin_out > 0) || (coast++ < 80)) ? 2 : 0);
    }
    led_run(0x0180u, 20, 0);
    assert((D.system_err == ERR_GUIDE_SENSOR) && !D.flag_turn && !D.tape_align && (D.turn_pulses == 0) && (D.control != RUN_TURN));
    assert((run_tape_last.kind == -1) && (run_tape_last.over_mm > 60.0f));

    h_drive_up(0);
    D.tape_stop_kind = 3;
    led_run(0x0180u, 5, 1);
    coast = 0;
    for (i = 0; (i < 900) && (D.flag_run || (i < 3)); i++) { led_run(0x01FFu, 1, ((D.vin_out > 0) || (coast++ < 40)) ? 2 : 0); }
    led_run(0x0180u, 5, 0);
    assert(!D.system_err && (run_tape_last.kind == 3) && (run_tape_last.over_mm > 20.0f));

    /* AGV_MOVE( 한계, TAPE_GD ): 지나쳤으면 도착(MOVE_DONE)이 아니라 MOVE_FAIL + 258 */
    boot();
    (void)param_set_ram(P_LEGACY_QUIRKS, 0);
    assert(tagact_move_tape(3000, 4) == 1);
    ticks(3);
    guide_led_reply(0x0180u);
    tick();
    led_run(0x0180u, 60, 1);
    led_run(0x01FFu, 2, 1);
    assert((D.move_on == MOVE_FOUND) && D.steer_hold);
    coast = 0;
    for (i = 0; (i < 900) && D.flag_run; i++) { led_run(0x0180u, 1, ((D.vin_out > 0) || (coast++ < 80)) ? 2 : 0); }
    assert((D.move_on != MOVE_DONE) && (D.system_err == ERR_GUIDE_SENSOR) && (run_tape_last.over_mm > 60.0f));   /* MOVE_FAIL 은 run_seq_init 이 0 으로 */
    printf("  gd overrun ok\n");
}

/* 분류 4 + 자동 출발: GD 중 명령 정지로 서 있는 동안 스스로 떠나지 않는다(예약을 거두어 되돌림). 다시 출발해 목표에 서면 건다 */
static void test_gd_autostart(void)
{
    int i;
    gd_to_cmd_stop(0, 3u);                      /* AS 2 초 */
    assert(!D.flag_run && !D.wait_start && (D.tape_autostart == 3u) && !D.system_err);
    for (i = 0; (i < 400) && !D.flag_run && !D.flag_ready; i++) { guide_led_reply(0x0180u); tick(); }
    assert(i == 400);                           /* 교통 정지를 어기지 않는다 */
    restart_to_stop();
    assert(!D.system_err && (run_tape_last.kind == 4) && D.wait_start && !D.tape_autostart);
    assert((run_tape_last.over_mm > -5.0f) && (run_tape_last.over_mm < 30.0f));   /* 이 흉내는 펄스가 속도와 무관해 20 mm 쯤 */
    for (i = 0; (i < 400) && !D.flag_run && !D.flag_ready; i++) { guide_led_reply(0x0180u); tick(); }
    assert((i > 150) && (i < 400));             /* 목표에 선 뒤 AS 쯤에 스스로 떠난다 */
    printf("  gd autostart ok\n");
}

/* 맡은 채 서 있는 동안 바퀴가 돌면(밀림) 다시 걸지 않고 258. 선 직후 굴러감(0.5 s 안)과 20 mm 안 떨림은 둔다 */
static void test_gd_pushed(void)
{
    int i;
    gd_to_cmd_stop(1, 0u);
    assert(!D.system_err && run_tape_turn_held());
    for (i = 0; i < 10; i++) { run_encoder(0, 20000); run_encoder(1, 20000); }   /* 선 직후 굴러감 */
    ticks(2);
    assert(!D.system_err && run_tape_turn_held());
    ticks(50);                                  /* 기준 자리가 잡힌다 */
    for (i = 0; i < 10; i++) { run_encoder(0, 20000); run_encoder(1, 20000); }
    ticks(2);
    assert(!D.system_err && run_tape_turn_held() && (10.0f * P.one_pulse_length < 20.0f));
    for (i = 0; i < 60; i++) { run_encoder(0, 20000); run_encoder(1, 20000); }
    ticks(2);
    assert((D.system_err == ERR_GUIDE_SENSOR) && !run_tape_turn_held());
    printf("  gd pushed ok\n");
}

/* 맡은 채 다른 회전·시퀀스가 시작되면 오류 없이 놓는다(그 뒤 다시 출발해도 헛 258 이 없다) */
static void test_gd_release(void)
{
    int i;
    gd_to_cmd_stop(1, 0u);
    D.turn_motion = MOTION_CW;                  /* 서 있는 자리에서 도는 회전(시퀀스 회전 등) */
    D.turn_pulses = 200;
    ticks(2);
    assert(!run_tape_turn_held() && !D.system_err);
    for (i = 0; i < 300; i++) { run_encoder(0, 20000); run_encoder(1, 20000); }
    ticks(2);
    assert(!D.system_err);

    gd_to_cmd_stop(1, 0u);
    seq_request(SEQ_LOAD);
    for (i = 0; (i < 20) && !seq_is_active(); i++) { guide_reply(0); tick(); }
    ticks(1);
    assert(seq_is_active() && !run_tape_turn_held() && !D.system_err);
    run_seq_init();

    gd_to_cmd_stop(0, 3u);                      /* 분류 4 + 자동 출발: 끊겨 예약을 되돌려 둔 채 시퀀스가 시작 */
    assert(D.tape_autostart == 3u);
    seq_request(SEQ_LOAD);
    for (i = 0; (i < 20) && !seq_is_active(); i++) { guide_reply(0); tick(); }
    ticks(1);
    assert(seq_is_active() && (D.tape_autostart == 0) && !D.system_err);
    run_seq_init();
    printf("  gd release ok\n");
}

/* GD 중 사람 STOP(서기 전에 뗌): 요청과 함께 자동 출발 보관도 지워져, 뒤의 자동 출발 없는 분류 3 정지에서 스스로 떠나지 않는다.
 * 새 분류 3·4 규칙은 그 규칙의 자동 출발만 남긴다(없으면 0) */
static void test_gd_stop_switch(void)
{
    int i;
    h_drive_up(0);
    D.tape_stop_kind = 4;
    D.tape_autostart = 3;
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 2, 1);
    led_run(0x0180u, 5, 1);
    assert(D.steer_hold && D.wait_start);
    raw_a(RAW_STOP, 0);
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0); }
    raw_a(0, RAW_STOP);
    led_run(0x0180u, 10, 0);
    assert(!D.flag_run && !D.wait_start && (D.tape_autostart == 0));
    press_start();
    for (i = 0; (i < 20) && !D.flag_run; i++) { guide_led_reply(0x0180u); tick(); }
    led_run(0x0180u, 80, 1);
    D.tape_stop_kind = 3;                       /* 뒤 태그: 자동 출발 없는 AGV_STOP( TAPE ) */
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 2, 1);
    for (i = 0; (i < 400) && D.flag_run; i++) { led_run(0x01FFu, 1, (D.vin_out > 0) ? 1 : 0); }
    led_run(0x01FFu, 5, 0);
    assert(!D.flag_run && !D.system_err && (run_tape_last.kind == 3));
    for (i = 0; (i < 600) && !D.flag_run && !D.flag_ready; i++) { guide_led_reply(0x0180u); tick(); }
    assert(i == 600);

    boot();
    drive_up();
    run_scan(20);
    D.tape_autostart = 9;                       /* 다른 길로 남은 낡은 값 */
    (void)tagact_run(41);                       /* SEQ_START( TAPE_GD, LOAD ): 자동 출발 없음 */
    assert((D.tape_stop_kind == TA_STOP_KIND_TAPE_GD) && (D.tape_autostart == 0));
    (void)tagact_run(39);                       /* AGV_AUTOSTART( 5 ), AGV_STOP( TAPE ) */
    assert((D.tape_stop_kind == TA_STOP_KIND_TAPE) && (D.tape_autostart == 6u));
    printf("  gd stop switch ok\n");
}

/* 맡은 채 다시 걸 때 목표를 60 mm 넘게 지났으면(끊긴 정지가 목표 뒤에 섰다) 달리는 중에는 오류 없이 제어 정지, 선 뒤 258, 회전 없음 */
static void test_gd_resume_overrun(void)
{
    int i, ran = 0;
    h_drive_up(1);
    D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0x10;
    led_run(0x0180u, 5, 1);
    led_run(0x01FFu, 3, 1);
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    for (i = 0; (i < 900) && D.flag_run; i++) { led_run(0x0180u, 1, (D.vin_out > 0) ? 20 : 0); }   /* 감속 중 크게 밀려 목표 뒤에 선다 */
    led_run(0x0180u, 5, 0);
    assert(!D.system_err && run_tape_turn_held());
    run_wait_start();
    for (i = 0; (i < 900) && (D.flag_run || D.flag_ready || (i < 20)); i++) {
        led_run(0x0180u, 1, (D.vin_out > 0) ? 1 : 0);
        if (D.flag_run && !D.system_err) { ran = 1; }   /* 오류 없이 달렸다가 제어 정지로 선다 */
    }
    led_run(0x0180u, 5, 0);
    assert(ran && (D.system_err == ERR_GUIDE_SENSOR) && !D.flag_turn && (D.control != RUN_TURN));
    assert((run_tape_last.kind == -1) && (run_tape_last.over_mm > 60.0f));
    printf("  gd resume overrun ok\n");
}

/* 맡은 교차선 회전이 있는 동안(회전 거부·GD 끊김)에도 새 태그는 ACK 없이 버린다 */
static void test_gd_held_tag(void)
{
    gd_to_cmd_stop(1, 0u);
    assert(run_tape_turn_held());
    host.sci[HAL_SCI_A].tx_len = 0;
    tag_frame(1, '2', 0);
    run_scan(1);
    assert(!tx_has('A', 'E') && run_tape_turn_held());
    printf("  gd held tag ok\n");
}

/* 추적기 덩어리 칸이 범위 밖(시작 칸 음수 → 0xFFFF)이어도 덩어리 기억이 망가지지 않는다: 한쪽(T) 교차선을 인정 */
static void test_tape_lane_range(void)
{
    h_drive_up(0);
    D.tape_stop_kind = 3;
    led_run(0x0180u, 3, 1);
    run_tape_rx(0xFFFFu, 1u);
    led_run(0x01FFu, 3, 1);                     /* 왼쪽(칸 1~7)만 찬 T */
    assert((D.tape_stop_kind == 0) && D.flag_stopping);
    printf("  tape lane range ok\n");
}

/* ---- 센서 정지 AGV_STOP( 거리, FROM_SENSOR( 입력, 유지틱 ) ) ---- */
static int s_fs_fb;                 /* 실행한 AGV_FORBACK 수(tagact_trace) */
static void fs_trace(uint16_t code, uint16_t next)
{
    (void)next;
    if ((uint16_t)(TA_KIND(code) << 8) == TA_FORBACK) { s_fs_fb++; }
}

/* 확장 입력 보드로 부팅해 달린다. FL 30 cm, 전후진 전환 켬 */
static void fs_up(int back)
{
    s_ext_board = 1;
    boot();
    (void)param_set(P_TURN_FORBACK, 10);
    assert(param_set_ram(P_STOP_LEN_FWD_MM, 30) == 0);
    run_seq_init();                             /* 앞 판의 시퀀스 요청이 START 를 가로채지 않게 */
    if (back) { run_relay(1, 0); }
    drive_up();
    run_scan(20);
    s_ref.on = s_ref.armed = 0;
    s_fs_fb = 0;
}

/* 선 뒤 다시 출발(START) */
static void fs_restart(void)
{
    int i;
    for (i = 0; (i < 400) && (D.flag_run || D.flag_stopping); i++) { run_scan(1); }
    for (i = 0; i < 5; i++) { guide_reply(0); tick(); }
    press_start();
    for (i = 0; (i < 20) && !D.flag_run; i++) { guide_reply(0); tick(); }
    assert(D.flag_run);
}

/* 설 때까지(펄스 없이) 틱, back 이면 전후진 전환(run_relay), ACS 출발 대기로 다시 달린다 */
static void fs_stop_go(int back)
{
    int i;
    for (i = 0; (i < 600) && (D.flag_run || D.flag_stopping); i++) { guide_reply(0); tick(); }
    assert(!D.flag_run && !D.flag_stopping);
    if (back) { run_relay((int16_t)!D.backward, 0); }
    run_wait_start();
    for (i = 0; (i < 50) && !(D.flag_run && !D.flag_stopping); i++) { guide_reply(0); tick(); }
    assert(D.flag_run && !D.flag_stopping);
}

typedef struct { int t, seq; int16_t sp; } fs_res_t;
/* n 틱 달린다(틱마다 두 바퀴 1 펄스). 입력 EXT_4 는 on_at 틱부터 켜짐, gap_at 부터 gap 틱 꺼짐. 처음 정지가 걸린 틱·정지 펄스·시퀀스 */
static void fs_run(int n, int on_at, int gap_at, int gap, fs_res_t *r)
{
    int t;
    r->t = -1;
    for (t = 0; t < n; t++) {
        ext_in(0x08u, (t >= on_at) && !((t >= gap_at) && (t < gap_at + gap)));
        run_scan(1);
        if ((r->t < 0) && D.flag_stopping) {
            r->t = t;
            r->sp = D.stop_pulses;
            r->seq = seq_request_flag[SEQ_LOAD];
        }
    }
    ext_in(0x08u, 0);
}

static void test_from_sensor(void)
{
    fs_res_t a, b;
    int i;

    tagact_trace = fs_trace;
    /* 노이즈 없는 입력: 사이트 C 참조와 같은 틱·같은 정지 거리. 처음 켜진 틱 10 + 유지 100 = 110. 뒤 동작은 확정 때 */
    for (i = 0; i < 2; i++) {                   /* 전진·후진 */
        fs_up(i);
        s_ref.on = 1;
        (void)tagact_run(52);
        fs_run(200, 10, 999, 0, &a);
        fs_up(i);
        (void)tagact_run(49);
        assert(!seq_request_flag[SEQ_LOAD] && (s_fs_fb == 0));   /* 태그 때는 서지도, 뒤 동작도 않는다 */
        fs_run(200, 10, 999, 0, &b);
        assert((a.t == 110) && (b.t == a.t) && (b.sp == a.sp) && a.seq && b.seq && (s_fs_fb == 1) && !D.stop_err);
        run_seq_init();
    }
    /* 꺼짐 3틱은 무시(기준·틱 그대로), 6틱은 처음부터(다시 켜진 틱이 새 기준) */
    fs_up(0);
    (void)tagact_run(49);
    fs_run(200, 10, 40, 3, &b);
    assert((b.t == 110) && (b.sp == a.sp));
    run_seq_init();
    fs_up(0);
    (void)tagact_run(49);
    fs_run(250, 10, 40, 6, &b);
    assert((b.t == 146) && (b.sp == a.sp));
    run_seq_init();
    /* 확정 때 이미 거리(10 mm)를 넘었다: 바로(1 mm) */
    fs_up(0);
    (void)tagact_run(50);
    fs_run(100, 5, 999, 0, &b);
    assert((b.t == 25) && (b.sp == (int16_t)(1.0f * P.one_length_pulse)));
    /* 다시 읽으면 처음부터 */
    fs_up(0);
    (void)tagact_run(49);
    fs_run(50, 0, 999, 0, &b);
    (void)tagact_run(49);
    fs_run(200, 0, 999, 0, &b);
    assert(b.t == 100);
    run_seq_init();
    /* 앞 동작은 태그 때, 뒤 동작은 확정 때 */
    fs_up(0);
    D.obs_use = 1;
    (void)tagact_run(53);
    assert((D.obs_use == 0) && (s_fs_fb == 0));
    fs_run(30, 0, 999, 0, &b);
    assert((b.t == 10) && (s_fs_fb == 1));
    /* 서 있거나 서는 동안은 기다린다: 다시 달린 첫 틱이 기준 */
    fs_up(0);
    (void)tagact_run(49);
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    ext_in(0x08u, 1);
    for (i = 0; (i < 400) && D.flag_run; i++) { run_scan(1); }
    for (i = 0; i < 150; i++) { guide_reply(0); tick(); }
    assert(!D.flag_run && !seq_request_flag[SEQ_LOAD]);    /* 선 채 켜진 150 틱은 세지 않았다 */
    ext_in(0x08u, 0);
    run_wait_start();
    for (i = 0; (i < 20) && !D.flag_run; i++) { guide_reply(0); tick(); }
    assert(D.flag_run && !D.flag_stopping);
    fs_run(200, 0, 999, 0, &b);
    assert((b.t == 100) && b.seq);
    run_seq_init();
    /* STOP·오류에서 지운다 */
    fs_up(0);
    (void)tagact_run(49);
    raw_a(RAW_STOP, 0);
    ticks(3);
    assert(D.stop_sw);
    raw_a(0, RAW_STOP);
    ticks(3);
    fs_restart();
    fs_run(200, 0, 999, 0, &b);
    assert(!seq_request_flag[SEQ_LOAD] && (s_fs_fb == 0));   /* 확정(뒤 동작)이 없다 */
    fs_up(0);
    (void)tagact_run(49);
    D.system_err = ERR_GUIDE_SENSOR;
    ticks(2);
    D.system_err = 0;
    fs_restart();
    fs_run(200, 0, 999, 0, &b);
    assert(!seq_request_flag[SEQ_LOAD] && (s_fs_fb == 0));
    /* ACS 작업 취소('C' 2·3, 'W' 'C' 2·3)에서 지운다: 선 뒤 다시 출발해도 감지 자리에서 서지 않는다 */
    for (i = 0; i < 4; i++) {
        char f[32];
        fs_up(0);
        (void)tagact_run(49);
        fs_run(20, 999, 999, 0, &b);
        if (i < 2) {
            sprintf(f, "C1B%02dC00%c", AGV_ID, '2' + i);    /* 'C' 프레임은 12 바이트 */
        }
        else {
            sprintf(f, "W1B%02dC00000000000", AGV_ID);
            f[8] = '0';                                  /* 교통 정지로 선 뒤 */
            send_frame(f, 0);
            f[2] = 'C';
            f[8] = (char)('2' + i - 2);
        }
        send_frame(f, 0);
        fs_stop_go(0);
        fs_run(200, 0, 999, 0, &b);
        assert((b.t < 0) && !seq_request_flag[SEQ_LOAD] && (s_fs_fb == 0));
        run_seq_init();
    }
    /* 전·후진 방향 전환(run_relay: AGV_FORBACK·수동 스위치·ACS 'C' 4·5 가 모두 이리로)에서 지운다. 명령 정지만으로는 남는다(위) */
    fs_up(0);
    (void)tagact_run(49);
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    fs_stop_go(1);
    assert(D.backward);
    fs_run(200, 0, 999, 0, &b);
    assert((b.t < 0) && !seq_request_flag[SEQ_LOAD]);
    run_seq_init();
    /* 5 m 안에 감지가 시작되지 않으면 서고 270(전진·후진) */
    for (i = 0; i < 2; i++) {
        int t, lim = (int)(5000.0f * P.one_length_pulse);
        fs_up(i);
        (void)tagact_run(49);
        for (t = 0; (t < 9000) && !D.stop_err; t++) { run_scan(1); }
        assert((D.stop_err == ERR_SENSOR_STOP_MISS) && (t >= lim) && (t <= lim + 3));
    }
    /* 범위 규칙의 센서 정지(펼친 표·압축 표 같은 틱·거리): 앞에서 태그 때는 아무것도, 뒤 동작은 확정 때.
     * 확정 때 보고의 TAG_NUM 도 읽은 태그 번호 */
    for (i = 0; i < 2; i++) {
        fs_res_t c;
        fs_up(0);
        if (i) { tagact_bind_packed(&s_tag_pk, 0); }
        (void)tagact_run(152);
        assert(!seq_request_flag[SEQ_LOAD] && (s_fs_fb == 0) && !D.flag_stopping);
        fs_run(40, 5, 999, 0, &c);
        assert((c.t == 15) && c.seq && (s_fs_fb == 1) && !D.stop_err);
        if (i == 0) { b = c; } else { assert((c.t == b.t) && (c.sp == b.sp)); }
        run_seq_init();
        fs_up(0);
        if (i) { tagact_bind_packed(&s_tag_pk, 0); }
        (void)tagact_run(163);
        host.sci[HAL_SCI_A].tx_len = 0;
        fs_run(40, 5, 999, 0, &c);
        ticks(2);
        assert((c.t == 15) && (tx_num('E', 'A') == 163));
        run_seq_init();
    }
    /* 교차선 요청과 한 규칙: 표 오류 */
    fs_up(0);
    (void)tagact_run(51);
    assert(D.stop_err == (int16_t)SEQ_ERR_TABLE);
    s_ext_board = 0;
    tagact_trace = 0;
    printf("  from sensor ok\n");
}

/* 전원을 켤 때 MODE + UP: 입력 번호 표시, 출력 번호 선택, SET 켬, RESET 끔 */
static void test_io_test(void)
{
    uint16_t d[5];
    int i, tx;

    host_reset();
    memset(&D, 0, sizeof(D));
    for (i = BUS_IN_A; i <= BUS_IN_D; i++) { host.bus_in[i] = 0xFF; }
    host.bus_in[BUS_EXT_IN_A] = host.bus_in[BUS_EXT_IN_B] = 0xFF;
    host.key = KEY_MODE + KEY_UP;
    sys_boot();
    ticks(60);
    assert(!sys_power_on());
    ui_digits(d);
    assert((d[0] == 30) && (d[1] == 15) && (d[2] == 29) && (d[3] == 30));   /* TEST */
    host.key = 0;
    ticks(5);
    assert(D.ui_mode == UI_IO_IN);
    assert(!sys_power_on());

    host.bus_in[BUS_IN_D] &= (uint16_t)~4u;           /* 번호 16 */
    ticks(2);
    ui_digits(d);
    assert((d[3] == 1) && (d[4] == 6));
    host.bus_in[BUS_IN_A] &= (uint16_t)~4u;           /* 번호 1 이 앞선다 */
    ticks(2);
    ui_digits(d);
    assert((d[3] == 0) && (d[4] == 1));
    host.bus_in[BUS_IN_A] |= 4u;
    host.bus_in[BUS_IN_D] |= 4u;

    key(KEY_MODE);
    assert(D.ui_mode == UI_IO_OUT);
    assert(host.bus_out[BUS_LAMP] == 0x7Fu);          /* 전부 꺼짐 */
    key(KEY_SET);                                     /* 1 = AGV_BACK */
    assert(!(host.bus_out[BUS_LAMP] & 0x08));
    key(KEY_RESET);
    assert(host.bus_out[BUS_LAMP] & 0x08);
    key(KEY_DOWN);                                    /* 아래 한계 1 */
    ui_digits(d);
    assert((d[3] == 0) && (d[4] == 1));

    key(KEY_SHIFT);
    key(KEY_UP);
    key(KEY_UP);                                      /* 21 = 확장 출력 1 */
    ui_digits(d);
    assert((d[3] == 2) && (d[4] == 1));
    key(KEY_SET);
    assert(!(host.bus_out[BUS_EXT_OUT_A] & 0x01));
    key(KEY_UP);
    key(KEY_UP);                                      /* 위 한계 36 = 확장 출력 16 */
    ui_digits(d);
    assert((d[3] == 3) && (d[4] == 6));
    key(KEY_SET);
    assert(!(host.bus_out[BUS_EXT_OUT_B] & 0x80));

    key(KEY_DOWN);
    key(KEY_DOWN);
    key(KEY_DOWN);                                    /* 6 */
    key(KEY_SHIFT);
    for (i = 0; i < 4; i++) { key(KEY_UP); }          /* 10 = 스피커 */
    key(KEY_SET);
    assert(host.sound == 1);
    assert(!(host.bus_out[BUS_AMP] & 0x44));          /* 앰프 런 꺼짐 */

    tx = host.sci[HAL_SCI_A].tx_len;
    tag_frame(2, '1', 0);
    ticks(2);
    assert(host.sci[HAL_SCI_A].tx_len == tx);         /* 시험 중에는 통신도 멈춘다 */
    printf("  io test ok\n");
}

#include "agv/steerlog.h"
#if AGV_DEV_TESTS
static void sl_scan(uint16_t leds)
{
    host.sci[HAL_SCI_B].rx_len = host.sci[HAL_SCI_B].rx_pos = 0;
    guide_led_reply(leds);
    run_encoder(0, 20000);
    run_encoder(1, 20000);
    tick();
}

static const steerlog_smp_t *sl_last(void)
{
    return &agv_steerlog.s[(agv_steerlog.w + STEERLOG_N - 1u) % STEERLOG_N];
}

static const steerlog_ev_t *sl_find(int16_t kind)
{
    uint16_t i, n = (agv_steerlog.ev_n < STEERLOG_EV) ? agv_steerlog.ev_n : STEERLOG_EV;
    for (i = 0; i < n; i++) { if (agv_steerlog.ev[i].kind == kind) { return &agv_steerlog.ev[i]; } }
    return 0;
}

/* 조향 기록기: 달리면 표본이 쌓이고(w_cmd 를 풀 수 있다), 위치가 튀면 사건이 직전·이번 LED 와 남고, 서면 언다 */
static void test_steerlog(void)
{
    const steerlog_smp_t *s;
    const steerlog_ev_t *e;
    uint32_t total;
    int k, p, off;

    boot();
    assert(agv_steerlog.total == 0 && agv_steerlog.runs == 0);
    assert(param_set_ram(P_LEGACY_QUIRKS, 0) == 0);
    drive_up();
    for (k = 0; k < 700; k++) {
        sl_scan(LED_CENTER);
        if (agv_steerlog.total > 0u) {      /* w_cmd 풀기 = motor_speed_pi 의 RUN_DRIVE 식 */
            s = sl_last();
            p = s->p2 >> 1;
            off = s->io >> 8;
            assert(D.w_cmd[0] == ((s->v_cmd + p + off > 0) ? s->v_cmd + p + off : 0));
            assert(D.w_cmd[1] == ((s->v_cmd - p - off > 0) ? s->v_cmd - p - off : 0));
        }
    }
    assert(D.vin_out > 1000 && agv_steerlog.runs == 1);
    assert(agv_steerlog.total > STEERLOG_N && agv_steerlog.n == STEERLOG_N);
    assert(agv_steerlog.w == agv_steerlog.total % STEERLOG_N);
    assert(agv_steerlog.cfg.los == P.low_speed && agv_steerlog.cfg.drs == P.target_speed && agv_steerlog.cfg.dcc == P.dcc);
    s = sl_last();
    assert((s->pf & 0xFFu) == 0 && ((s->pf >> 8) & 7u) == (uint16_t)(D.step - DRIVE_CHANGE_SPEED));
    assert(s->v_cmd == D.v_cmd && s->rpm[0] == D.rpm[0] && s->rpm[1] == D.rpm[1]);
    assert(agv_steerlog.ev_n == 0);

    D.pause_stop = 1;                               /* 일시정지로 서면 언다 */
    for (k = 0; (k < 500) && (D.vin_out != 0); k++) { sl_scan(LED_CENTER); }
    assert((D.vin_out == 0) && (D.control == RUN_DRIVE));
    total = agv_steerlog.total;
    for (k = 0; k < 50; k++) { sl_scan(LED_CENTER); }
    assert(agv_steerlog.total == total && agv_steerlog.runs == 1);
    /* 다시 출발: 표본·사건을 새로 시작. 일시정지 재출발 절차 대신 기록기 입력(vin_out)만 바꿔 직접 부른다 */
    D.vin_out = 300;
    steerlog_tick();
    assert(agv_steerlog.runs == 2 && agv_steerlog.total == 1u && agv_steerlog.n == 1u && agv_steerlog.w == 1u);
    D.vin_out = 0;
    steerlog_tick();
    assert(agv_steerlog.total == 1u);
    boot();                                         /* 위치 튐 사건은 새 주행에서 */
    assert(agv_steerlog.total == 0 && agv_steerlog.runs == 0);
    assert(param_set_ram(P_LEGACY_QUIRKS, 0) == 0);
    drive_up();
    for (k = 0; k < 400; k++) { sl_scan(LED_CENTER); }
    assert(D.vin_out > 1000 && agv_steerlog.ev_n == 0 && agv_steerlog.runs == 1);

    sl_scan(LED_CENTER << 6);                       /* 8·9번 칸 → 14·15번 칸: 위치 0 → +12 */
    assert(D.guide_pos == 12 && D.risk_stop == 1);
    e = sl_find(STEERLOG_EV_JUMP);
    assert(e && e->prev_pos == 0 && e->pos == 12 && e->prev_lo == LED_CENTER && e->lo == (LED_CENTER << 6) && e->hi == 0);
    assert(e->dist_mm > 0 && e->vin_out == D.vin_out && e->tick == (uint16_t)(agv_steerlog.total - 1u));
    e = sl_find(STEERLOG_EV_RISK);
    assert(e && e->prev_pos == 0 && e->pos == 12 && e->prev_lo == LED_CENTER && e->lo == (LED_CENTER << 6));
    assert((sl_last()->p2 & 1) == 1 && (sl_last()->pf & 0xFFu) == 12u);
    total = agv_steerlog.total;
    printf("  steerlog ok (%lu 표본, 사건 %u)\n", (unsigned long)total, agv_steerlog.ev_n);
}

#else
#ifdef STEERLOG_N
#error "AGV_DEV_TESTS 0 이미지에는 조향 기록기가 없어야 한다"
#endif
#endif

int main(void)
{
    setvbuf(stdout, 0, _IONBF, 0);
    signal(SIGABRT, on_abort);
    test_boot_garbage();
    test_link();
    test_boot();
    test_scheduler();
    test_storage();
    test_keep_restore();
    test_start_drive();
    test_ready_timeout();
    test_guide_dropout();
    test_guide_lost_distance();
    test_sensor_switch();
    test_estop();
    test_bumper();
    test_amp_alarm();
    test_stop_switch_aborts();
    test_tag();
    test_rx_tag_reader();
    test_tag_cond();
    test_compensate();
    test_rfid_retry();
    test_cross();
    test_cross_out();
    test_status_report();
    test_ui();
    test_menu5();
#if AGV_DEV_TESTS
    test_s_tag();                       /* 'S' 처리는 두 빌드 같다(comm.c). 개발 빌드에서만 돌린다 */
    test_move_tag_ignore();
    test_tst();
    test_wt_ff();
#endif
    test_load_profile();
    test_load_while_running();
    test_para_init();
    test_lamp();
    test_align_turn();
    test_turn_speed();
    test_seq_turn_speed();
    test_tag_standby();
    test_tag_stop_class();
    test_tape_turn_request();
    test_move_cross_tag();
    test_tune_entry();
    test_ee_probe();
    test_tune_menu();
    test_tune_nav();
    test_tune_inputs();
    test_fnd_fit();
    test_tune_ee_layout();
    test_tune_t2_compare();
    test_tune_t24_amp();
    test_tune_guide();
#if AGV_DEV_TESTS
    test_tune_gr();
#endif
    test_guide_h_only();
    test_curvelog();
#if AGV_DEV_TESTS
    test_steerlog();
#endif
    test_edge_release();
    test_turn_scale();
    test_tape_turn();
    test_tape_rule();
    test_tape_guard();
    test_tape_rule2();
    test_tape_turn_paused();
    test_tape_gd_cmd_stop();
    test_move_gd_cmd();
    test_gd_traffic_reset();
    test_tape_seek_len();
    test_gd_overrun();
    test_gd_autostart();
    test_gd_pushed();
    test_gd_release();
    test_gd_held_tag();
    test_gd_stop_switch();
    test_gd_resume_overrun();
    test_tape_lane_range();
    test_from_sensor();
    test_io_test();
    test_acs_off();
    test_keep_results();
    printf("test_sys ok\n");
    return 0;
}
