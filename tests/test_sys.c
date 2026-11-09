/* test_sys.c - 2·3단계 통합 시험. hal/host 위에서 부팅부터 10 ms 스캔까지 돌린다 */
#include "agv/tune.h"
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "agv/acs.h"
#include "agv/comm.h"
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
#include "agv/seq_dsl.h"
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

static const seq_def_t s_seqs[] = { { 0, SEQ_LOAD, s_load_steps, sizeof(s_load_steps) / sizeof(s_load_steps[0]) } };
static const seq_def_t s_acts[] = { { 0, ACT_HOLD, a_hold_steps, sizeof(a_hold_steps) / sizeof(a_hold_steps[0]) } };
static int stop_len_call(seq_ctx_t *c) { (void)c; tagact_stop_length(); return 1; }
static const seq_call_fn s_calls[] = { stop_len_call };
static const seq_site_t s_site = { s_seqs, 1, s_acts, 1, s_calls, 1, 0, 0 };

/* 태그 표 */
static const tag_rule_t s_rules[] = {
    { 1, 0, 0, 0, 0, { TA_STOP, 0, 0, 0 } },
    { 2, 0, 0, 0, 0, { TA_SPEED | 2, 0, 0, 0 } },
    { 4, 0, 0, 0, 0, { TA_SEQ | SEQ_LOAD, 0, 0, 0 } },
    { 5, 0, 0, 0, 0, { TA_ACT | ACT_HOLD, 0, 0, 0 } },
    { 8, 0, 0, 0, 0, { TA_CALL | 0, 0, 0, 0 } },
    { 9, 0, TAG_COND_BACKWARD, 0, 0, { TA_STOP, 0, 0, 0 } },
    /* 확장 조건 둘 + 입력. 시험은 tagact_dry_run 으로 맞은 규칙만 본다 */
    TAG_IF( 20, 0, IF_X2_IN( X_ID(AGV_ID, AGV_ID), X_STATION(1, 42, 42), IN( ON(IN_START_SW) ) ), A_SPEED_LOW )
    TAG_IF( 20, 0, IF_X2( X_ID(AGV_ID, AGV_ID), X_STATION(1, 42, 42) ), A_SPEED_MIN )
    TAG_IF( 20, 0, IF_ID( 8, 9 ), A_SPEED_HIGH )
    TAG_IF( 21, 0, IF_TAG_MINUS_EQ_STATION( 0, 10 ), A_OBS_ON )
    TAG_IF( 21, 0, IF_TAG_EQ_STATION( 0 ), A_OBS_OFF )
    TAG_IF( 22, 0, IF_STATION_MOD( 0, 10, 3 ), A_OBS_ON )
    TAG_IF( 22, 0, IF_STATION_DIV( 0, 10, 4 ), A_OBS_OFF )
    TAG_IF( 23, 0, IF_IN( ANY_ON(IN_EXT_16, IN_START_SW) ), A_OBS_ON )
    TAG_IF( 23, 0, IF_X( X_NOT(X_CROSS_IN) ), A_OBS_OFF )
};

static int s_manual_count;
void site_register(void)
{
    param_default_types(0, 0, 0);
    (void)seq_bind(&s_site);
    tagact_bind(s_rules, sizeof(s_rules) / sizeof(s_rules[0]), 0);
}
void site_manual(void) { s_manual_count++; }

/* ---- 도구 ---- */

static void tick(void)
{
    int i;
    for (i = 0; i < 5; i++) { sys_tick_2ms(); }
    sys_scan();
}
static void ticks(int n) { while (n-- > 0) { tick(); } }

static void boot(void)
{
    host_reset();
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
static void test_tag_cond(void)
{
    uint16_t a[4];

    boot();
    acs_station_set(1, 42, 2);
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == A_SPEED_MIN));
    raw_a(RAW_START, 0);                        /* START 누름 */
    tick();
    assert(io_in(IN_START_SW));
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == A_SPEED_LOW));
    assert((tagact_dry_run(23, a, 4) == 1) && (a[0] == A_OBS_ON));
    raw_a(0, RAW_START);
    tick();
    assert((tagact_dry_run(23, a, 4) == 1) && (a[0] == A_OBS_OFF));
    D.cross_in_flag = 1;
    assert(tagact_dry_run(23, a, 4) == 0);
    D.cross_in_flag = 0;
    acs_station_set(1, 43, 2);
    assert(tagact_dry_run(20, a, 4) == 0);
    acs_station_set(1, 42, 2);
    (void)param_set(P_AGV_ID, 8);
    assert((tagact_dry_run(20, a, 4) == 1) && (a[0] == A_SPEED_HIGH));
    (void)param_set(P_AGV_ID, AGV_ID);

    acs_station_set(0, 11, 2);                  /* 21 - 10 == 11 */
    assert((tagact_dry_run(21, a, 4) == 1) && (a[0] == A_OBS_ON));
    acs_station_set(0, 21, 2);
    assert((tagact_dry_run(21, a, 4) == 1) && (a[0] == A_OBS_OFF));
    acs_station_set(0, 43, 2);
    assert((tagact_dry_run(22, a, 4) == 1) && (a[0] == A_OBS_ON));
    acs_station_set(0, 45, 2);
    assert((tagact_dry_run(22, a, 4) == 1) && (a[0] == A_OBS_OFF));
    acs_station_set(0, 55, 2);
    assert(tagact_dry_run(22, a, 4) == 0);
    acs_station_reset();
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
    key(KEY_SET);                                     /* USE */
    key(KEY_SET);                                     /* 첫 번호 */
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
    printf("  ui ok\n");
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
    key(KEY_UP);                                      /* OTHER */
    key(KEY_SET);
    for (i = 0; (i < 40) && (ui_para_number() != P_PARA_INIT); i++) { key(KEY_UP); }
    assert(ui_para_number() == P_PARA_INIT);
    key(KEY_SET);
    /* 값 88 = 십의 자리 8 */
    key(KEY_SHIFT);
    for (i = 0; i < 8; i++) { key(KEY_UP); }
    key(KEY_SHIFT);
    for (i = 0; i < 8; i++) { key(KEY_UP); }
    key(KEY_SET);
    assert(param_get(P_SPEED_HIGH) == 25);            /* 세트 2 */
    printf("  77/88/99 ok\n");
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
    agv_coast.p[3].dist_mm = 700;
    agv_stopt.state = 2;
    boot();
    assert(agv_stopt.state == 0);
    assert(agv_ident.state == 0 && agv_ident.track_mm == 0.0f && test_keep_mark != 0);
    assert(agv_coast.state == 0 && agv_coast.p[3].dist_mm == 0);
    agv_ident.state = 2;
    agv_ident.track_mm = 5000.0f;
    agv_top.state = 1;
    agv_all.state = 1;
    agv_all.fail = 2034;
    agv_coast.state = 1;
    agv_coast.p[3].dist_mm = 700;
    agv_rpmt.state = 1;
    agv_stopt.state = 1;
    agv_stopt.p[5].overshoot_mm = 150.0f;
    boot();
    assert(agv_ident.state == 2 && agv_ident.track_mm == 5000.0f && agv_ident.fail == 0);
    assert(agv_top.state == 3 && agv_top.fail == -1);
    assert(agv_all.state == 3 && agv_all.fail == 2034);
    assert(agv_coast.state == 3 && agv_coast.fail == -1 && agv_coast.p[3].dist_mm == 700);
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
    turn_scan(LED_CENTER, 0);
    assert((D.tape_align == 0) && (D.turn_pulses == 0));
    assert(D.system_err == 0);

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
    assert(tune_active() && (tune_state() == TUNE_WAIT) && (tune_menu() == 100));
    assert(shows("T1   ") && (D.ui_mode == UI_AUTO));
    printf("  tune entry ok\n");
}

/* 메뉴: MODE 같은 단계 다음, SET 아래로, RESET 위로. MODE 는 파라미터·조그 화면으로 가지 않는다 */
static void test_tune_menu(void)
{
    uint16_t d[5];
    int k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_MODE);
    assert(tune_menu() == 200 && shows("T2   "));
    key(KEY_MODE);
    assert(tune_menu() == 300 && shows("T3   "));
    key(KEY_MODE);
    assert(tune_menu() == 100);
    key(KEY_SET);                                     /* t1 입력: 지름, GD, 상한, L */
    assert(tune_menu() == 110 && shows("DI202"));
    key(KEY_MODE);
    assert(tune_menu() == 120 && shows("GD250"));
    key(KEY_MODE);
    assert(tune_menu() == 130 && shows("TP060"));     /* 빈 EEPROM 상한 60 */
    key(KEY_MODE);
    assert(tune_menu() == 140 && shows("L0000"));
    key(KEY_MODE);
    assert(tune_menu() == 110);
    key(KEY_RESET);
    assert(tune_menu() == 100);
    key(KEY_MODE);
    key(KEY_SET);
    assert(tune_menu() == 210 && shows("T21CH"));
    ui_digits(d);
    for (k = 0; k < 3; k++) { key(KEY_MODE); }
    assert(tune_menu() == 240 && shows("T24AP"));
    key(KEY_SET);                                     /* L 입력 */
    assert(tune_menu() == 241 && shows("L0000"));
    key(KEY_MODE);
    assert(tune_menu() == 241);                       /* 한 칸뿐 */
    key(KEY_RESET);
    key(KEY_MODE);
    assert(tune_menu() == 210);
    key(KEY_MODE);
    key(KEY_MODE);
    key(KEY_SET);                                     /* t2.3 은 L 이 없다 */
    assert(tune_menu() == 230 && shows("T23GS"));
    key(KEY_RESET);
    key(KEY_RESET);
    assert(tune_menu() == 200);
    key(KEY_MODE);
    key(KEY_SET);
    assert(tune_menu() == 310 && tune_ident_mode() && shows("T31ID"));
    for (k = 0; k < 4; k++) { key(KEY_MODE); }
    assert(tune_temp_mode() && !tune_all_mode() && shows("T35TE"));
    key(KEY_MODE);
    assert(tune_hs_mode() && shows("T36HS"));
    key(KEY_SET);                                     /* 속도 입력. 기본 1번(30), 1번 위로는 못 간다 */
    assert(tune_menu() == 361 && shows("SP030"));
    key(KEY_SET);
    key(KEY_UP);
    key(KEY_SET);
    assert(shows("SP030") && tune_hs_mpm() == 30);
    key(KEY_SET);
    key(KEY_DOWN);
    key(KEY_SET);
    assert(shows("SP029") && tune_hs_mpm() == 29 && host.ee[TUNE_EE_CAP] != 29);   /* RAM 에만 */
    key(KEY_RESET);
    assert(tune_menu() == 360);
    key(KEY_MODE);
    assert(tune_ident_mode());
    assert(D.ui_mode == UI_AUTO && tune_active() && !D.flag_jog_on);
    printf("  tune menu ok\n");
}

/* 입력: SET 수정, SHIFT 자리, UP·DOWN 값, SET 확정 즉시 저장, RESET 취소. L·상한은 다시 켜도 남는다 */
static void test_tune_inputs(void)
{
    int k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    key(KEY_SET);                                     /* t1 지름 */
    key(KEY_SET);
    assert(tune_editing() && tune_flick() == 0x10u);
    key(KEY_UP);
    key(KEY_SHIFT);
    assert(tune_flick() == 0x08u);
    key(KEY_UP);
    assert(shows("DI213") && param_get(P_WHEEL_DIA_MM) == 202);   /* 확정 전 */
    key(KEY_SET);
    assert(!tune_editing() && param_get(P_WHEEL_DIA_MM) == 213);
    ticks(10);
    assert(host.ee[P_WHEEL_DIA_MM] == 213);
    key(KEY_SET);
    key(KEY_UP);
    key(KEY_RESET);                                   /* 취소 */
    assert(!tune_editing() && param_get(P_WHEEL_DIA_MM) == 213 && tune_menu() == 110);
    key(KEY_MODE);
    key(KEY_MODE);                                    /* 상한 */
    key(KEY_SET);
    key(KEY_SHIFT);
    key(KEY_DOWN);                                    /* 50 */
    key(KEY_SHIFT);                                   /* 두 자리 칸은 1 → 2 → 1 */
    for (k = 0; k < 5; k++) { key(KEY_UP); }
    key(KEY_SET);
    assert(shows("SAVE "));                           /* 저장 알림 1 s */
    ticks(100);
    assert(shows("TP055"));
    key(KEY_MODE);                                    /* L 네 자리 */
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
    assert(host.ee[TUNE_EE_TRACK] == 7005 && host.ee[TUNE_EE_CAP] == 55);
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_SET);
    key(KEY_MODE);
    key(KEY_MODE);
    assert(shows("TP055"));
    key(KEY_MODE);
    assert(shows("L7005"));
    printf("  tune inputs ok\n");
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

/* 결과 쪽을 SAVE 까지 UP 으로 넘긴다 */
static void to_save(void)
{
    int k;
    for (k = 0; (k < 30) && !shows("SAVE "); k++) { key(KEY_UP); }
    assert(shows("SAVE "));
}

/* t1 결과: 채택 저장 1·13·35 는 SET 채택, RESET 건너뛰기, SAVE 에서만 쓴다. 51번 추천은 없다. 기준값을 쓰고 RBOT */
static void test_tune_t1_save(void)
{
    uint16_t before[128];
    int a, k;

    boot_keys(KEY_MODE + KEY_SET, 0);
    drive_up();                      /* t1 에서 START */
    run_scan(3);
    assert(tune_state() == TUNE_BLD && ident_build_mode() && agv_build.mask != 0);
    assert(shows("B1008"));          /* 단계 1, 0회, 선 찾기 */
    agv_build.high_rec = 25;
    agv_build.dcc_rec = 35;
    agv_build.cid_rec = 40;
    agv_build.se_x10 = 43;           /* 편차 단계 4.3 mm */
    agv_build.min_rec = 7;
    agv_build.wb_new = 61;
    agv_build.top_rpm = 3000;
    agv_build.dec_rpm_s = 3500;
    agv_build.lr_permille = -4;
    agv_build.amp = 'S';
    fake_build_done();
    ticks(200);
    memcpy(before, host.ee, sizeof(before));
    /* 1번: 지금 30 과 새 25 를 1 s 씩. 새 값이면 둘째 자리 점 */
    for (k = 0; (k < 250) && !shows(" 1025"); k++) { tick(); }
    assert(shows(" 1025"));
    for (k = 0; (k < 250) && !shows(" 1030"); k++) { tick(); }
    assert(shows(" 1030"));
    key(KEY_SET);                    /* 채택 → 다음 */
    assert(tune_page() == 1);
    key(KEY_RESET);                  /* 13번 건너뛰기 */
    key(KEY_SET);                    /* 35 */
    assert(tune_page() == 3);        /* 값 쪽. 51번 채택 쪽은 없다 */
    key(KEY_DOWN);
    key(KEY_DOWN);
    key(KEY_DOWN);                   /* 1번으로 돌아가도 채택 표시(첫 자리 점)가 남는다 */
    assert(tune_page() == 0);
    ticks(5);
    {
        char c[5];
        assert(tune_text(c) & 1u);
    }
    /* 저장 전에는 아무것도 쓰지 않았다 */
    assert(param_get(P_SPEED_HIGH) == 30 && param_get(P_CROSS_IN_CM) == 60 && param_get(P_OPT_FLAGS) == 0);
    assert(!tune_base_valid());
    to_save();
    key(KEY_SET);
    ticks(100);
    assert(tune_state() == TUNE_SAVED && shows("RBOT "));
    ticks(50);
    assert(param_get(P_SPEED_HIGH) == 25 && param_get(P_DCC) == 40 && param_get(P_CROSS_IN_CM) == 40);
    assert(param_get(P_OPT_FLAGS) == 0 && host.ee[116] == before[116]);
    assert(host.ee[P_SPEED_HIGH] == 25 && host.ee[P_DCC] == 40 && host.ee[P_CROSS_IN_CM] == 40);
    assert(tune_base_valid() && tune_base(TUNE_BASE_TOP) == 3000 && tune_base(TUNE_BASE_DEC) == 3500);
    assert(tune_base(TUNE_BASE_LR) == -4 && ((uint16_t)tune_base(TUNE_BASE_AMP_SE) >> 8) == 'S');
    assert(((uint16_t)tune_base(TUNE_BASE_AMP_SE) & 0xFFu) == 43u);
    /* 쓴 칸: 체크섬 0, 채택한 1·35, 기준값 110~114·119·125 뿐 */
    for (a = 0; a < 128; a++) {
        int base = (a == TUNE_EE_CHECK) || (a == 111) || (a == 112) || (a == 113) || (a == 114) || (a == 119) || (a == 125);
        int adopt = (a == 0) || (a == P_SPEED_HIGH) || (a == P_CROSS_IN_CM);
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
    printf("  tune ee layout ok\n");
}

/* t2.1 빠른 점검: 기준 대비 %, NG 는 이름이 깜박인다. 추천은 채택하지 않으면 안 쓴다. SAVE 에서 RESET 은 메뉴로 */
static void test_tune_t2_compare(void)
{
    int k;

    test_tune_t1_save();             /* 기준값 TOP 3000, DEC 3500 */
    boot_keys(KEY_MODE + KEY_SET, 1);
    key(KEY_MODE);
    key(KEY_SET);                    /* t2.1 */
    assert(tune_base_valid());
    drive_up();
    run_scan(3);
    assert(tune_state() == TUNE_BLD && shows("C1008"));
    agv_build.high_rec = 22;
    agv_build.top_rpm = 2700;        /* -10% NG */
    agv_build.dec_rpm_s = 3400;      /* -2% */
    agv_build.over_mm = 25;          /* NG */
    agv_build.lr_permille = 15;      /* NG */
    agv_build.se_x10 = 70;           /* 기준 43 의 +63% NG */
    fake_build_done();
    assert(tune_page() == 0 && tune_flick() == 0);
    key(KEY_RESET);                  /* 1번 건너뛰기 */
    for (k = 0; (k < 20) && !shows("T-010"); k++) { key(KEY_UP); }
    assert(shows("T-010") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("D-002") && tune_flick() == 0);
    key(KEY_UP);
    assert(shows("FL---"));          /* 정지가 없었다 */
    key(KEY_UP);
    assert(shows("ST025") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("SE070") && tune_flick() == 0x03u);
    key(KEY_UP);
    assert(shows("LR015") && tune_flick() == 0x03u);
    to_save();
    key(KEY_RESET);                  /* 저장 없이 메뉴로 */
    assert(tune_state() == TUNE_WAIT && tune_menu() == 210);
    ticks(50);
    assert(param_get(P_SPEED_HIGH) == 25 && host.ee[P_SPEED_HIGH] == 25);
    assert(tune_base(TUNE_BASE_TOP) == 3000);
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
    key(KEY_MODE);
    key(KEY_SET);
    for (k = 0; k < 3; k++) { key(KEY_MODE); }   /* t2.4 */
    assert(tune_base_valid() && tune_menu() == 240);
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
        tune_on_h(&rx);
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

/* 위치 소스 'h' 만(38번 2): 연속 시험 선 찾기 중에도 매 틱 'H' 만 보낸다('a'·'h' 번갈음 없음).
 * 위치는 LED 덩어리에 h_scale 을 곱해 'a' 단위로 쓰고, 덩어리 폭이 커브 판정 입력이다 */
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
    assert(param_set_ram(P_GUIDE_CURVE_DETECT, GUIDE_SRC_LED_ONLY) == 0);   /* 주행 중 바꾼다 */
    for (k = 0; k < 6; k++) {
        host.sci[HAL_SCI_B].tx_len = 0;
        guide_led_reply(LED_CENTER);
        run_encoder(0, 20000);
        run_encoder(1, 20000);
        tick();
        assert(host.sci[HAL_SCI_B].tx_len == 4 && host.sci[HAL_SCI_B].tx[1] == 'H');
    }
    assert(D.guide_pos == 0 && D.guide_span == 2 && D.stop_err == 0 && D.flag_run);
    guide_cfg_get(&cfg);
    cfg.h_scale = 1.5f;
    guide_configure(&cfg);
    guide_led_reply(LED_NEAR);                      /* 'h' 위치 -6 → 'a' 단위 -9 */
    tick();
    assert(D.guide_pos == -9);
    cfg.h_scale = 1.0f;
    guide_configure(&cfg);
    assert(param_set_ram(P_GUIDE_CURVE_DETECT, GUIDE_SRC_POSITION) == 0);
    printf("  guide h only ok\n");
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
    assert(D.speed_cmd == P.min_speed);
    assert(D.tape_align == 0);
    guide_led_reply(0x03FFu);
    tick();
    assert(D.tape_turn_dir == 0);
    assert(D.tape_align == 1);
    assert(D.turn_pulses == P.turn_90);
    assert(D.flag_stopping == 1);
    assert(D.stop_pulses == (int16_t)(P.guide_dist_mm * P.one_length_pulse + 0.5f));   /* 교차선은 GD */
    printf("  tape turn ok\n");
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
    test_para_init();
    test_lamp();
    test_align_turn();
    test_tune_entry();
    test_tune_menu();
    test_tune_inputs();
    test_tune_ee_layout();
    test_tune_t2_compare();
    test_tune_t24_amp();
    test_tune_guide();
    test_guide_h_only();
    test_tape_turn();
    test_io_test();
    test_acs_off();
    test_keep_results();
    printf("test_sys ok\n");
    return 0;
}
