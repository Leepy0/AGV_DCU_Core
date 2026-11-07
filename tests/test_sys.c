/* test_sys.c - 2·3단계 통합 시험. hal/host 위에서 부팅부터 10 ms 스캔까지 돌린다 */
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
#include "agv/param.h"
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

    /* 다른 호기 */
    send_frame("E1199T00200000000", 0);
    run_scan(1);
    assert(comm_recent_num() == 2);

    /* 회전 중 무시: ACK 없음 */
    D.turn_pulses = 10;
    host.sci[HAL_SCI_A].tx_len = 0;
    tag_frame(1, '3', 0);
    run_scan(1);
    assert(!tx_has('A', 'E'));
    D.turn_pulses = 0;

    printf("  tag ok\n");
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
    test_compensate();
    test_rfid_retry();
    test_cross();
    test_status_report();
    test_ui();
    test_para_init();
    test_lamp();
    test_acs_off();
    printf("test_sys ok\n");
    return 0;
}
