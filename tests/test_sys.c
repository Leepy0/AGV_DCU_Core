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

/* 시험 결과는 리셋에 남는다. 표시가 틀리면(전원 투입) 지운다. 진행 중이던 결과는 끊김으로 둔다 */
static void test_keep_results(void)
{
    test_keep_mark = 0;
    agv_ident.state = 2;
    agv_ident.track_mm = 5000.0f;
    boot();
    assert(agv_ident.state == 0 && agv_ident.track_mm == 0.0f && test_keep_mark != 0);
    agv_ident.state = 2;
    agv_ident.track_mm = 5000.0f;
    agv_top.state = 1;
    agv_all.state = 1;
    agv_all.fail = 2034;
    agv_rpmt.state = 1;
    boot();
    assert(agv_ident.state == 2 && agv_ident.track_mm == 5000.0f && agv_ident.fail == 0);
    assert(agv_top.state == 3 && agv_top.fail == -1);
    assert(agv_all.state == 3 && agv_all.fail == 2034);
    assert(agv_rpmt.state == 3 && agv_rpmt.fail == -1);
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

/* 튜닝 측정: 'h' 부호가 반대이고 배율이 2 이면 저장 뒤 되살아난다 */
static void test_tune_guide(void)
{
    guide_rx_t rx;
    guide_cfg_t cfg;
    int i;

    boot();
    tune_arm();
    drive_up();
    run_scan(5);
    assert(tune_state() == TUNE_GO_1);
    memset(&rx, 0, sizeof(rx));
    rx.kind = GUIDE_RX_LED;
    for (i = 0; i < 40; i++) {
        int pos = (i % 7) - 3;                    /* 'a' 위치 */
        int cell = 8 - pos;                       /* 'h' 두 칸 c, c+1: 위치 = 2c - 16 = -2pos */
        tune_on_a(GUIDE_NORMAL, (int16_t)(2 * pos));
        rx.led_lo = 3UL << (cell - 1);            /* 부호 반대, 배율 1 */
        tune_on_h(&rx);
    }
    assert(tune_value(TUNE_PAGE_LED) == 2);
    assert(tune_value(TUNE_PAGE_GUIDE) == -100);
    /* 저장값을 부팅 때 되살린다 */
    keep_set(KEEP_GUIDE_INV, 1);
    keep_set(KEEP_GUIDE_SCALE, 1250);
    keep_set(KEEP_GUIDE_MARK, 0x7E57);
    tune_load();
    guide_cfg_get(&cfg);
    assert((cfg.invert_fwd == 1) && (cfg.invert_back == 0) && (cfg.h_scale == 1.25f));
    {   /* 튜닝 중 저전압 280 은 표시·부저에 나오지 않는다 */
        uint16_t d[5];
        tune_arm();
        host.key = 0;
        host.adc = 100u << 4;
        ticks(1100);
        assert(D.action_err == ERR_BAT_LOW);
        ui_digits(d);
        assert(d[0] != 15u);                      /* 'E' 가 아니다 */
        assert(host.bus_out[BUS_LAMP] & 0x40u);   /* 부저 꺼짐 (저활성) */
    }
    printf("  tune guide ok\n");
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
    test_compensate();
    test_rfid_retry();
    test_cross();
    test_status_report();
    test_ui();
    test_para_init();
    test_lamp();
    test_align_turn();
    test_tune_guide();
    test_tape_turn();
    test_io_test();
    test_acs_off();
    test_keep_results();
    printf("test_sys ok\n");
    return 0;
}
