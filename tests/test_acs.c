/* test_acs.c - 슬롯 스케줄러, 교차로, 스테이션 큐, 오류 보고 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/acs.h"
#include "agv/frame.h"
#include "fakes.h"

#define D agv_drive

static void setup(int retry_sec, int retry_legacy, int cross_mode)
{
    fake_reset();
    fake_params();
    memset(&D, 0, sizeof(D));
    D.ui_mode = UI_AUTO;
    (void)param_set(P_AGV_ID, 7);
    (void)param_set(P_ACS_RETRY_SEC, (int16_t)retry_sec);
    (void)param_set(P_ACS_RETRY_LEGACY, (int16_t)retry_legacy);
    (void)param_set(P_CROSS_MODE, (int16_t)cross_mode);
    (void)param_set(P_BAT_DV, 245);
    frame_reset();
    acs_reset();
}

static void ticks(int n) { for (int i = 0; i < n; i++) { acs_schedule(); } }

static int sent_info(int info)
{
    int c = 0;
    for (int i = 0; (i < fake_tx_count) && (i < 256); i++) {
        if (fake_tx[i][6] == (uint16_t)info) { c++; }
    }
    return c;
}

/* 다음 프레임까지 걸린 틱 */
static int gap(int limit)
{
    int start = fake_tx_count, c = 0;
    while ((fake_tx_count == start) && (c < limit)) { acs_schedule(); c++; }
    return c;
}

int main(void)
{
    /* 최초 송신은 다음 호출, 호출당 1개, 오래된 것부터 */
    setup(1, 5, 0);
    acs_report('L', 12);
    acs_report('U', 13);
    acs_report('A', 14);
    acs_schedule();
    assert(fake_tx_count == 1 && fake_tx[0][1] == 'E' && fake_tx[0][6] == 'L' && fake_tx[0][8] == '1');
    assert(fake_tx[0][10] == '2');   /* 배터리 */
    acs_schedule();
    assert(fake_tx_count == 2 && fake_tx[1][6] == 'U');
    acs_schedule();
    assert(fake_tx_count == 3 && fake_tx[2][6] == 'A');

    /* 무응답 슬롯이 있어도 새 슬롯은 곧바로 나간다 (선두 대기 없음) */
    acs_on_ack('U');
    acs_on_ack('A');
    acs_report('F', 1);
    acs_schedule();
    assert(fake_tx[fake_tx_count - 1][6] == 'F');

    /* 슬롯 3개 중 하나만 무응답이어도 나머지 재전송이 막히지 않는다 */
    setup(1, 5, 0);
    acs_report('L', 1);
    acs_report('U', 2);
    acs_report('A', 3);
    ticks(3);
    acs_on_ack('L');
    ticks(250);
    assert(sent_info('L') == 1 && sent_info('U') >= 3 && sent_info('A') >= 3);

    /* 간격: CT=1 → 100틱 + 지터 [0, 10) */
    setup(1, 5, 0);
    acs_report('L', 1);
    acs_schedule();
    { int c = gap(200); assert(c >= 100 && c < 110); }

    /* CT=0 → WIRELESS × 10틱 = 50 + 지터 [0, 5) */
    setup(0, 5, 0);
    acs_report('L', 1);
    acs_schedule();
    { int c = gap(200); assert(c >= 50 && c < 55); }

    /* 짧은 간격의 지터 폭 하한 3틱 */
    setup(0, 1, 0);
    acs_report('L', 1);
    acs_schedule();
    { int c = gap(200); assert(c >= 10 && c < 13); }

    /* 큰 CT 도 16비트를 넘지 않는다: 989초 */
    setup(989, 5, 0);
    acs_report('L', 1);
    acs_schedule();
    { int c = gap(120000); assert(c >= 98900 && c < 108790); }

    /* MsgID 는 10회마다 재발급, 99회에 포기 */
    setup(0, 1, 0);
    acs_report('L', 1);
    for (int i = 0; (i < 2000) && acs_pending(); i++) { acs_schedule(); }
    assert(fake_tx_count == ACS_MAX_TRY);
    assert(acs_stat()->give_up == 1);
    assert(acs_stat()->retry == ACS_MAX_TRY - 1);
    assert(acs_pending() == 0);
    assert(fake_tx[0][3] == fake_tx[9][3]);
    assert(fake_tx[9][3] != fake_tx[10][3]);
    assert(fake_tx[10][3] == fake_tx[19][3]);
    assert(fake_tx[19][3] != fake_tx[20][3]);

    /* 큐가 차면 슬롯이 남아 다음 호출에 나간다 */
    setup(1, 5, 0);
    fake_link_full = 1;
    acs_report('L', 1);
    ticks(3);
    assert(acs_pending() == 1 && acs_stat()->tx == 0);
    fake_link_full = 0;
    acs_schedule();
    assert(fake_tx_count == 1);

    /* AUTO 가 아니면 보내지 않고 시도 횟수도 쓰지 않는다 */
    setup(1, 5, 0);
    D.ui_mode = UI_JOG;
    acs_report('L', 1);
    ticks(5);
    assert(fake_tx_count == 0 && acs_pending() == 1);
    D.ui_mode = UI_AUTO;
    acs_schedule();
    assert(fake_tx_count == 1);

    /* P_ACS_RETRY_LEGACY == 0 이면 미사용 */
    setup(1, 0, 1);
    acs_report('L', 1);
    acs_cross_in(3);
    assert(acs_send_ack('1', 'W', 'C', 'P') == 0);
    ticks(10);
    assert(fake_tx_count == 0 && acs_pending() == 0 && !D.cross_in_flag);
    assert(acs_send_tag_ack('1', 'E', 'T', 'W') == 1);     /* 태그 ACK 는 ACS 미사용과 무관 */
    assert(fake_tx_count == 1);

    /* ACK: 같은 Info 중 가장 오래된 슬롯. 'C' 는 9·16·21 공유 */
    setup(1, 5, 0);
    acs_report_slot(ACS_SLOT_ERR_CLEAR, 0);
    acs_report_slot(ACS_SLOT_STANDBY_CLEAR, 0);
    acs_report('C', 0);
    assert(acs_pending() == 3);
    acs_on_ack('C');
    assert(acs_pending() == 2);
    ticks(2);
    /* 가장 오래된 16 번('R')이 지워졌고 9 번('E'), 21 번('L') 만 나갔다 */
    assert(fake_tx_count == 2 && fake_tx[0][1] == 'E' && fake_tx[1][1] == 'L');

    /* 'L' ACK 는 호출 슬롯을 지운다 */
    setup(1, 5, 0);
    acs_report('C', 3);
    acs_on_ack('L');
    assert(acs_pending() == 0);

    /* 응답은 1회 송신 후 해제 */
    setup(1, 5, 0);
    acs_call_ack('5');
    acs_schedule();
    assert(fake_tx_count == 1 && fake_tx[0][1] == 'A' && fake_tx[0][3] == '5');
    assert(fake_tx[0][6] == 'N' && fake_tx[0][7] == '0' && fake_tx[0][9] == 'P');
    assert(acs_pending() == 0);
    ticks(300);
    assert(fake_tx_count == 1);

    /* 즉시 ACK */
    setup(1, 5, 0);
    acs_set_pan('4');
    assert(acs_send_ack('7', 'E', 'T', 'W') == 1);
    assert(fake_tx[0][2] == '4' && fake_tx[0][3] == '7' && fake_tx[0][7] == 'T' && fake_tx[0][9] == 'W');

    /* 표시 보고는 번호 > 0 일 때만 */
    setup(1, 5, 0);
    acs_report('D', 0);
    assert(acs_pending() == 0);
    acs_report('D', 5);
    assert(acs_pending() == 1);

    /* 교차로 간격: CT=1 → 10틱 */
    setup(1, 5, 1);
    (void)param_set(P_CROSS_IN_TIME, 99);
    acs_cross_in(12);
    ticks(1);
    { int c = gap(100); assert(c >= 10 && c < 14); }

    /* 교차로 간격: 0 → 50틱, 큰 값 → 500틱 */
    setup(0, 5, 1);
    (void)param_set(P_CROSS_IN_TIME, 99);
    acs_cross_in(12);
    ticks(1);
    assert(fake_tx_count == 1 && fake_tx[0][6] == 'I' && fake_tx[0][7] == '0' && fake_tx[0][8] == '1' && fake_tx[0][9] == '2');
    { int c = gap(100); assert(c >= 50 && c < 55); }

    setup(99, 5, 1);
    (void)param_set(P_CROSS_RETRY_100MS, 99);
    (void)param_set(P_CROSS_IN_TIME, 99);
    acs_cross_in(12);
    ticks(1);
    { int c = gap(700); assert(c >= 500 && c < 550); }

    /* ACK 뒤 500틱 교체. Cross-In/Out 은 서로 독립 */
    setup(0, 5, 1);
    (void)param_set(P_CROSS_RETRY_100MS, 5);    /* 50틱 */
    (void)param_set(P_CROSS_IN_TIME, 99);
    acs_cross_in(1);
    acs_cross_out(2);
    D.cross_in_on = 1;
    ticks(2);
    assert(sent_info('I') == 1 && sent_info('O') == 1);    /* 한 호출에 하나씩 */
    acs_on_ack('I');
    assert(acs_cross_num(ACS_CROSS_IN) == 1);
    fake_tx_count = 0;
    ticks(450);
    assert(sent_info('I') == 0);                /* In 은 keep-alive 로 늘었다 */
    assert(sent_info('O') >= 7);                /* Out 은 In 의 간격에 끌려가지 않는다 */
    ticks(120);
    assert(sent_info('I') == 1);
    acs_on_ack('O');
    assert(!D.cross_out_flag && !D.cross_out_on && acs_cross_num(ACS_CROSS_OUT) == 0);

    /* 20회째 ack_error (286 근거) */
    setup(0, 5, 1);
    (void)param_set(P_CROSS_IN_TIME, 99);
    acs_cross_in(1);
    D.cross_in_on = 1;
    { int guard = 0;
      while (!D.cross_ack_error && (guard < 5000)) { acs_schedule(); guard++; }
      assert(sent_info('I') == 20); }
    acs_on_ack('I');
    ticks(1);
    assert(D.cross_ack_error == 0);

    /* Flag 뒤 지연이 지나면 On, 꺼지면 상태 전부 초기화 */
    setup(0, 5, 1);
    (void)param_set(P_CROSS_IN_TIME, 3);
    acs_cross_in(1);
    ticks(30);
    assert(D.cross_in_flag == 1 && !D.cross_in_on);
    ticks(1);
    assert(!D.cross_in_flag && D.cross_in_on == 1);
    D.cross_in_on = 0;
    ticks(1);
    assert(acs_cross_num(ACS_CROSS_IN) == 0);

    /* 시퀀스가 돌고 있으면 바로 On: seq_is_active 는 test_seq 범위. 여기서는 꺼진 상태만 본다 */

    /* 교차로 꺼진 설정도 다른 슬롯은 나간다 */
    setup(1, 5, 10);    /* 일의 자리 0 */
    assert(param_dv.collision_type == 0);
    acs_cross_in(1);
    acs_report('L', 1);
    acs_schedule();
    assert(sent_info('L') == 1 && sent_info('I') == 0);

    /* 스테이션 큐 상한·중복 */
    setup(1, 5, 0);
    for (int i = 1; i <= ACS_STATION_MAX; i++) { assert(acs_station_push((int16_t)i) == 1); }
    assert(acs_station_push(99) == 0);
    assert(acs_station_count() == ACS_STATION_MAX);
    acs_station_reset();
    assert(acs_station_push(5) == 1 && acs_station_push(5) == 0 && acs_station_push(0) == 0);
    assert(acs_station_get(0) == 5);

    /* 오류 보고: set[40]=10 은 16비트 곱이면 영구 미송신이었다 */
    setup(1, 5, 0);
    (void)param_set(P_ERR_REPORT_MIN, 10);
    D.stop_err = 277;
    for (long i = 0; i < 500; i++) { acs_error_tick(); }
    ticks(1);
    assert(sent_info('X') == 1 && fake_tx[0][7] == '2' && fake_tx[0][8] == '7' && fake_tx[0][9] == '7');
    acs_on_ack('X');
    for (long i = 500; i < 60000L; i++) { acs_error_tick(); }
    assert(acs_pending() == 0);
    acs_error_tick();
    assert(acs_pending() == 1);
    D.stop_err = 0;
    acs_error_tick();
    ticks(3);
    assert(sent_info('E') == 1 && sent_info('C') == 1);

    /* 오류가 풀리면 누적 틱을 지운다: 'X' 조기 발송 없음 */
    setup(1, 5, 0);
    D.action_err = 281;
    for (int i = 0; i < 499; i++) { acs_error_tick(); }
    D.action_err = 0;
    acs_error_tick();
    D.action_err = 281;
    for (int i = 0; i < 499; i++) { acs_error_tick(); }
    assert(acs_pending() == 0);

    puts("test_acs ok");
    return 0;
}
