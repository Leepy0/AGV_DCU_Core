/* branch.c - 좌우 분기 (DA_Branch_Action). 고정 편이 지령 + 4-LED 창 이동 감시. 동작은 원본 그대로다 */
#include "agv/branch.h"
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/tagact.h"

#define D agv_drive
#define P param_dv

#define BRANCH_CONFIRM_TICKS 10

static void clear_state(void)
{
    D.branch_start = 0;
    D.branch_step = 0;
    D.branch_return_cnt = 0;
    D.cnt_branch_l = 0;
    D.cnt_branch_r = 0;
    D.p_cmd = 0;
}

static void fail(void)
{
    D.stop_err = ERR_BRANCH_TIMEOUT;
    D.flag_stopping = 1;
    D.branch_ready = 0;
    clear_state();
}

static int16_t zone_confirmed(uint16_t zone)
{
    if (zone == 0) {
        D.branch_return_cnt = 0;
        return 0;
    }
    if (D.branch_return_cnt < BRANCH_CONFIRM_TICKS) { D.branch_return_cnt++; }
    if (D.branch_return_cnt >= BRANCH_CONFIRM_TICKS) {
        D.branch_return_cnt = 0;
        return 1;
    }
    return 0;
}

static uint16_t nibble(uint32_t v, uint16_t first_bit)
{
    return (uint16_t)((v >> first_bit) & 0xFUL);
}

/* 감시 창의 시작 비트. 왼쪽 LED13~16 → 9~12 → 5~8, 오른쪽 LED1~4 → 5~8 → 9~12.
 * wide 는 왼쪽 LED21~24 → 17~20 → 13~16, 오른쪽 LED9~12 → 13~16 → 17~20 */
static uint16_t led_zone(int16_t left)
{
    static const uint16_t narrow[2][3] = { { 0, 4, 8 }, { 12, 8, 4 } };
    static const uint16_t wide[2][3]   = { { 8, 12, 16 }, { 20, 16, 12 } };

    if (D.guide_wide == 1) { return nibble(D.guide_led32, wide[left][D.branch_step]); }
    return nibble(D.guide_led, narrow[left][D.branch_step]);
}

void branch_run(void)
{
    int16_t gap = param_digit(param_get(P_BRANCH_OFFSET), 1);
    int16_t dir;

    if ((D.branch_ready != 0) && (D.branch_start == 0)) {
        D.straight_cnt = 0;
        if ((D.branch_ready != BRANCH_LEFT) && (D.branch_ready != BRANCH_RIGHT)) {
            fail();
            return;
        }
        /* 전진은 좌측, 후진은 우측 태그 센서가 거리 측정 시작점이다 */
        if (!D.backward ? D.tag_l : D.tag_r) {
            D.cnt_branch_l = 0;
            D.cnt_branch_r = 0;
        }
        if (((int32_t)D.cnt_branch_l + (int32_t)D.cnt_branch_r) > (int32_t)P.tag_pulse[8]) {
            dir = D.branch_ready;
            D.branch_ready = 0;
            clear_state();
            D.branch_start = dir;
            D.guide_led = 0;       /* 이전 패턴으로 첫 단계가 통과되지 않게 한다 */
        }
    }

    dir = D.branch_start;
    if (dir == 0) {
        if (D.branch_ready == 0) {
            D.branch_step = 0;
            D.branch_return_cnt = 0;
            D.cnt_branch_l = 0;
            D.cnt_branch_r = 0;
            D.p_cmd = 0;
        }
        return;
    }
    if ((dir != BRANCH_LEFT) && (dir != BRANCH_RIGHT)) {
        fail();
        return;
    }
    D.straight_cnt = 0;

    /* 1.5 m 를 가도 창 순서가 안 끝나면 테이프 이탈이나 슬립으로 본다 */
    if (((int32_t)D.cnt_branch_l > P.straight_len) || ((int32_t)D.cnt_branch_r > P.straight_len)) {
        fail();
        return;
    }

    D.p_cmd = (dir == BRANCH_LEFT) ? (int16_t)(-gap) : gap;

    if (D.branch_step == 3) {
        D.branch_ready = 0;
        clear_state();
        return;
    }
    if ((D.branch_step < 0) || (D.branch_step > 3)) {
        fail();
        return;
    }
    if (zone_confirmed(led_zone(dir == BRANCH_LEFT))) {
        D.branch_step++;
        /* 중앙 테이프로 돌아오는 방향으로 한 주기 동안 작은 보정을 준다 */
        if (D.branch_step == 3) { D.p_cmd = (dir == BRANCH_LEFT) ? 1 : -1; }
    }
}
