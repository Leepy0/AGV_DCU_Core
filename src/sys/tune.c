/* tune.c - 회전 튜닝 모드. 전원을 켤 때 MODE + DOWN.
 * 직진 1 m, 태그 없이 180도 정렬 회전, 직진 1 m 뒤 멈춰 휠 간격 값을 계산한다.
 * 라인 중앙에서 다시 중앙까지 돈 펄스가 실제 180도다. SET 저장, RESET 다시 시험 */
#include "agv/tune.h"
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/tagact.h"
#include "agv/ui.h"

#define D agv_drive
#define P param_dv

#define TUNE_DIST_MM 1000.0f

static struct {
    uint16_t state;
    uint16_t turn_seen;
    uint32_t start;
    int16_t  wheel_base;
} s_tune;

void     tune_reset(void)  { s_tune.state = TUNE_OFF; s_tune.turn_seen = 0; }
void     tune_arm(void)    { s_tune.state = TUNE_WAIT; }
int      tune_active(void) { return s_tune.state != TUNE_OFF; }
uint16_t tune_state(void)  { return s_tune.state; }
int16_t  tune_result(void) { return s_tune.wheel_base; }

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }

static int reached(void)
{
    return (float)(travel() - s_tune.start) >= (TUNE_DIST_MM * P.one_length_pulse);
}

static void go_slow(void)
{
    if (D.speed_cmd != P.min_speed) {
        D.speed_cmd = P.min_speed;
        D.step = DRIVE_CHANGE_SPEED;
    }
}

static void begin_leg(uint16_t next)
{
    s_tune.start = travel();
    s_tune.state = next;
}

static int16_t compute_wheel_base(void)
{
    int16_t low, up;
    float value = (float)param_get(P_WHEEL_BASE_MM) * (float)D.tape_turned / (float)D.tape_total_pulse;
    int16_t rounded = (int16_t)(value + 0.5f);

    param_limit(P_WHEEL_BASE_MM, &low, &up);
    if (rounded < low) { rounded = low; }
    if (rounded > up)  { rounded = up; }
    return rounded;
}

void tune_run(void)
{
    int stopped = !D.flag_run && !D.flag_ready;

    switch (s_tune.state) {
    case TUNE_WAIT:
        if (D.flag_run) { begin_leg(TUNE_GO_1); }
        break;
    case TUNE_GO_1:
        if (stopped) { s_tune.state = TUNE_WAIT; break; }
        go_slow();
        if (reached()) {
            D.tape_slow = 1;
            s_tune.turn_seen = 0;
            tagact_turn_align(TURN_CW, 180u, 0u);
            if (D.tape_align) { s_tune.state = TUNE_TURN; }
            else              { D.tape_slow = 0; tagact_stop(); s_tune.state = TUNE_WAIT; }
        }
        break;
    case TUNE_TURN:
        if (D.flag_turn) { s_tune.turn_seen = 1; }
        else if (s_tune.turn_seen && !D.tape_align && !D.flag_stopping && (D.turn_motion == 0)) {
            D.tape_slow = 0;
            if ((D.system_err != 0) || (D.stop_err != 0)) {
                s_tune.state = TUNE_WAIT;
            }
            else {
                s_tune.wheel_base = compute_wheel_base();
                run_restart_now();
                s_tune.state = TUNE_RESTART;
            }
        }
        break;
    case TUNE_RESTART:
        if (D.flag_run) { begin_leg(TUNE_GO_2); }
        break;
    case TUNE_GO_2:
        if (stopped) { s_tune.state = TUNE_WAIT; break; }
        go_slow();
        if (reached()) {
            tagact_stop();
            s_tune.state = TUNE_DONE;
        }
        break;
    default:
        break;
    }
}

int tune_key(uint16_t key)
{
    if ((s_tune.state != TUNE_DONE) && (s_tune.state != TUNE_SAVED)) { return 0; }
    if (D.flag_run || D.flag_stopping) { return 0; }
    if (key == KEY_SET) {
        (void)param_set(P_WHEEL_BASE_MM, s_tune.wheel_base);
        s_tune.state = TUNE_SAVED;
        return 1;
    }
    if (key == KEY_RESET) {
        s_tune.state = TUNE_WAIT;
        return 1;
    }
    return 0;
}
