/* sys.c - 부팅과 10 ms 스케줄러. 순서: docs/01_아키텍처.md 2절 */
#include <string.h>
#include "agv/sys.h"
#include "agv/acs.h"
#include "agv/comm.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/guide.h"
#include "agv/hal.h"
#include "agv/io.h"
#include "agv/ioport.h"
#include "agv/link.h"
#include "agv/log.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/site.h"
#include "agv/steer.h"
#include "agv/stop.h"
#include "agv/storage.h"
#include "agv/tune.h"
#include "agv/ui.h"

#define D agv_drive

#define TICKS_PER_10MS   5u
#define ADC_START_TICK   3u     /* 10 ms 작업 한 틱 전에 변환을 시작한다 */
#define POWER_ON_SCANS   50u    /* 전원 안정 500 ms */
#define IO_TEST_KEYS     (KEY_MODE + KEY_UP)   /* 전원을 켤 때 누르면 IO 시험 */
#define TUNE_KEYS        (KEY_MODE + KEY_DOWN) /* 전원을 켤 때 누르면 회전 튜닝 */
enum { IO_TEST_NONE = 0, IO_TEST_WAIT, IO_TEST_ON };

static volatile uint16_t s_ticks, s_tick10;
static uint16_t s_phase, s_scans, s_power_on, s_alive, s_overrun, s_io_test;

uint16_t sys_overrun(void)  { return s_overrun; }
uint16_t sys_ticks(void)    { return s_ticks; }
int      sys_power_on(void) { return s_power_on != 0u; }

void sys_tick_2ms(void)
{
    s_ticks++;
    ui_isr(io_isr_read());
    if (s_phase == ADC_START_TICK) { hal_adc_start(); }
    if (++s_phase >= TICKS_PER_10MS) {
        s_phase = 0;
        s_tick10++;
    }
}

void sys_encoder(uint16_t wheel, uint32_t period) { run_encoder(wheel, period); }
void sys_scia_rx(uint16_t byte)                   { comm_rx_byte(byte); }

void sys_boot(void)
{
    int16_t err;

    /* RAM 은 codestart 가 지운다. 앰프 출력과 직결된 구조체는 여기서 한 번 더 지운다 */
    memset((void *)&agv_drive, 0, sizeof(agv_drive));
    s_ticks = s_tick10 = 0;
    s_phase = s_scans = s_power_on = s_alive = s_overrun = s_io_test = 0;
    hal_sys_init();
    motor_reset();
    stop_reset();
    steer_reset();
    frame_reset();
    acs_reset();
    comm_reset();
    io_reset();
    ui_reset();
    storage_reset();
    guide_poll_reset();
    tune_reset();
    site_register();

    err = param_boot(&storage_blocking, SYS_VERSION);
    param_store_set(&storage_queued);
    storage_load();
    tune_load();
    seq_pick_profile();
    run_reset();
    run_boot_restore();
    D.system_err = 0;
    D.stop_err = err;
    link_select();
    link_guide_init();
    hal_start();
}

/* 10 ms 작업 순서는 고정이다 */
static void scan_10ms(void)
{
    D.action_err = 0;              /* 상태형 오류는 매 틱 다시 계산한다 */
    io_latch();
    if (s_io_test == IO_TEST_ON) {   /* 주행과 ACS 는 멈춘다. 끝내려면 전원을 다시 켠다 */
        ui_iotest_run();
        return;
    }
    comm_poll();
    if (s_power_on) {
        guide_poll();
        run_coordinate();
        if (tune_active()) { tune_run(); }
        motor_run();
        run_seq();
        site_manual();
        run_after_motor();
        ui_run();
    }
    else if (s_io_test == IO_TEST_WAIT) {
        ui_iotest_wait();
        if (ui_key() == 0u) {
            s_io_test = IO_TEST_ON;
            ui_iotest_enter();
        }
    }
    else if (++s_scans >= POWER_ON_SCANS) {
        io_probe_ext_board();
        if (ui_key() == IO_TEST_KEYS) { s_io_test = IO_TEST_WAIT; }
        else {
            if (ui_key() == TUNE_KEYS) { tune_arm(); }
            s_power_on = 1;
        }
    }
    acs_schedule();
    io_write();
}

void sys_scan(void)
{
    if (s_tick10 != 0u) {
        uint16_t irq, late;

        scan_10ms();
        irq = hal_irq_save();
        late = s_tick10;
        s_tick10 = 0;
        hal_irq_restore(irq);
        if (late >= 2u) {
            s_overrun++;
            log_event(LOG_OVERRUN, (int16_t)late, 0);
        }
        s_alive = 1;
    }
    storage_poll();
    /* 타이머가 죽으면 차지 않아 리셋된다 */
    if (s_alive) {
        s_alive = 0;
        hal_wdog_kick();
    }
}

#ifndef AGV_HOST_BUILD
void main(void)
{
    sys_boot();
    for (;;) { sys_scan(); }
}
#endif
