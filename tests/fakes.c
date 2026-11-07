/* fakes.c - 호스트 시험용 가짜 상위 계층 */
#include "fakes.h"
#include "agv/io.h"
#include "agv/tagact.h"
#include <signal.h>
#include <string.h>
#include <unistd.h>

/* assert 가 abort 로 끝날 때 오류 대화 상자를 기다리며 프로세스가 남지 않게 한다 */
static void on_abort(int sig) { (void)sig; _exit(3); }
__attribute__((constructor)) static void fakes_init(void) { signal(SIGABRT, on_abort); }

uint32_t fake_in, fake_out, fake_seq_off_count;
int      fake_restart_count, fake_link_full, fake_tx_count;
uint16_t fake_tx[256][20];
int      fake_speed_arg, fake_forback_arg, fake_turn_count;
uint16_t fake_ee[128];
int      fake_ee_writes, fake_ee_cut = -1;

void fake_reset(void)
{
    fake_in = fake_out = fake_seq_off_count = 0;
    fake_restart_count = fake_link_full = fake_tx_count = 0;
    fake_speed_arg = fake_forback_arg = -1;
    fake_turn_count = 0;
    fake_ee_writes = 0;
    fake_ee_cut = -1;
}

uint32_t io_in_mask(void)  { return fake_in; }
uint32_t io_out_mask(void) { return fake_out; }
void     io_out_masks(uint32_t set, uint32_t clr) { fake_out = (fake_out & ~clr) | set; }
void     io_seq_outputs_off(void) { fake_seq_off_count++; fake_out = 0; }

void run_request_restart(uint16_t sec) { (void)sec; fake_restart_count++; }
void log_event(uint16_t code, int16_t a, int16_t b) { (void)code; (void)a; (void)b; }

int link_send(const uint16_t *buf, uint16_t n)
{
    if (fake_link_full) { return 0; }
    if (fake_tx_count < 256) { memcpy(fake_tx[fake_tx_count], buf, n * sizeof(uint16_t)); }
    fake_tx_count++;
    return 1;
}

void tagact_speed(uint16_t level) { fake_speed_arg = level; }
void tagact_forback(dir_t d)      { fake_forback_arg = d; }
void tagact_turn(turn_dir_t d, uint16_t deg, dir_t fb, uint16_t autostart, uint16_t obs)
{
    (void)d; (void)deg; (void)fb; (void)autostart; (void)obs;
    fake_turn_count++;
}

static uint16_t ee_read(uint16_t a) { return fake_ee[a]; }
static void ee_write(uint16_t a, uint16_t v)
{
    if ((fake_ee_cut >= 0) && (fake_ee_writes >= fake_ee_cut)) { return; }
    fake_ee[a] = v;
    fake_ee_writes++;
}
const param_store_t fake_store = { ee_read, ee_write };

void fake_params(void)
{
    memset(fake_ee, 0xFF, sizeof(fake_ee));
    param_default_types(0, 0, 0);
    (void)param_boot(&fake_store, 2611);
}
