/* storage.c - EEPROM 비차단 저장과 쓰기 뒤 읽기 검증, KEEP 변경 감지 */
#include "agv/storage.h"
#include "agv/drive.h"
#include "agv/hal.h"
#include "agv/log.h"
#include "agv/sys.h"

#define EE_WORDS     128u
#define DIRTY_WORDS  (EE_WORDS / 16u)
/* 2 ms 틱 4개 = 6~8 ms. 칩 쓰기 시간 5 ms 를 넘긴다 */
#define WRITE_TICKS  4u
#define KEEP_COUNT   (KEEP_LAST - KEEP_FIRST + 1)

enum { ST_IDLE = 0, ST_WRITING };

static uint16_t s_val[EE_WORDS];        /* 칩에 있거나 곧 쓸 값 */
static uint16_t s_dirty[DIRTY_WORDS];
static int16_t  s_keep[KEEP_COUNT];
static uint16_t s_state, s_addr, s_try, s_start;

void storage_reset(void)
{
    uint16_t i;
    for (i = 0; i < DIRTY_WORDS; i++) { s_dirty[i] = 0; }
    s_state = ST_IDLE;
    s_try = 0;
}

void storage_request(uint16_t addr, uint16_t value)
{
    if (addr >= EE_WORDS) { return; }
    s_val[addr] = value;
    s_dirty[addr >> 4] |= (uint16_t)(1u << (addr & 15u));
}

uint16_t storage_pending(void)
{
    uint16_t i, n = (s_state != ST_IDLE) ? 1u : 0u;
    for (i = 0; i < DIRTY_WORDS; i++) {
        uint16_t w = s_dirty[i];
        while (w != 0u) {
            w = (uint16_t)(w & (w - 1u));
            n++;
        }
    }
    return n;
}

static int16_t next_dirty(void)
{
    uint16_t i, b;
    for (i = 0; i < DIRTY_WORDS; i++) {
        if (s_dirty[i] != 0u) {
            for (b = 0; b < 16u; b++) {
                if (s_dirty[i] & (1u << b)) { return (int16_t)(i * 16u + b); }
            }
        }
    }
    return -1;
}

static void finish_write(void)
{
    hal_ee_write_end();
    s_state = ST_IDLE;
    if (hal_ee_read(s_addr) == s_val[s_addr]) {
        s_try = 0;
    }
    else if (++s_try >= STORAGE_RETRY) {
        /* 다시 쓰지 않는다. 틀린 칸을 붙들고 다른 저장을 막지 않게 한다 */
        log_event(LOG_EE_FAIL, (int16_t)s_addr, 0);
        agv_drive.stop_err = ERR_PARAM_LIMIT;
        s_try = 0;
    }
    else {
        s_dirty[s_addr >> 4] |= (uint16_t)(1u << (s_addr & 15u));
    }
}

void storage_poll(void)
{
    int16_t addr;

    if (s_state == ST_WRITING) {
        if ((uint16_t)(sys_ticks() - s_start) >= WRITE_TICKS) { finish_write(); }
        return;
    }
    addr = next_dirty();
    if (addr < 0) { return; }
    s_addr = (uint16_t)addr;
    s_dirty[s_addr >> 4] &= (uint16_t)~(1u << (s_addr & 15u));
    hal_ee_write_begin(s_addr, s_val[s_addr]);
    s_start = sys_ticks();
    s_state = ST_WRITING;
}

/* ---- 부팅 ---- */

static uint16_t boot_read(uint16_t addr) { return hal_ee_read(addr); }

static void boot_write(uint16_t addr, uint16_t value)
{
    hal_ee_write_begin(addr, value);
    hal_delay_us(HAL_EE_WRITE_US + 500u);
    hal_ee_write_end();
}

static uint16_t queue_read(uint16_t addr) { return (addr < EE_WORDS) ? s_val[addr] : 0u; }

const param_store_t storage_blocking = { boot_read, boot_write };
const param_store_t storage_queued   = { queue_read, storage_request };

ee_probe_t ee_probe;

/* 위 128 워드를 읽기만 한다(쓰지 않는다). 칩이 128 워드면 주소 7비트를 무시해 0~127 이 다시 읽힌다 */
static void probe_size(void)
{
    uint16_t a;
    ee_probe.diff_n = ee_probe.first_addr = ee_probe.first_lo = ee_probe.first_hi = 0;
    for (a = EE_WORDS; a < 2u * EE_WORDS; a++) {
        uint16_t v = hal_ee_read(a);
        if (v == s_val[a & (EE_WORDS - 1u)]) { continue; }
        if (ee_probe.diff_n == 0u) {
            ee_probe.first_addr = a;
            ee_probe.first_lo = s_val[a & (EE_WORDS - 1u)];
            ee_probe.first_hi = v;
        }
        ee_probe.diff_n++;
    }
    ee_probe.ee_words = (ee_probe.diff_n != 0u) ? (uint16_t)(2u * EE_WORDS) : (uint16_t)EE_WORDS;
}

void storage_load(void)
{
    uint16_t a;
    for (a = 0; a < EE_WORDS; a++) { s_val[a] = hal_ee_read(a); }
    for (a = 0; a < (uint16_t)KEEP_COUNT; a++) { s_keep[a] = (int16_t)s_val[KEEP_FIRST + a]; }
    probe_size();
}

/* ---- KEEP ---- */

int16_t keep_get(uint16_t addr)
{
    return ((addr >= KEEP_FIRST) && (addr <= KEEP_LAST)) ? s_keep[addr - KEEP_FIRST] : 0;
}

void keep_set(uint16_t addr, int16_t value)
{
    if ((addr >= KEEP_FIRST) && (addr <= KEEP_LAST)) { s_keep[addr - KEEP_FIRST] = value; }
}

void keep_flush_if_changed(void)
{
    uint16_t i;
    for (i = 0; i < (uint16_t)KEEP_COUNT; i++) {
        uint16_t addr = (uint16_t)(KEEP_FIRST + i);
        if ((uint16_t)s_keep[i] != s_val[addr]) { storage_request(addr, (uint16_t)s_keep[i]); }
    }
}
