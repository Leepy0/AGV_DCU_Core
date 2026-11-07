/* io.c - 버스 입출력과 논리 I/O. 입력은 2 ms ISR 이 읽고 10 ms 시작에 한 번 고정한다 */
#include "agv/io.h"
#include "agv/drive.h"
#include "agv/hal.h"
#include "agv/ioport.h"
#include "agv/param.h"
#include "agv/ui.h"

/* 물리 입력 위치: 바이트 번호 << 4 | 비트 */
enum { RAW_A = 0, RAW_B, RAW_C, RAW_D, RAW_EXT_A, RAW_EXT_B, RAW_COUNT };
#define POS(byte, bit) (((byte) << 4) | (bit))

static const uint16_t s_in_pos[32] = {
    POS(RAW_EXT_A, 0), POS(RAW_EXT_A, 1), POS(RAW_EXT_A, 2), POS(RAW_EXT_A, 3),
    POS(RAW_EXT_A, 4), POS(RAW_EXT_A, 5), POS(RAW_EXT_A, 6), POS(RAW_EXT_A, 7),
    POS(RAW_EXT_B, 0), POS(RAW_EXT_B, 1), POS(RAW_EXT_B, 2), POS(RAW_EXT_B, 3),
    POS(RAW_EXT_B, 4), POS(RAW_EXT_B, 5), POS(RAW_EXT_B, 6), POS(RAW_EXT_B, 7),
    POS(RAW_A, 2), POS(RAW_A, 3), POS(RAW_B, 0), POS(RAW_A, 4), POS(RAW_A, 5), POS(RAW_B, 1), POS(RAW_B, 2),
    POS(RAW_A, 7), POS(RAW_A, 6), POS(RAW_C, 4), POS(RAW_C, 5),
    POS(RAW_C, 7), POS(RAW_D, 0), POS(RAW_D, 1), POS(RAW_D, 2), POS(RAW_C, 6)
};

/* E-STOP·범퍼는 보드에서 고활성이다 */
#define BUILTIN_HIGH ((1UL << IN_ESTOP) | (1UL << IN_BUMPER_1) | (1UL << IN_BUMPER_2))

#define AMP_DIR_BITS     0x22u     /* AMP_x_DIR: 원본이 초기값 1 에서 바꾸지 않는다 */
#define AMP_RUN_BITS     0x44u
#define AMP_RESET_BITS   0x11u
#define AMP_L_CW         0x80u
#define AMP_R_CW         0x08u
#define EXT_BOARD_JUMPER 0x08u     /* 입력 B b3 */

static volatile uint16_t s_raw[RAW_COUNT];
static volatile uint16_t s_ext_board;
static uint32_t s_in, s_high_in, s_high_out, s_out;
static uint16_t s_in_amp;
static uint16_t s_enabled;

void io_reset(void)
{
    uint16_t i;
    s_high_in = BUILTIN_HIGH;
    s_high_out = 0;
    s_out = 0;
    s_ext_board = 0;
    s_enabled = 0;
    for (i = 0; i < RAW_COUNT; i++) { s_raw[i] = 0xFFFFu; }
}

void io_bind(const io_map_t *map)
{
    uint16_t i;
    s_high_in = BUILTIN_HIGH;
    s_high_out = 0;
    for (i = 0; i < map->nin; i++) {
        if (map->active_high_in[i] < 32u) { s_high_in |= 1UL << map->active_high_in[i]; }
    }
    for (i = 0; i < map->nout; i++) {
        if (map->active_high_out[i] < 32u) { s_high_out |= 1UL << map->active_high_out[i]; }
    }
}

uint16_t io_isr_read(void)
{
    s_raw[RAW_A] = hal_bus_read(BUS_IN_A);
    s_raw[RAW_B] = hal_bus_read(BUS_IN_B);
    s_raw[RAW_C] = hal_bus_read(BUS_IN_C);
    s_raw[RAW_D] = hal_bus_read(BUS_IN_D);
    if (s_ext_board) {
        s_raw[RAW_EXT_A] = hal_bus_read(BUS_EXT_IN_A);
        s_raw[RAW_EXT_B] = hal_bus_read(BUS_EXT_IN_B);
    }
    return (uint16_t)(s_raw[RAW_C] & 7u);
}

void io_probe_ext_board(void)
{
    if (s_raw[RAW_B] & EXT_BOARD_JUMPER) { s_ext_board = 1; }
}

uint16_t io_jog_bits(void) { return (uint16_t)((s_raw[RAW_B] >> 4) & 0xFu); }

void io_latch(void)
{
    uint16_t i;
    uint32_t m = 0;

    for (i = 0; i < 32u; i++) {
        uint16_t p = s_in_pos[i];
        if ((s_raw[p >> 4] >> (p & 15u)) & 1u) { m |= 1UL << i; }
    }
    /* 고활성 비트는 그대로, 나머지는 뒤집는다 */
    s_in = (m & s_high_in) | (~m & ~s_high_in);
    s_in_amp = (uint16_t)(~s_raw[RAW_A] & 3u);
}

uint16_t io_in(uint16_t id)
{
    if (id < 32u) { return (uint16_t)((s_in >> id) & 1UL); }
    if (id < IN_MAX) { return (uint16_t)((s_in_amp >> (id - 32u)) & 1u); }
    return 0;      /* IO_NONE 포함 */
}

uint32_t io_in_mask(void) { return s_in; }

void io_out(uint16_t id, uint16_t on)
{
    if (id >= 32u) { return; }       /* 앰프 신호는 agv_drive 에서 만든다 */
    if (on) { s_out |= 1UL << id; }
    else    { s_out &= ~(1UL << id); }
}

uint32_t io_out_mask(void) { return s_out; }

void io_out_masks(uint32_t set, uint32_t clr) { s_out = (s_out & ~clr) | set; }

static uint32_t s_seq_keep;
void io_seq_keep(uint32_t mask) { s_seq_keep = mask; }
void io_seq_outputs_off(void)
{
    s_out &= ~((0xFFFFUL | (1UL << OUT_IF_1) | (1UL << OUT_IF_2)) & ~s_seq_keep);
}

static uint16_t pin(uint16_t id)
{
    uint16_t on = (uint16_t)((s_out >> id) & 1UL);
    uint16_t high = (uint16_t)((s_high_out >> id) & 1UL);
    return (uint16_t)(on == high);   /* 저활성이면 켜짐 = 0 */
}

static uint16_t amp_image(void)
{
    uint16_t v = AMP_DIR_BITS;

    if (agv_drive.amp_run)   { v |= AMP_RUN_BITS; }
    if (agv_drive.amp_reset) { v |= AMP_RESET_BITS; }
    switch (agv_drive.amp_dir) {
    case MOTION_FWD:  v |= AMP_L_CW; break;
    case MOTION_BACK: v |= AMP_R_CW; break;
    case MOTION_CW:   v |= AMP_L_CW | AMP_R_CW; break;
    default: break;
    }
    return v;
}

/* 램프 외부 복제. 설정값 1~3 은 램프 바이트 b3~b5, 21~28 은 확장 A, 29~36 은 확장 B */
static void lamp_mirror(uint16_t *lamp, uint16_t *ext_a, uint16_t *ext_b)
{
    uint16_t i;
    if (param_get(P_LAMP_EXT_USE) <= 0) { return; }
    for (i = 0; i < UI_LAMP_EXT_COUNT; i++) {
        int16_t n = ui_lamp_ext(i);
        uint16_t on = (uint16_t)((*lamp >> i) & 1u);
        uint16_t *byte, bit;
        if (n >= 29)                   { byte = ext_b; bit = (uint16_t)(n - 29); }
        else if (n >= 21)              { byte = ext_a; bit = (uint16_t)(n - 21); }
        else if ((n >= 1) && (n <= 3)) { byte = lamp;  bit = (uint16_t)(n + 2); }
        else { continue; }
        if (bit <= 7u) { *byte = (uint16_t)((*byte & ~(1u << bit)) | (uint16_t)(on << bit)); }
    }
}

void io_write(void)
{
    uint16_t i, lamp = 0, ext_a = 0, ext_b = 0;

    /* 램프 바이트 b0~b6. b7 은 ioport.h 설명대로 0 */
    for (i = 0; i < 7u; i++) { lamp |= (uint16_t)(pin((uint16_t)(OUT_LAMP_G + i)) << i); }
    for (i = 0; i < 8u; i++) {
        ext_a |= (uint16_t)(pin(i) << i);
        ext_b |= (uint16_t)(pin((uint16_t)(i + 8u)) << i);
    }
    lamp_mirror(&lamp, &ext_a, &ext_b);
    hal_bus_write(BUS_AMP, amp_image());
    hal_bus_write(BUS_LAMP, lamp);
    if (s_ext_board) {
        hal_bus_write(BUS_EXT_OUT_A, ext_a);
        hal_bus_write(BUS_EXT_OUT_B, ext_b);
    }
    hal_dac_write(0, (uint16_t)agv_drive.tq_cmd[0]);
    hal_dac_write(1, (uint16_t)agv_drive.tq_cmd[1]);
    hal_sound((uint16_t)agv_drive.sound_on);
    if (!s_enabled) {
        /* OE 를 열기 전에 확장 래치를 모두 끈다. 보드 유무 판정은 500 ms 뒤에 끝난다 */
        if (!s_ext_board) {
            hal_bus_write(BUS_EXT_OUT_A, 0xFFu);
            hal_bus_write(BUS_EXT_OUT_B, 0xFFu);
        }
        hal_bus_enable();
        s_enabled = 1;
    }
}
