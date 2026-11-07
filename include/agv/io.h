/* io.h - 논리 입출력. 극성은 사이트 io_map 에서 흡수하며 앱 코드는 항상 1 = 활성.
 *
 * 시퀀스 조건은 32비트 마스크이므로 ID 0~31 만 쓸 수 있다.
 * 앰프 신호는 시퀀스·원격 강제 대상이 아니라서 32 이상에 둔다. */
#ifndef AGV_IO_H
#define AGV_IO_H
#include "agv/types.h"

#define IO_NONE 0xFF   /* 미연결. 표에서 쓰면 아무 일도 하지 않는다 */

/* 논리 입력 */
enum {
    IN_EXT_1 = 0, IN_EXT_2, IN_EXT_3, IN_EXT_4, IN_EXT_5, IN_EXT_6, IN_EXT_7, IN_EXT_8,
    IN_EXT_9, IN_EXT_10, IN_EXT_11, IN_EXT_12, IN_EXT_13, IN_EXT_14, IN_EXT_15, IN_EXT_16,
    IN_START_SW, IN_STOP_SW, IN_ESTOP, IN_BUMPER_1, IN_BUMPER_2, IN_OBS_F, IN_OBS_B,
    IN_TAG_L, IN_TAG_R, IN_REMOTE_START, IN_REMOTE_STOP,
    IN_IF_1, IN_IF_2, IN_IF_3, IN_IF_4, IN_IF_5,
    IN_AMP_L_OVERLOAD = 32, IN_AMP_R_OVERLOAD,
    IN_MAX
};
/* 논리 출력 */
enum {
    OUT_EXT_1 = 0, OUT_EXT_2, OUT_EXT_3, OUT_EXT_4, OUT_EXT_5, OUT_EXT_6, OUT_EXT_7, OUT_EXT_8,
    OUT_EXT_9, OUT_EXT_10, OUT_EXT_11, OUT_EXT_12, OUT_EXT_13, OUT_EXT_14, OUT_EXT_15, OUT_EXT_16,
    OUT_LAMP_G, OUT_LAMP_R, OUT_LAMP_Y, OUT_AGV_BACK, OUT_IF_1, OUT_IF_2, OUT_BUZZER, OUT_BRAKE_1, OUT_BRAKE_2,
    OUT_AMP_L_RESET = 32, OUT_AMP_L_DIR, OUT_AMP_L_RUN, OUT_AMP_L_CW,
    OUT_AMP_R_RESET, OUT_AMP_R_DIR, OUT_AMP_R_RUN, OUT_AMP_R_CW,
    OUT_MAX
};

/* 기본은 active-low. 반대로 배선된 논리 ID 만 사이트가 등록한다 */
typedef struct {
    const uint16_t *active_high_in;   uint16_t nin;
    const uint16_t *active_high_out;  uint16_t nout;
} io_map_t;

void     io_bind(const io_map_t *map);
uint16_t io_in(uint16_t id);
void     io_out(uint16_t id, uint16_t on);
uint32_t io_in_mask(void);                         /* 입력 ID 0~31 */
uint32_t io_out_mask(void);                        /* 출력 ID 0~31 래치 */
void     io_out_masks(uint32_t set, uint32_t clr); /* clr 먼저, 그다음 set */
void     io_force(uint16_t id, uint16_t on, uint16_t ms);   /* 관리 프로토콜 OF */
void     io_force_clear(uint16_t id);
void     io_seq_outputs_off(void);                 /* Auto_Loading_Init 의 출력 OFF 대체 */
void     io_write(void);                           /* 10 ms 끝 */
#endif
