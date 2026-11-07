/* ioport.h - io 모듈의 코어 내부 함수. 앱은 io.h 만 쓴다.
 *
 * BRAKE_1·2 비트: 원본은 버스 쓰기마다 두 비트를 뒤집지만 쓰기가 항상 짝수 회라 핀에서는
 * 주소 3 의 b7 = 0, 주소 2 의 b7 = 1 로 고정된다. 메인과 ISR 이 나뉜 뒤로는 뒤집기가 끼어들 수
 * 있어 값을 고정으로 낸다. OUT_BRAKE_1·2 는 쓰지 않는다. 배선 의미는 실측 대상이다. */
#ifndef AGV_IOPORT_H
#define AGV_IOPORT_H
#include "agv/types.h"
void     io_reset(void);
uint16_t io_isr_read(void);          /* 2 ms: 입력 버스를 읽고 키 입력 3비트를 돌려준다 */
void     io_latch(void);             /* 10 ms 시작 */
void     io_probe_ext_board(void);   /* 전원 안정 뒤 한 번 */
uint16_t io_jog_bits(void);

/* IO 시험. 번호 1~10 은 보드 출력, 21~36 은 확장 출력 1~16. 값 0 = 켜짐 */
#define IO_TEST_OUT_LAST 36u
#define IO_TEST_BOARD_LAST 10u
uint16_t io_test_ext_board(void);
uint16_t io_test_input(void);        /* 첫 활성 입력 번호, 없으면 0 */
void     io_test_write(const uint16_t *v);   /* v[0..IO_TEST_OUT_LAST] */
#endif
