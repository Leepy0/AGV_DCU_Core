/*
 * storage.h - EEPROM 저장 큐와 KEEP 변수
 *
 * 저장은 메인 루프에서 한 번에 한 워드씩 한다. 같은 주소에 다시 요청하면 새 값으로 합친다.
 * ISR 에서 부르지 않는다.
 * 주소: 0~45 파라미터, 50 표시, 51~106 KEEP (번호는 현장 EEPROM 과 호환).
 * 체크섬 밖: 115~118 파라미터(param.c), 120~124 미확인 OUT 큐(acs.h).
 * 칩 크기: 128 워드로 쓴다. 부팅 때 128~255 를 읽기만 해서 0~127 과 견주어 실제 크기를 ee_probe 에 남긴다.
 */
#ifndef AGV_STORAGE_H
#define AGV_STORAGE_H
#include "agv/types.h"
#include "agv/param.h"

#define KEEP_FIRST          51
#define KEEP_LAST           109
#define KEEP_BACKWARD       51
#define KEEP_TURN_SCALE     54    /* 튜닝 모드가 잰 제자리 회전 보정비 × 1000. 범위 밖이면 보정 없음(tune.h) */
#define KEEP_STATION0       57    /* 스테이션 큐 10칸 */
#define KEEP_STATION_SLOTS  10
#define KEEP_TAG_INDEX0     67    /* 태그 이력 종류 5칸 */
#define KEEP_TAG_NUM0       72    /* 태그 이력 번호 5칸 */
#define KEEP_STATION_CNT    92
#define KEEP_PAN            93
#define KEEP_LAMP_EXT0      104   /* 램프 외부 출력 3칸 */
#define KEEP_GUIDE_MARK     107   /* 튜닝 모드가 저장한 가이드 'h' 부호·배율 */
#define KEEP_GUIDE_INV      108   /* b0 전진 센서, b1 후진 센서 */
#define KEEP_GUIDE_SCALE    109   /* 배율 × 1000 */

#define STORAGE_RETRY       3     /* 읽기 검증 실패 뒤 다시 쓰는 횟수 */

/* 칩 크기 시험 결과(부팅, 읽기만). 128~255 가 모두 addr & 0x7F 의 값과 같으면 주소 7비트를 무시하는 128 워드 칩 */
typedef struct {
    uint16_t ee_words;       /* 128 또는 256 */
    uint16_t diff_n;         /* 128~255 중 0~127 과 다른 워드 수 */
    uint16_t first_addr;     /* 처음 다른 주소(128~255). 없으면 0 */
    uint16_t first_lo, first_hi;   /* 그 주소 & 0x7F 의 값, 그 주소의 값 */
} ee_probe_t;
extern ee_probe_t ee_probe;

extern const param_store_t storage_blocking;   /* 부팅: 끝날 때까지 기다린다 */
extern const param_store_t storage_queued;     /* 운전 중: 큐에 넣고 돌아온다 */

void     storage_reset(void);
void     storage_load(void);                   /* 부팅: 칩 내용을 읽어 둔다. 인터럽트 허용 전 */
void     storage_request(uint16_t addr, uint16_t value);
void     storage_poll(void);                   /* 메인 루프. 호출마다 칩 동작 하나 */
uint16_t storage_pending(void);
int16_t  keep_get(uint16_t addr);
void     keep_set(uint16_t addr, int16_t value);
void     keep_flush_if_changed(void);          /* 정지 상태에서 부른다 */
#endif
