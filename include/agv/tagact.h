/* tagact.h - 태그 액션 API. 태그 규칙 표(tag_rule_t)와 사용자 정의 코드가 호출한다.
 * 설계: docs/03_시퀀스_DSL.md 4절 */
#ifndef AGV_TAGACT_H
#define AGV_TAGACT_H
#include "agv/types.h"

typedef enum { DIR_FWD = 1, DIR_BACK = 2, DIR_TOGGLE = 3, DIR_FWD_AFTER_TURN = 4, DIR_BACK_AFTER_TURN = 5 } dir_t;
typedef enum { TURN_CW = 1, TURN_CCW = 2 } turn_dir_t;
typedef enum { BRANCH_LEFT = 1, BRANCH_RIGHT = 2 } branch_dir_t;

typedef struct {
    uint16_t tag;        /* 태그 번호 */
    uint16_t profile;    /* 0 = 모든 프로파일 */
    uint16_t cond;       /* 하위 8비트 TAG_COND_*, 상위 8비트 스테이션 큐 칸 */
    uint16_t cond_a;     /* 조건 인자 */
    uint16_t cond_b;
    uint16_t act[4];     /* TA_* 액션 코드, 0 종료. TA_TURN은 2칸 */
} tag_rule_t;

enum { TAG_COND_NONE = 0, TAG_COND_FORBACK_MODE, TAG_COND_LOW_BATTERY, TAG_COND_FORWARD, TAG_COND_BACKWARD,
       TAG_COND_STATION_RANGE, /* cond_a <= 큐[칸] <= cond_b */
       TAG_COND_STATION_RUNNING /* 위 조건 + 아직 정지가 걸리지 않음 */ };

/* 액션 코드: 상위 8비트 종류, 하위 8비트 인자 */
#define TA_KIND(code)     ((code) >> 8)
#define TA_ARG(code)      ((code) & 0xFF)
#define TA_STOP           0x0100
#define TA_STOP_AUTOSTART 0x0200   /* 인자: 초, 0 = P_AUTO_START_SEC */
#define TA_SPEED          0x0300   /* 인자: 0 HIGH, 1 LOW, 2 LLOW, 3 MIN, 10~ m/min */
#define TA_FORBACK        0x0400   /* 인자: dir_t */
#define TA_TURN           0x0500   /* 인자: dir | (deg/90)<<2 ; 다음 칸: fb | auto<<4 | obs<<8 */
#define TA_BRANCH         0x0600   /* 인자: branch_dir_t */
#define TA_SEQ            0x0700   /* 인자: seq_id_t (정지 + 시퀀스 요청) */
#define TA_TAPE_STOP      0x0800   /* 인자: seq_id_t (테이프 탐색 정지 뒤 시퀀스) */
#define TA_ACS            0x0900   /* 인자: ACS_INFO_* ; 번호는 스테이션 큐[0] */
#define TA_ACS_NUM        0x0A00   /* 인자: 번호 상수 (다음 칸에 Info) */
#define TA_OBS            0x0B00   /* 인자: on/off */
#define TA_STATE          0x0C00   /* 인자: 사이트 상태 id (다음 칸: 값) */
#define TA_STATION_RESET  0x0E00
#define TA_ACT            0x0F00   /* 인자: ACT ID. 주행을 멈추지 않고 병렬 동작 시작 */
#define TA_CALL           0x0D00   /* 인자: 사용자 함수 표 인덱스 */
#define TA_TURN_ALIGN     0x1000   /* TA_TURN 과 같은 인자. 목표각 근방에서 주행로에 정렬 */
#define TA_TURN_TAPE      0x1100   /* TA_TURN_ALIGN 과 같다. 교차 테이프를 찾은 뒤 시작 */

void     tagact_bind(const tag_rule_t *rules, uint16_t n, uint16_t profile);
uint16_t tagact_display_tag(void);
int      tagact_run(uint16_t tag);     /* DA_Tag_Selection 대체. 반환 1 = 이 태그로 새 정지가 시작됨 */
uint16_t tagact_dry_run(uint16_t tag, uint16_t *acts, uint16_t cap);  /* 실행하지 않고 액션 열거 (관리 TT/TR) */

/* 개별 액션 (사용자 정의 코드용) */
void tagact_stop(void);          /* 정지 거리 없이 멈춘다. A_STOP */
void tagact_stop_length(void);   /* 방향별 정지 거리 뒤 멈춘다 */
void tagact_speed(uint16_t level);
void tagact_forback(dir_t d);
void tagact_turn(turn_dir_t d, uint16_t deg, dir_t fb, uint16_t autostart, uint16_t obs);
void tagact_turn_align(turn_dir_t d, uint16_t deg, uint16_t next);   /* next: fb | auto<<4 | obs<<8 */
void tagact_turn_align_stopped(turn_dir_t d, uint16_t deg, uint16_t next);   /* 멈춘 자리에서 바로 */
void tagact_branch(branch_dir_t d);
void tagact_compensate(int32_t pulses);   /* 태그 처리 지연·RFID 재시도 거리 보정 */
#endif
