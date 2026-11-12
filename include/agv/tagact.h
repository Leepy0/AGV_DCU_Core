/* tagact.h - 태그 액션 API. 태그 규칙 표(tag_rule_t)와 사용자 정의 코드가 호출한다.
 * 설계: docs/03_시퀀스_DSL.md 4절 */
#ifndef AGV_TAGACT_H
#define AGV_TAGACT_H
#include "agv/types.h"
#include "agv/seq.h"

typedef enum { DIR_FWD = 1, DIR_BACK = 2, DIR_TOGGLE = 3, DIR_FWD_AFTER_TURN = 4, DIR_BACK_AFTER_TURN = 5 } dir_t;
typedef enum { TURN_CW = 1, TURN_CCW = 2 } turn_dir_t;
typedef enum { BRANCH_LEFT = 1, BRANCH_RIGHT = 2 } branch_dir_t;

/* 조건 셋을 모두 만족하는 규칙만 맞는다. 새 칸은 끝에 두어 예전 {tag, profile, cond, a, b, {act}} 초기화가 그대로 된다 */
typedef struct {
    uint16_t tag;        /* 태그 번호 */
    uint16_t profile;    /* 0 = 모든 프로파일 */
    uint16_t cond;       /* 확장 조건 1: XC_* (agv/seq.h). 태그 번호 조건은 이 규칙의 tag 를 쓴다 */
    uint16_t cond_a;     /* 조건 인자 */
    uint16_t cond_b;
    uint16_t act[4];     /* TA_* 액션 코드, 0 종료. TA_TURN은 2칸 */
    uint16_t cond2;      /* 확장 조건 2 */
    uint16_t cond2_a;
    uint16_t cond2_b;
    uint32_t in_on;      /* 입력 조건. 시퀀스 WHEN 과 같은 인코딩 (양쪽에 선 비트 = 하나라도 ON) */
    uint32_t in_off;
} tag_rule_t;            /* C28x 16워드 */

/* 압축 표(생성기용). 규칙은 3워드이고 조건·입력·동작은 중복을 없앤 표의 번호다. 번호 0 = 없음(조건 없음, 입력 없음, 동작 없음).
 * 표 번호는 0~255, 태그 0~1023, 프로파일 0~63. 맞은 규칙은 tag_rule_t 로 펼쳐 같은 길로 돈다 */
typedef struct { uint16_t cond, a, b; } tag_x_t;           /* 확장 조건 하나 (3워드) */
typedef struct { uint32_t on, off; } tag_in_t;              /* 입력 조건 (4워드) */
typedef struct { uint16_t act[4]; } tag_act_t;              /* 동작 칸 (4워드) */
typedef struct {
    uint16_t tp;         /* 태그 | 프로파일 << 10 */
    uint16_t x;          /* 확장 조건 1 번호 | 확장 조건 2 번호 << 8 */
    uint16_t ai;         /* 동작 번호 | 입력 조건 번호 << 8 */
} tag_pack_t;
typedef struct {
    const tag_pack_t *rules;
    const tag_x_t    *x;
    const tag_in_t   *in;
    const tag_act_t  *act;
    uint16_t          n;
} tag_packed_t;

/* 예전 이름 */
#define TAG_COND_NONE            XC_NONE
#define TAG_COND_FORBACK_MODE    XC_FORBACK_MODE
#define TAG_COND_LOW_BATTERY     XC_LOW_BATTERY
#define TAG_COND_FORWARD         XC_FORWARD
#define TAG_COND_BACKWARD        XC_BACKWARD
#define TAG_COND_STATION_RANGE   XC_STATION
#define TAG_COND_STATION_RUNNING XC_STATION_RUNNING

#ifdef AGV_HOST_BUILD
extern void (*tagact_trace)(uint16_t code, uint16_t next);   /* 호스트 시험 전용: 실행 동작 기록 훅. 0 = 끔 */
#endif

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
#define TA_ACS            0x0900   /* 인자: DSL_INFO_* ; 번호는 스테이션 큐[0] */
#define TA_ACS_NUM        0x0A00   /* 인자: 번호 상수 (다음 칸에 Info) */
#define TA_OBS            0x0B00   /* 인자: on/off */
#define TA_STATE          0x0C00   /* 인자: 사이트 상태 id (다음 칸: 값) */
#define TA_STATION_RESET  0x0E00
#define TA_ACT            0x0F00   /* 인자: ACT ID. 주행을 멈추지 않고 병렬 동작 시작 */
#define TA_CALL           0x0D00   /* 인자: 사용자 함수 표 인덱스 */
#define TA_TURN_ALIGN     0x1000   /* TA_TURN 과 같은 인자. 목표각 근방에서 주행로에 정렬 */
#define TA_TURN_TAPE      0x1100   /* TA_TURN_ALIGN 과 같다. 교차 테이프를 찾은 뒤 시작 */

/* 시퀀스 MOVE_MM: 서 있을 때 mm 앞에 위치 정지하는 이동을 건다. 달리고 있거나 mm < 1 이면 무시(D.move_on 0 그대로) */
void     tagact_move_mm(int16_t mm);
void     tagact_bind(const tag_rule_t *rules, uint16_t n, uint16_t profile);
void     tagact_bind_packed(const tag_packed_t *t, uint16_t profile);   /* 압축 표 */
uint16_t tagact_display_tag(void);
int      tagact_run(uint16_t tag);     /* DA_Tag_Selection 대체. 반환 1 = 이 태그로 새 정지가 시작됨 */
uint16_t tagact_dry_run(uint16_t tag, uint16_t *acts, uint16_t cap);  /* 실행하지 않고 액션 열거 (관리 TT/TR) */

/* 개별 액션 (사용자 정의 코드용) */
void tagact_stop(void);          /* 정지 거리 없이 멈춘다. AGV_STOP */
void tagact_stop_length(void);   /* 방향별 정지 거리 뒤 멈춘다 */
void tagact_stop_after_mm(int32_t mm);   /* 지금 자리에서 mm 만큼 더 가서 멈춘다. 현장 센서 정지용(레거시 FL_check: 센서가 켜지면 FL 만큼 가서 정지) */
void tagact_speed(uint16_t level);
void tagact_forback(dir_t d);
void tagact_turn(turn_dir_t d, uint16_t deg, dir_t fb, uint16_t autostart, uint16_t obs);
void tagact_turn_align(turn_dir_t d, uint16_t deg, uint16_t next);   /* next: fb | auto<<4 | obs<<8 */
void tagact_turn_align_stopped(turn_dir_t d, uint16_t deg, uint16_t next);   /* 멈춘 자리에서 바로 */
void tagact_turn_align_cross(turn_dir_t d, uint16_t deg, uint16_t next);     /* 교차선 기준: GD 뒤 정지 */
void tagact_branch(branch_dir_t d);
void tagact_compensate(int32_t pulses);   /* 태그 처리 지연·RFID 재시도 거리 보정 */
#endif
