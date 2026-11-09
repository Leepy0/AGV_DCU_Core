/*
 * seq.h - 시퀀스(정지 후 동작)와 ACT(주행 중에도 도는 병렬 동작)의 실행 인터페이스
 *
 * 사이트는 seq_dsl.h 매크로로 표를 쓰고, 코어가 10 ms마다 실행한다.
 * 모든 시간 인자는 1/100초(10 ms 틱) 단위다. 300 = 3초.
 * 설계: docs/03_시퀀스_DSL.md
 */
#ifndef AGV_SEQ_H
#define AGV_SEQ_H

#include "agv/types.h"

/* 시퀀스 요청 ID. 겹친 요청은 사이트 대응표(site_seqs) 순서로 실행한다 */
typedef enum {
    SEQ_NONE = 0,
    SEQ_LOAD,            /* Loading_Ready_Flag */
    SEQ_BOX_LOAD,        /* Box_Loading_Ready_Flag */
    SEQ_LOAD_FINISH,     /* Finish_Loading_Flag */
    SEQ_LOAD_LAST,       /* Last_Loading_Flag */
    SEQ_LOAD_EMPTY,      /* Empty_Loading_Flag */
    SEQ_LOAD_PART,       /* Part_Loading_Flag */
    SEQ_CART_LOAD,
    SEQ_TROLLEY_LOAD,
    SEQ_LOAD_CHECK,
    SEQ_UNLOAD,          /* Unloading_Ready_Flag */
    SEQ_BOX_UNLOAD,
    SEQ_UNLOAD_FINISH,
    SEQ_UNLOAD_LAST,
    SEQ_UNLOAD_EMPTY,
    SEQ_UNLOAD_PART,
    SEQ_CART_UNLOAD,
    SEQ_TROLLEY_UNLOAD,
    SEQ_MATERIAL_UNLOAD,
    SEQ_SITE_BASE = 32,  /* 사이트 전용 ID는 여기부터 */
    SEQ_MAX = 64
} seq_id_t;

/* 스텝 종류 */
typedef enum {
    SK_END_RESTART = 0,   /* 종료, 재출발 예약. ACT 금지 */
    SK_END_STOP,          /* 종료, 재출발 없음 */
    SK_END_ERROR,         /* 시퀀스: action_err = arg0 후 종료. ACT: stop_err = arg0 (주행 정지) */
    SK_LABEL,             /* 분기 목적지 표시. 실행 시간 없음 */
    SK_OUT,               /* on = 켤 출력, off = 끌 출력. 끄기를 먼저 적용 */
    SK_WAIT,              /* 입력 조건 대기. timeout 있으면 target 으로 */
    SK_WAIT_STABLE,       /* 조건이 arg0 틱 초과 연속 참이면 통과. timeout 있으면 target 으로 */
    SK_WAIT_OUT,          /* 출력 래치 조건 대기 */
    SK_DELAY,             /* arg0 틱 대기 */
    SK_OUT_PULSE,         /* out 을 arg0 틱 켜고 arg1 틱 끄기를 timeout 회 반복 */
    SK_OUT_FLICKER,       /* out 을 arg0 틱 간격으로 토글, 조건 참이면 끄고 다음. timeout 있으면 target */
    SK_HANDSHAKE,         /* out 켬, 입력 켜짐 대기, out 끔, 입력 꺼짐 대기. timeout 있으면 target */
    SK_ACS,               /* arg0 = Info 문자, arg1 = 번호 출처 SEQ_NUM_*, timeout = 상수 번호 */
    SK_ACS_WAIT_ACK,      /* 미확인 슬롯이 없을 때까지 대기 */
    SK_ACS_STATION,       /* arg0 = 큐 칸, arg1 = 값, timeout = 큐 길이 */
    SK_STATION_RESET,
    SK_SPEED,             /* arg0 = SEQ_SPD_* 또는 m/min. ACT 금지 */
    SK_FORBACK,           /* arg0 = dir_t. ACT 금지 */
    SK_TURN,              /* arg0 = turn_dir_t, arg1 = 각도, timeout = fb | auto<<4 | obs<<8. 완료 대기 포함. ACT 금지 */
    SK_OBS,               /* arg0 = on/off */
    SK_SET_STATE,         /* 사이트 상태 arg0 = arg1 */
    SK_WAIT_STATE,        /* 사이트 상태 arg0 == arg1 대기 */
    SK_GOTO,              /* target 으로 */
    SK_WHEN,              /* 연속한 WHEN 을 위에서 평가, 첫 참의 target 으로. 전부 거짓이면 대기 */
    SK_WHEN_STATE,        /* WHEN 의 사이트 상태 조건판: 상태 arg0 == arg1 */
    SK_WHEN_TIMEOUT,      /* WHEN 묶음의 시간 제한: timeout 넘으면 target 으로 */
    SK_ERROR_HOLD,        /* action_err = arg0 유지, 조건 참이면 target 으로. ACT 금지 */
    SK_ACT_START,         /* ACT arg0 시작 */
    SK_WAIT_ACT,          /* ACT arg0 끝날 때까지 대기 */
    SK_CALL,              /* 사용자 정의 함수: arg0 = 함수 표 인덱스 */
    SK_DELAY_PARAM,       /* 파라미터 arg0 x arg1 틱 대기 */
    SK_WAIT_PARAM,        /* SK_WAIT 와 같고 제한 시간이 파라미터 arg0 x arg1 틱. 값 0 이하 = 제한 없음 */
    SK_TURN_NOWAIT,       /* SK_TURN 과 같고 완료를 기다리지 않는다. 다음 스캔에 다음 스텝. ACT 금지 */
    SK_KIND_MAX
} seq_kind_t;

/* 번호 출처 (SK_ACS) */
enum { SEQ_NUM_CONST = 0, SEQ_NUM_STATION0, SEQ_NUM_STATION0_PLUS1, SEQ_NUM_AGV_ID };
#define SEQ_NUM_STATION_K   0x100u
#define SEQ_NUM_STATION(k)  (SEQ_NUM_STATION_K | (k))   /* 스테이션 큐[k] */

/* 확장 조건. WHEN·WAIT 스텝과 태그 규칙이 같이 쓴다.
 * 16비트 코드: 하위 7비트 종류, 비트 7 부정(XC_NOT), 상위 8비트 칸(큐 칸·파라미터 번호). 인자 a, b */
enum {
    XC_NONE = 0,          /* 항상 참 */
    XC_FORBACK_MODE,      /* ForBack_Mode > 0 */
    XC_LOW_BATTERY,
    XC_FORWARD,           /* 주행 방향 전진 */
    XC_BACKWARD,
    XC_STATION,           /* a <= 큐[칸] <= b */
    XC_STATION_RUNNING,   /* 위 조건 + 아직 정지가 걸리지 않음 */
    XC_CROSS,             /* 교차로. 칸 비트0: 0 IN, 1 OUT. 비트1: 0 대기 또는 유지, 1 유지만 */
    XC_ERR_EQ,            /* action_err == a */
    XC_PARAM_EQ,          /* 파라미터[칸] == a */
    XC_PARAM_GE,          /* 파라미터[칸] >= a */
    XC_PARAM_LE,          /* 파라미터[칸] <= a */
    XC_ID,                /* a <= P_AGV_ID <= b */
    XC_KEY,               /* 지금 눌린 키 == a. 키를 소비하지 않는다 */
    XC_STATION_MOD,       /* 큐[칸] % a == b */
    XC_STATION_DIV,       /* 큐[칸] / a == b */
    XC_TAG_MINUS_EQ_ST,   /* 태그 - a == 큐[칸]. 태그 표 전용 */
    XC_TAG_EQ_ST          /* 태그 == 큐[칸]. 태그 표 전용 */
};
#define XC_NOT  0x80u

/* 속도 수준 (SK_SPEED) */
enum { SEQ_SPD_HIGH = 0xFFF0, SEQ_SPD_LOW = 0xFFF1, SEQ_SPD_LLOW = 0xFFF2, SEQ_SPD_MIN = 0xFFF3 };

#define SEQ_NO_LABEL     0u       /* target 없음. 라벨 값은 1부터 */
#define SEQ_TO_UNMATCHED 0x8000u  /* timeout 최상위 비트: 조건이 거짓인 틱만 누적 (최대 32767 = 327초) */

#define SEQ_ERR_CALL     290u     /* 사용자 정의 함수가 실패(2)를 돌려준 경우 */
#define SEQ_ERR_TABLE    290u     /* 없는 라벨·함수 인덱스 */
#define SEQ_STATE_COUNT  16       /* 사이트 상태 변수 칸 수 */

typedef struct {
    uint16_t kind;      /* seq_kind_t */
    uint16_t target;    /* 분기·타임아웃 라벨 */
    uint16_t arg0;
    uint16_t arg1;
    uint16_t timeout;
    uint16_t xc;        /* WHEN·WAIT 확장 조건 XC_*, 인자는 arg0·arg1. C28x 정렬 빈칸 자리 */
    uint32_t on;        /* 조건 must_on, SK_OUT 은 켤 출력 */
    uint32_t off;       /* 조건 must_off, SK_OUT 은 끌 출력 */
    uint32_t out;       /* 펄스·점멸·핸드셰이크 대상 출력. WHEN·WAIT 는 두 번째 "하나라도 ON" 묶음 */
} seq_step_t;           /* C28x 12워드 */

typedef struct {
    uint16_t profile;           /* 0 = 모든 프로파일 */
    uint16_t id;                /* seq_id_t 또는 사이트 ACT ID */
    const seq_step_t *steps;
    uint16_t nsteps;
} seq_def_t;

typedef struct {
    uint16_t sub;        /* 사용자 정의 서브 스텝 */
    uint16_t timer;      /* 스텝 진입 후 틱 */
    uint16_t scratch[4];
    uint16_t id;
    uint16_t step;
} seq_ctx_t;

/* 반환: 0 대기, 1 완료(다음 스텝), 2 실패 */
typedef int (*seq_call_fn)(seq_ctx_t *ctx);

/* agv_type 0 = 타입 무관. 빌드가 곧 사이트인 경우에 쓴다 */
typedef struct { uint16_t profile, agv_type, id_from, id_to; } profile_row_t;

typedef struct {
    const seq_def_t *seqs;          uint16_t nseqs;
    const seq_def_t *acts;          uint16_t nacts;
    const seq_call_fn *calls;       uint16_t ncalls;
    const profile_row_t *profiles;  uint16_t nprofiles;
} seq_site_t;

/* 부팅 시 등록. P_AGV_TYPE·P_AGV_ID 로 프로파일을 고르고, ACT 표의 금지 스텝을 검사한다.
 * 반환 0 정상, 그 외 첫 금지 스텝이 있는 ACT ID. */
uint16_t seq_bind(const seq_site_t *site);
uint16_t seq_profile(void);
void     seq_pick_profile(void);   /* 파라미터를 읽은 뒤나 바꾼 뒤 프로파일을 다시 고른다 */
int      seq_call_once(uint16_t idx);   /* 사용자 함수를 한 번 부른다. 태그 A_CALL */

/* 시퀀스: 정지 완료 뒤 run 모듈이 seq_run() 을 호출한다. 한 번에 하나만 활성 */
void     seq_request(uint16_t id);
void     seq_cancel(uint16_t id);
int      seq_is_active(void);
uint16_t seq_active_id(void);
uint16_t seq_active_step(void);
void     seq_run(void);          /* 10 ms */
void     seq_abort_all(void);    /* STOP·E-STOP·W 리셋 */

/* 확장 조건 평가. tag 는 태그 번호 조건용 (시퀀스는 0) */
uint16_t seq_xcond(uint16_t c, uint16_t a, uint16_t b, uint16_t tag);
/* on·off 에 같이 선 비트는 "하나라도 ON", 나머지는 전부 ON / 전부 OFF */
uint16_t seq_mask_true(uint32_t v, uint32_t on, uint32_t off);

/* 사이트 상태 변수 (SET_STATE, WAIT_STATE, WHEN_STATE) */
uint16_t seq_state_get(uint16_t id);
void     seq_state_set(uint16_t id, uint16_t value);

/* ACT: 주행·정지와 무관하게 10 ms 마다 돈다. 슬롯 4개 */
#define ACT_SLOTS 4
int      act_start(uint16_t id);  /* 0 시작, 1 이미 실행 중(무시), 2 슬롯 없음 또는 미정의 */
int      act_is_busy(uint16_t id);
void     act_run(void);           /* 10 ms. seq_run 다음에 호출 */
void     act_abort_all(void);     /* 실행 중 ACT 를 멈추고 그 ACT 가 켠 출력만 끈다 */

/* 레거시 호환: DA_Guide_Stop_Action = &Flag 를 쓰는 앱 코드용 */
extern int seq_request_flag[SEQ_MAX];

#endif /* AGV_SEQ_H */
