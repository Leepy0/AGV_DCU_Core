/*
 * motor.h - rpm 피드백, 속도 PI, S커브, 주행 단계. 결과는 agv_drive 에 남고
 * tq_cmd·amp_run·amp_dir·sound_on 을 HAL 이 내보낸다. rpm 은 모터축 값이다.
 */
#ifndef AGV_MOTOR_H
#define AGV_MOTOR_H

#include "agv/types.h"
#include "agv/param.h"      /* AGV_DEV_TESTS 기본값, motor_speed_ff */

#define MOTOR_TORQUE_LIMIT   4000.0f
#define MOTOR_SPEED_KP       2.2f
#define MOTOR_SPEED_KI       0.2f
#define MOTOR_RPM_SPIKE      3500

void    motor_reset(void);
void    motor_encoder_pulse(uint16_t wheel);    /* eCAP ISR */
int16_t motor_idle_ticks(uint16_t wheel);       /* 마지막 펄스 뒤 틱 */
int16_t motor_fresh_pulse(uint16_t wheel);      /* 이번 틱에 펄스가 있었으면 1 */
uint16_t motor_pulse_seq(uint16_t wheel);       /* 누적 펄스. 16비트로 돈다. 차로만 쓴다 */
void    motor_stall_check(void);                /* P_ENC_STALL_TICKS. 한쪽 엔코더 고착이면 202·203 */

void    motor_feedback_update(void);            /* MotorFeedback_Update */
void    motor_speed_pi(int16_t input);          /* Set_Velocity */
void    motor_velocity(int16_t input);          /* velocity */
int16_t motor_scv_speed(int16_t target, int16_t acc, int16_t dcc);    /* DA_SCV_Speed */
void    motor_drive_step(void);                 /* Driving_auto */
int16_t motor_turn_adjust(void);                /* DA_Tape_Turn_Angle_Adj */
void    motor_run(void);                        /* Motor_Run_Process. 오류 판정과 수동 동작은 호출자 몫 */

/* 바닥 rpm 시험. 속도 PI 를 거치지 않고 두 바퀴 DAC 를 고정한다. 오류·E-STOP 이면 0 */
#define MOTOR_DAC_MAX        4095
void    motor_dac_force(int16_t left, int16_t right);
void    motor_dac_release(void);
int16_t motor_dac_forced(void);               /* 1 = 고정 중 */
int16_t motor_raw_rpm(uint16_t wheel);          /* 스파이크 컷 전 rpm. 32767 에서 자른다 */
void    motor_spike_set(int16_t rpm);           /* 측정 rpm 스파이크 컷. 0 = 기본 MOTOR_RPM_SPIKE */
int16_t motor_spike(void);

/* 속도 루프 시험(TST, AGV_DEV_TESTS 에서만 정의). 0 = 지금, 1 = 새1(1표본 되먹임 Kp 0.8 Ki 0.16 FF 0.999),
 * 2 = 새2(2표본 평균 Kp 0.8 Ki 0.12 FF 0.999). PI 오차의 되먹임만 바꾸고 D.rpm_avg 는 그대로다. 값은 motor.c s_spi */
#if AGV_DEV_TESTS
extern int16_t speed_pi_test;
float   motor_speed_ff(void);                   /* 지금 쓰는 피드포워드 계수(rpm 당 토크). 새1·새2 면 고정값 */
#else
#define motor_speed_ff() (param_dv.speed_ff)
#endif

/* DA_SCV_Speed_Core. 상태를 인자로 받는 순수 함수. 저크 min(2.4, 한계/8) rpm/틱²(정지 램프 llow -1 은 2.4), acc·dcc 는 가감속도 상한.
 * flags SCV_JERK_OLD = 예전 저크(한계/8)와 목표 0 일 때 stop_floor 아래 0 으로 한 번에(QUIRK_JERK_OLD 도 같다).
 * b0(예전 DCC 하한 20)과 min_speed 는 쓰지 않는다: P_DCC 한계가 20~60 이라 늘 DCC 그대로였다 */
#define SCV_JERK_OLD         0x02
int16_t motor_scv_core(int16_t vin, int16_t *vout, int16_t acc, int16_t dcc, int16_t llow,
                       int16_t min_speed, int16_t stop_floor, int16_t flags, int16_t *accel);

#endif /* AGV_MOTOR_H */
