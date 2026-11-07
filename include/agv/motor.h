/*
 * motor.h - rpm 피드백, 속도 PI, S커브, 주행 단계. 결과는 agv_drive 에 남고
 * tq_cmd·amp_run·amp_dir·sound_on 을 HAL 이 내보낸다. rpm 은 모터축 값이다.
 */
#ifndef AGV_MOTOR_H
#define AGV_MOTOR_H

#include "agv/types.h"

#define MOTOR_TORQUE_LIMIT   4000.0f
#define MOTOR_SPEED_KP       2.2f
#define MOTOR_SPEED_KI       0.2f
#define MOTOR_RPM_SPIKE      3500

void    motor_reset(void);
void    motor_encoder_pulse(uint16_t wheel);    /* eCAP ISR */
int16_t motor_idle_ticks(uint16_t wheel);       /* 마지막 펄스 뒤 틱 */
int16_t motor_fresh_pulse(uint16_t wheel);      /* 이번 틱에 펄스가 있었으면 1 */

void    motor_feedback_update(void);            /* MotorFeedback_Update */
void    motor_speed_pi(int16_t input);          /* Set_Velocity */
void    motor_velocity(int16_t input);          /* velocity */
int16_t motor_scv_speed(int16_t target, int16_t acc, int16_t dcc);    /* DA_SCV_Speed */
void    motor_drive_step(void);                 /* Driving_auto */
int16_t motor_turn_adjust(void);                /* DA_Tape_Turn_Angle_Adj */
void    motor_run(void);                        /* Motor_Run_Process. 오류 판정과 수동 동작은 호출자 몫 */

/* DA_SCV_Speed_Core. 상태를 인자로 받는 순수 함수 */
int16_t motor_scv_core(int16_t vin, int16_t *vout, int16_t acc, int16_t dcc, int16_t llow,
                       int16_t min_speed, int16_t stop_floor, int16_t use_dcc_floor, int16_t *accel);

#endif /* AGV_MOTOR_H */
