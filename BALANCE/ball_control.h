#ifndef __BALL_CONTROL_H
#define __BALL_CONTROL_H

#include "sys.h"

#define BALL_TARGET_CENTER_0P1MM              0.0f
#define BALL_TARGET_POSITIVE_5CM_0P1MM        500.0f
#define BALL_TARGET_NEGATIVE_5CM_0P1MM       (-500.0f)

#define BALL_SERVO_DEFAULT_MID_US             1350.0f
#define BALL_SERVO_DEFAULT_MIN_US             1000.0f
#define BALL_SERVO_DEFAULT_MAX_US             2000.0f
#define BALL_SERVO_DEFAULT_US_PER_DEG         11.11f
#define BALL_SERVO_DEFAULT_DIR                1.0f

#define BALL_IMU_ACCEL_AXIS_X                 0
#define BALL_IMU_ACCEL_AXIS_Y                 1
#define BALL_IMU_ACCEL_AXIS_Z                 2
#define BALL_IMU_FF_AXIS                      BALL_IMU_ACCEL_AXIS_X
#define BALL_IMU_FF_SIGN                      1.0f

typedef struct
{
    float alpha;
    float kp;
    float ki;
    float kd;
    float kf;
    float integral_limit;
    float angle_limit_deg;
    float deadband_deg;
    float rate_limit_deg;
    float servo_mid_us;
    float servo_min_us;
    float servo_max_us;
    float us_per_degree;
    float servo_dir;
    uint16_t vision_timeout_ms;
    uint16_t stable_error_0p1mm;
    uint16_t stable_speed_0p1mm_per_s;
    uint16_t stable_count_required;
} BallControlParam_t;

typedef struct
{
    uint8_t enabled;
    uint8_t online;
    uint8_t ball_lost;
    uint8_t filter_inited;
    int16_t raw_pos_0p1mm;
    float filtered_pos_0p1mm;
    float target_pos_0p1mm;
    float error_0p1mm;
    float last_error_0p1mm;
    float position_speed_0p1mm_per_s;
    float integral;
    float pid_angle_deg;
    float ff_angle_deg;
    float target_angle_deg;
    float output_angle_deg;
    uint16_t output_pwm_us;
    uint16_t stable_count;
    uint16_t lost_count;
    uint16_t safe_mode;
} BallControlState_t;

extern BallControlParam_t BallControlParam;
extern BallControlState_t BallControlState;

void Ball_Control_Init(void);
void Ball_Control_Enable(uint8_t enable);
void Ball_Control_Reset(void);
void Ball_Control_SetTarget(float target_0p1mm);
void Ball_Control_SetVisionPosition(int16_t pos_0p1mm);
void Ball_Control_SetVisionLost(void);
void Ball_Control_SetVisionHeartbeat(void);
void Ball_Control_Update(uint16_t period_ms);
void Ball_Control_StopSafe(void);
uint8_t Ball_Control_IsStable(void);
uint8_t Ball_Control_IsLost(void);
float Ball_Control_GetAbsError(void);
uint16_t Ball_Control_GetServoPwm(void);
uint8_t Ball_Control_IsServoOverrideEnabled(void);
float Ball_Control_GetFilteredPosition(void);

#endif
