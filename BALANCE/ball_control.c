#include "system.h"
#include "ball_control.h"

BallControlParam_t BallControlParam;
BallControlState_t BallControlState;

static float Ball_Control_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Ball_Control_LimitFloat(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    else if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint16_t Ball_Control_FloatToPwm(float pwm_us)
{
    if(BallControlParam.servo_min_us <= 0.0f || BallControlParam.servo_max_us <= 0.0f)
    {
        return (uint16_t)(BALL_SERVO_DEFAULT_MID_US + 0.5f);
    }

    pwm_us = Ball_Control_LimitFloat(pwm_us,
                                     BallControlParam.servo_min_us,
                                     BallControlParam.servo_max_us);

    return (uint16_t)(pwm_us + 0.5f);
}

static int16_t Ball_Control_GetImuAccelByAxis(uint8_t axis)
{
#if (BALL_IMU_FF_AXIS == BALL_IMU_ACCEL_AXIS_Y)
    (void)axis;
    return imu.accel.y;
#elif (BALL_IMU_FF_AXIS == BALL_IMU_ACCEL_AXIS_Z)
    (void)axis;
    return imu.accel.z;
#else
    (void)axis;
    return imu.accel.x;
#endif
}

static void Ball_Control_UpdateStableJudge(void)
{
    if(Ball_Control_AbsFloat(BallControlState.error_0p1mm) <= (float)BallControlParam.stable_error_0p1mm)
    {
        if(BallControlState.stable_count < BallControlParam.stable_count_required)
        {
            BallControlState.stable_count++;
        }
    }
    else
    {
        BallControlState.stable_count = 0;
    }
}

void Ball_Control_Init(void)
{
    BallControlParam.alpha = 0.25f;
    BallControlParam.kp = 0.024f;
    BallControlParam.ki = 0.012f;
    BallControlParam.kd = 0.010f;
    BallControlParam.kf = 6.700f;
    BallControlParam.integral_limit = 3000.0f;
    BallControlParam.angle_limit_deg = 40.0f;
    BallControlParam.deadband_deg = 1.0f;
    BallControlParam.rate_limit_deg = 2.0f;
    BallControlParam.servo_mid_us = BALL_SERVO_DEFAULT_MID_US;
    BallControlParam.servo_min_us = BALL_SERVO_DEFAULT_MIN_US;
    BallControlParam.servo_max_us = BALL_SERVO_DEFAULT_MAX_US;
    BallControlParam.us_per_degree = BALL_SERVO_DEFAULT_US_PER_DEG;
    BallControlParam.servo_dir = - BALL_SERVO_DEFAULT_DIR;
    BallControlParam.vision_timeout_ms = 200;
    BallControlParam.stable_error_0p1mm = 120;
    BallControlParam.stable_speed_0p1mm_per_s = 300;
    BallControlParam.stable_count_required = 50;

    Ball_Control_Reset();
}

void Ball_Control_Enable(uint8_t enable)
{
    BallControlState.enabled = (enable != 0) ? 1 : 0;
    if(BallControlState.enabled == 0)
    {
        Ball_Control_StopSafe();
    }
    else
    {
        BallControlState.safe_mode = 0;
        BallControlState.stable_count = 0;
    }
}

void Ball_Control_Reset(void)
{
    BallControlState.enabled = 0;
    BallControlState.online = 0;
    BallControlState.ball_lost = 0;
    BallControlState.filter_inited = 0;
    BallControlState.raw_pos_0p1mm = 0;
    BallControlState.filtered_pos_0p1mm = 0.0f;
    BallControlState.target_pos_0p1mm = BALL_TARGET_CENTER_0P1MM;
    BallControlState.error_0p1mm = 0.0f;
    BallControlState.last_error_0p1mm = 0.0f;
    BallControlState.position_speed_0p1mm_per_s = 0.0f;
    BallControlState.integral = 0.0f;
    BallControlState.pid_angle_deg = 0.0f;
    BallControlState.ff_angle_deg = 0.0f;
    BallControlState.target_angle_deg = 0.0f;
    BallControlState.output_angle_deg = 0.0f;
    BallControlState.output_pwm_us = Ball_Control_FloatToPwm(BallControlParam.servo_mid_us);
    BallControlState.stable_count = 0;
    BallControlState.lost_count = 0;
    BallControlState.safe_mode = 0;
}

void Ball_Control_SetTarget(float target_0p1mm)
{
    BallControlState.target_pos_0p1mm = target_0p1mm;
}

void Ball_Control_SetVisionPosition(int16_t pos_0p1mm)
{
    BallControlState.raw_pos_0p1mm = pos_0p1mm;
    BallControlState.online = 1;
    BallControlState.ball_lost = 0;
    BallControlState.lost_count = 0;
}

void Ball_Control_SetVisionLost(void)
{
    BallControlState.online = 1;
}

void Ball_Control_SetVisionHeartbeat(void)
{
    BallControlState.online = 1;
}

void Ball_Control_Update(uint16_t period_ms)
{
    float dt_s;
    float raw_pos;
    float last_filtered_pos;
    float derivative;
    float pid_output;
    float accel_g;
    float target_angle;
    float angle_delta;
    float pwm_us;

    if(period_ms == 0)
    {
        return;
    }

    dt_s = (float)period_ms * 0.001f;

    if(BallControlState.enabled == 0)
    {
        Ball_Control_StopSafe();
        return;
    }

    BallControlState.ball_lost = 0;
    BallControlState.lost_count = 0;
    BallControlState.safe_mode = 0;
    raw_pos = (float)BallControlState.raw_pos_0p1mm;
    last_filtered_pos = BallControlState.filtered_pos_0p1mm;

    if(BallControlState.filter_inited == 0)
    {
        BallControlState.filtered_pos_0p1mm = raw_pos;
        BallControlState.filter_inited = 1;
        BallControlState.position_speed_0p1mm_per_s = 0.0f;
    }
    else
    {
        BallControlParam.alpha = Ball_Control_LimitFloat(BallControlParam.alpha, 0.0f, 1.0f);
        BallControlState.filtered_pos_0p1mm = BallControlParam.alpha * raw_pos +
                                             (1.0f - BallControlParam.alpha) * BallControlState.filtered_pos_0p1mm;
        BallControlState.position_speed_0p1mm_per_s = (BallControlState.filtered_pos_0p1mm - last_filtered_pos) / dt_s;
    }

    BallControlState.last_error_0p1mm = BallControlState.error_0p1mm;
    BallControlState.error_0p1mm = - (BallControlState.target_pos_0p1mm - BallControlState.filtered_pos_0p1mm);

    BallControlState.integral += BallControlState.error_0p1mm * dt_s;
    BallControlState.integral = Ball_Control_LimitFloat(BallControlState.integral,
                                                        -BallControlParam.integral_limit,
                                                        BallControlParam.integral_limit);

    derivative = (BallControlState.error_0p1mm - BallControlState.last_error_0p1mm) / dt_s;
    pid_output = BallControlParam.kp * BallControlState.error_0p1mm +
                 BallControlParam.ki * BallControlState.integral +
                 BallControlParam.kd * derivative;
    BallControlState.pid_angle_deg = pid_output;

    accel_g = ((float)Ball_Control_GetImuAccelByAxis(BALL_IMU_FF_AXIS) / 16384.0f) * BALL_IMU_FF_SIGN;
    BallControlState.ff_angle_deg = BallControlParam.kf * accel_g;

    target_angle = BallControlState.pid_angle_deg + BallControlState.ff_angle_deg;
    target_angle = Ball_Control_LimitFloat(target_angle,
                                           -BallControlParam.angle_limit_deg,
                                           BallControlParam.angle_limit_deg);

    if(Ball_Control_AbsFloat(target_angle) < BallControlParam.deadband_deg)
    {
        target_angle = 0.0f;
    }

    BallControlState.target_angle_deg = target_angle;

    angle_delta = BallControlState.target_angle_deg - BallControlState.output_angle_deg;
    angle_delta = Ball_Control_LimitFloat(angle_delta,
                                          -BallControlParam.rate_limit_deg,
                                          BallControlParam.rate_limit_deg);
    BallControlState.output_angle_deg += angle_delta;

    pwm_us = BallControlParam.servo_mid_us +
             BallControlParam.servo_dir * BallControlState.output_angle_deg * BallControlParam.us_per_degree;
    BallControlState.output_pwm_us = Ball_Control_FloatToPwm(pwm_us);

    Ball_Control_UpdateStableJudge();
}

void Ball_Control_StopSafe(void)
{
    BallControlState.safe_mode = 1;
    BallControlState.integral = 0.0f;
    BallControlState.pid_angle_deg = 0.0f;
    BallControlState.ff_angle_deg = 0.0f;
    BallControlState.target_angle_deg = 0.0f;
    BallControlState.output_angle_deg = 0.0f;
    BallControlState.output_pwm_us = Ball_Control_FloatToPwm(BallControlParam.servo_mid_us);
}

uint8_t Ball_Control_IsStable(void)
{
    return (BallControlState.stable_count >= BallControlParam.stable_count_required) ? 1 : 0;
}

uint8_t Ball_Control_IsLost(void)
{
    return 0;
}

float Ball_Control_GetAbsError(void)
{
    return Ball_Control_AbsFloat(BallControlState.error_0p1mm);
}

uint16_t Ball_Control_GetServoPwm(void)
{
    return BallControlState.output_pwm_us;
}

uint8_t Ball_Control_IsServoOverrideEnabled(void)
{
    return (BallControlState.enabled != 0 || BallControlState.safe_mode != 0) ? 1 : 0;
}

float Ball_Control_GetFilteredPosition(void)
{
    return BallControlState.filtered_pos_0p1mm;
}
