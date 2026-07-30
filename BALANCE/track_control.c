#include "system.h"
#include "track_control.h"

TrackControlState_t TrackControlState;

static float Track_LimitFloat(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static float Track_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void Track_PID_Init(TrackPid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    if(pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void Track_PID_Reset(TrackPid_t *pid)
{
    if(pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

float Track_PID_UpdateError(TrackPid_t *pid, float error)
{
    float derivative;
    float output;

    if(pid == 0)
    {
        return 0.0f;
    }

    pid->integral += error;
    pid->integral = Track_LimitFloat(pid->integral, -pid->integral_limit, pid->integral_limit);
    derivative = error - pid->last_error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Track_LimitFloat(output, -pid->output_limit, pid->output_limit);

    pid->last_error = error;
    pid->output = output;

    return output;
}

float Track_PID_Update(TrackPid_t *pid, float target, float feedback)
{
    return Track_PID_UpdateError(pid, target - feedback);
}

void Track_Control_Init(void)
{
    Track_Control_Reset();

    Track_PID_Init(&TrackControlState.line_pid, 0.10f, 0.0f, 0.04f,
                   TRACK_DEFAULT_INTEGRAL_LIMIT, TRACK_DEFAULT_TURN_LIMIT);
    Track_PID_Init(&TrackControlState.yaw_pid, 0.015f, 0.0f, 0.004f,
                   TRACK_DEFAULT_INTEGRAL_LIMIT, TRACK_DEFAULT_YAW_LIMIT);

    Track_Control_SetStopParam(0, 0, 0, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
}

void Track_Control_Reset(void)
{
    TrackControlState.enable = 0;
    TrackControlState.mode = TRACK_MODE_STOP;
    TrackControlState.straight_yaw_enable = 0;
    TrackControlState.stop_request = 0;
    TrackControlState.stop_reason = TRACK_STOP_NONE;
    TrackControlState.base_speed = TRACK_DEFAULT_BASE_SPEED;
    TrackControlState.turn_delta_speed = 0.0f;
    TrackControlState.yaw_delta_speed = 0.0f;
    TrackControlState.left_target_speed = 0.0f;
    TrackControlState.right_target_speed = 0.0f;
    TrackControlState.yaw_ref = 0.0f;
    TrackControlState.yaw_error = 0.0f;
    TrackControlState.yaw_ref_valid = 0;
    TrackControlState.run_time_ms = 0;
    TrackControlState.stop_param.window_start_ms = 0;
    TrackControlState.stop_param.window_end_ms = 0;
    TrackControlState.stop_param.force_stop_ms = 0;
    TrackControlState.stop_param.wide_stable_count = TRACK_WIDE_STABLE_DEFAULT;
    TrackControlState.stop_param.lost_stop_count = TRACK_LOST_STOP_DEFAULT;

    Track_PID_Reset(&TrackControlState.line_pid);
    Track_PID_Reset(&TrackControlState.yaw_pid);
}

void Track_Control_Start(uint8_t mode, float base_speed)
{
    Track_Control_Reset();
    TrackControlState.enable = 1;
    TrackControlState.mode = mode;
    TrackControlState.base_speed = base_speed;
}

void Track_Control_Stop(uint8_t reason)
{
    TrackControlState.enable = 0;
    TrackControlState.stop_request = 1;
    TrackControlState.stop_reason = reason;
    TrackControlState.left_target_speed = 0.0f;
    TrackControlState.right_target_speed = 0.0f;
    TrackControlState.turn_delta_speed = 0.0f;
    TrackControlState.yaw_delta_speed = 0.0f;
}

void Track_Control_SetStopParam(uint32_t window_start_ms, uint32_t window_end_ms, uint32_t force_stop_ms,
                                 uint8_t wide_stable_count, uint16_t lost_stop_count)
{
    TrackControlState.stop_param.window_start_ms = window_start_ms;
    TrackControlState.stop_param.window_end_ms = window_end_ms;
    TrackControlState.stop_param.force_stop_ms = force_stop_ms;
    TrackControlState.stop_param.wide_stable_count = wide_stable_count;
    TrackControlState.stop_param.lost_stop_count = lost_stop_count;
}

void Track_Control_SetLinePid(float kp, float ki, float kd, float output_limit)
{
    Track_PID_Init(&TrackControlState.line_pid, kp, ki, kd, TRACK_DEFAULT_INTEGRAL_LIMIT, output_limit);
}

void Track_Control_SetYawPid(float kp, float ki, float kd, float output_limit)
{
    Track_PID_Init(&TrackControlState.yaw_pid, kp, ki, kd, TRACK_DEFAULT_INTEGRAL_LIMIT, output_limit);
}

void Track_Control_SetBaseSpeed(float base_speed)
{
    TrackControlState.base_speed = base_speed;
}

uint8_t Track_Control_IsStraightSegment(void)
{
    if(TrackIrState.line_valid == 0)
    {
        return 0;
    }

    if(Track_AbsFloat(TrackIrState.line_error) > TRACK_STRAIGHT_ERR_TH)
    {
        return 0;
    }

    if(TrackIrState.active_count > TRACK_STRAIGHT_WIDTH_TH)
    {
        return 0;
    }

    if(TrackIrState.wide_line)
    {
        return 0;
    }

    return 1;
}

static void Track_Control_UpdateStopJudge(void)
{
    uint8_t wide_line_stable;

    if(TrackControlState.stop_request)
    {
        return;
    }

    if(TrackIrState.lost_count >= TrackControlState.stop_param.lost_stop_count)
    {
        Track_Control_Stop(TRACK_STOP_BY_LOST_LINE);
        return;
    }

    wide_line_stable = Track_IR_IsWideLine(&TrackIrState, TrackControlState.stop_param.wide_stable_count);

    if((TrackControlState.stop_param.window_end_ms > TrackControlState.stop_param.window_start_ms) &&
       (TrackControlState.run_time_ms >= TrackControlState.stop_param.window_start_ms) &&
       (TrackControlState.run_time_ms <= TrackControlState.stop_param.window_end_ms) &&
       wide_line_stable)
    {
        Track_Control_Stop(TRACK_STOP_BY_MARKER);
        return;
    }

    if((TrackControlState.stop_param.force_stop_ms > 0) &&
       (TrackControlState.run_time_ms >= TrackControlState.stop_param.force_stop_ms))
    {
        Track_Control_Stop(TRACK_STOP_BY_TIMEOUT);
    }
}

static void Track_Control_UpdateYaw(float current_yaw_deg)
{
    if(Track_Control_IsStraightSegment())
    {
        TrackControlState.straight_yaw_enable = 1;

        if(TrackControlState.yaw_ref_valid == 0)
        {
            TrackControlState.yaw_ref = current_yaw_deg;
            TrackControlState.yaw_ref_valid = 1;
            Track_PID_Reset(&TrackControlState.yaw_pid);
        }

        TrackControlState.yaw_error = TrackControlState.yaw_ref - current_yaw_deg;
        TrackControlState.yaw_delta_speed = TRACK_YAW_DIR * Track_PID_UpdateError(&TrackControlState.yaw_pid,
                                                                                   TrackControlState.yaw_error);
    }
    else
    {
        TrackControlState.straight_yaw_enable = 0;
        TrackControlState.yaw_ref_valid = 0;
        TrackControlState.yaw_error = 0.0f;
        TrackControlState.yaw_delta_speed = 0.0f;
        Track_PID_Reset(&TrackControlState.yaw_pid);
    }
}

void Track_Control_Update(uint16_t period_ms, float current_yaw_deg)
{
    float line_error;

    Track_IR_Update(&TrackIrState);

    if(TrackControlState.enable == 0)
    {
        TrackControlState.left_target_speed = 0.0f;
        TrackControlState.right_target_speed = 0.0f;
        return;
    }

    TrackControlState.run_time_ms += period_ms;

    Track_Control_UpdateStopJudge();
    if(TrackControlState.stop_request)
    {
        TrackControlState.left_target_speed = 0.0f;
        TrackControlState.right_target_speed = 0.0f;
        return;
    }

    if(TrackIrState.line_valid)
    {
        line_error = 0.0f - TrackIrState.line_error;
        TrackControlState.turn_delta_speed = TRACK_TURN_DIR * Track_PID_UpdateError(&TrackControlState.line_pid,
                                                                                     line_error);
    }
    else
    {
        Track_PID_Reset(&TrackControlState.line_pid);
        TrackControlState.turn_delta_speed = TRACK_TURN_DIR * Track_LimitFloat(TrackIrState.last_line_error * 0.10f,
                                                                               -TrackControlState.line_pid.output_limit,
                                                                               TrackControlState.line_pid.output_limit);
    }

    Track_Control_UpdateYaw(current_yaw_deg);

    TrackControlState.left_target_speed = TrackControlState.base_speed - TrackControlState.turn_delta_speed - TrackControlState.yaw_delta_speed;
    TrackControlState.right_target_speed = TrackControlState.base_speed + TrackControlState.turn_delta_speed + TrackControlState.yaw_delta_speed;

    TrackControlState.left_target_speed = Track_LimitFloat(TrackControlState.left_target_speed,
                                                           -TRACK_DEFAULT_OUTPUT_LIMIT,
                                                           TRACK_DEFAULT_OUTPUT_LIMIT);
    TrackControlState.right_target_speed = Track_LimitFloat(TrackControlState.right_target_speed,
                                                            -TRACK_DEFAULT_OUTPUT_LIMIT,
                                                            TRACK_DEFAULT_OUTPUT_LIMIT);
}

uint8_t Track_Control_IsStopRequested(void)
{
    return TrackControlState.stop_request;
}
