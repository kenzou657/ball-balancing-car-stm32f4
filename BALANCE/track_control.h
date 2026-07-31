#ifndef __TRACK_CONTROL_H
#define __TRACK_CONTROL_H

#include "sys.h"
#include "track_ir.h"

#define TRACK_MODE_STOP                0
#define TRACK_MODE_LAP_A               1
#define TRACK_MODE_AB                  2

#define TRACK_TURN_DIR                 1.0f
#define TRACK_YAW_DIR                  -1.0f

#define TRACK_DEFAULT_BASE_SPEED       0.35f
#define TRACK_DEFAULT_TURN_LIMIT       0.35f
#define TRACK_DEFAULT_YAW_LIMIT        0.18f
#define TRACK_DEFAULT_OUTPUT_LIMIT     0.50f
#define TRACK_DEFAULT_INTEGRAL_LIMIT   100.0f

#define TRACK_STRAIGHT_ERR_TH          0.75f
#define TRACK_STRAIGHT_WIDTH_TH        3
#define TRACK_WIDE_STABLE_DEFAULT      3
#define TRACK_LOST_STOP_DEFAULT        30

typedef enum
{
    TRACK_STOP_NONE = 0,
    TRACK_STOP_BY_MARKER,
    TRACK_STOP_BY_TIMEOUT,
    TRACK_STOP_BY_LOST_LINE,
    TRACK_STOP_BY_USER
} TrackStopReason_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output;
    float integral_limit;
    float output_limit;
} TrackPid_t;

typedef struct
{
    uint32_t window_start_ms;
    uint32_t window_end_ms;
    uint32_t force_stop_ms;
    uint8_t wide_stable_count;
    uint16_t lost_stop_count;
} TrackStopParam_t;

typedef struct
{
    uint8_t enable;
    uint8_t mode;
    uint8_t yaw_enable;              /* 是否允许公共循迹 yaw 环参与输出，任务3弯道调试时关闭。 */
    uint8_t straight_yaw_enable;
    uint8_t stop_request;
    uint8_t stop_reason;

    float base_speed;
    float turn_delta_speed;
    float yaw_delta_speed;
    float left_target_speed;
    float right_target_speed;

    float yaw_ref;
    float yaw_error;
    uint8_t yaw_ref_valid;

    uint32_t run_time_ms;
    TrackStopParam_t stop_param;

    TrackPid_t line_pid;
    TrackPid_t yaw_pid;
} TrackControlState_t;

extern TrackControlState_t TrackControlState;

void Track_PID_Init(TrackPid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void Track_PID_Reset(TrackPid_t *pid);
float Track_PID_UpdateError(TrackPid_t *pid, float error);
float Track_PID_Update(TrackPid_t *pid, float target, float feedback);

void Track_Control_Init(void);
void Track_Control_Reset(void);
void Track_Control_Start(uint8_t mode, float base_speed);
void Track_Control_Stop(uint8_t reason);
void Track_Control_SetStopParam(uint32_t window_start_ms, uint32_t window_end_ms, uint32_t force_stop_ms,
                                 uint8_t wide_stable_count, uint16_t lost_stop_count);
void Track_Control_SetLinePid(float kp, float ki, float kd, float output_limit);
void Track_Control_SetYawPid(float kp, float ki, float kd, float output_limit);
void Track_Control_SetBaseSpeed(float base_speed);
void Track_Control_SetYawEnable(uint8_t enable);
void Track_Control_Update(uint16_t period_ms, float current_yaw_deg);
uint8_t Track_Control_IsStraightSegment(void);
uint8_t Track_Control_IsStopRequested(void);

#endif
