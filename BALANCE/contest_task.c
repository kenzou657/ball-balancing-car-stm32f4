#include "system.h"
#include "contest_task.h"
#include "track_control.h"

ContestTaskContext_t ContestTaskContext;

#define CONTEST_TASK1_MOTOR_DEBUG_SPEED        0.20f

#define CONTEST_TASK2_STRAIGHT_DISTANCE_M      1.50f
#define CONTEST_TASK2_CURVE_RADIUS_M           0.50f
#define CONTEST_TASK2_CURVE_DISTANCE_M         (3.1416f * CONTEST_TASK2_CURVE_RADIUS_M)
#define CONTEST_TASK2_BASE_SPEED               0.28f
#define CONTEST_TASK2_YAW_KP                   0.010f
#define CONTEST_TASK2_YAW_LIMIT                0.08f
#define CONTEST_TASK2_CURVE_YAW_DEG            170.0f
#define CONTEST_TASK2_CURVE_TIMEOUT_MS         12000u
#define CONTEST_TASK2_MARKER_STABLE_COUNT      3u

#define CONTEST_TASK2_PHASE_IDLE               0
#define CONTEST_TASK2_PHASE_STRAIGHT1          1
#define CONTEST_TASK2_PHASE_CURVE1             2
#define CONTEST_TASK2_PHASE_STRAIGHT2          3
#define CONTEST_TASK2_PHASE_CURVE2             4

static void Contest_Task_Finish(void);

static void Contest_Task_ClearMotion(void)
{
    Move_X = 0.0f;
    Move_Y = 0.0f;
    Move_Z = 0.0f;
    MOTOR_A.Target = 0.0f;
    MOTOR_B.Target = 0.0f;
    MOTOR_C.Target = 0.0f;
    MOTOR_D.Target = 0.0f;
}

static void Contest_Task_ResetRuntime(void)
{
    ContestTaskContext.state_time_ms = 0;
    ContestTaskContext.phase_time_ms = 0;
    ContestTaskContext.phase_distance_m = 0.0f;
    ContestTaskContext.yaw_deg = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = 0.0f;
    ContestTaskContext.task2_phase = CONTEST_TASK2_PHASE_IDLE;
    ContestTaskContext.finished_task = CONTEST_TASK_NONE;
    ContestTaskContext.finished_time_ms = 0;
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Track_IR_Reset(&TrackIrState);
    Contest_Task_ClearMotion();
}

static uint8_t Contest_Task_IsExecutableTask(uint8_t task_id)
{
    return (task_id == CONTEST_TASK_1 ||
            task_id == CONTEST_TASK_2 ||
            task_id == CONTEST_TASK_4 ||
            task_id == CONTEST_TASK_5 ||
            task_id == CONTEST_TASK_6) ? 1 : 0;
}

static float Contest_Task_UpdateYaw(uint16_t period_ms)
{
    /* ICM20948 陀螺仪配置为 +-500dps，灵敏度约 65.5 LSB/(deg/s)。 */
    ICM20948_Get_Gyroscope();
    ContestTaskContext.yaw_deg += ((float)imu.gyro.z / 65.5f) * ((float)period_ms / 1000.0f);
    return ContestTaskContext.yaw_deg;
}

static void Contest_Task_SetDiffTarget(float left, float right)
{
    /* 赛题底盘固定为两轮差速：MotorA 左轮，MotorB 右轮。 */
    Move_X = (left + right) * 0.5f;
    Move_Y = 0.0f;
    Move_Z = 0.0f;
    MOTOR_A.Target = left;
    MOTOR_B.Target = right;
    MOTOR_C.Target = 0.0f;
    MOTOR_D.Target = 0.0f;
}

static void Contest_Task_ApplyTrackTarget(void)
{
    Contest_Task_SetDiffTarget(TrackControlState.left_target_speed, TrackControlState.right_target_speed);
}

static float Contest_Task_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Contest_Task_LimitFloat(float value, float min_value, float max_value)
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

static void Contest_Task_ResetPhase(uint8_t phase)
{
    ContestTaskContext.task2_phase = phase;
    ContestTaskContext.phase_time_ms = 0;
    ContestTaskContext.phase_distance_m = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = ContestTaskContext.yaw_deg;
    Track_IR_Reset(&TrackIrState);
    Track_PID_Reset(&TrackControlState.line_pid);
    Track_PID_Reset(&TrackControlState.yaw_pid);
}

static void Contest_Task_UpdatePhaseDistance(uint16_t period_ms)
{
    float left_speed = Contest_Task_AbsFloat(MOTOR_A.Encoder);
    float right_speed = Contest_Task_AbsFloat(MOTOR_B.Encoder);

    ContestTaskContext.phase_time_ms += period_ms;
    ContestTaskContext.phase_distance_m += ((left_speed + right_speed) * 0.5f) * ((float)period_ms / 1000.0f);
}

static void Contest_Task1_Update(uint16_t period_ms)
{
    (void)period_ms;

    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;

        case CONTEST_STATE_TRACK:
        
            // Set_Pwm(-16000,  16000,  0, 0, 0    );
            Contest_Task_SetDiffTarget(CONTEST_TASK1_MOTOR_DEBUG_SPEED,
                                       CONTEST_TASK1_MOTOR_DEBUG_SPEED);
            break;

        default:
            break;
    }
}

static void Contest_Task2_StartCurve(uint8_t curve_phase)
{
    Contest_Task_ResetPhase(curve_phase);
    Track_Control_Start(TRACK_MODE_LAP_A, CONTEST_TASK2_BASE_SPEED);
    Track_Control_SetStopParam(0, 0, 0, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
}

static void Contest_Task2_UpdateStraight(uint16_t period_ms, uint8_t next_phase)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);
    float yaw_error;
    float yaw_delta;

    Contest_Task_UpdatePhaseDistance(period_ms);

    yaw_error = ContestTaskContext.phase_start_yaw_deg - yaw_deg;
    yaw_delta = Contest_Task_LimitFloat(yaw_error * CONTEST_TASK2_YAW_KP,
                                        -CONTEST_TASK2_YAW_LIMIT,
                                        CONTEST_TASK2_YAW_LIMIT);

    Contest_Task_SetDiffTarget(CONTEST_TASK2_BASE_SPEED - yaw_delta,
                                CONTEST_TASK2_BASE_SPEED + yaw_delta);

    if(ContestTaskContext.phase_distance_m >= CONTEST_TASK2_STRAIGHT_DISTANCE_M)
    {
        Contest_Task2_StartCurve(next_phase);
    }
}

static uint8_t Contest_Task2_CurveComplete(void)
{
    float yaw_delta = Contest_Task_AbsFloat(ContestTaskContext.yaw_deg - ContestTaskContext.phase_start_yaw_deg);

    if(yaw_delta >= CONTEST_TASK2_CURVE_YAW_DEG)
    {
        return 1;
    }

    if(ContestTaskContext.phase_distance_m >= CONTEST_TASK2_CURVE_DISTANCE_M)
    {
        return 1;
    }

    if(ContestTaskContext.phase_time_ms >= CONTEST_TASK2_CURVE_TIMEOUT_MS)
    {
        return 1;
    }

    return 0;
}

static void Contest_Task2_UpdateCurve(uint16_t period_ms, uint8_t next_phase)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);

    Contest_Task_UpdatePhaseDistance(period_ms);
    Track_Control_Update(period_ms, yaw_deg);

    if(Track_Control_IsStopRequested())
    {
        Contest_Task_Finish();
        return;
    }

    Contest_Task_ApplyTrackTarget();

    if(Contest_Task2_CurveComplete())
    {
        Track_Control_Stop(TRACK_STOP_BY_USER);
        Contest_Task_ResetPhase(next_phase);
    }
}

static void Contest_Task2_UpdateFinalCurve(uint16_t period_ms)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);
    uint8_t marker_detected;

    Contest_Task_UpdatePhaseDistance(period_ms);
    Track_Control_Update(period_ms, yaw_deg);

    if(Track_Control_IsStopRequested())
    {
        Contest_Task_Finish();
        return;
    }

    Contest_Task_ApplyTrackTarget();

    marker_detected = Track_IR_IsWideLine(&TrackIrState, CONTEST_TASK2_MARKER_STABLE_COUNT);
    if((Contest_Task2_CurveComplete() && marker_detected) ||
       (ContestTaskContext.phase_time_ms >= CONTEST_TASK2_CURVE_TIMEOUT_MS))
    {
        Contest_Task_Finish();
    }
}

static void Contest_Task_StartTrack(uint8_t task_id)
{
    switch(task_id)
    {
        case CONTEST_TASK_2:
            Track_Control_Start(TRACK_MODE_LAP_A, 0.30f);
            Track_Control_SetStopParam(3000, 25000, 30000, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
            break;

        case CONTEST_TASK_4:
            Track_Control_Start(TRACK_MODE_AB, 0.32f);
            Track_Control_SetStopParam(2000, 20000, 25000, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
            break;

        case CONTEST_TASK_5:
            Track_Control_Start(TRACK_MODE_AB, 0.28f);
            Track_Control_SetStopParam(2000, 25000, 30000, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
            break;

        case CONTEST_TASK_6:
            Track_Control_Start(TRACK_MODE_AB, 0.25f);
            Track_Control_SetStopParam(2000, 30000, 35000, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
            break;

        default:
            break;
    }
}

static void Contest_Task_UpdateTrack(uint16_t period_ms)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);

    Track_Control_Update(period_ms, yaw_deg);

    if(Track_Control_IsStopRequested())
    {
        Contest_Task_Finish();
    }
    else
    {
        Contest_Task_ApplyTrackTarget();
    }
}

static void Contest_Task2_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_Task_ResetPhase(CONTEST_TASK2_PHASE_STRAIGHT1);
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;

        case CONTEST_STATE_TRACK:
            switch(ContestTaskContext.task2_phase)
            {
                case CONTEST_TASK2_PHASE_STRAIGHT1:
                    Contest_Task2_UpdateStraight(period_ms, CONTEST_TASK2_PHASE_CURVE1);
                    break;

                case CONTEST_TASK2_PHASE_CURVE1:
                    Contest_Task2_UpdateCurve(period_ms, CONTEST_TASK2_PHASE_STRAIGHT2);
                    break;

                case CONTEST_TASK2_PHASE_STRAIGHT2:
                    Contest_Task2_UpdateStraight(period_ms, CONTEST_TASK2_PHASE_CURVE2);
                    break;

                case CONTEST_TASK2_PHASE_CURVE2:
                    Contest_Task2_UpdateFinalCurve(period_ms);
                    break;

                default:
                    Contest_Task_ResetPhase(CONTEST_TASK2_PHASE_STRAIGHT1);
                    break;
            }
            break;

        default:
            break;
    }
}

static void Contest_Task4_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_Task_StartTrack(CONTEST_TASK_4);
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;
        case CONTEST_STATE_TRACK:
            Contest_Task_UpdateTrack(period_ms);
            break;
        default:
            break;
    }
}

static void Contest_Task5_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_Task_StartTrack(CONTEST_TASK_5);
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;
        case CONTEST_STATE_TRACK:
            Contest_Task_UpdateTrack(period_ms);
            break;
        default:
            break;
    }
}

static void Contest_Task6_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_Task_StartTrack(CONTEST_TASK_6);
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;
        case CONTEST_STATE_TRACK:
            Contest_Task_UpdateTrack(period_ms);
            break;
        default:
            break;
    }
}

void Contest_Task_Init(void)
{
    ContestTaskContext.selected_task = CONTEST_TASK_1;
    ContestTaskContext.running_task = CONTEST_TASK_NONE;
    ContestTaskContext.state = CONTEST_STATE_IDLE;
    Contest_Task_ResetRuntime();
    OLED_Clear();
}

void Contest_Task_Select(uint8_t task_id)
{
    if(task_id > CONTEST_TASK_6)
    {
        task_id = CONTEST_TASK_NONE;
    }

    if(ContestTaskContext.state != CONTEST_STATE_IDLE)
    {
        Contest_Task_Stop();
    }

    ContestTaskContext.selected_task = task_id;
    ContestTaskContext.finished_task = CONTEST_TASK_NONE;
    ContestTaskContext.finished_time_ms = 0;
}

void Contest_Task_StartSelected(void)
{
    if(ContestTaskContext.selected_task == CONTEST_TASK_NONE)
    {
        return;
    }

    Contest_Task_ResetRuntime();
    ContestTaskContext.running_task = ContestTaskContext.selected_task;
    ContestTaskContext.state = CONTEST_STATE_START;
}

void Contest_Task_KeyAction(uint8_t key_action)
{
    uint8_t next_task;

    if(key_action == single_click)
    {
        if(Contest_Task_IsRunning())
        {
            return;
        }

        next_task = ContestTaskContext.selected_task + 1;
        if(next_task > CONTEST_TASK_6)
        {
            next_task = CONTEST_TASK_1;
        }
        Contest_Task_Select(next_task);
    }
    else if(key_action == long_click)
    {
        if(Contest_Task_IsRunning())
        {
            Contest_Task_Stop();
            return;
        }

        Contest_Task_StartSelected();
    }
}

void Contest_Task_Stop(void)
{
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Contest_Task_ClearMotion();
    ContestTaskContext.running_task = CONTEST_TASK_NONE;
    ContestTaskContext.state = CONTEST_STATE_IDLE;
    ContestTaskContext.state_time_ms = 0;
}

static void Contest_Task_Finish(void)
{
    ContestTaskContext.finished_task = ContestTaskContext.running_task;
    ContestTaskContext.finished_time_ms = ContestTaskContext.state_time_ms;
    ContestTaskContext.state = CONTEST_STATE_FINISH;
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Contest_Task_ClearMotion();
}

void Contest_Task_Update(uint16_t period_ms)
{
    if(ContestTaskContext.state == CONTEST_STATE_IDLE)
    {
        return;
    }

    ContestTaskContext.state_time_ms += period_ms;

    if(Contest_Task_IsExecutableTask(ContestTaskContext.running_task) == 0)
    {
        Contest_Task_Finish();
    }

    switch(ContestTaskContext.running_task)
    {
        case CONTEST_TASK_1:
            Contest_Task1_Update(period_ms);
            break;
        case CONTEST_TASK_2:
            Contest_Task2_Update(period_ms);
            break;
        case CONTEST_TASK_4:
            Contest_Task4_Update(period_ms);
            break;
        case CONTEST_TASK_5:
            Contest_Task5_Update(period_ms);
            break;
        case CONTEST_TASK_6:
            Contest_Task6_Update(period_ms);
            break;
        default:
            break;
    }

    if(ContestTaskContext.state == CONTEST_STATE_FINISH)
    {
        ContestTaskContext.running_task = CONTEST_TASK_NONE;
        ContestTaskContext.state = CONTEST_STATE_IDLE;
    }
}

static void Contest_Task_OLEDShowText6(u8 x, u8 y, const u8 *p)
{
    while(*p != '\0')
    {
        if(x > 122 || y > 52)
        {
            return;
        }
        OLED_ShowChar(x, y, *p, 12, 1);
        x += 6;
        p++;
    }
}

static void Contest_Task_OLEDShowNumber6(u8 x, u8 y, u32 num, u8 len)
{
    u8 t;
    u8 temp;

    for(t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + 6 * t, y, temp + '0', 12, 1);
    }
}

static void Contest_Task_OLEDShowSigned6(u8 x, u8 y, float value, uint16_t scale, uint8_t len)
{
    long show_value;

    if(value < 0.0f)
    {
        OLED_ShowChar(x, y, '-', 12, 1);
        show_value = (long)(-value * (float)scale);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 12, 1);
        show_value = (long)(value * (float)scale);
    }
    Contest_Task_OLEDShowNumber6(x + 6, y, (u32)show_value, len);
}

static void Contest_Task_OLEDShowMotorDebug(uint8_t task_id, uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"MDBG T");
    Contest_Task_OLEDShowNumber6(36, 0, task_id, 1);
    Contest_Task_OLEDShowText6(54, 0, (const u8 *)"TIME");
    Contest_Task_OLEDShowNumber6(84, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(108, 0, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"A");
    Contest_Task_OLEDShowSigned6(12, 13, MOTOR_A.Encoder, 1000, 4);
    Contest_Task_OLEDShowText6(60, 13, (const u8 *)"B");
    Contest_Task_OLEDShowSigned6(72, 13, MOTOR_B.Encoder, 1000, 4);

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"TAR");
    Contest_Task_OLEDShowSigned6(24, 26, CONTEST_TASK1_MOTOR_DEBUG_SPEED, 1000, 4);
    Contest_Task_OLEDShowText6(66, 26, (const u8 *)"mm/s");

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"KP");
    Contest_Task_OLEDShowNumber6(18, 39, (uint32_t)Velocity_KP, 4);
    Contest_Task_OLEDShowText6(60, 39, (const u8 *)"KI");
    Contest_Task_OLEDShowNumber6(78, 39, (uint32_t)Velocity_KI, 4);

    Contest_Task_OLEDShowText6(0, 52, (const u8 *)"S:SEL  L:STOP");
    OLED_Refresh_Gram();
}

void Contest_Task_OLEDShow(void)
{
    uint32_t show_time;
    uint8_t task_id;

    if(Contest_Task_IsRunning())
    {
        task_id = ContestTaskContext.running_task;
        show_time = ContestTaskContext.state_time_ms;
    }
    else if(ContestTaskContext.finished_time_ms != 0)
    {
        task_id = ContestTaskContext.finished_task;
        show_time = ContestTaskContext.finished_time_ms;
    }
    else
    {
        task_id = ContestTaskContext.selected_task;
        show_time = 0;
    }

    if(task_id == CONTEST_TASK_1)
    {
        Contest_Task_OLEDShowMotorDebug(task_id, show_time);
        return;
    }

    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"CONTEST DIFF");
    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"TASK");
    Contest_Task_OLEDShowNumber6(30, 13, task_id, 1);

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"TIME");
    Contest_Task_OLEDShowNumber6(30, 26, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(54, 26, (const u8 *)".");
    Contest_Task_OLEDShowNumber6(60, 26, (show_time % 1000) / 100, 1);
    Contest_Task_OLEDShowText6(72, 26, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"PHASE");
    Contest_Task_OLEDShowNumber6(42, 39, ContestTaskContext.task2_phase, 1);

    Contest_Task_OLEDShowText6(0, 52, (const u8 *)"S:SEL  L:START");
    OLED_Refresh_Gram();
}

uint8_t Contest_Task_IsRunning(void)
{
    return (ContestTaskContext.state != CONTEST_STATE_IDLE) ? 1 : 0;
}

uint8_t Contest_Task_GetSelected(void)
{
    return ContestTaskContext.selected_task;
}
