#include "system.h"
#include "contest_task.h"
#include "track_control.h"

ContestTaskContext_t ContestTaskContext;

#define CONTEST_TASK1_MOTOR_DEBUG_SPEED        0.20f   /* OLED 任务1电机调试目标线速度，单位 m/s，左右轮同速前进。 */

#define CONTEST_MERGED_BASE_SPEED              0.25f   /* 任务4/5/6合并任务默认循迹基础速度，单位 m/s。 */
#define CONTEST_MERGED_MIN_SPEED_SCALE         0.35f   /* 任务4/5/6根据滚球误差动态降速时允许的最低速度比例。 */
#define CONTEST_MERGED_SLOW_ERR_0P1MM          300.0f  /* 任务4/5/6开始明显降速的滚球误差阈值，单位 0.1mm，300 表示 3cm。 */
#define CONTEST_MERGED_PRE_BALANCE_HOLD_MS     3000u   /* 任务4/5/6起步前滚球稳定保持时间，达到后才开始循迹并开始运动计时。 */
#define CONTEST_MERGED_PRE_BALANCE_TIMEOUT_MS  20000u  /* 任务4/5/6预平衡最长等待时间，超时立即停车保护。 */
#define CONTEST_MERGED_AB_MIN_RUN_MS           2000u   /* 任务4选择 AB 模式时，允许识别终点宽线前的最短运行时间，单位 ms。 */
#define CONTEST_MERGED_AA_MIN_RUN_MS           3000u   /* 任务5/6选择 AA 整圈模式时，允许识别回到 A 点前的最短运行时间，单位 ms。 */
#define CONTEST_MERGED_AB_TIMEOUT_MS           30000u  /* 任务4 AB 模式运动阶段强制停车超时时间，单位 ms。 */
#define CONTEST_MERGED_AA_TIMEOUT_MS           45000u  /* 任务5/6 AA 模式运动阶段强制停车超时时间，单位 ms。 */

#define CONTEST_TASK3_PHASE_HOLD_MS            1500u   /* 滚球稳定每个目标位置至少保持的时间，单位 ms。 */
#define CONTEST_TASK3_PHASE_TIMEOUT_MS         8000u   /* 滚球稳定单个目标位置的最长等待时间，超时后切换下一阶段，单位 ms。 */

#define CONTEST_TASK2_CURVE_RADIUS_M           0.50f   /* 赛题任务2半圆弯道半径参考值，单位 m，用于给弯道阶段里程提供初始估算。 */

#define CONTEST_TASK2_STRAIGHT1_DISTANCE_M     1.44f   /* 赛题任务2第一段直行目标里程，单位 m，由左右轮编码器平均速度积分得到。 */
#define CONTEST_TASK2_CURVE1_DISTANCE_M        ((3.1416f * CONTEST_TASK2_CURVE_RADIUS_M) + 0.04f) /* 赛题任务2第一个半圆弯道目标里程，单位 m。 */
#define CONTEST_TASK2_STRAIGHT2_DISTANCE_M     1.25f   /* 赛题任务2第二段直行目标里程，单位 m，可独立补偿第二段直行误差。 */
#define CONTEST_TASK2_CURVE2_DISTANCE_M        ((3.1416f * CONTEST_TASK2_CURVE_RADIUS_M) + 0.04f) /* 赛题任务2第二个半圆弯道参考里程，单位 m；最终停车仍优先由总里程停车宏决定。 */

#define CONTEST_TASK2_STOP_DISTANCE_M          5.72f   /* 赛题任务2总里程停车位置，单位 m，到达该总里程立即停车，不再由各阶段里程相加决定终点。 */
#define CONTEST_TASK2_BASE_SPEED               0.38f   /* 赛题任务2直线段基础速度，单位 m/s。 */
#define CONTEST_TASK2_CURVE_SPEED              0.28f   /* 赛题任务2弯道循迹基础速度，单位 m/s，独立于直线速度便于弯道降速调试。 */
#define CONTEST_TASK2_STRAIGHT1_YAW_DEG        0.0f    /* 赛题任务2第一段直行 yaw 目标角，单位 deg。 */
#define CONTEST_TASK2_STRAIGHT2_YAW_DEG        -180.0f  /* 赛题任务2第二段直行 yaw 目标角，单位 deg。 */
#define CONTEST_TASK2_YAW_DIR                  1.0f    /* 赛题任务2直行 yaw 环输出方向，角度环极性不对时优先改这里为 -1.0f。 */
#define CONTEST_TASK2_YAW_KP                   0.008f  /* 赛题任务2直线段 yaw 位置环 P 系数。 */
#define CONTEST_TASK2_YAW_KI                   0.000f  /* 赛题任务2直线段 yaw 位置环 I 系数，先保持 0，确认无稳态偏差后再小量增加。 */
#define CONTEST_TASK2_YAW_KD                   0.010f   /* 赛题任务2直线段 yaw 位置环 D 系数，用于抑制直线摆动。 */
#define CONTEST_TASK2_YAW_INTEGRAL_LIMIT       30.0f   /* 赛题任务2直线段 yaw 积分限幅，单位 deg 累加量。 */
#define CONTEST_TASK2_YAW_LIMIT                0.08f   /* 赛题任务2直线段 yaw PID 输出限幅，单位 m/s。 */
#define CONTEST_TASK2_STRAIGHT1_YAW_SCALE      0.30f   /* 赛题任务2第一段直行 yaw/IMU 环输出比例，1.00 表示完整使用 yaw PID 输出。 */
#define CONTEST_TASK2_STRAIGHT1_LINE_SCALE     0.70f   /* 赛题任务2第一段直行红外循迹环输出比例，调小可降低直线贴线修正力度。 */
#define CONTEST_TASK2_STRAIGHT2_YAW_SCALE      0.70f   /* 赛题任务2第二段直行 yaw/IMU 环输出比例，可与第一段独立调节。 */
#define CONTEST_TASK2_STRAIGHT2_LINE_SCALE     0.30f   /* 赛题任务2第二段直行红外循迹环输出比例，可与第一段独立调节。 */
#define CONTEST_TASK2_CURVE_FF_DELTA           0.08f   /* 赛题任务2弯道基础转向前馈差速，单位 m/s，红外 PID 只负责修正残差。 */
#define CONTEST_TASK2_CURVE_FF_RAMP_M          0.40f   /* 赛题任务2弯道前馈渐入里程，单位 m，避免入弯瞬间阶跃打舵。 */
#define CONTEST_TASK2_CURVE_FF_SCALE           0.70f   /* 赛题任务2弯道前馈输出比例，调小可降低固定转弯量。 */
#define CONTEST_TASK2_CURVE_LINE_SCALE         0.30f   /* 赛题任务2弯道红外 PID 输出比例，调小可降低循迹反馈修正力度。 */
#define CONTEST_TASK2_CURVE1_FF_DIR            -1.0f   /* 赛题任务2第一个弯道前馈方向，方向反时改为 1.0f。 */
#define CONTEST_TASK2_CURVE2_FF_DIR            -1.0f   /* 赛题任务2第二个弯道前馈方向，方向反时改为 1.0f。 */

#define CONTEST_MERGED_AB_STRAIGHT1_DISTANCE_M 1.80f   /* 任务4 A->B 第一段直行目标里程，独立于任务5/6 A->A 调参。 */
#define CONTEST_MERGED_AA_STRAIGHT1_DISTANCE_M 1.44f   /* 任务5/6 A->A 第一段直行目标里程，独立于任务4 A->B 调参。 */
#define CONTEST_MERGED_CURVE1_DISTANCE_M       ((3.1416f * CONTEST_TASK2_CURVE_RADIUS_M) + 0.04f) /* 任务4/5/6第一个半圆弯道目标里程。 */
#define CONTEST_MERGED_STRAIGHT2_DISTANCE_M    1.25f   /* 任务4/5/6第二段直行目标里程，独立于任务3调参。 */
#define CONTEST_MERGED_CURVE2_DISTANCE_M       ((3.1416f * CONTEST_TASK2_CURVE_RADIUS_M) + 0.04f) /* 任务4/5/6第二个半圆弯道目标里程。 */
#define CONTEST_MERGED_STOP_DISTANCE_M         6.10f   /* 任务4/5/6总里程停车位置，后续按任务4 AB/任务5 AA 分别微调。 */
#define CONTEST_MERGED_AB_STRAIGHT_SPEED       0.37f   /* 任务4 A->B 直线段基础速度，单位 m/s，独立于任务5/6 A->A 调参。 */
#define CONTEST_MERGED_AB_CURVE_SPEED          0.25f   /* 任务4 A->B 弯道循迹基础速度，单位 m/s，独立于任务5/6 A->A 调参。 */
#define CONTEST_MERGED_AA_STRAIGHT_SPEED       0.23f   /* 任务5/6 A->A 直线段基础速度，单位 m/s，独立于任务4 A->B 调参。 */
#define CONTEST_MERGED_AA_CURVE_SPEED          0.23f   /* 任务5/6 A->A 弯道循迹基础速度，单位 m/s，独立于任务4 A->B 调参。 */
#define CONTEST_MERGED_ACCEL_LIMIT_MPS2        0.05f   /* 任务4/5/6慢启动加速度限制，单位 m/s^2；越小起步越柔和，0.05表示约7.4s升到0.37m/s。 */
#define CONTEST_MERGED_STRAIGHT1_YAW_SCALE     0.35f   /* 任务4/5/6第一段直行 yaw/IMU 环输出比例。 */
#define CONTEST_MERGED_STRAIGHT1_LINE_SCALE    0.65f   /* 任务4/5/6第一段直行红外循迹环输出比例。 */
#define CONTEST_MERGED_STRAIGHT2_YAW_SCALE     0.70f   /* 任务4/5/6第二段直行 yaw/IMU 环输出比例。 */
#define CONTEST_MERGED_STRAIGHT2_LINE_SCALE    0.30f   /* 任务4/5/6第二段直行红外循迹环输出比例。 */
#define CONTEST_MERGED_CURVE_FF_DELTA          0.06f   /* 任务4/5/6弯道基础转向前馈差速，单位 m/s。 */
#define CONTEST_MERGED_CURVE_FF_RAMP_M         0.40f   /* 任务4/5/6弯道前馈渐入里程，单位 m。 */
#define CONTEST_MERGED_CURVE_FF_SCALE          0.60f   /* 任务4/5/6弯道前馈输出比例。 */
#define CONTEST_MERGED_CURVE_LINE_SCALE        0.30f   /* 任务4/5/6弯道红外 PID 修正比例。 */

#define CONTEST_TASK2_PHASE_IDLE               0       /* 赛题任务2状态机空闲阶段。 */
#define CONTEST_TASK2_PHASE_STRAIGHT1          1       /* 赛题任务2第一段直行阶段。 */
#define CONTEST_TASK2_PHASE_CURVE1             2       /* 赛题任务2第一个半圆弯道阶段。 */
#define CONTEST_TASK2_PHASE_STRAIGHT2          3       /* 赛题任务2第二段直行阶段。 */
#define CONTEST_TASK2_PHASE_CURVE2             4       /* 赛题任务2第二个半圆弯道阶段，结束后停车。 */

static void Contest_Task_Finish(void);
static float Contest_Task_AbsFloat(float value);
static float Contest_Task_LimitFloat(float value, float min_value, float max_value);
static void Contest_Task2_PathStep(uint16_t period_ms);
static void Contest_MergedTask_FailStop(void);
static void Contest_MergedTask_Update(uint16_t period_ms);
static void Contest_Task_DebugPrint(uint16_t period_ms);

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
    ContestTaskContext.total_distance_m = 0.0f;
    ContestTaskContext.yaw_deg = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = 0.0f;
    ContestTaskContext.task2_phase = CONTEST_TASK2_PHASE_IDLE;
    ContestTaskContext.task3_phase = CONTEST_TASK3_PHASE_IDLE;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_PRE_BALANCE;
    ContestTaskContext.merged_pre_balance_ms = 0;
    ContestTaskContext.merged_stable_hold_ms = 0;
    ContestTaskContext.merged_motion_time_ms = 0;
    ContestTaskContext.merged_motion_time_frozen_ms = 0;
    ContestTaskContext.merged_fail_stop = 0;
    Ball_Control_Enable(0);
    Ball_Control_Reset();
    Ball_Control_StopSafe();
    ContestTaskContext.finished_task = CONTEST_TASK_NONE;
    ContestTaskContext.finished_time_ms = 0;
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Track_IR_Reset(&TrackIrState);
    Contest_Task_ClearMotion();
}

static uint8_t Contest_Task_IsExecutableTask(uint8_t task_id)
{
    return (task_id >= CONTEST_TASK_1 && task_id <= CONTEST_TASK_MAX) ? 1 : 0;
}

static uint8_t Contest_Task_IsMergedTask(uint8_t task_id)
{
    return (task_id == CONTEST_TASK_5 ||
            task_id == CONTEST_TASK_6 ||
            task_id == CONTEST_TASK_7) ? 1 : 0;
}

static float Contest_Task_UpdateYaw(uint16_t period_ms)
{
    if(SysVal.HardWare_Ver == V1_0)
    {
        MPU6050_Get_Gyroscope();
        MPU6050_Get_Accelscope();
    }
    else if(SysVal.HardWare_Ver == V1_1)
    {
        ICM20948_Get_Gyroscope();
        ICM20948_Get_Accel();
    }

    ContestTaskContext.yaw_deg += ((float)imu.gyro.z / 65.5f) * ((float)period_ms / 1000.0f);
    return ContestTaskContext.yaw_deg;
}

static void Contest_Task_SetDiffTarget(float left, float right)
{
    float speed_ref;
    float accel_limit_speed;
    float start_scale;

    if(Contest_Task_IsMergedTask(ContestTaskContext.running_task) &&
       ContestTaskContext.merged_run_phase == CONTEST_MERGED_RUN_MOTION)
    {
        speed_ref = Contest_Task_AbsFloat(left);
        if(Contest_Task_AbsFloat(right) > speed_ref)
        {
            speed_ref = Contest_Task_AbsFloat(right);
        }

        if(speed_ref > 0.001f)
        {
            accel_limit_speed = CONTEST_MERGED_ACCEL_LIMIT_MPS2 * ((float)ContestTaskContext.merged_motion_time_ms * 0.001f);
            start_scale = Contest_Task_LimitFloat(accel_limit_speed / speed_ref, 0.0f, 1.0f);
            left *= start_scale;
            right *= start_scale;
        }
    }

    /* 赛题底盘固定为两轮差速：MotorD 左轮，MotorA 右轮。 */
    Move_X = (left + right) * 0.5f;
    Move_Y = 0.0f;
    Move_Z = 0.0f;
    MOTOR_A.Target = right;
    MOTOR_B.Target = 0.0f;
    MOTOR_C.Target = 0.0f;
    MOTOR_D.Target = left;
}

static void Contest_Task_ApplyTrackTarget(void)
{
    Contest_Task_SetDiffTarget(TrackControlState.left_target_speed, TrackControlState.right_target_speed);
}

static void Contest_Task_ApplyTrackTargetScale(float speed_scale)
{
    speed_scale = Contest_Task_LimitFloat(speed_scale,
                                          CONTEST_MERGED_MIN_SPEED_SCALE,
                                          1.0f);
    Contest_Task_SetDiffTarget(TrackControlState.left_target_speed * speed_scale,
                                TrackControlState.right_target_speed * speed_scale);
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

static float Contest_Task2_GetStraightYawTarget(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_STRAIGHT2)
    {
        return CONTEST_TASK2_STRAIGHT2_YAW_DEG;
    }

    return CONTEST_TASK2_STRAIGHT1_YAW_DEG;
}

static float Contest_Task2_GetStraightDistance(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_STRAIGHT2)
    {
        return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
               CONTEST_MERGED_STRAIGHT2_DISTANCE_M : CONTEST_TASK2_STRAIGHT2_DISTANCE_M;
    }

    if(ContestTaskContext.running_task == CONTEST_TASK_5)
    {
        return CONTEST_MERGED_AB_STRAIGHT1_DISTANCE_M;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_AA_STRAIGHT1_DISTANCE_M : CONTEST_TASK2_STRAIGHT1_DISTANCE_M;
}

static float Contest_Task2_GetCurveDistance(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_CURVE2)
    {
        return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
               CONTEST_MERGED_CURVE2_DISTANCE_M : CONTEST_TASK2_CURVE2_DISTANCE_M;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_CURVE1_DISTANCE_M : CONTEST_TASK2_CURVE1_DISTANCE_M;
}

static float Contest_Task2_GetStraightYawScale(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_STRAIGHT2)
    {
        return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
               CONTEST_MERGED_STRAIGHT2_YAW_SCALE : CONTEST_TASK2_STRAIGHT2_YAW_SCALE;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_STRAIGHT1_YAW_SCALE : CONTEST_TASK2_STRAIGHT1_YAW_SCALE;
}

static float Contest_Task2_GetStraightLineScale(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_STRAIGHT2)
    {
        return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
               CONTEST_MERGED_STRAIGHT2_LINE_SCALE : CONTEST_TASK2_STRAIGHT2_LINE_SCALE;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_STRAIGHT1_LINE_SCALE : CONTEST_TASK2_STRAIGHT1_LINE_SCALE;
}

static float Contest_Task2_GetCurveFeedforwardDir(void)
{
    if(ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_CURVE2)
    {
        return CONTEST_TASK2_CURVE2_FF_DIR;
    }

    return CONTEST_TASK2_CURVE1_FF_DIR;
}

static float Contest_Task2_GetCurveFeedforward(void)
{
    float ramp_scale = 1.0f;
    float ff_ramp_m = Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
                      CONTEST_MERGED_CURVE_FF_RAMP_M : CONTEST_TASK2_CURVE_FF_RAMP_M;
    float ff_delta = Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
                     CONTEST_MERGED_CURVE_FF_DELTA : CONTEST_TASK2_CURVE_FF_DELTA;

    if(ff_ramp_m > 0.0f)
    {
        ramp_scale = ContestTaskContext.phase_distance_m / ff_ramp_m;
        ramp_scale = Contest_Task_LimitFloat(ramp_scale, 0.0f, 1.0f);
    }

    return Contest_Task2_GetCurveFeedforwardDir() * ff_delta * ramp_scale;
}

static void Contest_Task_ApplyCurveTrackTarget(void)
{
    float curve_line_scale = Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
                             CONTEST_MERGED_CURVE_LINE_SCALE : CONTEST_TASK2_CURVE_LINE_SCALE;
    float curve_ff_scale = Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
                           CONTEST_MERGED_CURVE_FF_SCALE : CONTEST_TASK2_CURVE_FF_SCALE;
    float line_pid_delta = TrackControlState.turn_delta_speed * curve_line_scale;
    float curve_ff_delta = Contest_Task2_GetCurveFeedforward() * curve_ff_scale;
    float mixed_delta = line_pid_delta + curve_ff_delta;

    TrackControlState.turn_delta_speed = mixed_delta;
    TrackControlState.left_target_speed = Contest_Task_LimitFloat(TrackControlState.base_speed - mixed_delta,
                                                                  -TRACK_DEFAULT_OUTPUT_LIMIT,
                                                                  TRACK_DEFAULT_OUTPUT_LIMIT);
    TrackControlState.right_target_speed = Contest_Task_LimitFloat(TrackControlState.base_speed + mixed_delta,
                                                                   -TRACK_DEFAULT_OUTPUT_LIMIT,
                                                                   TRACK_DEFAULT_OUTPUT_LIMIT);

    Contest_Task_ApplyTrackTarget();
}

static void Contest_Task_ResetPhase(uint8_t phase)
{
    ContestTaskContext.task2_phase = phase;
    ContestTaskContext.phase_time_ms = 0;
    ContestTaskContext.phase_distance_m = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = ContestTaskContext.yaw_deg;
    Track_IR_Reset(&TrackIrState);
    Track_PID_Reset(&TrackControlState.line_pid);
    Track_PID_Init(&TrackControlState.yaw_pid,
                   CONTEST_TASK2_YAW_KP,
                   CONTEST_TASK2_YAW_KI,
                   CONTEST_TASK2_YAW_KD,
                   CONTEST_TASK2_YAW_INTEGRAL_LIMIT,
                   CONTEST_TASK2_YAW_LIMIT);
}

static void Contest_Task_UpdatePhaseDistance(uint16_t period_ms)
{
    float left_speed = Contest_Task_AbsFloat(MOTOR_D.Encoder);
    float right_speed = Contest_Task_AbsFloat(MOTOR_A.Encoder);
    float distance_delta_m = ((left_speed + right_speed) * 0.5f) * ((float)period_ms / 1000.0f);

    ContestTaskContext.phase_time_ms += period_ms;
    ContestTaskContext.phase_distance_m += distance_delta_m;
    ContestTaskContext.total_distance_m += distance_delta_m;
}

static float Contest_Task2_GetStraightSpeed(void)
{
    if(ContestTaskContext.running_task == CONTEST_TASK_5)
    {
        return CONTEST_MERGED_AB_STRAIGHT_SPEED;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_AA_STRAIGHT_SPEED : CONTEST_TASK2_BASE_SPEED;
}

static float Contest_Task2_GetCurveSpeed(void)
{
    if(ContestTaskContext.running_task == CONTEST_TASK_5)
    {
        return CONTEST_MERGED_AB_CURVE_SPEED;
    }

    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_AA_CURVE_SPEED : CONTEST_TASK2_CURVE_SPEED;
}

static float Contest_Task2_GetStopDistance(void)
{
    return Contest_Task_IsMergedTask(ContestTaskContext.running_task) ?
           CONTEST_MERGED_STOP_DISTANCE_M : CONTEST_TASK2_STOP_DISTANCE_M;
}

static uint8_t Contest_Task2_StopDistanceReached(void)
{
    return (ContestTaskContext.total_distance_m >= Contest_Task2_GetStopDistance()) ? 1 : 0;
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
    Track_Control_Start(TRACK_MODE_LAP_A, Contest_Task2_GetCurveSpeed());
    Track_Control_SetYawEnable(0); /* 任务3弯道调试：禁用公共 yaw 环，只保留红外线 PID。 */
    Track_Control_SetStopParam(0, 0, 0, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
}

static void Contest_Task2_UpdateStraight(uint16_t period_ms, uint8_t next_phase)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);
    float yaw_error;
    float line_error;
    float yaw_delta;
    float line_delta;
    float mixed_delta;

    Contest_Task_UpdatePhaseDistance(period_ms);
    Track_IR_Update(&TrackIrState);

    yaw_error = Contest_Task2_GetStraightYawTarget() - yaw_deg;
    yaw_delta = CONTEST_TASK2_YAW_DIR * Track_PID_UpdateError(&TrackControlState.yaw_pid, yaw_error);

    if(TrackIrState.line_valid)
    {
        line_error = 0.0f - TrackIrState.line_error;
        line_delta = TRACK_TURN_DIR * Track_PID_UpdateError(&TrackControlState.line_pid, line_error);
    }
    else
    {
        Track_PID_Reset(&TrackControlState.line_pid);
        line_delta = 0.0f;
    }

    TrackControlState.yaw_error = yaw_error;
    TrackControlState.yaw_delta_speed = yaw_delta * Contest_Task2_GetStraightYawScale();
    TrackControlState.turn_delta_speed = line_delta * Contest_Task2_GetStraightLineScale();
    mixed_delta = TrackControlState.yaw_delta_speed + TrackControlState.turn_delta_speed;

    Contest_Task_SetDiffTarget(Contest_Task2_GetStraightSpeed() - mixed_delta,
                                Contest_Task2_GetStraightSpeed() + mixed_delta);

    if(Contest_Task2_StopDistanceReached())
    {
        Contest_Task_Finish();
        return;
    }

    if(ContestTaskContext.phase_distance_m >= Contest_Task2_GetStraightDistance())
    {
        if(ContestTaskContext.running_task == CONTEST_TASK_5 &&
           ContestTaskContext.task2_phase == CONTEST_TASK2_PHASE_STRAIGHT1)
        {
            Contest_Task_Finish();
            return;
        }

        Contest_Task2_StartCurve(next_phase);
    }
}

static uint8_t Contest_Task2_PhaseDistanceReached(float target_distance_m)
{
    return (ContestTaskContext.phase_distance_m >= target_distance_m) ? 1 : 0;
}

static uint8_t Contest_Task2_CurveComplete(void)
{
    return Contest_Task2_PhaseDistanceReached(Contest_Task2_GetCurveDistance());
}

static void Contest_Task2_UpdateCurve(uint16_t period_ms, uint8_t next_phase)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);

    Contest_Task_UpdatePhaseDistance(period_ms);
    Track_Control_Update(period_ms, yaw_deg);

    if(Track_Control_IsStopRequested())
    {
        if(Contest_Task_IsMergedTask(ContestTaskContext.running_task) &&
           TrackControlState.stop_reason == TRACK_STOP_BY_TIMEOUT)
        {
            Contest_MergedTask_FailStop();
        }
        else
        {
            Contest_Task_Finish();
        }
        return;
    }

    Contest_Task_ApplyCurveTrackTarget();

    if(Contest_Task2_StopDistanceReached())
    {
        Track_Control_SetYawEnable(1);
        Contest_Task_Finish();
        return;
    }

    if(Contest_Task2_CurveComplete())
    {
        Track_Control_Stop(TRACK_STOP_BY_USER);
        Track_Control_SetYawEnable(1);
        Contest_Task_ResetPhase(next_phase);
    }
}

static void Contest_Task2_UpdateFinalCurve(uint16_t period_ms)
{
    float yaw_deg = Contest_Task_UpdateYaw(period_ms);

    Contest_Task_UpdatePhaseDistance(period_ms);
    Track_Control_Update(period_ms, yaw_deg);

    if(Track_Control_IsStopRequested())
    {
        if(Contest_Task_IsMergedTask(ContestTaskContext.running_task) &&
           TrackControlState.stop_reason == TRACK_STOP_BY_TIMEOUT)
        {
            Contest_MergedTask_FailStop();
        }
        else
        {
            Contest_Task_Finish();
        }
        return;
    }

    Contest_Task_ApplyCurveTrackTarget();

    if(Contest_Task2_StopDistanceReached())
    {
        Track_Control_SetYawEnable(1);
        Contest_Task_Finish();
    }
}

static void Contest_Task_StartTrack(uint8_t task_id)
{
    switch(task_id)
    {
        case CONTEST_TASK_3:
            Track_Control_Start(TRACK_MODE_LAP_A, CONTEST_TASK2_CURVE_SPEED);
            Track_Control_SetYawEnable(0); /* 任务3只在弯道使用公共循迹模块，先禁用 yaw 环隔离极性问题。 */
            Track_Control_SetStopParam(0, 0, 0, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
            break;

        case CONTEST_TASK_5:
            Track_Control_Start(TRACK_MODE_AB, 0.28f);
            Track_Control_SetYawEnable(1);
            Track_Control_SetStopParam(2000, 25000, 30000, TRACK_WIDE_STABLE_DEFAULT, TRACK_LOST_STOP_DEFAULT);
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

static void Contest_MergedTask_LoadConfig(void)
{
    if(ContestTaskContext.running_task == CONTEST_TASK_5)
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AB;
        ContestTaskContext.merged_balance_cm = 0;
    }
    else if(ContestTaskContext.running_task == CONTEST_TASK_6)
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AA;
        ContestTaskContext.merged_balance_cm = 0;
    }
    else
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AA;
        ContestTaskContext.merged_balance_cm = ContestTaskContext.merged_task6_target_cm;
    }
}

static uint32_t Contest_MergedTask_GetMotionTimeoutMs(void)
{
    return (ContestTaskContext.running_task == CONTEST_TASK_5) ?
           CONTEST_MERGED_AB_TIMEOUT_MS : CONTEST_MERGED_AA_TIMEOUT_MS;
}

static void Contest_MergedTask_FreezeMotionTime(void)
{
    ContestTaskContext.merged_motion_time_frozen_ms = ContestTaskContext.merged_motion_time_ms;
}

static uint8_t Contest_MergedTask_IsTrackFailsafeStop(void)
{
    if(Track_Control_IsStopRequested() == 0)
    {
        return 0;
    }

    return (TrackControlState.stop_reason == TRACK_STOP_BY_TIMEOUT) ? 1 : 0;
}

static void Contest_MergedTask_StartPreBalance(void)
{
    float target_0p1mm;

    Contest_MergedTask_LoadConfig();
    target_0p1mm = (float)ContestTaskContext.merged_balance_cm * 100.0f;

    ContestTaskContext.phase_time_ms = 0;
    ContestTaskContext.phase_distance_m = 0.0f;
    ContestTaskContext.total_distance_m = 0.0f;
    ContestTaskContext.yaw_deg = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = 0.0f;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_PRE_BALANCE;
    ContestTaskContext.merged_pre_balance_ms = 0;
    ContestTaskContext.merged_stable_hold_ms = 0;
    ContestTaskContext.merged_motion_time_ms = 0;
    ContestTaskContext.merged_motion_time_frozen_ms = 0;
    ContestTaskContext.merged_fail_stop = 0;

    Track_Control_Stop(TRACK_STOP_BY_USER);
    Ball_Control_Reset();
    Ball_Control_SetTarget(target_0p1mm);
    Ball_Control_Enable(1);
    Contest_Task_ClearMotion();
}

static void Contest_MergedTask_StartMotion(void)
{
    Contest_Task_ResetPhase(CONTEST_TASK2_PHASE_STRAIGHT1);
    ContestTaskContext.total_distance_m = 0.0f;
    ContestTaskContext.yaw_deg = 0.0f;
    ContestTaskContext.phase_start_yaw_deg = 0.0f;
    ContestTaskContext.merged_motion_time_ms = 0;
    ContestTaskContext.merged_motion_time_frozen_ms = 0;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_MOTION;
}

static void Contest_MergedTask_FailStop(void)
{
    Contest_MergedTask_FreezeMotionTime();
    ContestTaskContext.finished_task = ContestTaskContext.running_task;
    ContestTaskContext.finished_time_ms = ContestTaskContext.merged_motion_time_frozen_ms;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_FAIL;
    ContestTaskContext.merged_fail_stop = 1;
    ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_FAIL_STOP;
    ContestTaskContext.state = CONTEST_STATE_IDLE;
    ContestTaskContext.running_task = CONTEST_TASK_NONE;

    Track_Control_Stop(TRACK_STOP_BY_USER);
    Ball_Control_Enable(0);
    Ball_Control_StopSafe();
    Contest_Task_ClearMotion();
}

static void Contest_MergedTask_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_MergedTask_StartPreBalance();
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_RUNNING;
            break;

        case CONTEST_STATE_TRACK:
            if(ContestTaskContext.merged_run_phase == CONTEST_MERGED_RUN_PRE_BALANCE)
            {
                ContestTaskContext.phase_time_ms += period_ms;
                ContestTaskContext.merged_pre_balance_ms += period_ms;
                Ball_Control_Update(period_ms);
                Contest_Task_ClearMotion();

                if(Ball_Control_IsStable())
                {
                    ContestTaskContext.merged_stable_hold_ms += period_ms;
                }
                else
                {
                    ContestTaskContext.merged_stable_hold_ms = 0;
                }

                if(ContestTaskContext.merged_stable_hold_ms >= CONTEST_MERGED_PRE_BALANCE_HOLD_MS)
                {
                    Contest_MergedTask_StartMotion();
                    break;
                }

                if(ContestTaskContext.merged_pre_balance_ms >= CONTEST_MERGED_PRE_BALANCE_TIMEOUT_MS)
                {
                    Contest_MergedTask_FailStop();
                    return;
                }
                break;
            }

            if(ContestTaskContext.merged_run_phase == CONTEST_MERGED_RUN_MOTION)
            {
                ContestTaskContext.merged_motion_time_ms += period_ms;
                if(ContestTaskContext.merged_motion_time_ms >= Contest_MergedTask_GetMotionTimeoutMs())
                {
                    Contest_MergedTask_FailStop();
                    return;
                }

                Contest_Task2_PathStep(period_ms);
                if(ContestTaskContext.state != CONTEST_STATE_TRACK)
                {
                    return;
                }

                if(Contest_MergedTask_IsTrackFailsafeStop())
                {
                    Contest_MergedTask_FailStop();
                    return;
                }

                Ball_Control_Update(period_ms);
            }
            break;

        default:
            break;
    }
}

static void Contest_Task2_PathStart(void)
{
    Contest_Task_ResetPhase(CONTEST_TASK2_PHASE_STRAIGHT1);
}

static void Contest_Task2_PathStep(uint16_t period_ms)
{
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
            Contest_Task2_PathStart();
            break;
    }
}

static void Contest_Task2_Update(uint16_t period_ms)
{
    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Contest_Task2_PathStart();
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;

        case CONTEST_STATE_TRACK:
            Contest_Task2_PathStep(period_ms);
            break;

        default:
            break;
    }
}

static void Contest_Task3_SetPhase(uint8_t phase, float target_0p1mm)
{
    ContestTaskContext.task3_phase = phase;
    ContestTaskContext.phase_time_ms = 0;
    Ball_Control_SetTarget(target_0p1mm);
    Ball_Control_Enable(1);
}

static void Contest_Task3_Update(uint16_t period_ms)
{
    Contest_Task_ClearMotion();
    Ball_Control_Update(period_ms);
    ContestTaskContext.phase_time_ms += period_ms;

    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Ball_Control_Reset();
            Contest_Task3_SetPhase(CONTEST_TASK3_PHASE_CENTER, BALL_TARGET_CENTER_0P1MM);
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;

        case CONTEST_STATE_TRACK:
            switch(ContestTaskContext.task3_phase)
            {
                case CONTEST_TASK3_PHASE_CENTER:
                    if((Ball_Control_IsStable() && ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_HOLD_MS) ||
                       ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_TIMEOUT_MS)
                    {
                        Contest_Task3_SetPhase(CONTEST_TASK3_PHASE_POS_5CM, BALL_TARGET_POSITIVE_5CM_0P1MM);
                    }
                    break;

                case CONTEST_TASK3_PHASE_POS_5CM:
                    if((Ball_Control_IsStable() && ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_HOLD_MS) ||
                       ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_TIMEOUT_MS)
                    {
                        Contest_Task3_SetPhase(CONTEST_TASK3_PHASE_NEG_5CM, BALL_TARGET_NEGATIVE_5CM_0P1MM);
                    }
                    break;

                case CONTEST_TASK3_PHASE_NEG_5CM:
                    if((Ball_Control_IsStable() && ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_HOLD_MS) ||
                       ContestTaskContext.phase_time_ms >= CONTEST_TASK3_PHASE_TIMEOUT_MS)
                    {
                        ContestTaskContext.task3_phase = CONTEST_TASK3_PHASE_FINISH;
                        Contest_Task_Finish();
                    }
                    break;

                default:
                    Contest_Task3_SetPhase(CONTEST_TASK3_PHASE_CENTER, BALL_TARGET_CENTER_0P1MM);
                    break;
            }
            break;

        default:
            break;
    }
}

static void Contest_TaskServoDebug_Update(uint16_t period_ms)
{
    Contest_Task_ClearMotion();
    Ball_Control_Update(period_ms);

    switch(ContestTaskContext.state)
    {
        case CONTEST_STATE_START:
            Ball_Control_Reset();
            Ball_Control_SetTarget(BALL_TARGET_CENTER_0P1MM);
            Ball_Control_Enable(1);
            ContestTaskContext.task3_phase = CONTEST_TASK3_PHASE_CENTER;
            ContestTaskContext.phase_time_ms = 0;
            ContestTaskContext.state = CONTEST_STATE_TRACK;
            break;

        case CONTEST_STATE_TRACK:
            ContestTaskContext.phase_time_ms += period_ms;
            Ball_Control_SetTarget(BALL_TARGET_CENTER_0P1MM);
            break;

        default:
            break;
    }
}

static void Contest_Task5_Update(uint16_t period_ms)
{
    Contest_MergedTask_Update(period_ms);
}

void Contest_Task_Init(void)
{
    ContestTaskContext.selected_task = CONTEST_TASK_1;
    ContestTaskContext.running_task = CONTEST_TASK_NONE;
    ContestTaskContext.state = CONTEST_STATE_IDLE;
    ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_IDLE;
    ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AB;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_PRE_BALANCE;
    ContestTaskContext.merged_balance_cm = CONTEST_MERGED_BALANCE_DEFAULT_CM;
    ContestTaskContext.merged_task6_target_cm = CONTEST_MERGED_BALANCE_DEFAULT_CM;
    ContestTaskContext.merged_fail_stop = 0;
    Contest_Task_ResetRuntime();
    OLED_Clear();
}

void Contest_Task_Select(uint8_t task_id)
{
    if(task_id > CONTEST_TASK_MAX)
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
    ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_IDLE;
    ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_PRE_BALANCE;
    ContestTaskContext.merged_fail_stop = 0;

    if(task_id == CONTEST_TASK_5)
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AB;
        ContestTaskContext.merged_balance_cm = 0;
    }
    else if(task_id == CONTEST_TASK_6)
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AA;
        ContestTaskContext.merged_balance_cm = 0;
    }
    else if(task_id == CONTEST_TASK_7)
    {
        ContestTaskContext.merged_track_mode = CONTEST_MERGED_TRACK_MODE_AA;
        ContestTaskContext.merged_balance_cm = ContestTaskContext.merged_task6_target_cm;
    }
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
    if(Contest_Task_IsMergedTask(ContestTaskContext.running_task))
    {
        ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_RUNNING;
    }
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

        if(ContestTaskContext.selected_task == CONTEST_TASK_7 &&
           ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_BALANCE_SELECT)
        {
            ContestTaskContext.merged_task6_target_cm += CONTEST_MERGED_BALANCE_STEP_CM;
            if(ContestTaskContext.merged_task6_target_cm > CONTEST_MERGED_BALANCE_MAX_CM)
            {
                ContestTaskContext.merged_task6_target_cm = CONTEST_MERGED_BALANCE_MIN_CM;
            }
            ContestTaskContext.merged_balance_cm = ContestTaskContext.merged_task6_target_cm;
            return;
        }

        next_task = ContestTaskContext.selected_task + 1;
        if(next_task > CONTEST_TASK_MAX)
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

        if(ContestTaskContext.selected_task == CONTEST_TASK_7)
        {
            if(ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_IDLE ||
               ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_FAIL_STOP)
            {
                ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_BALANCE_SELECT;
                ContestTaskContext.merged_balance_cm = ContestTaskContext.merged_task6_target_cm;
                ContestTaskContext.merged_fail_stop = 0;
                return;
            }
            if(ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_BALANCE_SELECT ||
               ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_READY)
            {
                Contest_Task_StartSelected();
                return;
            }
        }

        Contest_Task_StartSelected();
    }
}

void Contest_Task_Stop(void)
{
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Ball_Control_Enable(0);
    Ball_Control_Reset();
    Ball_Control_StopSafe();
    Contest_Task_ClearMotion();
    ContestTaskContext.running_task = CONTEST_TASK_NONE;
    ContestTaskContext.state = CONTEST_STATE_IDLE;
    ContestTaskContext.state_time_ms = 0;
    ContestTaskContext.phase_time_ms = 0;
    ContestTaskContext.task3_phase = CONTEST_TASK3_PHASE_IDLE;
    if(ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_RUNNING)
    {
        Contest_MergedTask_FreezeMotionTime();
        ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_FINISH;
        ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_READY;
    }
}

static void Contest_Task_Finish(void)
{
    ContestTaskContext.finished_task = ContestTaskContext.running_task;
    ContestTaskContext.finished_time_ms = ContestTaskContext.state_time_ms;
    ContestTaskContext.state = CONTEST_STATE_FINISH;
    if(Contest_Task_IsMergedTask(ContestTaskContext.running_task))
    {
        Contest_MergedTask_FreezeMotionTime();
        ContestTaskContext.finished_time_ms = ContestTaskContext.merged_motion_time_frozen_ms;
        ContestTaskContext.merged_run_phase = CONTEST_MERGED_RUN_FINISH;
        ContestTaskContext.merged_config_state = CONTEST_MERGED_CFG_READY;
    }
    Track_Control_Stop(TRACK_STOP_BY_USER);
    Ball_Control_Enable(0);
    Ball_Control_StopSafe();
    Contest_Task_ClearMotion();
}

static void Contest_Task_DebugPrint(uint16_t period_ms)
{
    static uint16_t debug_time_ms = 0;
    uint16_t mask = TrackIrState.line_mask & 0x01FFu;

    debug_time_ms += period_ms;
    if(debug_time_ms < 100u)
    {
        return;
    }
    debug_time_ms = 0;

    printf("%.2f,%d,%d,%d,%c%c%c%c%c%c%c%c%c\n",
           ContestTaskContext.yaw_deg,
           imu.accel.x,
           imu.accel.y,
           imu.accel.z,
           (mask & (1u << 8)) ? '1' : '0',
           (mask & (1u << 7)) ? '1' : '0',
           (mask & (1u << 6)) ? '1' : '0',
           (mask & (1u << 5)) ? '1' : '0',
           (mask & (1u << 4)) ? '1' : '0',
           (mask & (1u << 3)) ? '1' : '0',
           (mask & (1u << 2)) ? '1' : '0',
           (mask & (1u << 1)) ? '1' : '0',
           (mask & (1u << 0)) ? '1' : '0');
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
            Contest_TaskServoDebug_Update(period_ms);
            break;
        case CONTEST_TASK_3:
            Contest_Task2_Update(period_ms);
            break;
        case CONTEST_TASK_4:
            Contest_Task3_Update(period_ms);
            break;
        case CONTEST_TASK_5:
        case CONTEST_TASK_6:
        case CONTEST_TASK_7:
            Contest_Task5_Update(period_ms);
            break;
        default:
            break;
    }

    Contest_Task_DebugPrint(period_ms);

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

static void Contest_Task_OLEDShowBallDebug(uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"BALL 0+5-5");
    Contest_Task_OLEDShowText6(54, 0, (const u8 *)"TIME");
    Contest_Task_OLEDShowNumber6(84, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(108, 0, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"PH");
    Contest_Task_OLEDShowNumber6(18, 13, ContestTaskContext.task3_phase, 1);
    Contest_Task_OLEDShowText6(36, 13, (const u8 *)"TG");
    Contest_Task_OLEDShowSigned6(54, 13, BallControlState.target_pos_0p1mm, 1, 4);

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"POS");
    Contest_Task_OLEDShowSigned6(24, 26, Ball_Control_GetFilteredPosition(), 1, 4);
    Contest_Task_OLEDShowText6(78, 26, (const u8 *)"0.1mm");

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"ERR");
    Contest_Task_OLEDShowSigned6(24, 39, BallControlState.error_0p1mm, 1, 4);
    Contest_Task_OLEDShowText6(78, 39, (const u8 *)"PWM");
    Contest_Task_OLEDShowNumber6(102, 39, Ball_Control_GetServoPwm(), 4);

    Contest_Task_OLEDShowText6(0, 52, (const u8 *)"S");
    Contest_Task_OLEDShowNumber6(12, 52, Ball_Control_IsStable(), 1);
    Contest_Task_OLEDShowText6(24, 52, (const u8 *)"L");
    Contest_Task_OLEDShowNumber6(36, 52, Ball_Control_IsLost(), 1);
    Contest_Task_OLEDShowText6(54, 52, (const u8 *)"L:STOP");
    OLED_Refresh_Gram();
}

static void Contest_Task_OLEDShowServoDebug(uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"SERVO DBG");
    Contest_Task_OLEDShowText6(66, 0, (const u8 *)"T");
    Contest_Task_OLEDShowNumber6(78, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(102, 0, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"ANG");
    Contest_Task_OLEDShowSigned6(24, 13, BallControlState.output_angle_deg, 10, 4);
    Contest_Task_OLEDShowText6(78, 13, (const u8 *)"PWM");
    Contest_Task_OLEDShowNumber6(102, 13, Ball_Control_GetServoPwm(), 4);

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"ERR");
    Contest_Task_OLEDShowSigned6(24, 26, BallControlState.error_0p1mm, 1, 4);
    Contest_Task_OLEDShowText6(78, 26, (const u8 *)"0.1");

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"POS");
    Contest_Task_OLEDShowSigned6(24, 39, Ball_Control_GetFilteredPosition(), 1, 4);
    Contest_Task_OLEDShowText6(78, 39, (const u8 *)"TG");
    Contest_Task_OLEDShowSigned6(96, 39, BallControlState.target_pos_0p1mm, 1, 3);

    Contest_Task_OLEDShowText6(0, 52, (const u8 *)"S");
    Contest_Task_OLEDShowNumber6(12, 52, Ball_Control_IsStable(), 1);
    Contest_Task_OLEDShowText6(24, 52, (const u8 *)"L");
    Contest_Task_OLEDShowNumber6(36, 52, Ball_Control_IsLost(), 1);
    Contest_Task_OLEDShowText6(54, 52, (const u8 *)"L:STOP");
    OLED_Refresh_Gram();
}

static void Contest_Task_OLEDShowMergedMode(u8 x, u8 y)
{
    if(ContestTaskContext.merged_track_mode == CONTEST_MERGED_TRACK_MODE_AA)
    {
        Contest_Task_OLEDShowText6(x, y, (const u8 *)"AA");
    }
    else
    {
        Contest_Task_OLEDShowText6(x, y, (const u8 *)"AB");
    }
}

static void Contest_Task_OLEDShowMergedCfg(void)
{
    if(ContestTaskContext.selected_task == CONTEST_TASK_5)
    {
        Contest_Task_OLEDShowText6(0, 0, (const u8 *)"T4 AB BAL0");
    }
    else if(ContestTaskContext.selected_task == CONTEST_TASK_6)
    {
        Contest_Task_OLEDShowText6(0, 0, (const u8 *)"T5 AA BAL0");
    }
    else
    {
        Contest_Task_OLEDShowText6(0, 0, (const u8 *)"T6 AA BAL");
    }
    Contest_Task_OLEDShowText6(72, 0, (const u8 *)"T");
    Contest_Task_OLEDShowNumber6(84, 0, ContestTaskContext.selected_task, 1);

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"PATH");
    Contest_Task_OLEDShowMergedMode(36, 13);
    Contest_Task_OLEDShowText6(60, 13, (const u8 *)"BAL");
    Contest_Task_OLEDShowSigned6(84, 13, (float)ContestTaskContext.merged_balance_cm, 1, 2);
    Contest_Task_OLEDShowText6(108, 13, (const u8 *)"cm");

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"SEL");
    Contest_Task_OLEDShowSigned6(30, 26, (float)ContestTaskContext.merged_task6_target_cm, 1, 2);
    Contest_Task_OLEDShowText6(54, 26, (const u8 *)"cm");
    if(ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_BALANCE_SELECT)
    {
        Contest_Task_OLEDShowText6(78, 26, (const u8 *)"<SEL");
    }

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"CFG");
    Contest_Task_OLEDShowNumber6(24, 39, ContestTaskContext.merged_config_state, 1);
    Contest_Task_OLEDShowText6(42, 39, (const u8 *)"FAIL");
    Contest_Task_OLEDShowNumber6(72, 39, ContestTaskContext.merged_fail_stop, 1);

    if(ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_BALANCE_SELECT)
    {
        Contest_Task_OLEDShowText6(0, 52, (const u8 *)"S:BAL  L:RUN");
    }
    else if(ContestTaskContext.selected_task == CONTEST_TASK_7)
    {
        Contest_Task_OLEDShowText6(0, 52, (const u8 *)"L:BAL S:TASK");
    }
    else
    {
        Contest_Task_OLEDShowText6(0, 52, (const u8 *)"L:RUN S:TASK");
    }
    OLED_Refresh_Gram();
}

static void Contest_Task_OLEDShowMergedRun(uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"MERGED RUN");
    Contest_Task_OLEDShowText6(72, 0, (const u8 *)"T");
    Contest_Task_OLEDShowNumber6(84, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(108, 0, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"MODE");
    Contest_Task_OLEDShowMergedMode(36, 13);
    Contest_Task_OLEDShowText6(60, 13, (const u8 *)"BAL");
    Contest_Task_OLEDShowSigned6(84, 13, (float)ContestTaskContext.merged_balance_cm, 1, 2);
    Contest_Task_OLEDShowText6(108, 13, (const u8 *)"cm");

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"ERR");
    Contest_Task_OLEDShowSigned6(24, 26, BallControlState.error_0p1mm, 1, 4);
    Contest_Task_OLEDShowText6(78, 26, (const u8 *)"0.1mm");

    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"STP");
    Contest_Task_OLEDShowNumber6(24, 39, TrackControlState.stop_request, 1);
    Contest_Task_OLEDShowText6(42, 39, (const u8 *)"LOST");
    Contest_Task_OLEDShowNumber6(72, 39, Ball_Control_IsLost(), 1);
    Contest_Task_OLEDShowText6(90, 39, (const u8 *)"F");
    Contest_Task_OLEDShowNumber6(102, 39, ContestTaskContext.merged_fail_stop, 1);

    if((ContestTaskContext.finished_time_ms != 0) &&
       (ContestTaskContext.merged_run_phase == CONTEST_MERGED_RUN_FINISH))
    {
        Contest_Task_OLEDShowText6(0, 52, (const u8 *)"FINISH TIME SAVED");
    }
    else
    {
        Contest_Task_OLEDShowText6(0, 52, (const u8 *)"L:STOP FAILSAFE");
    }
    OLED_Refresh_Gram();
}

static void Contest_Task_OLEDShowTask3Run(uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"TRACK T2");
    Contest_Task_OLEDShowNumber6(24, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(42, 0, (const u8 *)".");
    Contest_Task_OLEDShowNumber6(48, 0, (show_time % 1000) / 100, 1);
    Contest_Task_OLEDShowText6(60, 0, (const u8 *)"PH");
    Contest_Task_OLEDShowNumber6(78, 0, ContestTaskContext.task2_phase, 1);

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"DIS");
    Contest_Task_OLEDShowNumber6(24, 13, (u32)(ContestTaskContext.total_distance_m * 1000.0f), 4);
    Contest_Task_OLEDShowText6(54, 13, (const u8 *)"mm");

    Contest_Task_OLEDShowText6(0, 26, (const u8 *)"YAW");
    Contest_Task_OLEDShowSigned6(24, 26, ContestTaskContext.yaw_deg, 1, 4);
    Contest_Task_OLEDShowText6(60, 26, (const u8 *)"deg");


    Contest_Task_OLEDShowText6(0, 39, (const u8 *)"LD");
    Contest_Task_OLEDShowSigned6(18, 39, MOTOR_D.Encoder, 1000, 4);
    Contest_Task_OLEDShowText6(60, 39, (const u8 *)"RA");
    Contest_Task_OLEDShowSigned6(78, 39, MOTOR_A.Encoder, 1000, 4);

    // Contest_Task_OLEDShowText6(0, 39, (const u8 *)"MASK");
    // Contest_Task_OLEDShowNumber6(30, 39, TrackIrState.line_mask & 0x01FFu, 3);
    // Contest_Task_OLEDShowText6(60, 39, (const u8 *)"RAW");
    // Contest_Task_OLEDShowNumber6(84, 39, TrackIrState.raw_mask & 0x01FFu, 3);

    Contest_Task_OLEDShowText6(0, 52, (const u8 *)"L:STOP  S:SEL");
    OLED_Refresh_Gram();
}

static void Contest_Task_OLEDShowMotorDebug(uint8_t task_id, uint32_t show_time)
{
    Contest_Task_OLEDShowText6(0, 0, (const u8 *)"MDBG T");
    Contest_Task_OLEDShowNumber6(36, 0, task_id, 1);
    Contest_Task_OLEDShowText6(54, 0, (const u8 *)"TIME");
    Contest_Task_OLEDShowNumber6(84, 0, show_time / 1000, 3);
    Contest_Task_OLEDShowText6(108, 0, (const u8 *)"s");

    Contest_Task_OLEDShowText6(0, 13, (const u8 *)"LD");
    Contest_Task_OLEDShowSigned6(18, 13, MOTOR_D.Encoder, 1000, 4);
    Contest_Task_OLEDShowText6(60, 13, (const u8 *)"RA");
    Contest_Task_OLEDShowSigned6(78, 13, MOTOR_A.Encoder, 1000, 4);

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
        show_time = Contest_Task_IsMergedTask(task_id) ? ContestTaskContext.merged_motion_time_ms : ContestTaskContext.state_time_ms;
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

    if(task_id == CONTEST_TASK_2)
    {
        Contest_Task_OLEDShowServoDebug(show_time);
        return;
    }

    if(task_id == CONTEST_TASK_3)
    {
        Contest_Task_OLEDShowTask3Run(show_time);
        return;
    }

    if(task_id == CONTEST_TASK_4)
    {
        Contest_Task_OLEDShowBallDebug(show_time);
        return;
    }

    if(Contest_Task_IsMergedTask(task_id))
    {
        if(Contest_Task_IsRunning() ||
           ContestTaskContext.finished_time_ms != 0 ||
           ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_RUNNING ||
           ContestTaskContext.merged_config_state == CONTEST_MERGED_CFG_FAIL_STOP)
        {
            Contest_Task_OLEDShowMergedRun(show_time);
        }
        else
        {
            Contest_Task_OLEDShowMergedCfg();
        }
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
