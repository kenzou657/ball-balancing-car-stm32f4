#ifndef __CONTEST_TASK_H
#define __CONTEST_TASK_H

#include "sys.h"

#define CONTEST_TASK_NONE             0
#define CONTEST_TASK_1                1   /* 电机调试 */
#define CONTEST_TASK_2                2   /* 舵机调试 */
#define CONTEST_TASK_3                3   /* 赛题任务2：整圈循迹 */
#define CONTEST_TASK_4                4   /* 0->+5->-5 滚球稳定 */
#define CONTEST_TASK_5                5   /* 赛题任务4：A->B 循迹 + 0刻度滚球保持 */
#define CONTEST_TASK_6                6   /* 赛题任务5：A->A 循迹 + 0刻度滚球保持 */
#define CONTEST_TASK_7                7   /* 赛题任务6：A->A 循迹 + 任意整数刻度滚球保持 */
#define CONTEST_TASK_MAX              CONTEST_TASK_7

#define CONTEST_TRACK_PERIOD_MS       10

#define CONTEST_MERGED_TRACK_MODE_AB  0
#define CONTEST_MERGED_TRACK_MODE_AA  1

#define CONTEST_MERGED_BALANCE_MIN_CM      (-12)
#define CONTEST_MERGED_BALANCE_MAX_CM      12
#define CONTEST_MERGED_BALANCE_DEFAULT_CM  0
#define CONTEST_MERGED_BALANCE_STEP_CM     1

#define CONTEST_MERGED_CFG_IDLE            0
#define CONTEST_MERGED_CFG_MODE_SELECT     1
#define CONTEST_MERGED_CFG_BALANCE_SELECT  2
#define CONTEST_MERGED_CFG_READY           3
#define CONTEST_MERGED_CFG_RUNNING         4
#define CONTEST_MERGED_CFG_FAIL_STOP       5

#define CONTEST_MERGED_RUN_PRE_BALANCE     0
#define CONTEST_MERGED_RUN_MOTION          1
#define CONTEST_MERGED_RUN_FINISH          2
#define CONTEST_MERGED_RUN_FAIL            3

#define CONTEST_TASK3_PHASE_IDLE      0
#define CONTEST_TASK3_PHASE_CENTER    1
#define CONTEST_TASK3_PHASE_POS_5CM   2
#define CONTEST_TASK3_PHASE_NEG_5CM   3
#define CONTEST_TASK3_PHASE_FINISH    4

typedef enum
{
    CONTEST_STATE_IDLE = 0,
    CONTEST_STATE_START,
    CONTEST_STATE_TRACK,
    CONTEST_STATE_FINISH,
    CONTEST_STATE_STOP
} ContestTaskState_t;

typedef struct
{
    uint8_t selected_task;
    uint8_t running_task;
    uint8_t state;
    uint8_t task2_phase;
    uint8_t task3_phase;
    uint8_t merged_config_state;
    uint8_t merged_track_mode;
    uint8_t merged_run_phase;
    uint8_t merged_fail_stop;
    int8_t merged_balance_cm;
    int8_t merged_task6_target_cm;
    uint8_t finished_task;
    uint32_t state_time_ms;
    uint32_t phase_time_ms;
    uint32_t merged_pre_balance_ms;
    uint32_t merged_stable_hold_ms;
    uint32_t merged_motion_time_ms;
    uint32_t merged_motion_time_frozen_ms;
    uint32_t finished_time_ms;
    float phase_distance_m;
    float total_distance_m;
    float yaw_deg;
    float phase_start_yaw_deg;
} ContestTaskContext_t;

extern ContestTaskContext_t ContestTaskContext;

void Contest_Task_Init(void);
void Contest_Task_Select(uint8_t task_id);
void Contest_Task_StartSelected(void);
void Contest_Task_Stop(void);
void Contest_Task_KeyAction(uint8_t key_action);
void Contest_Task_Update(uint16_t period_ms);
void Contest_Task_OLEDShow(void);
uint8_t Contest_Task_IsRunning(void);
uint8_t Contest_Task_GetSelected(void);

#endif
