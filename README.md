# 球平衡小车项目

本工程基于 STM32F4 + FreeRTOS + Keil MDK。
## 1. 当前工程分层

- `USER/`：启动入口与 Keil 工程配置。启动链路为 `main()` -> `systemInit()` -> `start_task()` -> 周期任务。
- `BALANCE/`：比赛控制逻辑，包含任务状态机、循迹控制、滚球控制、电机闭环等。
- `HARDWARE/`：外设驱动，包含电机 PWM、编码器、9 路红外、USART 摄像头、OLED 等。
- `DOC/`：方案文档与任务拆分说明。

核心周期仍复用 `Balance_task()` 的 100Hz 控制节拍，优先不新增 FreeRTOS 任务，循迹、滚球、计时和安全输出统一在 10ms 周期内调度。

## 2. 算法架构

### 2.1 总体调度

```text
Balance_task 100Hz
    -> 编码器速度换算
    -> Contest_Task_Update(10ms)
        -> 按当前任务运行状态机
        -> Track_Control_Update()
        -> Ball_Control_Update()
        -> 生成左右轮目标速度 / 舵机 PWM
    -> 电机速度 PI
    -> Set_Pwm()
```

任务选择与执行集中在 `BALANCE/contest_task.c` / `BALANCE/contest_task.h`：

- 任务 1：电机调试，验证左右轮方向、编码器和速度 PI。
- 任务 2：舵机调试，验证 PC9 / TIM8 CH4 输出、舵机中位和方向。
- 任务 3：赛题任务 2，整圈循迹回 A，主要验证差速循迹和停车判定。
- 任务 4：赛题任务 3，静态滚球稳定，中心 -> +5cm -> -5cm。
- 任务 5：赛题任务 4/5/6 合并入口，选择 AB/AA 循迹模式和滚球平衡点。

### 2.2 循迹控制

模块：`HARDWARE/track_ir.c` / `HARDWARE/track_ir.h`、`BALANCE/track_control.c` / `BALANCE/track_control.h`。

数据流：

```text
9 路红外 GPIO
    -> 9bit line_mask
    -> 加权平均 line_error（左负右正）
    -> 循迹外环 PD
    -> 左右轮目标速度差
    -> 电机速度内环 PI
    -> PWM 输出
```

红外权重：IR0 到 IR8 对应 `-4, -3, -2, -1, 0, +1, +2, +3, +4`。

直行段可叠加 ICM20948 yaw 角度环，弯道、宽线、丢线时主要由红外外环和状态机处理。停车采用“时间窗口 + 宽线多路触发 + 强制停车时间”策略，避免刚启动误判 A 点。

### 2.3 滚球稳定

模块：`BALANCE/ball_control.c` / `BALANCE/ball_control.h`，摄像头入口在 `HARDWARE/usartx.c` / `HARDWARE/usartx.h`。

数据流：

```text
USART1 摄像头 5 字节帧
    -> 球位置 raw_pos，单位 0.1mm
    -> 一阶低通 filtered_pos
    -> 位置 PID 输出目标倾角
    -> ICM20948 加速度前馈
    -> 死区 / 限幅 / 变化率限制
    -> MG996R 舵机 PWM，PC9 / TIM8 CH4
```

摄像头协议：`AA CMD DATA_H DATA_L BB`，`CMD=0x01` 为位置，`0x02` 为丢球，`0x03` 为心跳。位置单位为 0.1mm，右侧为正、左侧为负。

### 2.4 循迹 + 滚球联动

OLED 任务 5 复用一个通用框架，对应赛题任务 4/5/6：

- 先选择循迹模式：AB 或 AA。
- 再选择滚球平衡点：-12cm 到 +12cm，内部换算为 0.1mm。
- 执行时每 10ms 同时更新循迹和滚球。
- 球误差变大时动态降低基础车速；丢球或长时间丢线直接保护停车。

## 3. 重点调试参数

### 3.1 红外与循迹

| 参数 | 位置 | 作用 | 调试建议 |
|---|---|---|---|
| `TRACK_BLACK_LEVEL` | `HARDWARE/track_ir.h` | 黑线有效电平 | 第一项确认，错了会导致 mask 全反 |
| IR0~IR8 引脚顺序 | `HARDWARE/track_ir.h` | 传感器左右顺序 | 手压黑线看 bit 顺序，必须左到右正确 |
| `TRACK_TURN_DIR` | `BALANCE/track_control.h` | 转向方向 | 车越修越偏时先反这个 |
| `line_pid.kp/kd/ki` | `BALANCE/track_control.c` 或接口设置 | 循迹外环 | 初期只用 PD，`ki=0` |
| `TRACK_WIDE_COUNT_TH` | `HARDWARE/track_ir.h` | 宽线/A/B 点候选 | 按赛道横线宽度调 |
| 停车时间窗口 | `BALANCE/contest_task.c` / `BALANCE/track_control.c` | A/B 点停车 | 先记录典型到达时间，再设窗口和强停 |
| `TRACK_LOST_STOP_DEFAULT` | `BALANCE/track_control.h` | 丢线保护周期数 | 100Hz 下 30 约 0.3s |

### 3.2 电机速度内环

| 参数 | 位置 | 作用 | 调试建议 |
|---|---|---|---|
| `Velocity_KP` / `Velocity_KI` | `BALANCE/system.c` 等全局参数 | 左右轮速度 PI | 先用任务 1 固定速度调通 |
| 编码器方向 | `HARDWARE/encoder.c` / 电机接线 | 速度反馈符号 | 目标正速度时反馈也应为正 |
| 电机 PWM 方向 | `BALANCE/balance.c` / `HARDWARE/motor.*` | 前进方向 | 左右轮方向不一致先修方向，不急着调 PID |
| 机械参数 | `BALANCE/robot_select_init.c` | 速度换算 | 当前差速底盘按 13 线、28 减速比、65mm 轮径 |

### 3.3 滚球控制

| 参数 | 位置 | 作用 | 初值/建议 |
|---|---|---|---|
| `alpha` | `BallControlParam` | 视觉低通 | 初值约 0.25，噪声大则减小 |
| `kp` | `BallControlParam` | 位置比例 | 初值约 0.010，先只开 P |
| `kd` | `BallControlParam` | 抑制过冲 | 有过冲后再逐步加 |
| `ki` | `BallControlParam` | 消除静差 | 初期 0，最后再加很小值 |
| `kf` | `BallControlParam` | IMU 加速度前馈 | OLED 任务 4 静态调试先 0，移动联调再开 |
| `servo_mid_us` | `BallControlParam` | 舵机机械中位 | 必须实测调到轨道水平 |
| `servo_dir` | `BallControlParam` | 舵机方向 | 球越控越远优先反这个 |
| `servo_min_us/max_us` | `BallControlParam` | PWM 机械限幅 | 初期 1000~2000us，确认不撞限再扩大 |
| `rate_limit_deg` | `BallControlParam` | 舵机变化率限制 | 抖动大可降低 |
| `stable_error_0p1mm` | `BallControlParam` | 稳定误差阈值 | 100 表示 1cm |
| `vision_timeout_ms` | `BallControlParam` | 摄像头超时 | 初值约 100~200ms |

## 4. 建议调试顺序

1. OLED 任务 1：固定左右轮目标速度，确认电机方向、编码器方向、速度 PI。
2. OLED 任务 2：舵机单独调试，确认 PC9 输出、机械中位、方向和安全限幅。
3. 红外输入：显示 `line_mask`、`active_count`、`line_error`，确认黑线电平和 IR 顺序。
4. OLED 任务 3：低速整圈循迹，先只调循迹 PD，不开复杂停车；方向错先改 `TRACK_TURN_DIR`。
5. 停车判定：记录 A/B 到达时间，调宽线阈值、停车窗口和强制停车时间。
6. 摄像头协议：确认心跳、位置正负、丢球和超时保护。
7. OLED 任务 4：静态滚球，中心 -> +5cm -> -5cm；先 P，再 D，最后小 I，前馈 `kf` 保持 0。
8. OLED 任务 5：先 AB + 0cm，低速联调，优先保球不追速度。
9. OLED 任务 5：再 AA + 0cm，逐步提高整圈速度。
10. OLED 任务 5：最后 AA + 非零平衡点，重点看目标换算和保持效果。

## 5. 已知注意事项

- 新增 `.c` 文件需要手动加入 Keil 工程，不能只放在目录里。
- 不修改 `USER/WHEELTEC.uvprojx`，避免破坏 Keil 工程配置。
- `TIM8 CH4 / PC9` 已用于舵机 PWM；电机 PWM 是 TIM1/TIM9/TIM10/TIM11。
- 滚球模块接管舵机时，要避免 `Set_Pwm()` 传入的舵机参数覆盖滚球输出。
- OLED 任务 5 对应赛题任务 4/5/6，不建议写三套重复状态机，当前思路是“循迹模式 + 平衡点”的合并框架。
- 现场调参先看 OLED/串口显示的原始量，不要直接盲调 PID。
