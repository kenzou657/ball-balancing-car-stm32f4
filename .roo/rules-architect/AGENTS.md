# Project Architecture Rules (Non-Obvious Only)

- Startup flow is `USER/main.c` -> `systemInit()` -> FreeRTOS `start_task`; application tasks are created only after hardware initialization succeeds.
- Hardware version is selected dynamically by IMU ID: MPU6050 means `V1_0` with legacy PS2/LED init, ICM20948 means `V1_1` with USB Host/new LED init, otherwise the MCU resets.
- `BALANCE/system.c` holds shared runtime state/PID/model globals exported through `system.h`; modules are coupled through these globals rather than dependency injection.
- Car-model configuration is potentiometer-driven through `Robot_Select()` during boot; `CAR_NUMBER` is 8 despite comments that still mention 6 models.
- Timer allocation is already constrained: TIM8 CH4/PC9 for servo, TIM1/TIM9/TIM10/TIM11 for motor PWM, TIM2/TIM3/TIM4/TIM5 for encoders.
- The repository is MDK-project authoritative, not folder-authoritative: a source file existing on disk is not built unless listed in `USER/WHEELTEC.uvprojx`.
