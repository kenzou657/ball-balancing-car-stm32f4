# AGENTS.md

This file provides guidance to agents when working with code in this repository.

- Build with Keil MDK-ARM 5.06u7 from `USER/WHEELTEC.uvprojx`; target is `FreeRTOS`, output is `OBJ/WHEELTEC.axf`, hex generation is enabled, and the configured post-build `fromelf --bin --output=..\wheeltec.bin ..\OBJ\WHEELTEC.axf` is present but disabled in the project.
- There is no discovered unit-test/lint setup; “single test” means build only the touched Keil target/file group in MDK, not a repository CLI test command.
- Do not modify `USER/WHEELTEC.uvprojx`; `DOC/AGENT任务.md` explicitly forbids it, so adding new source files also requires manual Keil project inclusion by the user.
- `keilkilll.bat` is a destructive artifact cleanup script run from repo root; it deletes `.o/.d/.axf/.map/.sct` recursively but intentionally does not delete Keil `.opt`/JLink settings.
- Source comments have been converted from GB18030/ANSI to UTF-8; do not run bulk encoding conversions again, and only edit localized comment text when needed for the touched code.
- New application logic should usually enter through `BALANCE/` or `HARDWARE/`; `FWLIB/`, `CORE/`, `FreeRTOS/`, and `OBJ/` are vendor/kernel/build-output areas.
- `system.h` is the project-wide include aggregator and exposes globals from `BALANCE/system.c`; existing modules commonly include only `system.h` instead of fine-grained headers.
- Runtime startup is `USER/main.c` -> `systemInit()` -> FreeRTOS `start_task`; business tasks are created only inside `start_task`.
- `systemInit()` auto-detects IMU: MPU6050 selects hardware `V1_0` plus legacy PS2/LED init, ICM20948 selects `V1_1` plus USB Host/new LED; unknown IMU causes `NVIC_SystemReset()`.
- `CAR_NUMBER` is 8 although comments still say 6 car types; do not trust older comments around car-model selection without checking code.
- TIM8 CH4 on PC9 is already the servo PWM (`Servo_PWM`/`TIM8->CCR4`) initialized at 100 Hz via `TIM8_SERVO_Init(9999,168-1)`; motor PWM timers run at ~10 kHz.
- Existing FreeRTOS tick is 1 kHz and helper macros `M2T()`/`F2T()` are defined in `FreeRTOS/include/FreeRTOSConfig.h`; prefer them for periodic task delays in this codebase.
- Keep competition pragmatism from `DOC/AGENT任务.md`: implement only necessary drivers/state-machine logic, avoid over-engineered commercial-style abstractions.
