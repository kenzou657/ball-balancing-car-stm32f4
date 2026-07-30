# Project Coding Rules (Non-Obvious Only)

- Do not edit `USER/WHEELTEC.uvprojx`; new `.c` files are invisible to Keil until the user manually adds them, so prefer modifying existing project-included files.
- Prefer `BALANCE/` for control/state-machine logic and `HARDWARE/` for peripheral drivers; avoid changes in `FWLIB/`, `CORE/`, `FreeRTOS/`, and generated `OBJ/` artifacts.
- Include `system.h` when following existing module style; it intentionally aggregates FreeRTOS, STM32, hardware, and `BALANCE/system.c` globals.
- Source files are now UTF-8 after GB18030/ANSI conversion; avoid project-wide encoding conversion or whole-file reformatting when making code changes.
- TIM8 CH4/PC9 is already reserved for servo PWM (`Servo_PWM` = `TIM8->CCR4`), initialized at 100 Hz; motor PWM uses TIM1/TIM9/TIM10/TIM11 at ~10 kHz.
- For periodic FreeRTOS code, use existing `M2T()`/`F2T()` helpers from `FreeRTOS/include/FreeRTOSConfig.h` rather than inventing another ms/tick conversion.
- Keep functions minimal and competition-focused per `DOC/AGENT任务.md`; avoid adding broad framework layers or unused utilities.
