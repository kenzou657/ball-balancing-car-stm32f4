# Project Documentation Rules (Non-Obvious Only)

- `DOC/AGENT任务.md` is authoritative for agent constraints: do not change `USER/WHEELTEC.uvprojx`, keep additions practical, and avoid unnecessary functions.
- `DOC/项目说明文档.md` contains the current architecture map, but re-check code for details that changed after the GB18030-to-UTF-8 source conversion.
- `DOC/舵机控制方案_v0.1.md` describes the current competition direction: MG996R servo, one-dimensional ball position feedback, and IMU disturbance feed-forward rather than a stepper/angle inner loop.
- `BALANCE/control.c` and `BALANCE/balance.c` coexist with overlapping names; check the Keil project membership before assuming a source file is built.
