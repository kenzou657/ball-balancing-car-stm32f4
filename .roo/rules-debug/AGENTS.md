# Project Debug Rules (Non-Obvious Only)

- Debug/build configuration is Keil/J-Link centered: use `USER/WHEELTEC.uvprojx`, target `FreeRTOS`, and debug configs under `USER/DebugConfig/`.
- Unknown IMU ID in `systemInit()` triggers immediate `NVIC_SystemReset()`, so repeated resets at boot often mean MPU6050/ICM20948 detection or I2C wiring failed.
- FreeRTOS `configASSERT()` prints through `printf("Error:%s,%d\r\n", ...)`; stack-overflow and malloc-failed hooks are disabled in `FreeRTOS/include/FreeRTOSConfig.h`.
- `keilkilll.bat` recursively deletes build artifacts including `.axf`, `.map`, `.sct`, `.o`, `.d`; run it only when a full clean is intended.
- Source comments are expected to be UTF-8 after conversion; if isolated legacy mojibake remains, treat it as comment text and do not perform broad encoding rewrites during debugging.
