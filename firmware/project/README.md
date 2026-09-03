# firmware/project

CubeMX + Keil project build guide.

Generate a new project instead of committing a third-party/generated tree:

1. STM32CubeMX: chip `STM32F405RGTx`; HSE crystal; SYSCLK 168 MHz; Serial Wire debug.
2. Peripheral plan by development stage (see `docs/04_开发路线与验收.md`):
   - S1: USART1 debug 115200;
   - S2: I2C1 for ICM-20948;
   - S3: TIM8 for DShot300 on PC6-PC9;
   - S5: UART5 for the upward laser;
   - S6: UART2/UART3 for radar/optical flow (one or both branches);
   - PS2 receiver: USB Host HID.
3. Add FreeRTOS (CMSIS-RTOS v2) and create tasks per `docs/02_软件架构.md`.
4. Add `../app/src/*.c` and `../app/include/` to the Keil project.
5. Keep only `.ioc`, handwritten sources and build settings in git; do not commit generated `Drivers/` from CubeMX if it can be regenerated.

The app modules are HAL-independent and call no driver yet; hardware read/write
functions are intentionally left to the generated project so the mapping stays
in the board-specific layer.
