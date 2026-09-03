# firmware/README

## STM32 firmware source modules

The `app/` directory contains self-written, HAL-independent application modules:

| Module | Purpose |
| --- | --- |
| `fc_types.h` | common types and flight states |
| `fc_pid.c/.h` | generic PID with anti-windup |
| `fc_attitude_estimator.c/.h` | basic complementary filter (Euler form, Stage S2) |
| `fc_mixer.c/.h` | Quad-X control allocation |
| `fc_safety.c/.h` | arm/disarm and failure state machine |

The CubeMX-generated HAL/FreeRTOS project is expected under `project/`. Keep the generated driver layer out of git, or track only the `.ioc` and handwritten sources.

## Integration notes

1. Configure CubeMX first: 168 MHz, SWD, USART1 debug, I2C1 IMU, TIM8 DShot, UARTs for laser/optical sensors, USB host for PS2.
2. Let CubeMX generate the FreeRTOS tasks, then call the app modules from the task bodies.
3. Always keep the state machine as the single owner of motor enablement: modules may compute outputs, but only `fc_safety` decides whether they reach the DShot driver.

See `docs/04_开发路线与验收.md` for the order in which each module should be completed and validated.
