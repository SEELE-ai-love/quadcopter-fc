# Added compiler-specific upstream files

- `startup_stm32f405xx.s`: STM32Cube_FW_F4_V1.28.3 local package, CMSIS Device F4 GCC template. Original copyright header retained; see LICENSE.ST.txt.
- `../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c` and `portmacro.h`: FreeRTOS-Kernel V10.3.1-kernel-only, commit `88e32327e975ddde97c390bc5b6c1f8e7d9d239e`. Original MIT license headers retained. Existing kernel files were not upgraded.
  https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/88e32327e975ddde97c390bc5b6c1f8e7d9d239e/portable/GCC/ARM_CM4F

`syscalls.c` and `STM32F405RG_APP.ld` are project-specific GCC integration files.
