# F405：VS Code + CMake + GCC 使用说明

现有 Keil 工程与 CMake 工程共用同一份 Core、F405_BSP、F405_APP、USB_HOST 和驱动源码。
原来的 `MDK-ARM/F405-_FlightControl.uvprojx`、ARM 启动文件和根目录 `F405.bin` 保留。
GCC 产物放在 `build/gcc-Debug` 或 `build/gcc-Release`，不会覆盖原 BIN。

## 每天如何使用

1. 在 VS Code 选择“打开文件夹”，打开本文件所在的工程根目录，不要只打开 MDK-ARM 子目录。
2. 正常修改 `.c/.h` 并保存。当前源码包含 GBK 和混合编码，保留原编码，不要批量转码或自动格式化。
3. 按 **Ctrl+Shift+B** 执行默认任务 `GCC: Build Debug`。它会配置 CMake，再执行增量构建。
4. 成功后，将 `build/gcc-Debug/F405.bin` 交给原有商家烧录上位机。
5. 首次更换编译器后的固件需要重新做硬件验收。构建成功不代表飞行行为已验证。

也可以在工程根目录的 PowerShell 中执行：

```powershell
.\tools\build.ps1 -Configuration Debug
.\tools\build.ps1 -Configuration Release
```

若本机 PowerShell 执行策略禁止脚本，可执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Debug
```

该参数只用于本次脚本进程，不修改系统执行策略。

脚本从 `CUBE_BUNDLE_PATH` 的 STM32Cube bundle 或 PATH 查找 CMake、Ninja、GNU Arm GCC。
也支持用 `ARM_GCC_ROOT` 指定 GCC 安装根目录。不需要重新下载独立 GCC。

## 在 CMake Tools 中配置

本机已生成且忽略提交的 `CMakeUserPresets.json`：

- 运行命令面板 `CMake: Select Configure Preset`，选择 **Local-Debug**。
- 运行 `CMake: Configure`，随后 `CMake: Build`。
- `Local-Release` 为尺寸优化版本。
- 根目录 `CMakePresets.json` 中的 Debug/Release 是可公开的通用预设。

新电脑需要准备 GNU Arm GCC、CMake >= 3.21、Ninja。将工具 bin 加入 PATH 后：

```text
cmake --preset Debug
cmake --build --preset Debug
```

`tools/build.ps1` 和 VS Code 默认任务适用于 Windows。CMake 配置本身不写死 D 盘路径。
Linux/macOS 可以用上面的命令，但本次只在 Windows 做了真实构建，其他系统尚未实测。

## 文件在哪里、何时需要修改

| 文件 | 用途 |
|---|---|
| `CMakeLists.txt` | 芯片参数、编译选项、链接选项和 BIN/HEX 生成 |
| `cmake/sources.cmake` | 明确列出参与构建的 C 文件和 include 目录 |
| `cmake/arm-none-eabi-gcc.cmake` | 定位 Arm 交叉编译工具链 |
| `GCC/startup_stm32f405xx.s` | GCC 启动文件，不替换 Keil 启动文件 |
| `GCC/STM32F405RG_APP.ld` | 应用 Flash 地址、SRAM、C 堆和 MSP 栈布局 |
| `GCC/syscalls.c` | newlib 的 UART、堆等底层接口，继续调用原 main.c 中的 fputc |
| `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F` | 与现有 V10.3.1 内核匹配的 GCC 移植层 |

修改已有函数后，直接重新构建即可。
新增 `.c` 文件时，将其路径加入 `cmake/sources.cmake` 的 `F405_SOURCES`。
新增头文件目录时，将目录加入 `F405_INCLUDE_DIRS`。普通 `.h` 文件无需逐个登记。
若还要让原 Keil 工程编译新增 `.c`，也需把该文件加入 Keil 项目；两份文件清单不会自动同步。

当前 `.ioc` 保持原 Keil 输出配置。以后 CubeMX 新增外设文件后，核对并补充 CMake 源码清单。
不要为了使用 CMake 直接重新生成并覆盖整套已修改的飞控工程。

## 和原来 C 代码的区别

业务层仍是 C99，HAL、FreeRTOS API、任务函数、PD/PID 公式不因为使用 VS Code 而改变。
编译器从 ARMCC5 换为 GCC 14.3.1，因此汇编语法、运行库、警告、优化及最终机器码会不同。
不能把 ARMCC 的目标文件直接混入 GCC 链接，也不能用旧 AXF 的符号解释新的 GCC BIN。

本次原有 C 文件只做了一项兼容改动：

```c
// 原先使用 const 变量作文件作用域数组长度，ARMCC 接受，标准 C/GCC 拒绝。
static uint8_t stp23L_buf[ sizeof(stp23L_frame) ] ;
```

该数组长度、原注释和运行逻辑保持原意。其余原有 C/H 文件和 Keil 工程文件保持迁移前字节内容。
头文件有三种大小写历史拼写，CMake 在 build 目录生成转发头处理，未修改共享源码。

## 地址与运行库：必须知道的区别

- 芯片：STM32F405RG，Cortex-M4F，Flash 1 MiB，普通 SRAM 128 KiB。
- 指令/FPU：`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`。
- 保留现有 Bootloader 空间：应用从 **0x08010000** 开始，VTOR 偏移 **0x10000**。
- 剩余应用 Flash 为 **960 KiB**。原 Keil scatter 从 0x08010000 仍填写 1 MiB，会越过芯片尾端；原 scatter 未改，GCC 使用实际剩余容量。
- 普通数据、RTOS 堆和 DMA 缓冲区使用普通 SRAM，不自动放入 CCM。
- GCC/newlib C 堆预留 8 KiB，MSP/中断栈预留 4 KiB。RTOS 的 `configTOTAL_HEAP_SIZE=51200` 和任务栈配置保持原样。
- MSP 栈显式放在静态数据和 C 堆之后，初始 SP 位于 SRAM 地址范围内；链接器检查总占用。
- 保留 `%f` 格式化支持。GCC 的 `_write` 继续转发到现有 `fputc`，日志仍走原调试串口。
- Debug 为 `-O0`、DWARF 3 调试信息；Release 为 `-Os`，不生成项目源码调试信息。
- GCC 14 预编译运行库自带 DWARF 5。链接脚本剔除这些库的调试节，保留项目源码的 DWARF 3，避免 Keil 遇到混合版本；因此不支持在 Keil 中源码单步进入这些预编译标准库。运行库代码本身保留。
- 编译器生成的代码大小和执行时间不同，Debug/Release 都要由人分别验收。

上位机没有从 ELF 读取地址的能力时，它必须仍按原有应用地址写入这个 BIN。
本次没有操作上位机、连接硬件、烧录或擦除 Bootloader。

## 使用 Keil 做 SWD 调试 GCC 固件

调试适配器如 ST-Link/J-Link 和 SWD 接线仍需实际具备；商家有线烧录工具不自动等于 SWD 调试器。

**应调试 `build/gcc-Debug/F405.elf` 对应的 GCC 固件，而不是重新编译旧 Keil 项目后调试旧 AXF。**

建议复制一份 Keil 调试配置供 GCC 使用，原 `.uvprojx` 本次未修改：

1. 先由你使用商家上位机烧录同一轮生成的 GCC Debug BIN。
2. 进入 Keil 的 Debug 配置，选择实际 SWD 调试适配器。
3. 关闭 `Load Application at Startup`，避免调试器自动装入旧 AXF。
4. 关闭 `Run to main()`，并确认 Utilities 中的 `Update Target before Debugging` 自动下载也关闭，避免旧映像覆盖当前 GCC 固件。
5. 在 Debug Initialization File 中选择 `MDK-ARM/GCC_Symbols_Only.ini`。
6. 该文件只执行 `LOAD "../build/gcc-Debug/F405.elf" NOCODE`，装载符号不下载程序。
7. 如果 Keil 无法按相对路径定位 ELF，改用当前 ELF 的完整路径；确认这只是初始化文件路径设置。
8. 启动调试后由人确认复位/停机方式、源码断点、调用栈与变量是否正常；这一步本次没有实机验证。

调试中修改并重建后，必须重新烧录配套 BIN 并重新载入配套 ELF。不能只换符号而不更新固件。
Debug 使用 DWARF 3 是为兼容 Keil；不同 Keil/探针版本仍需现场验证。
参考：
- https://www.keil.com/support/man/docs/uv4/uv4_cm_load.asp
- https://www.keil.com/support/docs/3902.htm

## 当前已知限制

本次完成的是构建迁移，未改变现有传感器映射、光流补偿、PD 参数或闭环启用标志。
现有代码仍有未使用变量/函数、ARMCC pragma 等警告，未为消除告警删除原注释和调试代码。
`show_task.c` 现有 `showbuffer` 的格式化已被注释，但后面仍调用 `strlen(showbuffer)`，存在未初始化缓冲区使用风险；本次按保留现有业务代码的范围没有顺带修复。
软件定时器回调中的阻塞延时、传感器与飞行行为等原有待验收项也不会因为构建通过而自动解决。

发布到 GitHub 时提交源码、CMake 配置、GCC 启动/运行库文件及本说明；`build/`、BIN/ELF、编译数据库、本机 `CMakeUserPresets.json` 已忽略。
按原有 ST 和 FreeRTOS 许可证保留第三方文件版权信息；迁移不替项目中其他代码补造许可证。
