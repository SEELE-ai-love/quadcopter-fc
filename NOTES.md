\---



###### \## 2026-09-18和2026-09-19  USB手柄（还未移植进入control\_task控制层）



##### //事先声明：本人驱动代码并非自己完成，而是借助claude和手柄商家的建议，对于开发者而言，USB协议旨在改动应用层的代码，使按键和轮盘映射进控制层从而实现不同的事件，USB协议内容冗杂，且不同的厂商手柄代码均不相同，但是这个USB文件依旧写入接口函数和驱动挂载



\### 硬件与配置

\- （cubemx） USB\_OTG\_FS，Host Only + VBUS使能

\- Middleware：USB\_HOST，Class选HID

\- 手柄：无线手柄（索尼PS） + USB接收器（接收器插飞控USB口）



\### 底层文件

\- USB\_HOST/App/usb\_host.c/h

\- Middlewares/ST/STM32\_USB\_Host\_Library/

\- stm32f4xx\_hal\_hcd.c/h

\- stm32f4xx\_ll\_usb.c/h



\### 根因踩坑

\- 系统完全无输出的根因：USB底层驱动版本不匹配，不是FreeRTOS配置问题

\- 之前误判成堆空间、优先级、栈溢出，浪费足足两小时

\- Codex覆盖USB底层驱动文件后系统恢复

\- Error\_Handler里不能写printf：fputc里while死等，UART未初始化时死循环，删掉后系统反而正常



\### 手柄数据接口

\- ps2\_info.LX / LY / RX / RY：摇杆，0\~255，中位约127

\- ps2\_info.getKeyState(bit)：电平查询，适合扳机键

\- ps2\_info.getKeyEvent(bit)：边沿事件，适合普通键

\- 拔出手柄时驱动自动复位ps2\_info到中位，按键全松



\### 按键语义（有意区分）

\- 扳机键（L1/L2/R1/R2）：电平查询，按住就一直响应，适合油门微调

\- 普通键（四色键、SELECT、START）：边沿事件，按一下触发一次，适合模式切换



\### 回调机制

\- USBH\_HID\_EventCallback 每收到一帧USB上报就调用一次

\- 频率约等于手柄poll interval（几毫秒一次）

\- 按一下扳机打印12行是正常：按住12帧，每帧都触发



\### 当前进度

\- 已完成：USB手柄数据读取，摇杆/按键都能实时读到

\- 下一步：

开始无人机专属的PID串级控制和使用Github推荐的无人机开源软件SerialPlot进行PID/LQR的曲线图像分析，初步舍弃vofa+上位机

做混控Mixer的时候，在 control\_task 里把摇杆映射到电机，先只打印验证（PS2\_task.c专门留着充当调试层）

