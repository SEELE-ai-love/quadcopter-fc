# 图片与记录索引

此目录收录项目历史图片，并按内容统一为可维护的英文文件名。图片保留原内容，未重画曲线、改动数值或补造测试条件；目录内封面为可编辑SVG。

| 文件 | 内容与出处 | 使用边界 |
| --- | --- | --- |
| [project_overview.svg](project_overview.svg) | 自研四旋翼飞控展示封面 | 软件模块主题，不是实物照片或硬件性能证据 |
| [dual_attitude_estimators.svg](dual_attitude_estimators.svg) | EKF / Mahony 双解算路线 | 启动时选择算法，图示不表示两者同时融合 |
| [recovery_boundaries.svg](recovery_boundaries.svg) | 接收、估计与位置控制恢复边界 | 描述源码中的状态路径 |
| [attitude_ekf_static.png](attitude_ekf_static.png) | 2026-09-22 22:49:31 串口截图，作者确认EKF | 与另一截图单位及静止条件相同；截图无完整校准记录和原始日志 |
| [attitude_mahony_static.png](attitude_mahony_static.png) | 2026-09-22 22:52:08 串口截图，作者确认Mahony | 同上；Yaw参考值不同，不直接比较绝对值 |
| [attitude_matlab_tracking.png](attitude_matlab_tracking.png) | 2026-09-21 MATLAB 姿态分析截图 | 图例是一维KF与四元数EKF；不作为EKF/Mahony实机同数据对照 |
| [attitude_matlab_bias.png](attitude_matlab_bias.png) | 2026-09-21 MATLAB 零偏分析截图 | 图中真实偏置属于当时分析；未提供当前实机真值 |
| [thrust_voltage_surface.png](thrust_voltage_surface.png) | 历史推力—电压—油门拟合曲面 | 对应原始采样表、拟合脚本尚未收录 |
| [throttle_fit_comparison.png](throttle_fit_comparison.png) | 历史油门拟合对照 | 图中统计未经本次重算，不构成当前固件验收 |
| [throttle_fit_residuals.png](throttle_fit_residuals.png) | 历史油门拟合残差 | 同上 |

两张串口截图的区间仅描述可见窗口，不等同于完整测试统计。当前公开版本的FIFO和位置控制路径不能由这些历史图片代替上机验证。
