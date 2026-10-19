# 油门模型分析 / Throttle Model Analysis

`fit_throttle_model.m` 使用 MATLAB 基础矩阵运算进行二次最小二乘拟合，不依赖 Curve Fitting Toolbox。输入是你提供的测量 CSV，输出系数、训练集残差、RMSE、MAE、R² 和对照图。

```matlab
result = fit_throttle_model('my_measurements.csv');
```

CSV 第一行为 `thrust_g,voltage_v,throttle`；后续每行分别为推力（克力数值）、电压（V）、油门指令值。至少需要六行且设计矩阵满秩，建议覆盖多个电压和推力区间，并保留独立测量集验证。

模型与固件 `Weight_To_Throttle()` 的项顺序一致：

```text
throttle = c0 + cF*F + cV*V + cFF*F² + cFV*F*V + cVV*V²
```

- `coefficients` 是本次输入数据的拟合结果；`firmware_coefficients` 是当前固件已有系数，二者分别保存。
- 指标只描述本次拟合输入，不能替代独立数据验证。脚本未自动剔除重复推力、零推力或异常值，需结合原始采样过程判断。
- 油门是控制指令，不是转速。输入按现有指令范围 48～1148 检查；多项式预测不裁剪，以保留真实残差。
- 不提供虚构测量数据，不自动替换固件系数，也不执行固件构建或电机操作。当前脚本尚未在 MATLAB 中运行验证。
- 历史图片见 [`docs/media`](../../docs/media/README.md)，本脚本不宣称复现其指标。外部配套文件的数据不视为本项目自行采集的数据。
