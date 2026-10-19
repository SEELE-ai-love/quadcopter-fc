function result = fit_throttle_model(csv_path)
%FIT_THROTTLE_MODEL 推力、电压到油门的二次最小二乘模型。
% 输入 CSV 列名：thrust_g, voltage_v, throttle。
% 示例：result = fit_throttle_model('my_measurements.csv');
% 只分析输入数据；不会写入飞控参数或执行电机操作。

data = readtable(csv_path);
required = {'thrust_g', 'voltage_v', 'throttle'};
if ~all(ismember(required, data.Properties.VariableNames))
    error('CSV must contain thrust_g, voltage_v and throttle columns.');
end
F = data.thrust_g;
V = data.voltage_v;
y = data.throttle;
if ~isnumeric(F) || ~isnumeric(V) || ~isnumeric(y)
    error('All three measurement columns must be numeric.');
end
if numel(y) < 6 || any(~isfinite([F; V; y]))
    error('At least six finite measurement rows are required.');
end
if any(F < 0) || any(V <= 0) || any(y < 48 | y > 1148)
    error('Check thrust, voltage and throttle units/ranges.');
end

% 标准化后拟合，减小平方项造成的数值尺度差异。
muF = mean(F);
muV = mean(V);
sF = std(F);
sV = std(V);
if sF == 0 || sV == 0
    error('Thrust and voltage must both vary across the measurements.');
end
f = (F - muF) / sF;
v = (V - muV) / sV;
A = [ones(size(f)), f, v, f.^2, f.*v, v.^2];
if rank(A) < 6
    error('Measurements do not identify all six polynomial coefficients.');
end
b = A \ y;

% 还原物理单位：T = c0 + cF*F + cV*V + cFF*F^2 + cFV*F*V + cVV*V^2。
cFF = b(4) / sF^2;
cFV = b(5) / (sF*sV);
cVV = b(6) / sV^2;
cF = b(2)/sF - 2*cFF*muF - cFV*muV;
cV = b(3)/sV - 2*cVV*muV - cFV*muF;
c0 = b(1) - b(2)*muF/sF - b(3)*muV/sV ...
     + cFF*muF^2 + cFV*muF*muV + cVV*muV^2;
result.coefficients = [c0; cF; cV; cFF; cFV; cVV];
result.fitted = A*b;
result.residual = y - result.fitted;
result.rmse = sqrt(mean(result.residual.^2));
result.mae = mean(abs(result.residual));
sst = sum((y - mean(y)).^2);
result.r_squared = NaN;
if sst > 0
    result.r_squared = 1 - sum(result.residual.^2)/sst;
end
result.sample_count = numel(y);
result.thrust_range_g = [min(F), max(F)];
result.voltage_range_v = [min(V), max(V)];

% 与固件中当前系数对照；它们是已有参数，不是本脚本的拟合结果。
result.firmware_coefficients = [1537.96029618; 8.70113970; ...
    -225.50577912; -0.00880428; -0.23781058; 9.01035198];
physical = [ones(size(F)), F, V, F.^2, F.*V, V.^2];
result.firmware_prediction = physical * result.firmware_coefficients;
fprintf('Coefficients [1 F V F^2 F*V V^2]:\n');
fprintf('%+.10f\n', result.coefficients);
fprintf('N=%d RMSE=%.6f MAE=%.6f R2=%.6f\n', ...
    result.sample_count, result.rmse, result.mae, result.r_squared);

figure('Name', 'Thrust-Voltage-Throttle / Model Analysis');
subplot(1, 2, 1);
scatter(y, result.fitted, 24, V, 'filled');
hold on;
plot([min(y), max(y)], [min(y), max(y)], 'k--');
hold off;
xlabel('Measured throttle'); ylabel('Fitted throttle');
title('Fit comparison'); colorbar; grid on;
subplot(1, 2, 2);
scatter(result.fitted, result.residual, 24, V, 'filled');
xlabel('Fitted throttle'); ylabel('Measured - fitted');
title('Residuals'); colorbar; grid on;
end
