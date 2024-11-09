function plotJointAngles(voltages, H_range, K_range, HR_vrange, HL_vrange, KR_vrange, KL_vrange)
    % Extract voltage readings
    HR_voltage = voltages(:, 1);
    HL_voltage = voltages(:, 2);
    KR_voltage = voltages(:, 3);
    KL_voltage = voltages(:, 4);

    % Define ranges for hip and knee joint
    
    hip_min_angle = H_range(1);
    hip_max_angle = H_range(2);
    knee_min_angle = K_range(1);
    knee_max_angle = K_range(2);

    % Calculate slopes and intercepts for each joint
    HR_slope = (hip_max_angle - hip_min_angle) / (HR_vrange(2) - HR_vrange(1));
    HR_intercept = hip_min_angle - HR_slope * HR_vrange(1);
    HL_slope = (hip_max_angle - hip_min_angle) / (HL_vrange(2) - HL_vrange(1));
    HL_intercept = hip_min_angle - HL_slope * HL_vrange(1);
    KR_slope = (knee_max_angle - knee_min_angle) / (KR_vrange(1) - KR_vrange(2));
    KR_intercept = knee_min_angle - KR_slope * KR_vrange(2);
    fprintf('KR_slope: %f\n', KR_slope);
    fprintf('KR_intercept: %f\n', KR_intercept);
    fprintf('KR_vrange(1): %f\n', KR_vrange(1));
    fprintf('KR_vrange(2): %f\n', KR_vrange(2));
    KL_slope = (knee_max_angle - knee_min_angle) / (KL_vrange(2) - KL_vrange(1));
    KL_intercept = knee_min_angle - KL_slope * KL_vrange(1);

    % Compute the angles based on the input voltages
    hipRightAngle = HR_slope * HR_voltage + HR_intercept;
    hipLeftAngle = HL_slope * HL_voltage + HL_intercept;
    kneeRightAngle = KR_slope * KR_voltage + KR_intercept;
    kneeLeftAngle = KL_slope * KL_voltage + KL_intercept;
    
    % Plot the voltages
figure('Position', [100, 100, 1200, 800]);
subplot(2, 2, 1);
plot(voltages(:, 1), 'LineWidth', 1.5);
title('Right Hip Voltage', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Voltage (V)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 2);
plot(voltages(:, 2), 'LineWidth', 1.5);
title('Left Hip Voltage', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Voltage (V)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 3);
plot(voltages(:, 3), 'LineWidth', 1.5);
title('Right Knee Voltage', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Voltage (V)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 4);
plot(voltages(:, 4), 'LineWidth', 1.5);
title('Left Knee Voltage', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Voltage (V)', 'FontSize', 16);
set(gca, 'FontSize', 16);

% Plot the angles
figure('Position', [100, 100, 1200, 800]);
subplot(2, 2, 1);
plot(hipRightAngle, 'LineWidth', 1.5);
title('Right Hip Angle', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Angle (degrees)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 2);
plot(hipLeftAngle, 'LineWidth', 1.5);
title('Left Hip Angle', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Angle (degrees)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 3);
plot(kneeRightAngle, 'LineWidth', 1.5);
title('Right Knee Angle', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Angle (degrees)', 'FontSize', 16);
set(gca, 'FontSize', 16);

subplot(2, 2, 4);
plot(kneeLeftAngle, 'LineWidth', 1.5);
title('Left Knee Angle', 'FontSize', 16);
xlabel('Sample', 'FontSize', 16);
ylabel('Angle (degrees)', 'FontSize', 16);
set(gca, 'FontSize', 16);
end