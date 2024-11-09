function [hipRightAngle, hipLeftAngle, kneeRightAngle, kneeLeftAngle] = voltagesToAngles(voltages, H_range, K_range, HR_vrange, HL_vrange, KR_vrange, KL_vrange)
    % Input validation
%     if length(voltages) ~= 4
%         error('The voltages array must contain exactly 4 elements.');
%     end

    % Extract voltage readings
    HR_voltage = voltages(:,1);
    HL_voltage = voltages(:,2);
    KR_voltage = voltages(:,3);
    KL_voltage = voltages(:,4);

    % Define ranges for hip and knee joints
    hip_min_angle = H_range(1);
    hip_max_angle = H_range(2);
    knee_min_angle = K_range(1);
    knee_max_angle = K_range(2);

    % Calculate slopes and intercepts for each joint
    HR_slope = (hip_max_angle - hip_min_angle) / (HR_vrange(2) - HR_vrange(1));
    HR_intercept = hip_min_angle - HR_slope * HR_vrange(1);
    HL_slope = (hip_max_angle - hip_min_angle) / (HL_vrange(2) - HL_vrange(1));
    HL_intercept = hip_min_angle - HL_slope * HL_vrange(1);
    KR_slope = (knee_max_angle - knee_min_angle) / (KR_vrange(2) - KR_vrange(1));
    KR_intercept = knee_min_angle - KR_slope * KR_vrange(1);
    print(KR_slope);
    print(KR_intercept);
    print(KR_vrange(1));
    print(KR_vrange(2);
    KL_slope = (knee_max_angle - knee_min_angle) / (KL_vrange(2) - KL_vrange(1));
    KL_intercept = knee_min_angle - KL_slope * KL_vrange(1);

    % Compute the angles based on the input voltages
    hipRightAngle = HR_slope * HR_voltage + HR_intercept;
    hipLeftAngle = HL_slope * HL_voltage + HL_intercept;
    kneeRightAngle = KR_slope * KR_voltage + KR_intercept;
    kneeLeftAngle = KL_slope * KL_voltage + KL_intercept;
end