clear all
close all

% Parameters
m = 80;            % Mass (kg)
l0 = 1;            % Rest length of the spring (m)
g = -9.81;         % Gravitational acceleration (m/s²)
x0 = 0;            % Initial x position (m)
y0 = 0.97;         % Initial y position (m)
vx0 = 1.2;        % Initial horizontal speed (m/s)
vy0 = 0;           % Initial vertical speed (m/s)
t_start = 0;
t_end = 10;
dt = 0.001;
tspan = t_start:dt:t_end;

% Define failure conditions
y_threshold = 0; % Walker falls if y position is below this threshold
vx_threshold = 10; % Velocity limit for failure condition
vy_threshold = 10;

% Ranges for k and alpha_TD
k_values = 0:1000:50000; % Spring constant values from 10000 to 20000 N/m
alpha_TD_values = deg2rad(50:0.5:85); % Touchdown angles from 65 to 75 degrees in radians

% Initialize storage for successful combinations
successful_combinations = false(length(k_values), length(alpha_TD_values));

for ki = 1:length(k_values)
    for ai = 1:length(alpha_TD_values)
        k = k_values(ki);
        alpha_TD = alpha_TD_values(ai);
        
        % Create instances of the left and right leg
        left_leg = SingleLeg;
        left_leg.left = true;
        left_leg.alpha_TD = alpha_TD;
        left_leg.k = k;
        left_leg.resetLeg(); % Call the public reset method

        right_leg = SingleLeg;
        right_leg.left = false;
        right_leg.alpha_TD = alpha_TD;
        right_leg.k = k;
        right_leg.resetLeg(); % Call the public reset method

        % Initialize positions and velocities
        x = x0;
        y = y0;
        vx = vx0;
        vy = vy0;

        % Initialize storage for results and TD/TO events
        results = zeros(length(tspan), 11);
        TD_events_left = [];
        TO_events_left = [];
        TD_events_right = [];
        TO_events_right = [];
        failure = false;
        failure_time = NaN;

        % Simulation loop
        for i = 1:length(tspan)
            t = tspan(i);

            % Compute forces from both legs
            [Fs_x_left, Fs_y_left, stance_phase_left, ~] = left_leg.computeForces(x, y, vy, t);
            [Fs_x_right, Fs_y_right, stance_phase_right, ~] = right_leg.computeForces(x, y, vy, t);

            % Sum forces
            Fs_x = Fs_x_left + Fs_x_right;
            Fs_y = Fs_y_left + Fs_y_right;

            % Integrate accelerations to get velocities
            ax = Fs_x / m;
            ay = (Fs_y + m * g) / m;
            vx = vx + ax * dt;
            vy = vy + ay * dt;

            % Integrate velocities to get positions
            x = x + vx * dt;
            y = y + vy * dt;

            % Store results
            results(i, :) = [t, x, y, vx, vy, Fs_y_right, Fs_y_left, Fs_x_right, Fs_x_left, stance_phase_right, stance_phase_left];

            % Track TD and TO events for left leg
            if i > 1
                if results(i-1, 11) == 0 && stance_phase_left == 1
                    TD_events_left = [TD_events_left; t]; % Left leg touchdown
                elseif results(i-1, 11) == 1 && stance_phase_left == 0
                    TO_events_left = [TO_events_left; t]; % Left leg takeoff
                end

                % Track TD and TO events for right leg
                if results(i-1, 10) == 0 && stance_phase_right == 1
                    TD_events_right = [TD_events_right; t]; % Right leg touchdown
                elseif results(i-1, 10) == 1 && stance_phase_right == 0
                    TO_events_right = [TO_events_right; t]; % Right leg takeoff
                end
            end

            % Check for failure conditions
            if y < y_threshold || abs(vx) > vx_threshold || abs(vy) > vy_threshold
                failure = true;
                failure_time = t;
                break;
            end
        end

        % Check if simulation completed without failure
        if ~failure
            successful_combinations(ki, ai) = true;
        end
    end
end

% Display successful combinations
disp('Successful combinations of k and alpha_TD (in degrees) with v = 1.2 (m/s):');
disp(successful_combinations);

% Plot successful combinations as a table
figure('Position', [100, 100, 600, 1000]);
imagesc(successful_combinations(end:-1:1, :)); % Flip the y-axis data
colormap([1 1 1; 0 0 1]); % White for failure, blue for success
set(gca, 'XTick', 1:length(alpha_TD_values), 'XTickLabel', rad2deg(alpha_TD_values));
set(gca, 'YTick', 1:length(k_values), 'YTickLabel', k_values(end:-1:1)); % Flip the y-axis labels
xlabel('Alpha TD (degrees)');
ylabel('Spring Constant k (N/m)');
title('Successful Combinations of k and Alpha TD');
grid on;

saveas(gcf, 'successful_combinations.png');