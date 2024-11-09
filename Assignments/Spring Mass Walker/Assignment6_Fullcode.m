clear all
close all

% Parameters
m = 80;            % Mass (kg)
l0 = 1;            % Rest length of the spring (m)
k = 10000;         % Spring constant (N/m)
g = -9.81;         % Gravitational acceleration (m/s²)
alpha_TD = deg2rad(68); % Angle of touchdown (radians)
x0 = 0;            % Initial x position (m)
y0 = 0.97;         % Initial y position (m)
vx0 = 1.05;        % Initial horizontal speed (m/s)
vy0 = 0;           % Initial vertical speed (m/s)
t_start = 0;
t_end = 20;
dt = 0.001;
tspan = t_start:dt:t_end;

% Define failure conditions
y_threshold = 0; % Walker falls if y position is below this threshold
vx_threshold = 10; % Velocity limit for failure condition
vy_threshold = 10;

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
    if y < y_threshold || abs(vx) > vx_threshold || abs(vy) > vy_threshold || stance_phase_right == 0 && stance_phase_left == 0
        failure = true;
        failure_time = t;
        disp('Simulation stopped due to failure condition.');
        break;
    end
end

% Plot results
figure1 = figure('Position', [100, 100, 1600, 400]);
hold on;

% Plot the vertical position y over time
plot(results(:,1), results(:,3), 'b', 'LineWidth', 1.5);

% Plot all touchdown events as downwards-facing triangles
for i = 1:length(TD_events_left)
    idx = find(results(:,1) == TD_events_left(i));
    plot(TD_events_left(i), results(idx, 3), 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
for i = 1:length(TD_events_right)
    idx = find(results(:,1) == TD_events_right(i));
    plot(TD_events_right(i), results(idx, 3), 'kv', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end

% Plot all takeoff events as upwards-facing triangles
for i = 1:length(TO_events_left)
    idx = find(results(:,1) == TO_events_left(i));
    plot(TO_events_left(i), results(idx, 3), 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
for i = 1:length(TO_events_right)
    idx = find(results(:,1) == TO_events_right(i));
    plot(TO_events_right(i), results(idx, 3), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end

xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs. Time');
legend('Vertical Position (y)');
grid on;
hold off;

saveas(figure1, 'position_vs_time.png'); % Save the figure as PNG

figure2 = figure('Position', [100, 100, 1000, 600]); % Non-elongated figure
hold on;

% Plot the vertical position y vs. vertical velocity vy
plot(results(:,3), results(:,5), 'b', 'LineWidth', 1.5);

% Identify the limit cycle
% Assuming the limit cycle is towards the end of the simulation
% Adjust the start and end indices based on the actual convergence behavior
start_idx = round(0.9 * length(results)); % Start at 90% of the total time
limit_cycle_data = results(start_idx:end, :);

% Plot the limit cycle in red
plot(limit_cycle_data(:,3), limit_cycle_data(:,5), 'r', 'LineWidth', 2.5);

xlabel('Vertical Position (m)');
ylabel('Vertical Velocity (m/s)');
title('Vertical Position vs. Vertical Velocity with Limit Cycle');
legend('Trajectory', 'Limit Cycle');
grid on;
hold off;

saveas(figure2, 'vertical_position_vs_velocity.png'); % Save the figure as PNG

figure3 = figure('Position', [100, 100, 1600, 400]);
hold on;

% Plot the vertical forces over time
plot(results(:,1), results(:,8), 'r', 'LineWidth', 1.5); % Right leg force
plot(results(:,1), results(:,9), 'k', 'LineWidth', 1.5); % Left leg force

% Plot all touchdown events as downwards-facing triangles at y=0
for i = 1:length(TD_events_left)
    plot(TD_events_left(i), 0, 'kv', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end
for i = 1:length(TD_events_right)
    plot(TD_events_right(i), 0, 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

% Plot all takeoff events as upwards-facing triangles at y=0
for i = 1:length(TO_events_left)
    plot(TO_events_left(i), 0, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end
for i = 1:length(TO_events_right)
    plot(TO_events_right(i), 0, 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

xlabel('Time (s)');
ylabel('Force horizontal (N)');
title('Horizontal Forces vs. Time');
legend('Right Leg Force', 'Left Leg Force');
grid on;
hold off;

saveas(figure3, 'horizontal_forces_vs_time.png'); % Save the figure as PNG

figure4 = figure('Position', [100, 100, 1600, 400]);
plot(results(:,1), results(:,6), 'r', 'LineWidth', 1.5); hold on;
plot(results(:,1), results(:,7), 'k', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Force vertical (N)');
title('Vertical Forces vs. Time');
legend('Right Leg Force', 'Left Leg Force');
grid on;

saveas(figure4, 'vertical_forces_vs_time.png'); % Save the figure as PNG

% Plot x Velocity over Time
figure5 = figure('Position', [100, 100, 1600, 400]);
plot(results(:,1), results(:,4), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('x Velocity (m/s)');
title('x Velocity vs. Time');
grid on;
saveas(figure5, 'x_velocity_vs_time.png'); % Save the figure as PNG

% Calculate average, minimal, and maximal velocities
avg_vx = mean(results(:,4));
min_vx = min(results(:,4));
max_vx = max(results(:,4));

avg_vy = mean(results(:,5));
min_vy = min(results(:,5));
max_vy = max(results(:,5));

% Create a table of velocity statistics
velocity_table = table([avg_vx; min_vx; max_vx], [avg_vy; min_vy; max_vy], ...
    'VariableNames', {'x_Velocity', 'y_Velocity'}, ...
    'RowNames', {'Average', 'Minimum', 'Maximum'});

% Display the table
disp(velocity_table);



if failure
    disp(['Failure occurred at time: ', num2str(failure_time)]);
end
