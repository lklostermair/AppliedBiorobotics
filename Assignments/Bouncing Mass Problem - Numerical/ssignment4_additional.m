clear all;
close all;
clc;

% Constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
m = 1;              % Mass (kg)
l0 = 1;             % Natural length of the spring (m)
h = 0.2;            % Initial height (m)
k_max = 150;
k_min = 50;
x_max = -0.6*l0;

% Time setup
t_total = 3;        % Total simulation time in seconds
dt = 0.001;         % Time step in seconds

% Solver settings
solver = 'ode45';
relTol = 1e-3;
absTol = 1e-6;

% Simulate Klostermair4 with variable k
model = 'Simulink_Klostermair4';
set_param(model, 'Solver', solver, 'SolverType', 'Variable-step', 'RelTol', num2str(relTol), 'AbsTol', num2str(absTol));
simOut_k = sim(model);
x_t_k = simOut_k.get('x_t');
E_Spring_k = simOut_k.get('E_Spring');

% Reset k to 100 for Klostermair2
k = 100;  % Spring constant reset to 100 N/m

% Simulate Klostermair2 with fixed k
model = 'Simulink_Klostermair2';
set_param(model, 'Solver', solver, 'SolverType', 'Variable-step', 'RelTol', num2str(relTol), 'AbsTol', num2str(absTol));
simOut = sim(model);
x_t = simOut.get('x_t');
E_Spring = simOut.get('E_Spring');

% Plot comparison of Spring Energy
figure;
hold on;
plot(x_t_k.Time, E_Spring_k.Data, 'b-', 'LineWidth', 2, 'DisplayName', 'Spring Energy variable k');
plot(x_t.Time, E_Spring.Data, 'r--', 'LineWidth', 2, 'DisplayName', 'Spring Energy constant k');
xlabel('Time (s)');
ylabel('Spring Energy (J)');
title('Comparison of Spring Energy between Models');
legend show;
grid on;

% Plot comparison of Displacement
figure;
hold on;
plot(x_t_k.Time, x_t_k.Data, 'b-', 'LineWidth', 2, 'DisplayName', 'Displacement variable k');
plot(x_t.Time, x_t.Data, 'r--', 'LineWidth', 2, 'DisplayName', 'Displacement constant k');
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Comparison of Displacement between Models');
legend show;
grid on;
