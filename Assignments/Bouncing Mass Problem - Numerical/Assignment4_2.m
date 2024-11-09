clear all;
close all;
clc;

% Constants
g = 9.81;           % Acceleration due to gravity (m/s^2)
m = 1;              % Mass (kg)
k = 100;            % Spring constant (N/m)
l0 = 1;             % Natural length of the spring (m)
omega = sqrt(k/m);  % Angular frequency (rad/s)
h=0.2;

% Time setup
t_total = 3;        % Total simulation time in seconds
dt = 0.001;         % Time step in seconds
N = t_total/dt;     % Number of time steps

% Initialize arrays
t = 0:dt:t_total;   % Time vector
x = zeros(1, N+1);
v = zeros(1, N+1);
E_Kin = zeros(1, N+1); % Kinetic energy
E_pot = zeros(1, N+1); % Potential energy
E_spring = zeros(1, N+1); % Spring energy

% Initial conditions
x(1) = 1.2;         % Initial height for flight (m above ground)
v(1) = 0;           % Initial velocity for flight (m/s)
phase = 'flight';   % Start in flight phase

% Compute initial energies
E_Kin(1) = 0.5 * m * v(1)^2;
E_pot(1) = m * g * (x(1));
E_spring(1) = 0;

% Simulation loop
for i = 1:N
    if strcmp(phase, 'flight')
        % Flight Phase Dynamics
        a = -g;
        v(i+1) = v(i) + a * dt;
        x(i+1) = x(i) + v(i+1) * dt;

        E_Kin(i+1) = 0.5 * m * v(i+1)^2;
        E_pot(i+1) = m * g * (x(i+1));
        E_spring(i+1) = 0;

        % Check if x crosses 1 meter
        if x(i+1) <= 1
            phase = 'compression';  % Switch to compression phase
        end
    else
        % Compression Phase Dynamics
        a = -(k/m) * (x(i) - l0) - g;
        v(i+1) = v(i) + a * dt;
        x(i+1) = x(i) + v(i+1) * dt;

        % Energy calculations
        E_Kin(i+1) = 0.5 * m * v(i+1)^2;
        E_pot(i+1) = m * g * (x(i+1));
        E_spring(i+1) = 0.5 * k * (x(i+1) - l0)^2;

        % Check if x crosses 1 meter
        if x(i+1) >= 1 && v(i+1) > 0
            phase = 'flight';  % Switch back to flight phase
        end
    end
end

% Initialize Simulink model
model = 'Simulink_Klostermair2';
open_system(model);

% Solver settings
solverSettings = {
    {'ode45', 'Variable-step', 1e-3, 1e-6}, % RelTol 1e-3, AbsTol 1e-6
    {'ode45', 'Variable-step', 1e-5, 1e-8}, % RelTol 1e-5, AbsTol 1e-8
    {'ode4', 'Fixed-step', 0.01}, % Fixed Step Size 0.01 s
    {'ode4', 'Fixed-step', 0.001}  % Fixed Step Size 0.001 s
};

% Prepare to store results
simResults = struct;

% Run simulations with different solver settings
for i = 1:numel(solverSettings)
    solver = solverSettings{i}{1};
    solverType = solverSettings{i}{2};

    % For variable-step solvers, use RelTol and AbsTol
    if strcmp(solverType, 'Variable-step')
        relTol = solverSettings{i}{3};
        absTol = solverSettings{i}{4};

        % Set solver parameters for variable-step solvers
        set_param(model, 'Solver', solver);
        set_param(model, 'SolverType', solverType);
        set_param(model, 'RelTol', num2str(relTol));
        set_param(model, 'AbsTol', num2str(absTol));
        simResults(i).solver = sprintf('%s (RelTol %g, AbsTol %g)', solver, relTol, absTol);
    else
        % Set fixed-step size
        fixedStepSize = solverSettings{i}{3};

        % Set solver parameters for fixed-step solvers
        set_param(model, 'Solver', solver);
        set_param(model, 'SolverType', solverType);
        set_param(model, 'FixedStep', num2str(fixedStepSize));
        simResults(i).solver = sprintf('%s (Fixed Step Size %g s)', solver, fixedStepSize);
    end

    % Simulate
    simOut = sim(model);

    % Extract signals
    x_t = simOut.get('x_t');
    E_kin = simOut.get('E_kin');
    E_Pot = simOut.get('E_Pot');
    E_Spring = simOut.get('E_Spring');

    % Store results
    simResults(i).t = x_t.Time;
    simResults(i).x = x_t.Data;
    simResults(i).E_kin = E_kin.Data;
    simResults(i).E_Pot = E_Pot.Data;
    simResults(i).E_Spring = E_Spring.Data;
end

% Plot comparison between numerical results
figure(1);
hold on;
plot(t, E_Kin, 'k', 'LineWidth', 2); % Analytical solution
for i = 1:numel(simResults)
    plot(simResults(i).t, simResults(i).E_kin, 'LineWidth', 1.5);
end
legend(['Analytical', {simResults.solver}], 'Location', 'Best');
xlabel('Time (s)');
ylabel('Kinetic Energy (J)');
title('Comparison of Numerical Solutions with Analytical Solution (E\_kin)');
grid on;

figure(2);
hold on;
plot(t, E_pot, 'k', 'LineWidth', 2); % Analytical solution
for i = 1:numel(simResults)
    plot(simResults(i).t, simResults(i).E_Pot, 'LineWidth', 1.5);
end
legend(['Analytical', {simResults.solver}], 'Location', 'Best');
xlabel('Time (s)');
ylabel('Potential Energy (J)');
title('Comparison of Numerical Solutions with Analytical Solution (E\_Pot)');
grid on;

figure(3);
hold on;
plot(t, E_spring, 'k', 'LineWidth', 2); % Analytical solution
for i = 1:numel(simResults)
    plot(simResults(i).t, simResults(i).E_Spring, 'LineWidth', 1.5);
end
legend(['Analytical', {simResults.solver}], 'Location', 'Best');
xlabel('Time (s)');
ylabel('Spring Energy (J)');
title('Comparison of Numerical Solutions with Analytical Solution (E\_Spring)');
grid on;

% Plot comparison between numerical results and analytical solution
figure(4);
hold on;
plot(t, x, 'k', 'LineWidth', 2); % Analytical solution
for i = 1:numel(simResults)
    plot(simResults(i).t, simResults(i).x, 'LineWidth', 1.5);
end
legend(['Analytical', {simResults.solver}], 'Location', 'Best');
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Comparison of Numerical Solutions with Analytical Solution (x\_t)');
grid on;

% Save figures to files
figure_paths = {'Displacement.png', 'KineticEnergy.png', 'PotentialEnergy.png', 'SpringEnergy.png', 'Comparison.png'};
figures = [1, 2, 3, 4]; % Corresponding figure numbers

for i = 1:length(figures)
    saveas(figure(figures(i)), figure_paths{i}, 'png'); % Save each figure as a PNG file
end


