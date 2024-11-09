% Define constants
g = 9.81;      % Acceleration due to gravity (m/s^2)
m = 1;         % Mass (kg)
k = 100;       % Spring constant (N/m)
omega = sqrt(k/m);  % Angular frequency (rad/s)
x0 = 0;      % Initial position for Phase Two (m)
v0 = -1.9806;        % Initial velocity for Phase Two (m/s)
h = 0.2;       % Initial position for Phase One (m)

% Define time vectors
t1 = linspace(0, 0.202, 11); % Time vector for Phase One (2 seconds)
t2 = linspace(0, 2, 100); % Time vector for Phase Two (2 seconds)

% Phase One Equations
a1 = -g * ones(size(t1));           % Constant acceleration
v1 = -g * t1;                       % Linearly increasing velocity
x1 = h-0.5 * g * t1.^2;              % Quadratic position

% Energies for Phase One
K1 = 0.5 * m * v1.^2;
Ug1 = m * g * (x1); % Negative because x1 is displacement downwards

% Phase Two Dynamics
x2 = m*g/k * (cos(omega * t2) - 1) + v0/omega * sin(omega * t2);
v2 = -m*g/omega/k * sin(omega * t2) + v0 * cos(omega * t2);
a2 = -omega^2 * (x0 * cos(omega * t2) + v0/omega * sin(omega * t2));

% Energies for Phase Two
K2 = 0.5 * m * v2.^2;
Ug2 = m * g * x2; % Positive upwards displacement
Ue2 = 0.5 * k * x2.^2;

% Plotting Energy Values for Phase One
figure;
plot(t1, K1, 'b-', t1, Ug1, 'r-');
title('Phase One Energy Values');
xlabel('Time (s)');
ylabel('Energy (Joules)');
legend('Kinetic Energy', 'Gravitational Potential Energy');
saveas(gcf, 'bouncing_mass_phaseone_energy.png')
% Plotting Energy Values for Phase Two
figure;
plot(t2, K2, 'b-', t2, Ug2, 'r-', t2, Ue2, 'g-');
title('Phase Two Energy Values');
xlabel('Time (s)');
ylabel('Energy (Joules)');
legend('Kinetic Energy', 'Gravitational Potential Energy', 'Elastic Potential Energy');
saveas(gcf, 'bouncing_mass_phasetwo_energy.png')
