clear all
close all
% Define constants
g = 3.71;      % Acceleration due to gravity (m/s^2)
m = 1;         % Mass (kg)
k = 100;       % Spring constant (N/m)
omega = sqrt(k/m);  % Angular frequency (rad/s)
x0 = 0;      % Initial position for Phase Two (m)
v0 = -1.22;        % Initial velocity for Phase Two (m/s)
h = 0.2;       % Initial position for Phase One (m)

% Define time vectors
t1 = linspace(0, 0.328, 17); % Time vector for Phase One (2 seconds)
t2 = linspace(0, 2, 100); % Time vector for Phase Two (2 seconds)

% Phase One Equations
a1 = -g * ones(size(t1));           % Constant acceleration
v1 = -g * t1;                       % Linearly increasing velocity
x1 = h-0.5 * g * t1.^2;              % Quadratic position

% Phase Two Equations
a2 = -omega^2 * (x0 * cos(omega * t2) + (v0/omega) * sin(omega * t2)); % Acceleration
v2 = -m*g/omega/k * sin(omega * t2) + v0 * cos(omega * t2);             % Velocity
x2 = m*g/k * (cos(omega * t2) - 1) + v0/omega * sin(omega * t2);        % Position

% Plotting Phase One
figure;
subplot(3,1,1); % Acceleration plot for Phase One
plot(t1, a1, 'r-');
title('Phase One - Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

subplot(3,1,2); % Velocity plot for Phase One
plot(t1, v1, 'b-');
title('Phase One - Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

subplot(3,1,3); % Position plot for Phase One
plot(t1, x1, 'g-');
title('Phase One - Position');
xlabel('Time (s)');
ylabel('Position (m)');
saveas(gcf, 'bouncing_mass_phaseone_eom_mars.png')
% Plotting Phase Two
figure;
subplot(3,1,1); % Acceleration plot for Phase Two
plot(t2, a2, 'r-');
title('Phase Two - Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

subplot(3,1,2); % Velocity plot for Phase Two
plot(t2, v2, 'b-');
title('Phase Two - Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

subplot(3,1,3); % Position plot for Phase Two
plot(t2, x2, 'g-');
title('Phase Two - Position');
xlabel('Time (s)');
ylabel('Position (m)');
saveas(gcf, 'bouncing_mass_phasetwo_eom_mars.png')
