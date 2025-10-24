%% SIMULATE_LEFT_TURN Demonstration of the four-wheel vehicle model.
% This script constructs the parameter set, integrates the nonlinear
% equations of motion, and plots key responses for a 30 km/h left-hand turn.
%
% The workflow is compatible with MATLAB R2024b and relies purely on base
% MATLAB and Simulink-supported constructs, enabling straightforward
% integration into a Simulink model via S-Function or MATLAB Function block.
%
% References:
%   [1] R. Rajamani, "Vehicle Dynamics and Control", 2nd ed., Springer, 2012.
%   [2] J.Y. Wong, "Theory of Ground Vehicles", 4th ed., Wiley, 2001.
%   [3] UWVehicleDynamics project, University of Waterloo Alternative Fuels
%       Team, GitHub repository, 2023 (github.com/uw-midsun/UWVehicleDynamics).
%
% Author: Automated MATLAB generation via ChatGPT (OpenAI), 2024.

clear; clc;

params = vehicleParameters();

% Provide sample time for driver integration logic
params.sampleTime = 0.01;

% Driver control callback handle
controlFcn = @(t, x) driverModel(t, x, params);

% Initial conditions: stationary lateral and yaw motion, forward speed 30 km/h
v0 = 30 / 3.6;
x0 = [0; 0; 0; v0; 0; 0];

% Simulation horizon
Tend = 8;

% Integrate using ODE45 with tight tolerances for repeatability
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
sol = ode45(@(t, x) fourWheelVehicleDynamics(t, x, controlFcn, params), [0 Tend], x0, opts);

t = linspace(0, Tend, ceil(Tend / params.sampleTime) + 1);
x = deval(sol, t);

% Extract outputs
X = x(1, :);
Y = x(2, :);
psi = x(3, :);
u = x(4, :);
vLat = x(5, :);
r = x(6, :);

% Compute curvature and lateral acceleration for diagnostics
curvature = r ./ max(u, 0.1);
ay = r .* u + gradient(u, t);

%% Plotting
figure('Name', 'Four-wheel vehicle left turn', 'NumberTitle', 'off');
subplot(3, 2, 1);
plot(t, u * 3.6, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [km/h]'); grid on; title('Longitudinal speed');

subplot(3, 2, 2);
plot(t, curvature, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Curvature [1/m]'); grid on; title('Path curvature');

subplot(3, 2, 3);
plot(t, rad2deg(psi), 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Yaw angle [deg]'); grid on; title('Yaw response');

subplot(3, 2, 4);
plot(t, rad2deg(r), 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Yaw rate [deg/s]'); grid on; title('Yaw rate');

subplot(3, 2, 5);
plot(X, Y, 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]'); grid on; axis equal; title('Vehicle path');

subplot(3, 2, 6);
plot(t, ay, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Lateral acceleration [m/s^2]'); grid on; title('Lateral acceleration');

sgtitle('Four-wheel vehicle model response for 30 km/h left turn');

%% Export key metrics for reporting
results.time = t;
results.states = x;
results.finalPosition = [X(end); Y(end)];
results.maxLateralAccel = max(abs(ay));
results.maxYawRate = max(abs(r));
results.lateralAcceleration = ay;

assignin('base', 'vehicleResults', results);

fprintf('Final position: X = %.2f m, Y = %.2f m\n', results.finalPosition(1), results.finalPosition(2));
fprintf('Peak lateral acceleration: %.2f m/s^2\n', results.maxLateralAccel);
fprintf('Peak yaw rate: %.2f deg/s\n', rad2deg(results.maxYawRate));
