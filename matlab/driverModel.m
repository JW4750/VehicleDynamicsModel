function control = driverModel(t, state, params)
%DRIVERMODEL Simple longitudinal speed hold and steering profile.
%
%   control = DRIVERMODEL(t, state, params) generates steering and drive
%   force commands for a left turn manoeuvre executed at 30 km/h. The
%   driver model holds a target longitudinal speed and applies a smooth
%   steering step at t = 1 s.
%
%   Inputs
%       t      - Simulation time [s]
%       state  - Current vehicle state vector
%                [X, Y, psi, u, v, r]
%       params - Vehicle parameter struct
%
%   Output structure fields
%       steerFL  - Front-left steer angle [rad]
%       steerFR  - Front-right steer angle [rad]
%       FxFL     - Front-left longitudinal force [N]
%       FxFR     - Front-right longitudinal force [N]
%       FxRL     - Rear-left longitudinal force [N]
%       FxRR     - Rear-right longitudinal force [N]
%
%   The steering law uses an Ackermann approximation in which a single
%   commanded wheel angle is converted to inner/outer angles based on the
%   current manoeuvre radius.

% target speed 30 km/h
vTarget = 30 / 3.6;

u = state(4);

% Simple PI controller to maintain speed using variable-step integration
persistent integ lastTime
if isempty(integ) || isempty(lastTime) || t <= 0
    integ = 0;
    lastTime = t;
end

speedError = vTarget - u;
integratorGain = 0.5;
propGain = 1200;
integralLimit = params.maxDrive;

dt = max(t - lastTime, params.sampleTime);
integ = integ + integratorGain * speedError * dt;
integ = max(min(integ, integralLimit), -integralLimit);
lastTime = t;

FxTotal = propGain * speedError + integ;
FxTotal = max(min(FxTotal, params.maxDrive), -params.maxDrive);

% Distribute drive force to front axle (front-wheel-drive assumption)
FxFL = 0.5 * FxTotal;
FxFR = 0.5 * FxTotal;
FxRL = 0;
FxRR = 0;

% Steering profile: smooth step using hyperbolic tangent ramp
steerPeak = deg2rad(8);
steerRate = 2; % 1/s
steerDelay = 1.0;

if t <= steerDelay
    steerCmd = 0;
else
    steerCmd = steerPeak * 0.5 * (1 + tanh(steerRate * (t - steerDelay - 0.5)));
end

% Ackermann conversion for inner/outer wheel angles
if abs(steerCmd) < 1e-6
    steerFL = 0;
    steerFR = 0;
else
    curvature = tan(steerCmd) / params.wheelBase;
    radius = 1 / curvature;
    innerAngle = atan(params.wheelBase / (radius - params.trackF / 2));
    outerAngle = atan(params.wheelBase / (radius + params.trackF / 2));
    if steerCmd > 0
        steerFL = innerAngle;
        steerFR = outerAngle;
    else
        steerFL = -outerAngle;
        steerFR = -innerAngle;
    end
end

control.steerFL = max(min(steerFL, params.maxSteer), -params.maxSteer);
control.steerFR = max(min(steerFR, params.maxSteer), -params.maxSteer);
control.FxFL = FxFL;
control.FxFR = FxFR;
control.FxRL = FxRL;
control.FxRR = FxRR;
end
