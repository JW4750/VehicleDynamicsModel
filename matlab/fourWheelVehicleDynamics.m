function dx = fourWheelVehicleDynamics(t, x, controlFcn, params)
%FOURWHEELVEHICLEDYNAMICS Continuous-time 6-DOF planar vehicle model.
%
%   dx = FOURWHEELVEHICLEDYNAMICS(t, x, controlFcn, params) evaluates the
%   nonlinear equations of motion for a four-wheel ground vehicle assuming
%   small roll and pitch motions (planar model). Tire forces are evaluated
%   on all four wheels using a linear tyre model valid for moderate slip
%   angles. This representation is commonly described in Rajamani (2012),
%   Wong (2001), and in open-source MATLAB vehicle dynamics templates such
%   as MathWorks' Vehicle Dynamics Blockset examples and UWVehicleDynamics
%   (GitHub, 2023).
%
%   State vector x = [X; Y; psi; u; v; r]
%       X   - Inertial x-position [m]
%       Y   - Inertial y-position [m]
%       psi - Yaw angle [rad]
%       u   - Longitudinal velocity in body frame [m/s]
%       v   - Lateral velocity in body frame [m/s]
%       r   - Yaw rate [rad/s]
%
%   controlFcn is a handle @(t, x) returning a struct with steering angles
%   and longitudinal forces for each wheel (fields steerFL, steerFR,
%   FxFL, FxFR, FxRL, FxRR).

% Unpack state
X = x(1);
Y = x(2);
psi = x(3);
u = x(4);
v = x(5);
r = x(6);

% Retrieve control inputs
control = controlFcn(t, x);

% Vehicle parameters
m = params.m;
Iz = params.Iz;
a = params.a;
b = params.b;
trackF = params.trackF;
trackR = params.trackR;
Cf = params.Cf;
Cr = params.Cr;
rho = params.rho;
Cd = params.Cd;
Af = params.Af;
Cr0 = params.Cr0;
mu = params.mu;

% Aerodynamic and rolling resistance forces
Fdrag = 0.5 * rho * Cd * Af * u * abs(u);
Froll = params.m * params.g * Cr0 * sign(u);
if abs(u) < 0.1
    Froll = params.m * params.g * Cr0 * (u / 0.1); % smooth near zero
end

% Wheel positions relative to CG
halfTrackF = trackF / 2;
halfTrackR = trackR / 2;

% Local velocities at each wheel center
[vxFL, vyFL] = wheelVelocity(u, v, r,  a,  halfTrackF);
[vxFR, vyFR] = wheelVelocity(u, v, r,  a, -halfTrackF);
[vxRL, vyRL] = wheelVelocity(u, v, r, -b,  halfTrackR);
[vxRR, vyRR] = wheelVelocity(u, v, r, -b, -halfTrackR);

% Slip angles (positive when wheel velocity vector points outward)
alphaFL = atan2(vyFL, vxFL) - control.steerFL;
alphaFR = atan2(vyFR, vxFR) - control.steerFR;
alphaRL = atan2(vyRL, vxRL);
alphaRR = atan2(vyRR, vxRR);

% Linear tyre forces with saturation to avoid unrealistic magnitudes
FyFL = saturateLateralForce(-Cf * alphaFL, params, control.steerFL, vxFL, vyFL);
FyFR = saturateLateralForce(-Cf * alphaFR, params, control.steerFR, vxFR, vyFR);
FyRL = saturateLateralForce(-Cr * alphaRL, params, 0,      vxRL, vyRL);
FyRR = saturateLateralForce(-Cr * alphaRR, params, 0,      vxRR, vyRR);

FxFL = control.FxFL;
FxFR = control.FxFR;
FxRL = control.FxRL;
FxRR = control.FxRR;

% Enforce friction ellipse constraint
[FyFL, FxFL] = frictionEllipse(FyFL, FxFL, params.mu, params.weightF / 2);
[FyFR, FxFR] = frictionEllipse(FyFR, FxFR, params.mu, params.weightF / 2);
[FyRL, FxRL] = frictionEllipse(FyRL, FxRL, params.mu, params.weightR / 2);
[FyRR, FxRR] = frictionEllipse(FyRR, FxRR, params.mu, params.weightR / 2);

% Summation of forces and moments
sumFx = FxFL + FxFR + FxRL + FxRR - Fdrag - Froll;
sumFy = FyFL + FyFR + FyRL + FyRR;

% Longitudinal and lateral equations of motion in body frame
udot = r * v + sumFx / m;
vdot = -r * u + sumFy / m;

% Yaw dynamics including both lateral and longitudinal force couples
momYaw = a * (FyFL + FyFR) - b * (FyRL + FyRR) ...
    + halfTrackF * (FxFR - FxFL) + halfTrackR * (FxRR - FxRL);
rdot = momYaw / Iz;

% Kinematic equations in global frame
Xdot = u * cos(psi) - v * sin(psi);
Ydot = u * sin(psi) + v * cos(psi);
psidot = r;

dx = [Xdot; Ydot; psidot; udot; vdot; rdot];
end

function [vx, vy] = wheelVelocity(u, v, r, xRel, yRel)
%WHEELVELOCITY Transform body velocities to wheel center velocities.
    vx = u - r * yRel;
    vy = v + r * xRel;
end

function Fy = saturateLateralForce(Fy, params, steer, vx, vy)
%SATURATELATERALFORCE Limit lateral force to friction circle estimate.
    Fz = params.m * params.g / 4; % assume static load per wheel
    FyMax = params.mu * Fz;
    Fy = max(min(Fy, FyMax), -FyMax);

    % Prevent numerical issues when wheel speed is near zero
    if hypot(vx, vy) < 0.1
        Fy = Fy * hypot(vx, vy) / 0.1;
    end
end

function [FySat, FxSat] = frictionEllipse(Fy, Fx, mu, Fz)
%FRICTIONELLIPSE Project combined force demands onto friction ellipse.
    limit = mu * Fz;
    if limit <= 0
        FySat = Fy;
        FxSat = Fx;
        return;
    end
    forceNorm = hypot(Fx / limit, Fy / limit);
    if forceNorm <= 1
        FySat = Fy;
        FxSat = Fx;
    else
        scale = 1 / forceNorm;
        FxSat = Fx * scale;
        FySat = Fy * scale;
    end
end
