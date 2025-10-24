function params = vehicleParameters()
%VEHICLEPARAMETERS Returns a struct with canonical C-segment passenger car parameters.
%
%   The parameter values are drawn from well-established vehicle dynamics
%   references such as Rajamani ("Vehicle Dynamics and Control", 2012) and
%   Wong ("Theory of Ground Vehicles", 2001), along with empirical data
%   shared by open-source vehicle modeling projects like the University of
%   Waterloo's UWVehicleDynamics (GitHub, 2023). These sources outline
%   representative masses, inertia properties, and tire cornering stiffness
%   suitable for mid-size passenger vehicles.
%
%   Output structure fields
%       m         - Vehicle mass [kg]
%       Iz        - Yaw moment of inertia about the CG [kg*m^2]
%       a         - Distance from CG to front axle [m]
%       b         - Distance from CG to rear axle [m]
%       trackF    - Front track width [m]
%       trackR    - Rear track width [m]
%       Cf        - Per-wheel cornering stiffness front [N/rad]
%       Cr        - Per-wheel cornering stiffness rear [N/rad]
%       mu        - Tire-road friction coefficient [-]
%       g         - Gravitational acceleration [m/s^2]
%       rho       - Air density [kg/m^3]
%       Cd        - Aerodynamic drag coefficient [-]
%       Af        - Vehicle frontal area [m^2]
%       Cr0       - Rolling resistance coefficient [-]
%       wheelBase - Wheelbase length [m]
%       weightF   - Static front axle load [N]
%       weightR   - Static rear axle load [N]
%       maxSteer  - Maximum steering command magnitude [rad]
%       maxDrive  - Maximum total drive force available [N]

params.m = 1575;                 % kg
params.Iz = 2875;                % kg*m^2
params.a = 1.2;                  % m (front axle to CG)
params.b = 1.65;                 % m (rear axle to CG)
params.trackF = 1.55;            % m
params.trackR = 1.55;            % m
params.Cf = 80000;               % N/rad (per wheel)
params.Cr = 80000;               % N/rad (per wheel)
params.mu = 0.95;                % asphalt tire-road friction
params.g = 9.81;                 % m/s^2
params.rho = 1.225;              % kg/m^3
params.Cd = 0.32;                % aerodynamic drag coefficient
params.Af = 2.2;                 % m^2 frontal area
params.Cr0 = 0.015;              % rolling resistance coefficient
params.wheelBase = params.a + params.b;
params.weightF = params.m * params.g * params.b / params.wheelBase;
params.weightR = params.m * params.g * params.a / params.wheelBase;
params.maxSteer = deg2rad(35);
params.maxDrive = 8000;          % N, combined at the wheels
params.sampleTime = 0.01;        % s, used by discrete driver controller logic
end
