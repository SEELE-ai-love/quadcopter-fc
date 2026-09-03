% quadcopter_parameters.m
% Returns physical/control parameters for the simulation.
% The values below are starting estimates; replace them with measured
% data after the real airframe is built.

function p = quadcopter_parameters()
    p.g     = 9.80665;   % gravity, m/s^2
    p.m     = 1.00;      % total mass, kg
    p.arm   = 0.18;      % rotor arm, m
    p.Ixx   = 0.0085;    % roll inertia, kg*m^2
    p.Iyy   = 0.0085;    % pitch inertia, kg*m^2
    p.Izz   = 0.0140;    % yaw inertia, kg*m^2
    p.maxT  = 2.4 * p.m * p.g; % total thrust authority, N
    p.maxTau = 1.0;      % per-axis torque authority, N*m
    p.dt    = 0.002;     % control period, s
end
