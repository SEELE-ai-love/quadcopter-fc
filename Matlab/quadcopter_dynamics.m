% quadcopter_dynamics.m
% Nonlinear 12-state dynamics of a quadcopter.
% State: [pn pe pd vn ve vd phi theta psi p q r]
%   p/v : NED position/velocity
%   phi/theta/psi : Euler angles (Z-Y-X)
%   p q r : body angular rates
% Inputs: thrust (N, along body z) and tau (N*m, body frame).

function xdot = quadcopter_dynamics(x, thrust, tau, p)
    v = x(4:6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    omega = x(10:12);

    R = rotZYX(psi, theta, phi);          % body -> NED
    % Rotor thrust points along -z_body (up when level), so hover trim is m*g.
    acc = [0; 0; p.g] - R * [0; 0; thrust / p.m];

    euler_dot = eulerRateMatrix(phi, theta) * omega;

    I = diag([p.Ixx, p.Iyy, p.Izz]);
    omega_dot = I \ (tau - cross(omega, I * omega));

    xdot = [v; acc; euler_dot; omega_dot];
end

function R = rotZYX(psi, theta, phi)
    cp = cos(phi);   sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);
    R = [cpsi*ct,              cpsi*st*sp - spsi*cp,  cpsi*st*cp + spsi*sp;
         spsi*ct,              spsi*st*sp + cpsi*cp,  spsi*st*cp - cpsi*sp;
         -st,                  ct*sp,                 ct*cp];
end

function W = eulerRateMatrix(phi, theta)
    cp = cos(phi); sp = sin(phi);
    tt = tan(theta); ct = cos(theta);
    W = [1,  sp*tt,  cp*tt;
         0,  cp,    -sp;
         0,  sp/ct,  cp/ct];
end
