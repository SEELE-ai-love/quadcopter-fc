% lqr_sim.m
% LQR attitude controller on the same nonlinear model used by pid_cascade_sim.
% The feedback gain is designed on the small-angle linearization at hover.

function lqr_sim()
    p = quadcopter_parameters();
    dt = p.dt;
    tEnd = 8.0;
    N = round(tEnd / dt);
    time = (0:N-1) * dt;

    % State used for feedback: [phi theta psi p q r]
    A = [zeros(3), eye(3);
         zeros(3), zeros(3)];
    B = [zeros(3); inv(diag([p.Ixx, p.Iyy, p.Izz]))];

    % Weights chosen so the torque stays within the model authority for a
    % ~0.25 rad step, comparable with the cascade PID experiment.
    Q = diag([2.5, 2.5, 1.5, 1, 1, 1]);   % angle and rate weights
    R = diag([0.25, 0.25, 0.40]);         % torque cost
    try
        K = lqr(A, B, Q, R);   % requires Control System Toolbox
    catch
        % Toolbox-free fallback: solve the scalar double-integrator LQR for
        % each decoupled axis analytically (the linear model is block
        % diagonal with this A/B structure).
        K = lqrDecoupled(p, Q, R);
    end

    x = zeros(12, 1);

    eulerHist = zeros(3, N);
    rateHist  = zeros(3, N);
    tauHist   = zeros(3, N);
    cmdHist   = zeros(3, N);

    for k = 1:N
        t = time(k);
        cmd = attitudeCommand(t);
        ang  = x(7:9);
        rate = x(10:12);

        angErr = wrap_to_pi(cmd - ang);
        z = [angErr; rate];
        tau = -K * z;
        tau = min(max(tau, -p.maxTau), p.maxTau);

        if (t >= 5.0) && (t < 5.05)
            tau = tau + [0.25; -0.10; 0.10];
        end

        thrust = p.m * p.g;
        x = x + quadcopter_dynamics(x, thrust, tau, p) * dt;

        eulerHist(:, k) = ang;
        rateHist(:, k)  = rate;
        tauHist(:, k)   = tau;
        cmdHist(:, k)   = cmd;
    end

    fprintf('LQR gain K:\n');
    disp(K);
    plotResults(time, eulerHist, rateHist, tauHist, cmdHist);
end

function K = lqrDecoupled(p, Q, R)
    % For one decoupled axis: xdot = [0 1; 0 0] x + [0; b] u with
    % Q = diag(q1,q2), R = r. The positive Riccati solution gives
    % k1 = sqrt(q1/r), k2 = (1 + sqrt(1 + q2*b^2/r)) / b.
    K = zeros(3, 6);
    b = [1 / p.Ixx; 1 / p.Iyy; 1 / p.Izz];
    for i = 1:3
        q1 = Q(i, i);
        q2 = Q(3 + i, 3 + i);
        r  = R(i, i);
        K(i, i) = sqrt(q1 / r);
        K(i, 3 + i) = (1 + sqrt(1 + q2 * b(i)^2 / r)) / b(i);
    end
end

function cmd = attitudeCommand(t)
    cmd = [0.25 * (t >= 1.0);
          -0.20 * (t >= 2.0);
           0.50 * (t >= 3.0)];
end

function plotResults(time, eulerHist, rateHist, tauHist, cmdHist)
    figure('Name', 'LQR simulation');

    subplot(3, 1, 1);
    plot(time, rad2deg(cmdHist(1, :)), '--', time, rad2deg(eulerHist(1, :)));
    legend('roll cmd', 'roll');
    ylabel('roll (deg)'); grid on;

    subplot(3, 1, 2);
    plot(time, rad2deg(cmdHist(2, :)), '--', time, rad2deg(eulerHist(2, :)));
    legend('pitch cmd', 'pitch');
    ylabel('pitch (deg)'); grid on;

    subplot(3, 1, 3);
    plot(time, rad2deg(cmdHist(3, :)), '--', time, rad2deg(eulerHist(3, :)));
    legend('yaw cmd', 'yaw');
    xlabel('time (s)'); ylabel('yaw (deg)'); grid on;

    figure('Name', 'LQR torques');
    plot(time, tauHist');
    legend('tau_x', 'tau_y', 'tau_z');
    xlabel('time (s)'); ylabel('torque (N*m)'); grid on;
end
