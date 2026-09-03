% pid_cascade_sim.m
% Cascade PID attitude controller on the nonlinear quadcopter model.
% Outer loop (angle) generates a rate reference; inner loop (rate) generates
% body torque. Run directly or through run_all_simulations.

function pid_cascade_sim()
    p = quadcopter_parameters();
    dt = p.dt;
    tEnd = 8.0;
    N = round(tEnd / dt);
    time = (0:N-1) * dt;

    x = zeros(12, 1);

    % Outer angle loop -> rate reference
    KpAngle = 4.5;
    maxRateRef = 3.0;

    % Inner rate loop -> torque
    KpRate = 0.25;
    KiRate = 0.45;
    KdRate = 0.004;
    maxInt = 0.15;

    eulerHist = zeros(3, N);
    rateHist  = zeros(3, N);
    tauHist   = zeros(3, N);
    cmdHist   = zeros(3, N);

    rateInt = zeros(3, 1);
    prevRateErr = zeros(3, 1);

    for k = 1:N
        t = time(k);
        cmd = attitudeCommand(t);
        ang  = x(7:9);
        rate = x(10:12);

        angErr   = wrap_to_pi(cmd - ang);
        rateRef  = KpAngle * angErr;
        rateRef  = min(max(rateRef, -maxRateRef), maxRateRef);

        rateErr  = rateRef - rate;
        rateInt  = rateInt + rateErr * dt;
        rateInt  = min(max(rateInt, -maxInt), maxInt);
        rateDer  = (rateErr - prevRateErr) / dt;

        tau = KpRate * rateErr + KiRate * rateInt + KdRate * rateDer;
        tau = min(max(tau, -p.maxTau), p.maxTau);

        % Disturbance torque, common with the LQR experiment
        if (t >= 5.0) && (t < 5.05)
            tau = tau + [0.25; -0.10; 0.10];
        end

        thrust = p.m * p.g;  % hover trim; altitude loop added later

        x = x + quadcopter_dynamics(x, thrust, tau, p) * dt;

        eulerHist(:, k) = ang;
        rateHist(:, k)  = rate;
        tauHist(:, k)   = tau;
        cmdHist(:, k)   = cmd;
        prevRateErr = rateErr;
    end

    plotResults(time, eulerHist, rateHist, tauHist, cmdHist);
end

function cmd = attitudeCommand(t)
    cmd = [0.25 * (t >= 1.0);
          -0.20 * (t >= 2.0);
           0.50 * (t >= 3.0)];
end

function plotResults(time, eulerHist, rateHist, tauHist, cmdHist)
    figure('Name', 'Cascade PID simulation');

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

    figure('Name', 'Cascade PID torques');
    plot(time, tauHist');
    legend('tau_x', 'tau_y', 'tau_z');
    xlabel('time (s)'); ylabel('torque (N*m)'); grid on;
end
