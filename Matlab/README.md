# Matlab/README

## MATLAB simulation

Base MATLAB only (no extra toolbox required). Add this folder to the MATLAB path, then run:

```matlab
run_all_simulations
```

Files:

- `quadcopter_parameters.m`: mass, inertia, thrust authority and control period.
- `quadcopter_dynamics.m`: nonlinear 12-state dynamics (NED position/velocity + Euler angles + body rates).
- `pid_cascade_sim.m`: cascade PID attitude control, plots angle tracking and torques.
- `lqr_sim.m`: LQR gain designed on the hover linearization, same disturbance as PID for comparison.
- `run_all_simulations.m`: runs both experiments.

The parameters are first-version estimates. After the real airframe is measured, update `quadcopter_parameters.m` and keep the change in git history.
