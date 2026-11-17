# Quadcopter-Carried Inverted Pendulum: Swing-Up and Stabilization Control

MATLAB/Simulink model and controller for a planar quadcopter carrying a pendulum pinned at its centre. The controller swings the pendulum up from hanging and balances it upright while the drone holds or tracks a position setpoint.

## Model

The drone (mass M, inertia J, tilt β, two rotor pairs with first-order motor lag and thrust limits) and the pendulum (point mass m, rod length L) are coupled through Lagrangian equations of motion with generalized coordinates q = [x, y, θ]:

```
(M+m) x'' + mL (cos θ θ'' − sin θ θ'²) = −F sin β
(M+m) y'' + mL (sin θ θ'' + cos θ θ'²) =  F cos β − (M+m) g
mL² θ''   + mL (cos θ x'' + sin θ y'')  = −mgL sin θ − b(θ' − β') + d
J β''                                   =  l_arm (T2 − T1) + b(θ' − β')
```

State vector: `[θ, ω, x, y, vx, vy, β, β']`

## Control

A cascaded controller:
- energy-based swing-up of the pendulum
- pole-placement balancing around the upright position
- attitude control of the drone tilt
- thrust allocation to the two rotor pairs

Robustness was checked under sensor noise and external pushes on the mass.

## Files

| File | Purpose |
|---|---|
| `pendulum_params.m` | Physical and controller parameters |
| `build_pendulum_model.m` | Builds `pendulum.slx` programmatically |
| `pendulum.slx` | The Simulink model (rebuilt by the script above if missing) |
| `run_pendulum.m` | Runs a simulation, plots the states and animates the result |
| `animate_pendulum.m` | Playback animation with play/replay/speed controls |
| `live_pendulum.m` | Real-time interactive run: move the setpoint with the arrow keys and push the mass with A/D |

## Run

```matlab
run_pendulum     % simulate, plot and animate
live_pendulum    % interactive real-time control
```

Requires MATLAB and Simulink.
