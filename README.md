# Project: Simulation of Systems (DLBENGPSS01)

MATLAB and Simulink implementation of a discrete-time PID control system for Task 3: Digital Control of a Discrete-Time System.

## Project Overview

This project implements and analyses a discrete-time closed-loop control system using a PID controller. The plant is modelled as a first-order discrete-time transfer function derived from the difference equation y[k+1] = 0.8y[k] + 0.2u[k], and the controller is designed and tuned in MATLAB, then verified in Simulink.

**Plant model:**

$$
G(z) = \frac{0.2}{z - 0.8}
$$

**Controller:** Discrete PID with:
- Proportional gain Kp = 6.0
- Integral gain Ki = 1.5
- Derivative gain Kd = 0
- Sample time Ts = 0.1 s

## Repository Structure

```
project-simulation-of-system/
├── matlab/
│   └── control_system.m       # Main MATLAB script
├── simulink/
│   └── control_model.slx      # Simulink block diagram
├── figures/
│   ├── fig1_open_loop_step.jpg           # Open-loop step response
│   ├── fig2_open_loop_pzmap.jpg          # Open-loop pole-zero map
│   ├── fig3_closed_loop_pid.jpg          # Closed-loop step response
│   ├── fig4_closed_loop_pzmap.jpg        # Closed-loop pole-zero map
│   ├── fig5_sampling_time_effect.jpg     # Sampling time comparison
│   ├── fig6_simulink_block_diagram.jpg   # Simulink block diagram
│   └── fig7_simulink_output.jpg          # Simulink scope output
└── README.md
```

## How to Run

### MATLAB
1. Open MATLAB (R2021a or later recommended).
2. Navigate to the `matlab/` folder.
3. Run `control_system.m` in the MATLAB editor or command window.
4. The script generates 5 figures automatically:
   - Figure 1: Open-loop step response (using `dstep`)
   - Figure 2: Open-loop pole-zero map
   - Figure 3: Closed-loop step response with PID (using `step`)
   - Figure 4: Closed-loop pole-zero map
   - Figure 5: Effect of sampling time (Ts = 0.1 s vs Ts = 0.05 s)

### Simulink
1. Open MATLAB and navigate to the `simulink/` folder.
2. Open `control_model.slx`.
3. Click **Run** to simulate.
4. Open the **Scope** block to view the closed-loop output.

## Results Summary

| Figure | Description |
|--------|-------------|
| Figure 1 | Open-loop step response of the plant (using `dstep`) |
| Figure 2 | Open-loop pole-zero map showing pole at z = 0.8 |
| Figure 3 | Closed-loop step response with improved PID controller |
| Figure 4 | Closed-loop pole-zero map confirming stability |
| Figure 5 | Effect of sampling time (Ts = 0.1 s vs Ts = 0.05 s) |
| Figure 6 | Simulink closed-loop block diagram |
| Figure 7 | Simulink closed-loop step response output |

## Key Findings

- The open-loop system is asymptotically stable with a single pole at z = 0.8 (|0.8| < 1), exhibiting a monotonic step response that settles to 1.0 without overshoot.
- The discrete PID controller (Kp = 6.0, Ki = 1.50, Kd = 0) significantly improves transient response, achieving fast rise time with approximately 21% overshoot and settling to 1.0.
- All closed-loop poles lie strictly inside the unit circle, confirming closed-loop stability.
- Reducing the sampling time from 0.1 s to 0.05 s reduces overshoot (from ~1.21 to ~1.06) and transient oscillations, consistent with digital control theory.
- The Simulink model validates the feedback loop structure; differences between MATLAB and Simulink responses are attributed to internal discretization method variations in Simulink's PID block.

## Tools Used

- MATLAB
- Simulink

