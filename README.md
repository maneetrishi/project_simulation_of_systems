# Project: Simulation of System

MATLAB and Simulink implementation of a discrete-time PID control system for a control systems simulation project.

## Project Overview

This project implements and analyses a discrete-time closed-loop control system using a PID controller. The plant is modelled as a first-order discrete-time transfer function, and the controller is designed and tuned in MATLAB, then verified in Simulink.

**Plant model:**

$$
G(z) = \frac{0.2}{z - 0.8}
$$

**Controller:** Discrete PID with:
- Proportional gain P = 1
- Integral gain I = 0.2
- Derivative gain D = 0
- Sample time Ts = 0.1 s

## Repository Structure

```
project-simulation-of-system/
├── matlab/
│   └── control_system.m       # Main MATLAB script
├── simulink/
│   └── control_model.slx      # Simulink block diagram
├── figures/
│   ├── fig1_open_loop_response.jpg
│   ├── fig2_closed_loop_pid_response.jpg
│   ├── fig3_sampling_time_effect.jpg
│   ├── fig4_simulink_block_diagram.jpg
│   └── fig5_simulink_output.jpg
└── README.md
```

## How to Run

### MATLAB
1. Open MATLAB (R2021a or later recommended).
2. Navigate to the `matlab/` folder.
3. Run `control_system.m` in the MATLAB editor or command window.
4. The figures will be generated automatically.

### Simulink
1. Open MATLAB and navigate to the `simulink/` folder.
2. Open `control_model.slx`.
3. Click **Run** to simulate.
4. Open the **Scope** block to view the closed-loop output.

## Results Summary

| Figure | Description |
|--------|-------------|
| Figure 1 | Open-loop step response of the plant |
| Figure 2 | Closed-loop step response with improved PID |
| Figure 3 | Effect of sampling time (Ts = 0.1 s vs Ts = 0.05 s) |
| Figure 4 | Simulink closed-loop block diagram |
| Figure 5 | Simulink closed-loop step response output |

## Key Findings

- The open-loop system reaches steady state around 2 to 3 seconds with no overshoot.
- Adding a discrete PID controller improves the transient response, with the output tracking the unit step input closely and settling near 1.0.
- Reducing the sampling time from 0.1 s to 0.05 s produces a slightly smoother response with reduced transient oscillations.
- The Simulink model confirms the MATLAB results with a stable, bounded closed-loop response.

## Tools Used

- MATLAB R2021a or later
- Simulink
