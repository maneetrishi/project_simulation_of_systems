%% Digital Control of a Discrete-Time System
% Task 3: DLBENGPSS01
% Clear workspace
clear; clc; close all;

%% System Parameters
Ts = 0.1;           % Sampling time [s]
N  = 100;           % Number of samples for dstep
t_end = N * Ts;     % Simulation end time [s]

%% Plant Definition
% Difference equation: y[k+1] = 0.8*y[k] + 0.2*u[k]
% Transfer function:   G(z) = 0.2 / (z - 0.8)
num = 0.2;
den = [1 -0.8];

% Create discrete LTI system object (modern equivalent of "dlti")
H = tf(num, den, Ts);
H.TimeUnit = 'seconds';

%% 1. Open-Loop Response using dstep (legacy)
% dstep requires polynomials in ascending powers of z^-1:
% 0.2/(z-0.8) = (0.2*z^-1) / (1 - 0.8*z^-1)
num_dstep = [0  0.2];   % 0 + 0.2*z^-1
den_dstep = [1 -0.8];   % 1 - 0.8*z^-1

y_ol = dstep(num_dstep, den_dstep, N);   % Only capture y
t_ol = (0:N-1)' * Ts;                    % Generate time vector manually

figure('Name','Fig1: Open-loop dstep');
stairs(t_ol, y_ol, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
title('Open-Loop Step Response using dstep')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;
ylim([0 1.2]);

% Modern step response for report figure
figure('Name','Fig1b: Open-loop Step');
step(H, t_end);
title('Open-Loop Step Response')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;

%% 2. Open-Loop Pole-Zero Map
figure('Name','FigPZ OL');
pzmap(H);
title('Open-Loop Pole-Zero Map')
grid on;
axis([-1.5 1.5 -1.5 1.5]);  


% Analytical stability check
p_ol = pole(H);
disp('--- Open-Loop Pole ---');
disp(p_ol);
disp(['Magnitude: ', num2str(abs(p_ol))]);
if abs(p_ol) < 1
    disp('Open-loop system is ASYMPTOTICALLY STABLE (|z| < 1).');
else
    disp('Open-loop system is UNSTABLE.');
end

%% 3. Discrete PID Controller Design
Kp = 8.0;
Ki = 2.0;
Kd = 0.1;

% Discrete PID controller (parallel form)
C = pid(Kp, Ki, Kd);
C.Ts = Ts;
C.TimeUnit = 'seconds';

disp(' ');
disp('--- Discrete PID Controller C(z) ---');
disp(C);

%% 4. Closed-Loop System
G  = series(C, H);      % Controller + Plant
CL = feedback(G, 1);    % Unity negative feedback

%% 4. Closed-Loop dstep
[num_cl, den_cl] = tfdata(CL, 'v');
n_den = length(den_cl);
n_num = length(num_cl);
num_cl_dstep = [zeros(1, n_den - n_num), num_cl];  % Pad leading zeros
den_cl_dstep = den_cl;

y_cl = dstep(num_cl_dstep, den_cl_dstep, N);   % Only capture y
t_cl = (0:N-1)' * Ts;                          % Generate time vector manually

figure('Name','Fig2: Closed-loop dstep');
stairs(t_cl, y_cl, 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980]);
title('Closed-Loop Step Response with PID using dstep')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;

% Modern step for report
figure('Name','Fig2b: Closed-loop Step');
step(CL, t_end);
title('Closed-Loop Step Response with Improved PID')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;

%% 5. Closed-Loop Pole-Zero Map
figure('Name','FigPZ CL');
pzmap(CL);
title('Closed-Loop Pole-Zero Map')
grid on;
axis([-1.5 1.5 -1.5 1.5]);   


% Stability check
p_cl = pole(CL);
p_cl_mag = abs(p_cl);
stable = all(p_cl_mag < 1);

disp(' ');
disp('--- Closed-Loop Poles ---');
disp(p_cl);
disp('Pole magnitudes:');
disp(p_cl_mag);
if stable
    disp('Closed-loop system is STABLE: all poles inside unit circle.');
else
    disp('WARNING: Closed-loop system is UNSTABLE.');
end

%% 6. Performance Metrics (Explicit 2% Settling Time Criterion)
S_ol = stepinfo(H, 'SettlingTimeThreshold', 0.02);
S_cl = stepinfo(CL, 'SettlingTimeThreshold', 0.02);

disp(' ');
disp('==================================================');
disp('  OPEN-LOOP METRICS (2% Settling Time Criterion) ');
disp('==================================================');
fprintf('Rise Time      : %.4f s\n', S_ol.RiseTime);
fprintf('Settling Time  : %.4f s\n', S_ol.SettlingTime);
fprintf('Overshoot      : %.2f %%\n', S_ol.Overshoot);
fprintf('Peak Amplitude : %.4f\n', S_ol.Peak);
fprintf('Peak Time      : %.4f s\n', S_ol.PeakTime);
fprintf('Steady-State   : %.4f\n', dcgain(H));

disp(' ');
disp('===================================================');
disp(' CLOSED-LOOP METRICS (2% Settling Time Criterion) ');
disp('===================================================');
fprintf('Rise Time      : %.4f s\n', S_cl.RiseTime);
fprintf('Settling Time  : %.4f s\n', S_cl.SettlingTime);
fprintf('Overshoot      : %.2f %%\n', S_cl.Overshoot);
fprintf('Peak Amplitude : %.4f\n', S_cl.Peak);
fprintf('Peak Time      : %.4f s\n', S_cl.PeakTime);
fprintf('Steady-State   : %.4f\n', dcgain(CL));

%% 7. Sampling-Time Effect (Ts = 0.1 s vs Ts = 0.05 s)
Ts2 = 0.05;
N2  = 200;              % Double samples to maintain duration
t_end2 = N2 * Ts2;

% Plant and controller at finer sampling
H2 = tf(num, den, Ts2);
H2.TimeUnit = 'seconds';

C2 = pid(Kp, Ki, Kd);
C2.Ts = Ts2;
C2.TimeUnit = 'seconds';

CL2 = feedback(series(C2, H2), 1);

% Common time vector for clean overlay
t_common = 0:0.01:min(t_end, t_end2);

[y1, ~] = step(CL, t_common);
[y2, ~] = step(CL2, t_common);

figure('Name','Fig3: Sampling Time Effect');
plot(t_common, y1, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_common, y2, 'r--', 'LineWidth', 1.5);
hold off;
legend('T_s = 0.1 s', 'T_s = 0.05 s', 'Location', 'best');
title('Effect of Sampling Time on Closed-Loop Response');
xlabel('Time (s)');
ylabel('Output y[k]');
grid on;
xlim([0 3]);        % Focus on transient region
ylim([0 1.5]);

%% 8. Export Figures for Report (optional)
% saveas(findobj('Name','Fig1b: Open-loop Step'), 'fig1_open_loop.png');
% saveas(findobj('Name','Fig2b: Closed-loop Step'), 'fig2_closed_loop.png');
% saveas(findobj('Name','Fig3: Sampling Time Effect'), 'fig3_sampling_time.png');
% saveas(findobj('Name','FigPZ OL'), 'figpz_open_loop.png');
% saveas(findobj('Name','FigPZ CL'), 'figpz_closed_loop.png');

%% 9. Simulink Verification Checklist
disp(' ');
disp('===================================================');
disp('  SIMULINK VERIFICATION CHECKLIST');
disp('===================================================');
disp('1. Use "Discrete PID Controller" block (NOT continuous PID)');
disp('2. Controller: Parallel form, Kp=8, Ki=2, Kd=0.1');
disp('   Sample time = 0.1, Discrete-time domain');
disp('3. Plant: "Discrete Transfer Fcn" block');
disp('   Numerator [0.2], Denominator [1 -0.8], Sample time = 0.1');
disp('4. Sum block: List of signs = |+-  (negative feedback)');
disp('5. Step input: Step time = 0, Final value = 1');
disp('6. Scope should show fast rise, overshoot, and settling to 1.0');
disp('   If output slowly reaches ~0.83, the feedback loop is broken');
disp('   or the PID block is running in continuous mode.');
disp('===================================================');
