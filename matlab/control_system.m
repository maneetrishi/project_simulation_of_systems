%% Digital Control of a Discrete-Time System
% Task 3: DLBENGPSS01 - Final Clean Script
% 5 figures: open-loop step, open-loop pzmap, closed-loop step,
%            closed-loop pzmap, sampling time effect
close all; clc;

%% System Parameters
Ts = 0.1;
N  = 100;
t_end = N * Ts;

%% Plant Definition
num = 0.2;
den = [1 -0.8];
H = tf(num, den, Ts);
H.TimeUnit = 'seconds';

%% FIGURE 1: Open-Loop Step Response using dstep
% dstep requires polynomials in ascending powers of z^-1
num_dstep = [0  0.2];
den_dstep = [1 -0.8];
y_ol = dstep(num_dstep, den_dstep, N);
t_ol = (0:N-1)' * Ts;

figure('Name','Fig1: Open-loop Step');
stairs(t_ol, y_ol, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
title('Open-Loop Step Response');
xlabel('Time (s)'); ylabel('Output y[k]');
grid on; ylim([0 1.2]);

%% FIGURE 2: Open-Loop Pole-Zero Map
figure('Name','Fig2: OL Pole-Zero Map');
pzmap(H);
title('Open-Loop Pole-Zero Map'); grid on;
axis([-1.5 1.5 -1.5 1.5]);

% Analytical stability check
p_ol = pole(H);
disp('=== Open-Loop Pole ===');
disp(p_ol);
fprintf('Magnitude: %.4f\n', abs(p_ol));
if abs(p_ol) < 1
    disp('Open-loop system is ASYMPTOTICALLY STABLE (|z| < 1).');
else
    disp('Open-loop system is UNSTABLE.');
end

%% Discrete PID Controller Design
Kp = 8.0;
Ki = 2.0;
Kd = 0.1;

C = pid(Kp, Ki, Kd);
C.Ts = Ts;
C.TimeUnit = 'seconds';

disp(' ');
disp('=== Discrete PID Controller C(z) ===');
disp(C);

%% Closed-Loop System
CL = feedback(series(C, H), 1);

%% FIGURE 3: Closed-Loop Step Response
figure('Name','Fig3: CL Step');
step(CL, t_end);
title('Closed-Loop Step Response with Improved PID');
xlabel('Time (s)'); ylabel('Output y[k]');
grid on;

%% FIGURE 4: Closed-Loop Pole-Zero Map
figure('Name','Fig4: CL Pole-Zero Map');
pzmap(CL);
title('Closed-Loop Pole-Zero Map'); grid on;
axis([-1.5 1.5 -1.5 1.5]);

% Stability check
p_cl = pole(CL);
p_cl_mag = abs(p_cl);
stable = all(p_cl_mag < 1);

disp(' ');
disp('=== Closed-Loop Poles ===');
disp(p_cl);
disp('Pole magnitudes:'); disp(p_cl_mag);
if stable
    disp('Closed-loop system is STABLE: all poles inside unit circle.');
else
    disp('WARNING: Closed-loop system is UNSTABLE.');
end

%% Performance Metrics (Explicit 2% Settling Time Criterion)
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

%% FIGURE 5: Sampling Time Effect (Ts = 0.1 vs 0.05)
Ts2 = 0.05;
H2 = tf(num, den, Ts2);
C2 = pid(Kp, Ki, Kd);
C2.Ts = Ts2;
CL2 = feedback(series(C2, H2), 1);

[y1, t1] = step(CL, 0:Ts:t_end);
[y2, t2] = step(CL2, 0:Ts2:(N*Ts2));

figure('Name','Fig5: Sampling Time');
plot(t1, y1, 'b-', 'LineWidth', 1.5); hold on;
plot(t2, y2, 'r--', 'LineWidth', 1.5); hold off;
legend('T_s = 0.1 s', 'T_s = 0.05 s', 'Location', 'best');
title('Effect of Sampling Time on Closed-Loop Response');
xlabel('Time (s)'); ylabel('Output y[k]'); grid on;
xlim([0 3]); ylim([0 1.5]);

disp(' ');
disp('=== All 5 figures generated successfully ===');
