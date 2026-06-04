clear; clc; close all;

%% System parameters
Ts = 0.1;
N = 100;

% Plant: H(z) = 0.2 / (z - 0.8)
num = 0.2;
den = [1 -0.8];
H = tf(num, den, Ts);

%% Open-loop response
figure(1)
step(H, N*Ts);
title('Open-loop Step Response')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;

S_ol = stepinfo(H);

%% Better PID gains
Kp = 8.0;
Ki = 2.0;
Kd = 0.1;

C = pid(Kp, Ki, Kd);
C.Ts = Ts;
C.TimeUnit = 'seconds';

%% Closed-loop system
G = series(C, H);
CL = feedback(G, 1);

figure(2)
step(CL, N*Ts);
title('Closed-loop Step Response with Improved PID')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;

S_cl = stepinfo(CL);

%% Pole-zero map
figure(3)
pzmap(CL);
title('Pole-Zero Map of Closed-loop System')
grid on;

%% Stability check
p = pole(CL);
p_mag = abs(p);
stable = all(p_mag < 1);

disp('Open-loop metrics:')
disp(S_ol)

disp('Closed-loop metrics:')
disp(S_cl)

disp('Closed-loop poles:')
disp(p)

disp('Pole magnitudes:')
disp(p_mag)

if stable
    disp('System is stable: all poles are inside the unit circle.')
else
    disp('System is unstable: at least one pole is outside the unit circle.')
end

%% Sampling-time effect
Ts2 = 0.05;

H2 = tf(num, den, Ts2);

C2 = pid(Kp, Ki, Kd);
C2.Ts = Ts2;
C2.TimeUnit = 'seconds';

CL2 = feedback(series(C2, H2), 1);

figure(4)
step(CL, N*Ts);
hold on;
step(CL2, N*Ts2);
hold off;
legend('Ts = 0.1 s', 'Ts = 0.05 s')
title('Effect of Sampling Time on Closed-loop Response')
xlabel('Time (s)')
ylabel('Output y[k]')
grid on;