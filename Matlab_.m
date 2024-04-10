clear; clc;
close all
% Resistance (ohm)
Rm = 8.4;
% Inductance (H)
Lm = 1.16e-3;
% Current-torque (N-m/A)
kt = 0.042;
% Back-emf constant (V-s/rad)
km = 0.042;
% Rotor inertia (kg-m^2)
Jr = 4e-6;
% Hub mass (kg)
mh = 0.0106; % 9 g
% Hub radius (m)
rh = 22.2/1000/2; % diameter 22.2 mm
% Hub inertia (kg-m^2)
Jh = 0.5*mh*rh^2;
% Disc mass (kg)
md = 0.053;
% Disc radius (m)
rd = 49.5/1000/2; % diameter = 49.5 mm
% Disc moment of inertia (kg-m^2)
Jd = 0.5*md*rd^2;
% Equivalent moment of inertia (kg-m^2)
Jeq = Jr + Jh + Jd;

syms s


A = [0 1; 
    0 -10.05];
B = [0; 239.4];
C = [1 0];
D = [0];

L = [17; 4.3];

[a1,b1] = ss2tf(A, B, C, D);

lol = tf(a1, b1);

%% Aim for 20% overshoot and a setteling time of 5s

OS = 5;
T_s = 2;

zeta = log(OS/100)/sqrt(pi^2+(log(OS/100))^2);
omega_n = 4/(zeta*T_s);

s1 = -zeta*omega_n + j*omega_n*sqrt(1-zeta^2);
s2 = -zeta*omega_n - j*omega_n*sqrt(1-zeta^2);
s3 = -zeta*omega_n*5 


desired_poles = [s1, s2];
K = place(A, B, desired_poles)
K1 = K(1);
KK = [0 K(2)];

A_cl = (A-B*K);
sys_cl = ss(A_cl, B, C, D);
eig(A_cl);

out = sim("Simulink_work");

sp = get(out,"set_point");
y = get(out,'position');
t1 = get(out,"time_data");

figure
hold on; grid on;
plot(t1, sp, t1, y, 'linewidth', 2)
legend('set point', 'angular position')

stepinfo(y)
sse = abs(sp(end)-y(end));

fprintf('steady State error = %d \n', sse)
