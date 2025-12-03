% MECH 467 Lab 3 Prelab Setup Script
% Ryan Edric Nashota
clear; clc;

%% Plant Parameters
% X Axis
Ka_x = 1; Kt_x = 0.49; Ke_x = 1.59; Je_x = 4.36e-4; B_x = 0.0094;
% Y Axis
Ka_y = 1; Kt_y = 0.49; Ke_y = 1.59; Je_y = 3e-4; B_y = 0.0091;

% Transfer Functions
s = tf('s');
Gx = (Ka_x * Kt_x * Ke_x) / (s * (Je_x * s + B_x));
Gy = (Ka_y * Kt_y * Ke_y) / (s * (Je_y * s + B_y));

%% Controller Design Parameters
Ts = 0.0001; % 0.1 ms

% Case 1: LBW (wc=20Hz, PM=60)
wc_lbw = 2*pi*20;
PM_target = 60;

% Design Lead X
[mag_x, phase_x] = bode(Gx, wc_lbw);
phi_req_x = -180 + PM_target - phase_x;
alpha_x = (1 + sind(phi_req_x))/(1 - sind(phi_req_x));
tau_x = 1 / (wc_lbw * sqrt(alpha_x));
K_lead_x = 1 / (mag_x * sqrt(alpha_x));
C_lead_x = K_lead_x * (alpha_x * tau_x * s + 1) / (tau_x * s + 1);

% Design Lead Y
[mag_y, phase_y] = bode(Gy, wc_lbw);
phi_req_y = -180 + PM_target - phase_y;
alpha_y = (1 + sind(phi_req_y))/(1 - sind(phi_req_y));
tau_y = 1 / (wc_lbw * sqrt(alpha_y));
K_lead_y = 1 / (mag_y * sqrt(alpha_y));
C_lead_y = K_lead_y * (alpha_y * tau_y * s + 1) / (tau_y * s + 1);

% Integral Action
Ki = wc_lbw / 10;
C_int = (s + Ki) / s;

% Total Controller Continuous
C_tot_x_c = C_lead_x * C_int;
C_tot_y_c = C_lead_y * C_int;

% Discretize
C_tot_x_d = c2d(C_tot_x_c, Ts, 'tustin');
C_tot_y_d = c2d(C_tot_y_c, Ts, 'tustin');

% Plant Discrete
Gx_d = c2d(Gx, Ts, 'zoh');
Gy_d = c2d(Gy, Ts, 'zoh');

disp('Controller Design Complete (Case 1 LBW)');
