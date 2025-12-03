% Define system parameters 
Ka = 1;  %[A/V] 
Kt = 0.49;  %[Nm/A]
Ke = 1.59;  %[mm/rad] 
Je_x = 4.36e-4;  %[kgm^2]
Je_y = 3e-4;  %[kgm^2] 
Be_x = 0.0094;  %[Nm/rad/s]
Be_y = 0.0091;  %[Nm/rad/s]
Ts = 0.0001; %[s]
high_w_crossover = 40*2*pi; %[rad/s]
low_w_crossover = 20*2*pi; %[rad/s]
Ki_high = high_w_crossover / 10;  
Ki_low = low_w_crossover / 10;

%% Determine Kp Controller For Both Axis and Both Crossover Frequencies 
%X-axis
Kp_high_x = calcKp(high_w_crossover,Je_x,Be_x);
Kp_low_x = calcKp(low_w_crossover,Je_x,Be_x);
% Y-axis
Kp_high_y = calcKp(high_w_crossover,Je_y,Be_y);
Kp_low_y = calcKp(low_w_crossover,Je_y,Be_y);

%% Determine Current Phase for both axis and both Crossover Frequencies
%X-axis
currPhase_high_x = findPhaseAtCrossover(Kp_high_x, Je_x, Be_x);
currPhase_low_x = findPhaseAtCrossover(Kp_low_x, Je_x, Be_x);
% Y-axis
currPhase_high_y = findPhaseAtCrossover(Kp_high_y, Je_y, Be_y);
currPhase_low_y = findPhaseAtCrossover(Kp_low_y, Je_y, Be_y);

%% Determine Peak Phase for both axis and both frequencies
%X-Axis
peakPhase_high_x = calcPeakPhase(currPhase_high_x);
peakPhase_low_x = calcPeakPhase(currPhase_low_x);
%Y-AxiTs
peakPhase_high_y = calcPeakPhase(currPhase_high_y);
peakPhase_low_y = calcPeakPhase(currPhase_low_y);

%% Determine Lead Compensator parameters
% X-Axis
[alpha_high_x, tau_high_x] = calculateLeadCompParameters(peakPhase_high_x, high_w_crossover);
[alpha_low_x, tau_low_x] = calculateLeadCompParameters(peakPhase_low_x, low_w_crossover);
% Y-Axis
[alpha_high_y, tau_high_y] = calculateLeadCompParameters(peakPhase_high_y, high_w_crossover);
[alpha_low_y, tau_low_y] = calculateLeadCompParameters(peakPhase_low_y, low_w_crossover);

%% Determine Lead Compensator Kp
% X-Axis
Kp_high_x = calculateLeadCompKp(alpha_high_x, tau_high_x, Je_x, Be_x, high_w_crossover);
Kp_low_x = calculateLeadCompKp(alpha_low_x, tau_low_x, Je_x, Be_x, low_w_crossover);
% Y-Axis
Kp_high_y = calculateLeadCompKp(alpha_high_y, tau_high_y, Je_y, Be_y, high_w_crossover);
Kp_low_y = calculateLeadCompKp(alpha_low_y, tau_low_y, Je_y, Be_y, low_w_crossover);

%% Develop Open Loop Integrator-Lead-Lag Controller Transfer Function
% X-Axis
Gs_high_x = calcLeadIntegratorFunction(Kp_high_x, Ki_high, alpha_high_x, tau_high_x, Je_x, Be_x);
Gs_low_x = calcLeadIntegratorFunction(Kp_low_x, Ki_low, alpha_low_x, tau_low_x, Je_x, Be_x);
% Y-Axis
Gs_high_y = calcLeadIntegratorFunction(Kp_high_y, Ki_high, alpha_high_y, tau_high_y, Je_y, Be_y);
Gs_low_y = calcLeadIntegratorFunction(Kp_low_y, Ki_low, alpha_low_y, tau_low_y, Je_y, Be_y);

%% Develop Continous Closed Loop Transfer Functions
% X-Axis
CL_low_x = feedback(Gs_low_x, 1);
CL_high_x = feedback(Gs_high_x, 1);
% Y-Axis
CL_low_y = feedback(Gs_low_y, 1);
CL_high_y = feedback(Gs_high_y, 1);

%% Develop Discrete closed loop transfer functions
% X-axis
CL_low_x_dis = c2d(CL_low_x, Ts, 'tustin');
CL_high_x_dis = c2d(CL_high_x, Ts, 'tustin');
% y-ayis
CL_low_y_dis = c2d(CL_low_y, Ts, 'tustin');
CL_high_y_dis = c2d(CL_high_y, Ts, 'tustin');

%% Determine Poles, Zeros, bandwidth, overshoot, and rise time for both axis, frequencies, and domains
% X-axis, continous
param_x_low_cont = analyzeTF(CL_low_x_dis);
param_x_high_cont = analyzeTF(CL_high_x_dis);
% X-axis, discrete
param_x_low_dis = analyzeTF(CL_low_x_dis);
param_x_high_dis = analyzeTF(CL_high_x_dis);
% Y-axis, continous
param_y_low_cont = analyzeTF(CL_low_y_dis);
param_y_high_cont = analyzeTF(CL_high_y_dis);
% Y-axis, discrete
param_y_low_dis = analyzeTF(CL_low_y_dis);
param_y_high_dis = analyzeTF(CL_high_y_dis);

%% Plot Bode Plots for Open-Loop Systems of X-Axis at both crossover frequencies
% Plot Open-Loop Bode Plots for X-Axis
figure;
bode(Gs_low_x);
hold on;
bode(Gs_high_x);
title('Open-Loop Bode Plots for X-Axis with LBW and HBW Controllers');
legend('LBW Controller', 'HBW Controller');
grid on;
% Plot Closed-Loop Bode Plots for X-Axis
figure;
bode(CL_low_x);
hold on;
bode(CL_high_x);
title('Closed-Loop Bode Plots for X-Axis with LBW and HBW Controllers');
legend('LBW Controller', 'HBW Controller');
grid on;

