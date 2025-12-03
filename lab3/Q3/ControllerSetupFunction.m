% Define 's' as a transfer function variable
s = tf('s');
% Define Constant System Parameters 
Ka = 1;  %[A/V] 
Kt = 0.49;  %[Nm/A]
Ke = 1.59;  %[mm/rad] 
Ts = 0.0001; %[s]
Kd = 1; %[V/mm] %Feedback Gain

%X-Low Controller
Je_x = 4.36e-4;  %[kgm^2]
Be_x = 0.0094;  %[Nm/rad/s]
wc_x = 20*2*pi; %[rad/s]
Ki_x = wc_x / 10;
Kp_x = 3.24;
alpha_x = 7.6583;
tau_x = 0.0029;

%X-High Controller
% Je_x = 4.36e-4;  %[kgm^2]
% Be_x = 0.0094;  %[Nm/rad/s]
% wc_x = 40*2*pi; %[rad/s]
% Ki_x = wc_x / 10;
% Kp_x = 11.1533;
% alpha_x = 10.1186;
% tau_x = 0.0013;

%Y-Low Controller
Je_y = 3e-4;  %[kgm^2] 
Be_y = 0.0091;  %[Nm/rad/s]
wc_y = 20*2*pi; %[rad/s]
Ki_y = wc_y / 10;
Kp_y = 2.5001;
alpha_y = 6.2602;
tau_y = 0.0032;

%Y-High Controller
% Je_y = 3e-4;  %[kgm^2] 
% Be_y = 0.0091;  %[Nm/rad/s]
% wc_y = 40*2*pi; %[rad/s]
% Ki_y = wc_y / 10;
% Kp_y = 8.1692;
% alpha_y = 8.9937;
% tau_y = 0.0013;


%Plant Transfer Function in continous
plant_x = 1/(Je_x*s+Be_x);
plant_y = 1/(Je_y*s+Be_y);

%Position Encoder Transfer Function in Continous
posEncoder = Ke / s;

%Compensator function in discrete
LeadComp_x = Kp_x*(alpha_x*tau_x*s+1)/(tau_x*s+1);
LeadComp_x_z = c2d(LeadComp_x,Ts);
LeadComp_y = Kp_y*(alpha_x*tau_x*s+1)/(tau_x*s+1);
LeadComp_y_z = c2d(LeadComp_y,Ts);

%Integration function in discrete
integrateblk_x=(Ki_x+s)/s;
integrateblk_x_z=c2d(integrateblk_x,Ts);
integrateblk_y=(Ki_y+s)/s;
integrateblk_y_z=c2d(integrateblk_y,Ts);