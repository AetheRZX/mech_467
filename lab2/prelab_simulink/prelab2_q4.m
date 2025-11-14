% Define system parameters 
Ka = 0.887;  
Kt = 0.72;  
Ke = (20/(2*pi));  %[mm/rad] 
Je = 7e-4;  
Be = 0.00612;  
mu = 0.3; 
Ts = 0.0002; 
hp = 20e-3; 

%Shelby's Method
% Define 's' as j60.
s=60j;
% Define the numerator of the open transfer function
numerator = Ka * Kt * Ke / (s * (Je * s + Be));
% Create the transfer function G(s)
Kp=1/abs(numerator)

%Determine Gain Margin by finding LRR & finding phase when LRR gain = 1
s=tf('s');
LRR = Kp * Ka * Kt * Ke / (s * (Je * s + Be));
bode(LRR);