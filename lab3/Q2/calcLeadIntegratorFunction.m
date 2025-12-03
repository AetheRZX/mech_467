function [G_s] = calcLeadIntegratorFunction(Kp, Ki, alpha, tau, Je, Be)
%Determines the forward path open loop function between Kp, Ki, Lead-Lag, &
%Our plant.
%Constants
Ka = 1;  %[A/V] 
Kt = 0.49;  %[Nm/A]
Ke = 1.59;  %[mm/rad] 
s=tf('s');

%Setup Transfer Functions
%Plant Transfer Function
Plant_s = Ka * Kt * Ke / (s * (Je * s + Be));
%Controller = Kp * Ki_s * Lead_s 
integrator_s = (Ki + s)/s; 
leadComp_s = (alpha*tau*s + 1)/(tau*s + 1);
G_s = Kp * integrator_s * leadComp_s * Plant_s;
end