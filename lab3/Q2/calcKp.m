function [Kp] = calcKp(crossoverFrequency, Je, Be)
    % Used to calculate our Kp Value
    %Constants
    Ka = 1;  %[A/V] 
    Kt = 0.49;  %[Nm/A]
    Ke = 1.59;  %[mm/rad] 
    s=crossoverFrequency*j;
    % Define the numerator of the open transfer function
    numerator = Ka * Kt * Ke / (s * (Je * s + Be));
    % Create the transfer function G(s)
    Kp = 1/abs(numerator);
end
