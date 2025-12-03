function Kp = calculateLeadCompKp(alpha, tau, Je, Be, wc)
    % Function to calculate Kp for a lead compensator
    % Arguments:
    %   alpha - Lead compensator parameter alpha
    %   tau - Lead compensator parameter tau [seconds]
    %   Je - Moment of inertia [kgm^2]
    %   Be - Damping coefficient [Nm/rad/s]
    %   wc - Gain crossover frequency [rad/s]
    % Returns:
    %   Kp - Proportional gain for the lead compensator

    % Constants
    Ka = 1;  %[A/V]
    Kt = 0.49;  %[Nm/A]
    Ke = 1.59;  %[mm/rad]

    % Transfer function of the lead compensator at the crossover frequency
    s = 1j * wc;
    compensator_gain = (1 + s * alpha*tau) / (1 + s * tau);

    % Loop return ratio without compensator
    LRR = Ka * Kt * Ke / (s * (Je * s + Be));

    % Calculate Kp such that |LRR| = 1 at the crossover frequency
    Kp = 1 / abs(compensator_gain * LRR);
end
