function currPhase = findPhaseAtCrossover(Kp, Je, Be)
    % Function to find the phase at the frequency where |LRR| = 1
    % Arguments:
    %   Kp - Proportional gain
    %   Je - Moment of inertia [kgm^2]
    %   Be - Damping coefficient [Nm/rad/s]
    % Returns:
    %   currPhase - Phase [radians] when |LRR| = 1
    
    % Constants
    Ka = 1;  %[A/V]
    Kt = 0.49;  %[Nm/A]
    Ke = 1.59;  %[mm/rad]
    
    % Frequency search range
    f_min = 0.1;  % Start search from 0.1 rad/s to avoid division by zero
    f_max = 1000; % Set an upper bound for the frequency range
    tolerance = 1e-6; % Tolerance for magnitude equality

    % Function for the magnitude of the LRR
    LRR_magnitude = @(omega) abs(Ka * Kt * Ke * Kp ./ ((1j * omega) .* (Je * 1j * omega + Be)));

    % Solve for the crossover frequency (where |LRR| = 1)
    crossoverFreq = fminbnd(@(omega) abs(LRR_magnitude(omega) - 1), f_min, f_max);

    % Compute the phase at the crossover frequency
    s = 1j * crossoverFreq;
    LRR = Kp * Ka * Kt * Ke / (s * (Je * s + Be));
    currPhase = rad2deg(angle(LRR)); % Phase in radians
end
