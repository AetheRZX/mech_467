function [alpha, tau] = calculateLeadCompParameters(phi_c_deg, wc)
    % Function to calculate lead compensator parameters
    % Arguments:
    %   phi_c_deg - Desired phase margin (degrees)
    %   wc - Gain crossover frequency (rad/s)
    % Returns:
    %   alpha - Lead compensator alpha parameter
    %   tau - Lead compensator tau parameter (seconds)

    % Convert phase margin to radians
    phi_c_rad = deg2rad(phi_c_deg);

    % Calculate alpha
    alpha = (1 + sin(phi_c_rad)) / (1 - sin(phi_c_rad));

    % Calculate tau
    tau = 1 / (wc * sqrt(alpha));
end
