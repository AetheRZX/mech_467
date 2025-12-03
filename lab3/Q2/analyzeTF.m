function [structOut] = analyzeTF(TF)
% Determine Poles, Zeros, bandwidth, overshoot rise time,
    % Analyze poles and zeros
    poles = pole(TF);
    zeros = zero(TF);

    % Compute step response information
    stepInfo = stepinfo(TF);

    %Extract bandwidth
    bandwidthVal = bandwidth(TF);

    % Prepare output structure
    structOut = struct();
    structOut.Poles = poles;
    structOut.Zeros = zeros;
    structOut.Bandwidth = bandwidthVal;
    structOut.Overshoot = stepInfo.Overshoot;
    structOut.RiseTime = stepInfo.RiseTime;
end
