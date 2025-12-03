function [t, pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, s, s_dot, s_ddot] = circularTraj(startPoint, centerPoint, feedRate, acceleration, deceleration, samplingTime, direction, angleTravelled)
    %% Circular Parameters
    radius = sqrt((startPoint(1) - centerPoint(1))^2 + (startPoint(2) - centerPoint(2))^2); % Circle radius
    startAngle = atan2(startPoint(2) - centerPoint(2), startPoint(1) - centerPoint(1)); % Start angle
    totalAngle = angleTravelled; % Full circle (360 degrees)
    arcLength = radius * totalAngle; % Circumference of the circle

    %% Time and Distances
    accelTime = feedRate / acceleration; % t = v/a
    decelTime = feedRate / abs(deceleration); % t = v/a
    accelDist = 0.5 * acceleration * accelTime^2; % D = 1/2 * a * t^2
    decelDist = 0.5 * deceleration * decelTime^2; % D = 1/2 * a * t^2
    constDist = arcLength - (accelDist + decelDist); % Remaining distance

    % Adjust for triangular profile if constDist < 0
    if constDist < 0
        accelTime = sqrt(2 * arcLength / (acceleration + deceleration));
        decelTime = accelTime;
        feedRate = acceleration * accelTime;
        constDist = 0;
        constTime = 0;
    else
        constTime = constDist / feedRate; % T = d/v
    end

    %% Adjust Times for Sampling Frequency
    numSamplesAccel = ceil(accelTime / samplingTime);
    numSamplesDecel = ceil(decelTime / samplingTime);
    numSamplesConst = ceil(constTime / samplingTime);

    accelTime = numSamplesAccel * samplingTime;
    decelTime = numSamplesDecel * samplingTime;
    constTime = numSamplesConst * samplingTime;

    %% Recalculate Distances and Accelerations
    acceleration = feedRate / accelTime;
    deceleration = feedRate / decelTime;
    accelDist = 0.5 * acceleration * accelTime^2;
    decelDist = 0.5 * deceleration * decelTime^2;
    if constDist ~= 0
        constDist = arcLength - (accelDist + decelDist);
    end

    %% Tangential Motion
    t_accel = (0:numSamplesAccel-1) * samplingTime;
    s_accel = 0.5 * acceleration * t_accel.^2;
    s_dot_accel = acceleration * t_accel;
    s_ddot_accel = acceleration * ones(size(t_accel));

    t_const = (1:numSamplesConst) * samplingTime;
    s_const = accelDist + feedRate * t_const;
    s_dot_const = feedRate * ones(size(t_const));
    s_ddot_const = zeros(size(t_const));

    t_decel = (1:numSamplesDecel) * samplingTime;
    s_decel = accelDist + constDist + feedRate * t_decel - 0.5 * deceleration * t_decel.^2;
    s_dot_decel = feedRate - deceleration * t_decel;
    s_ddot_decel = -deceleration * ones(size(t_decel));

    t = [t_accel, t_const + t_accel(end) + samplingTime, t_decel + t_accel(end) + constTime + samplingTime];
    s = [s_accel, s_const, s_decel];
    s_dot = [s_dot_accel, s_dot_const, s_dot_decel];
    s_ddot = [s_ddot_accel, s_ddot_const, s_ddot_decel];

    %% Circular Trajectory
    angles = startAngle + direction * s / radius; % Angular positions
    pos_x = centerPoint(1) + radius * cos(angles); % X positions
    pos_y = centerPoint(2) + radius * sin(angles); % Y positions

    vel_x = -direction * s_dot .* sin(angles); % X velocities
    vel_y = direction * s_dot .* cos(angles); % Y velocities

    accel_x = -direction * (s_ddot .* sin(angles) + (s_dot.^2 / radius) .* cos(angles));
    accel_y = direction * (s_ddot .* cos(angles) - (s_dot.^2 / radius) .* sin(angles));
end
