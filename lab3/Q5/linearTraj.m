function [t, pos_x, pos_y, vel_x, vel_y, accel_x, accel_y, s, s_dot, s_ddot] = linearTraj(startPoint, endPoint, feedRate, acceleration, deceleration, samplingTime)
%Determine straight line traj planning
    
    %% Determine Linear Distances
    deltaX = endPoint(1) - startPoint(1);
    deltaY = endPoint(2) - startPoint(2);
    totalDistance = sqrt(deltaX^2 + deltaY^2);

    %% Determine Time Arrays & determine distances
    accelTime = feedRate / acceleration; %t = v/a
    decelTime = feedRate / abs(deceleration); % t = v/a
    accelDist = 0.5 * acceleration * accelTime^2; %D=1/2at^2
    decelDist = 0.5 * deceleration * decelTime^2; %D=1/2at^2
    constDist = totalDistance - (accelDist + decelDist); %Remaining Dist Calc
    
    %% Modify values if constDist < 0 / no constDistances
    if constDist < 0
        %Recalculate accel times assuming symmetric accel & decel values
        accelTime = sqrt(2 * totalDistance / (acceleration + deceleration));
        decelTime = accelTime; %Triangular Profile
        %Recalculate accel distances
        accelDist = 0.5 * acceleration * accelTime^2;
        decelDist = accelDist; %Triangular Profile
        %Set Const Velocity Values
        constDist = 0;
        constTime = 0;
        %set new achievable feedrate
        feedRate = acceleration * accelTime;
    else
        %Set constTime
        constTime = constDist / feedRate; %T = d/v
    end

    %% Adjust times to align with sampling Frequency
    % Calculate number of samples for each phase
    numSamplesAccel = ceil(accelTime / samplingTime);
    numSamplesDecel = ceil(decelTime / samplingTime);
    numSamplesConst = ceil(constTime / samplingTime);

    % Adjust times to be exact multiples of samplingTime
    accelTime = numSamplesAccel * samplingTime;
    decelTime = numSamplesDecel * samplingTime;
    if constTime ~= 0
        constTime = numSamplesConst * samplingTime;
    end

    %% Recalculate acceleration, distances, & Velocity based on adjusted Times
    % recalculate acceleration values
    acceleration = feedRate / accelTime;
    deceleration = feedRate / decelTime;

    % recalculate distances
    accelDist = 0.5 * acceleration * accelTime^2;
    decelDist = 0.5 * deceleration * decelTime^2;
    if (constDist ~= 0) 
        constDist = totalDistance - (accelDist + decelDist);
    end

    %% Determine Scalar Distances, Velocities, and Accelerations 
    % Acceleration Phase
    t_accel = (0 : numSamplesAccel - 1) * samplingTime;  % [sec]
    s_accel = 0.5 * acceleration * t_accel.^2;         % [mm]
    s_dot_accel = acceleration * t_accel;              % [mm/sec]
    s_ddot_accel = acceleration * ones(size(t_accel)); % [mm/sec^2]
    
    % Constant Velocity Phase
    if constDist > 0
        t_const = (1 : numSamplesConst) * samplingTime;  % [sec]
        s_const = accelDist + feedRate * t_const;        % [mm]
        s_dot_const = feedRate * ones(size(t_const));    % [mm/sec]
        s_ddot_const = zeros(size(t_const));            % [mm/sec^2]
    else
        t_const = [];
        s_const = [];
        s_dot_const = [];
        s_ddot_const = [];
    end
    
    % Deceleration Phase
    if constDist > 0
        % Deceleration starts after constant velocity phase
        t_decel_start = accelTime + constTime;
    else
        % Deceleration starts immediately after acceleration phase
        t_decel_start = accelTime;
    end
    t_decel = (1 : numSamplesDecel) * samplingTime;     % [sec]
    s_decel = accelDist + constDist + feedRate * t_decel - 0.5 * deceleration * t_decel.^2; % [mm]
    s_dot_decel = feedRate - deceleration * t_decel;   % [mm/sec]
    s_ddot_decel = -deceleration * ones(size(t_decel));% [mm/sec^2]
    
    % Combine all profiles
    s = [s_accel, s_const, s_decel];                     % [mm]
    s_dot = [s_dot_accel, s_dot_const, s_dot_decel];     % [mm/sec]
    s_ddot = [s_ddot_accel, s_ddot_const, s_ddot_decel]; % [mm/sec^2]
    if constDist > 0
        t = [t_accel, t_const + t_accel(end) + samplingTime, t_decel + t_accel(end) + constTime + samplingTime];
    else
        t = [t_accel, t_decel + t_accel(end) + samplingTime];
    end
    
    %% Determine Final Directional Distances, Velocities, and Accelerations
    % Direction cosines
    cos_theta = deltaX / totalDistance;
    sin_theta = deltaY / totalDistance;

    % Axis positions
    pos_x = startPoint(1) + s * cos_theta;                % [mm]
    pos_y = startPoint(2) + s * sin_theta;                % [mm]

    % Axis velocities
    vel_x = s_dot * cos_theta;                            % [mm/sec]
    vel_y = s_dot * sin_theta;                            % [mm/sec]

    % Axis accelerations
    accel_x = s_ddot * cos_theta;                         % [mm/sec^2]
    accel_y = s_ddot * sin_theta;                         % [mm/sec^2]
end