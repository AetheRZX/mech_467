clc; close all;
% Generates and visualizes a toolpath consisting of straight lines and a circular arc.

%% Parameters
feedRate = 200; % Desired feedrate [mm/sec]
acceleration = 1000; % Acceleration [mm/sec^2]
deceleration = -1000; % Deceleration [mm/sec^2]
samplingTime = 0.1 / 1000; % Sampling time [sec]
point1 = [0, 0];
point2 = [40, 30];
point3 = [60, 30];
point4 = [90, 30]; % Circle center
circleRadius = 30; % Radius of the circle
startAngle = -180; % Start angle (degrees)
sweepAngle = 360; % Full circle
cw = -1;
ccw = 1;

%% Initialize Trajectory Storage
trajectoryX = [];
trajectoryY = [];
timeVector = [];

%% Generate Straight Paths
[straightX1, straightY1, time1, velocity1, accel1, displacement1] = generateStraightPath(point1, point2, feedRate, acceleration, deceleration, samplingTime);
[straightX2, straightY2, time2, velocity2, accel2, displacement2] = generateStraightPath(point2, point3, feedRate, acceleration, deceleration, samplingTime);

%% Generate Circular Path
[circleX, circleY, time3, velocity3, accel3, displacement3] = generateCircularPath(point3, point4, circleRadius, sweepAngle, feedRate, acceleration, deceleration, samplingTime, ccw);

%% Concatenate Trajectories
trajectoryX = [straightX1(:); straightX2(:); circleX(:)];
trajectoryY = [straightY1(:); straightY2(:); circleY(:)];
timeVector = [time1(:); time2(:) + time1(end); time3(:) + (time2(end) + time1(end))];
velocities = [velocity1(:); velocity2(:); velocity3(:)];
accelerations = [accel1(:); accel2(:); accel3(:)];
displacement = [displacement1(:); displacement2(:) + displacement1(end); displacement3(:) + displacement1(end) + displacement2(end)];

%% Plot the Combined Toolpath
figure;
plot(trajectoryX, trajectoryY, 'LineWidth', 2);
title('Toolpath');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;
axis equal;

%% Plot Velocity vs Time
figure;
plot(timeVector, velocities, 'LineWidth', 2);
title('Tool Velocity vs Time');
xlabel('Time [s]');
ylabel('Velocity [mm/s]');
grid on;

%% Plot Acceleration vs Time
figure;
plot(timeVector, accelerations, 'LineWidth', 2);
title('Tool Acceleration vs Time');
xlabel('Time [s]');
ylabel('Acceleration [mm/s^2]');
grid on;

%% Plot Displacement vs Time
figure;
plot(timeVector, displacement, 'LineWidth', 2);
title('Displacement vs Time');
xlabel('Time [s]');
ylabel('Displacement [mm]');
grid on;

%% Functions

function [x, y, t, v, a, s] = generateStraightPath(startPoint, endPoint, feedRate, acceleration, deceleration, samplingTime)
    deltaX = endPoint(1) - startPoint(1);
    deltaY = endPoint(2) - startPoint(2);
    totalDistance = sqrt(deltaX^2 + deltaY^2);

    % Calculate time for each phase
    timeAccel = feedRate / acceleration;
    timeDecel = feedRate / abs(deceleration);
    distanceAccel = 0.5 * acceleration * timeAccel^2;
    distanceDecel = 0.5 * abs(deceleration) * timeDecel^2;
    distanceConst = totalDistance - (distanceAccel + distanceDecel);

    if distanceConst < 0
        timeAccel = sqrt(2 * totalDistance / (acceleration + abs(deceleration)));
        timeDecel = timeAccel;
        timeConst = 0;
        feedRate = acceleration * timeAccel;
    else
        timeConst = distanceConst / feedRate;
    end

    % Adjust times to align with sampling
    timeAccel = ceil(timeAccel / samplingTime) * samplingTime;
    timeConst = ceil(timeConst / samplingTime) * samplingTime;
    timeDecel = ceil(timeDecel / samplingTime) * samplingTime;

    % Generate time vector
    t = 0:samplingTime:(timeAccel + timeConst + timeDecel);
    v = zeros(size(t));
    a = zeros(size(t));
    s = zeros(size(t));

    % Acceleration phase
    for i = 1:length(t)
        if t(i) <= timeAccel
            s(i) = 0.5 * acceleration * t(i)^2;
            v(i) = acceleration * t(i);
            a(i) = acceleration;
        elseif t(i) <= timeAccel + timeConst
            s(i) = distanceAccel + feedRate * (t(i) - timeAccel);
            v(i) = feedRate;
            a(i) = 0;
        else
            timeD = t(i) - timeAccel - timeConst;
            s(i) = distanceAccel + distanceConst + feedRate * timeD + 0.5 * deceleration * timeD^2;
            v(i) = feedRate + deceleration * timeD;
            a(i) = deceleration;
        end
    end

    % Calculate positions
    x = startPoint(1) + (deltaX / totalDistance) * s;
    y = startPoint(2) + (deltaY / totalDistance) * s;
end

function [x, y, t, v, a, s] = generateCircularPath(currentPosition, center, radius, sweepAngle, feedRate, acceleration, deceleration, samplingTime, direction)
    sweepAngle = deg2rad(sweepAngle);
    deltaX = currentPosition(1) - center(1);
    deltaY = currentPosition(2) - center(2);
    startAngle = atan2(deltaY, deltaX);

    if direction == 1
        endAngle = startAngle + sweepAngle;
    elseif direction == -1
        endAngle = startAngle - sweepAngle;
    else
        error('Invalid direction. Use 1 for CCW or -1 for CW.');
    end

    arcLength = abs(radius * sweepAngle);

    timeAccel = feedRate / acceleration;
    timeDecel = feedRate / abs(deceleration);
    timeConst = (arcLength - 0.5 * acceleration * timeAccel^2 - 0.5 * abs(deceleration) * timeDecel^2) / feedRate;

    timeAccel = ceil(timeAccel / samplingTime) * samplingTime;
    timeDecel = ceil(timeDecel / samplingTime) * samplingTime;
    timeConst = max(ceil(timeConst / samplingTime) * samplingTime, 0);

    t = 0:samplingTime:(timeAccel + timeConst + timeDecel);
    s = zeros(size(t));
    v = zeros(size(t));
    a = zeros(size(t));

    for i = 1:length(t)
        if t(i) <= timeAccel
            s(i) = 0.5 * acceleration * t(i)^2;
            v(i) = acceleration * t(i);
            a(i) = acceleration;
        elseif t(i) <= timeAccel + timeConst
            s(i) = 0.5 * acceleration * timeAccel^2 + feedRate * (t(i) - timeAccel);
            v(i) = feedRate;
            a(i) = 0;
        else
            timeD = t(i) - timeAccel - timeConst;
            s(i) = arcLength - 0.5 * abs(deceleration) * timeD^2;
            v(i) = feedRate + deceleration * timeD;
            a(i) = deceleration;
        end
    end

    angles = linspace(startAngle, endAngle, length(s)); 
    x = center(1) + radius * cos(angles);
    y = center(2) + radius * sin(angles);
end
