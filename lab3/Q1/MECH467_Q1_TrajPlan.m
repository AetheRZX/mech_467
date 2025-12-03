% Main Script for Linear Trajectory Planning and Plotting

% Clear workspace and figures
clear;
close all;
clc;

%% Constants - Original System Parameters
% Ti = 0.0001;          % Interpolation period [sec]
% F = 200;              % Desired feedrate [mm/sec]
% A = 1000;             % Acceleration [mm/sec^2]
% D = 1000;             % Deceleration [mm/sec^2]
% P1 = [0, 0];          % Point 1
% P2 = [40, 30];        % Point 2
% P3 = [60, 30];        % Point 3
% P_Circle = [90, 30];  % Center of circular radius 
% Direction = 1;        % Counter-clockwise direction
% AngleTrav = 2*pi;

%% Constants - Original System Parameters
Ti = 0.0001;          % Interpolation period [sec]
F = 100;              % Desired feedrate [mm/sec]
A = 250;             % Acceleration [mm/sec^2]
D = 250;             % Deceleration [mm/sec^2]
P1 = [0, 0];          % Point 1
P2 = [20, 50];        % Point 2
P3 = [30, 0];        % Point 3
P_Circle = [30, 30];  % Center of circular radius 
Direction = 1;        % counter-clockwise direction
AngleTrav = pi/2;

%% Trajectory Planning for Segment P1 to P2
% Call the linearTraj function for P1 to P2
[t1, pos_x1, pos_y1, vel_x1, vel_y1, accel_x1, accel_y1, s1, s_dot1, s_ddot1] = ...
    linearTraj(P1, P2, F, A, D, Ti);

%% Trajectory Planning for Segment P2 to P3
% Call the linearTraj function for P2 to P3
[t2, pos_x2, pos_y2, vel_x2, vel_y2, accel_x2, accel_y2, s2, s_dot2, s_ddot2] = ...
    linearTraj(P2, P3, F, A, D, Ti);

%% Trajectory Planning for Circular Motion (P3 to P3 via circle)
[t3, pos_x3, pos_y3, vel_x3, vel_y3, accel_x3, accel_y3, s3, s_dot3, s_ddot3] = ...
    circularTraj(P3, P_Circle, F, A, D, Ti, Direction, AngleTrav);


%% Combine Trajectory Data for Continuous Plotting
% Adjust time vectors to ensure continuity
t2 = t2 + t1(end) + Ti;  % Offset the second segment's time
t3 = t3 + t2(end) + Ti;            % Offset the circular segment's time
% Offset motion profiles for continuity
s2 = s2 + s1(end);       % Offset the second segment's trajectory
s3 = s3 + s2(end);       % Offset the third segment's trajectory


%% Combine Trajectory Data for Continuous Plotting
t_combined = [t1, t2, t3];
pos_x_combined = [pos_x1, pos_x2, pos_x3];
pos_y_combined = [pos_y1, pos_y2, pos_y3];
vel_x_combined = [vel_x1, vel_x2, vel_x3];
vel_y_combined = [vel_y1, vel_y2, vel_y3];
accel_x_combined = [accel_x1, accel_x2, accel_x3];
accel_y_combined = [accel_y1, accel_y2, accel_y3];
s_combined = [s1, s2, s3];
s_dot_combined = [s_dot1, s_dot2, s_dot3];
s_ddot_combined = [s_ddot1, s_ddot2, s_ddot3];

%% Plotting the Toolpath
figure('Name', 'Toolpath');
plot(pos_x_combined, pos_y_combined, 'b-', 'LineWidth', 2);
hold on;
plot(P1(1), P1(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Start Point
plot(P2(1), P2(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Intermediate Point
plot(P3(1), P3(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm'); % End Point
plot(P_Circle(1), P_Circle(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Circle Center
title('Generated Toolpath');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
legend('Toolpath', 'P1 Start', 'P2 Intermediate', 'P3 Intermediate', 'Circle Center');
grid on;
axis equal;
hold off;

%% Plotting X & Y vs Time
% Plotting Displacement vs. Time
figure('Name', 'X & Y vs. Time');

subplot(3,1,1);
plot(t_combined, pos_x_combined, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_combined, pos_y_combined, 'b-', 'LineWidth', 1.5);
title('Displacement vs. Time');
xlabel('Time [sec]');
ylabel('Displacement [mm]');
legend('X Position', 'Y Position');
grid on;
hold off;

% Plotting Velocity vs. Time
subplot(3,1,2);
plot(t_combined, vel_x_combined, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_combined, vel_y_combined, 'b-', 'LineWidth', 1.5);
title('Velocity vs. Time');
xlabel('Time [sec]');
ylabel('Velocity [mm/sec]');
legend('X Velocity', 'Y Velocity');
grid on;
hold off;

% Plotting Acceleration vs. Time
subplot(3,1,3);
plot(t_combined, accel_x_combined, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_combined, accel_y_combined, 'b-', 'LineWidth', 1.5);
title('Acceleration vs. Time');
xlabel('Time [sec]');
ylabel('Acceleration [mm/sec^2]');
legend('X Acceleration', 'Y Acceleration');
grid on;
hold off;

%% Plotting Profiles vs Time
% Plotting Displacement vs. Time
figure('Name', 'Profiles vs. Time');

subplot(3,1,1);
plot(t_combined, s_combined, 'r-', 'LineWidth', 1.5);
title('Displacement vs. Time');
xlabel('Time [sec]');
ylabel('Displacement [mm]');
grid on;
hold off;

% Plotting Velocity vs. Time
subplot(3,1,2);
plot(t_combined, s_dot_combined, 'r-', 'LineWidth', 1.5);
title('Velocity vs. Time');
xlabel('Time [sec]');
ylabel('Velocity [mm/sec]');
grid on;
hold off;

% Plotting Acceleration vs. Time
subplot(3,1,3);
plot(t_combined, s_ddot_combined, 'r-', 'LineWidth', 1.5);
title('Acceleration vs. Time');
xlabel('Time [sec]');
ylabel('Acceleration [mm/sec^2]');
grid on;
hold off;
