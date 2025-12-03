%% Load Experimental Data & Trajectory Data (Y)
dataG = readtable("standard.csv");
tMeasured = dataG.Time/1000;
xTraj = dataG.X_Traj;
yTraj = dataG.Y_Traj;
xMeasured = dataG.X_ActPos;
yMeasured = dataG.Y_ActPos;


% Reload Input and Simulated Data
timeLow = simDataLow.Time;          % Time vector for low bandwidth
output_x_low = simDataLow.Data(:, 3); % Simulated x position - Low BW
output_y_low = simDataLow.Data(:, 5); % Simulated y position - Low BW
x_pos_combined = pos_x_combined;     % Reference trajectory input (r_x)


%% Load Input (r) Data
xInput = pos_x_combined;
yInput = pos_y_combined;
tInput = t_combined;

%% Compute Errors
ErrorXInput = xInput - output_x_low;
ErroryInput = yInput - output_y_low;
ErrorXMeasured = xMeasured - xTraj;
ErrorYMeasured = yMeasured - yTraj;

%% G - Plot Error graphs
% Plot X-axis tracking errors (low vs high bandwidth)
figure;
plot(tInput, ErrorXInput, 'LineWidth', 1.5, 'DisplayName', 'Error X (Low BW)');
hold on;
plot(tMeasured, ErrorXMeasured, '--', 'LineWidth', 1.5, 'DisplayName', 'Error X (High BW)');
xlabel('Time [s]');
ylabel('X Error [mm]');
title('X-Axis Tracking Errors: Simulated & Measured');
legend show;
grid on;

% Plot Y-axis tracking errors (low vs high bandwidth)
figure;
plot(tInput, ErroryInput, 'LineWidth', 1.5, 'DisplayName', 'Error X (Low BW)');
hold on;
plot(tMeasured, ErrorYMeasured, '--', 'LineWidth', 1.5, 'DisplayName', 'Error X (High BW)');
xlabel('Time [s]');
ylabel('Y Error [mm]');
title('Y-Axis Tracking Errors: Simulated & Measured');
legend show;
grid on;

%% E2 Code below
%% Combine and Plot Simulated Tool Motion with Reference Toolpath
% Simulated Tool Motion (Low and High Bandwidth) = Simulation Output
% Reference Toolpath = trajectory created inputs

% Full Plot: Reference Toolpath and Simulated Paths
figure;
plot(xinput, xinput, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xTraj, yTraj, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low BW)');
plot(xMeasured, yMeasured, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High BW)');
plot(xTraj, yMeasured, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Mismatched Dyn.)');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('Toolpath: Reference vs Simulated (Low and High Bandwidth)');
legend show;
grid on;

%% Critical Regions R1, R2, R3, and R4
% Define approximate limits for critical regions
R1_limits = [19, 21, 14, 16]; % [xmin, xmax, ymin, ymax]
R2_limits = [38, 42, 28, 32];
R3_limits = [58, 62, 28, 32];
R4_limits = [102, 108, 52, 58];

% Create a 2x2 subplot for zoomed-in critical regions
figure;

subplot(2, 2, 1); % R1
plot(xinput, yinput, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xTraj, yTraj, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low BW)');
plot(xMeasured, yMeasured, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High BW)');
axis(R1_limits); % Zoom to R1
title('Region R1');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 2); % R2
plot(xinput, xinput, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xTraj, yTraj, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low BW)');
plot(xMeasured, yMeasured, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High BW)');
axis(R2_limits); % Zoom to R2
title('Region R2');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 3); % R3
plot(xinput, xinput, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xTraj, yTraj, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low BW)');
plot(xMeasured, yMeasured, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High BW)');
axis(R3_limits); % Zoom to R3
title('Region R3');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 4); % R4
plot(xinput, xinput, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xTraj, yTraj, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low BW)');
plot(xMeasured, yMeasured, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High BW)');
axis(R4_limits); % Zoom to R4
title('Region R4');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;


