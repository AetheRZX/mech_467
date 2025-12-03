% Reload Experimental Data (Measured Trajectory)
dataG = readtable("standard.csv");

% Extract measured trajectory and time
tMeasured = dataG.Time / 1000; % Convert to seconds
xTraj = dataG.X_Traj;          % Reference trajectory (measured x)
yTraj = dataG.Y_Traj;          % Reference trajectory (measured y)
xMeasured = dataG.X_ActPos;    % Measured x position
yMeasured = dataG.Y_ActPos;    % Measured y position



% Reload Input and Simulated Data
% Extract data from the low-bandwidth simulation - low feedrate
simDataLow = outLow.simout;
timeLow = simDataLow.Time;          % Time vector for low bandwidth
output_x_low = simDataLow.Data(:, 3); % Output x (simulated x position) - Low BW
output_y_low = simDataLow.Data(:, 5); % Output y (simulated y position) - Low BW
x_pos_combined = pos_x_combined;     % Reference trajectory input (r_x)
y_pos_combined = pos_y_combined;     % Reference trajectory input (r_y)

% Ensure Comparison Alignment (Use the smaller length for x and y separately)
minLengthInputSimX = min(length(x_pos_combined), length(output_x_low));
minLengthInputSimY = min(length(y_pos_combined), length(output_y_low));
minLengthMeasuredTrajX = min(length(xMeasured), length(xTraj));
minLengthMeasuredTrajY = min(length(yMeasured), length(yTraj));

% Trim Data to Match Lengths for Comparisons (X)
xInputAligned = x_pos_combined(1:minLengthInputSimX);
xSimAligned = output_x_low(1:minLengthInputSimX);

xMeasuredAligned = xMeasured(1:minLengthMeasuredTrajX);
xTrajAligned = xTraj(1:minLengthMeasuredTrajX);

% Trim Data to Match Lengths for Comparisons (Y)
yInputAligned = y_pos_combined(1:minLengthInputSimY);
ySimAligned = output_y_low(1:minLengthInputSimY);

yMeasuredAligned = yMeasured(1:minLengthMeasuredTrajY);
yTrajAligned = yTraj(1:minLengthMeasuredTrajY);

% Compute Errors (X and Y)
ErrorXInput = xInputAligned - xSimAligned;        % Reference input vs Simulated (X)
ErrorYInput = yInputAligned - ySimAligned;        % Reference input vs Simulated (Y)
ErrorXMeasured = xMeasuredAligned - xTrajAligned; % Measured vs Reference trajectory (X)
ErrorYMeasured = yMeasuredAligned - yTrajAligned; % Measured vs Reference trajectory (Y)

% Downsampling Factor (Optional for Plotting)
downsampleFactor = 10; % Adjust as needed for performance
xInput_ds = xInputAligned(1:downsampleFactor:end);
yInput_ds = yInputAligned(1:downsampleFactor:end);
xSim_ds = xSimAligned(1:downsampleFactor:end);
ySim_ds = ySimAligned(1:downsampleFactor:end);
xMeasured_ds = xMeasuredAligned(1:downsampleFactor:end);
yMeasured_ds = yMeasuredAligned(1:downsampleFactor:end);
xTraj_ds = xTrajAligned(1:downsampleFactor:end);
yTraj_ds = yTrajAligned(1:downsampleFactor:end);
tInput_ds = timeLow(1:downsampleFactor:end);
tMeasured_ds = tMeasured(1:downsampleFactor:end);

%% Plot Error Graphs (X and Y Axes)
% X-axis tracking errors
figure;
plot(tInput_ds, ErrorXInput(1:downsampleFactor:end), 'LineWidth', 1.5, 'DisplayName', 'Error X (Input vs Simulated)');
hold on;
plot(tMeasured_ds, ErrorXMeasured(1:downsampleFactor:end), '--', 'LineWidth', 1.5, 'DisplayName', 'Error X (Measured vs Reference)');
xlabel('Time [s]');
ylabel('X Error [mm]');
title('X-Axis Tracking Errors');
legend show;
grid on;

% Y-axis tracking errors
figure;
plot(tInput_ds, ErrorYInput(1:downsampleFactor:end), 'LineWidth', 1.5, 'DisplayName', 'Error Y (Input vs Simulated)');
hold on;
plot(tMeasured_ds, ErrorYMeasured(1:downsampleFactor:end), '--', 'LineWidth', 1.5, 'DisplayName', 'Error Y (Measured vs Reference)');
xlabel('Time [s]');
ylabel('Y Error [mm]');
title('Y-Axis Tracking Errors');
legend show;
grid on;

%% Plot Toolpath with Reference, Simulated, and Measured (Both Axes)
figure;
plot(xInput_ds, yInput_ds, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Input Toolpath');
hold on;
plot(xSim_ds, ySim_ds, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Toolpath');
plot(xMeasured_ds, yMeasured_ds, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Toolpath');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('Toolpath: Reference, Simulated, and Measured');
legend show;
grid on;

%% Critical Regions (Zoomed-In Plots for Both Axes)
R1_limits = [19, 21, 14, 16]; % [xmin, xmax, ymin, ymax]
R2_limits = [38, 42, 28, 32];
R3_limits = [58, 62, 28, 32];
R4_limits = [102, 108, 52, 58];

figure;

subplot(2, 2, 1); % R1
plot(xInput_ds, yInput_ds, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xSim_ds, ySim_ds, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Toolpath');
plot(xMeasured_ds, yMeasured_ds, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Toolpath');
axis(R1_limits); % Zoom to R1
title('Region R1');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 2); % R2
plot(xInput_ds, yInput_ds, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xSim_ds, ySim_ds, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Toolpath');
plot(xMeasured_ds, yMeasured_ds, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Toolpath');
axis(R2_limits); % Zoom to R2
title('Region R2');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 3); % R3
plot(xInput_ds, yInput_ds, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xSim_ds, ySim_ds, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Toolpath');
plot(xMeasured_ds, yMeasured_ds, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Toolpath');
axis(R3_limits); % Zoom to R3
title('Region R3');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 4); % R4
plot(xInput_ds, yInput_ds, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(xSim_ds, ySim_ds, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Toolpath');
plot(xMeasured_ds, yMeasured_ds, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Toolpath');
axis(R4_limits); % Zoom to R4
title('Region R4');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;
