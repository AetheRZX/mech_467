% Extract data from the low-bandwidth simulation - low feedrate
simDataLow = outLow.simout;
timeLow = simDataLow.Time;          % Time vector for low bandwidth
input_x_low = simDataLow.Data(:, 2); % Input x (reference trajectory r_x) - Low BW
output_x_low = simDataLow.Data(:, 3); % Output x (simulated x position) - Low BW
input_y_low = simDataLow.Data(:, 4); % Input y (reference trajectory r_y) - Low BW
output_y_low = simDataLow.Data(:, 5); % Output y (simulated y position) - Low BW

% Extract data from the low-bandwidth simulation - high feedrate
simDataHigh = outHighFeedrate.simout;
timeHigh = simDataHigh.Time;          % Time vector for high bandwidth
input_x_high = simDataHigh.Data(:, 2); % Input x (reference trajectory r_x) - High BW
output_x_high = simDataHigh.Data(:, 3); % Output x (simulated x position) - High BW
input_y_high = simDataHigh.Data(:, 4); % Input y (reference trajectory r_y) - High BW
output_y_high = simDataHigh.Data(:, 5); % Output y (simulated y position) - High BW

%% F1 - Plot Error graphs
% Calculate X-axis tracking errors
error_x_low = input_x_low - output_x_low; % X-axis error for low bandwidth
error_x_high = input_x_high - output_x_high; % X-axis error for high bandwidth

% Calculate Y-axis tracking errors
error_y_low = input_y_low - output_y_low; % Y-axis error for low bandwidth
error_y_high = input_y_high - output_y_high; % Y-axis error for high bandwidth

% Plot X-axis tracking errors (low vs high bandwidth)
figure;
plot(timeLow, error_x_low, 'LineWidth', 1.5, 'DisplayName', 'Error X (Low Feedrate)');
hold on;
plot(timeHigh, error_x_high, '--', 'LineWidth', 1.5, 'DisplayName', 'Error X (High Feedrate)');
xlabel('Time [s]');
ylabel('X Error [mm]');
title('X-Axis Tracking Errors: Low vs High Feedrate');
legend show;
grid on;

% Plot Y-axis tracking errors (low vs high bandwidth)
figure;
plot(timeLow, error_y_low, 'LineWidth', 1.5, 'DisplayName', 'Error Y (Low Feedrate)');
hold on;
plot(timeHigh, error_y_high, '--', 'LineWidth', 1.5, 'DisplayName', 'Error Y (High Feedrate)');
xlabel('Time [s]');
ylabel('Y Error [mm]');
title('Y-Axis Tracking Errors: Low vs High Feedrate');
legend show;
grid on;

%% F2 Code below
%% Combine and Plot Simulated Tool Motion with Reference Toolpath
% Simulated Tool Motion (Low and High Bandwidth) = Simulation Output
% Reference Toolpath = trajectory created inputs

% Full Plot: Reference Toolpath and Simulated Paths
figure;
plot(input_x_low, input_y_low, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(output_x_low, output_y_low, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low Feedrate)');
plot(output_x_high, output_y_high, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High Feedrate)');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('Toolpath: Reference vs Simulated (Low and High Feedrate)');
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
plot(input_x_low, input_y_low, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(output_x_low, output_y_low, 'b-', 'LineWidth', 2, 'DisplayName', 'Simulated Path (Low Feedrate)');
plot(output_x_high, output_y_high, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High Feedrate)');
axis(R1_limits); % Zoom to R1
title('Region R1');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 2); % R2
plot(input_x_low, input_y_low, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(output_x_low, output_y_low, 'b-', 'LineWidth', 2, 'DisplayName', 'Simulated Path (Low Feedrate)');
plot(output_x_high, output_y_high, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High Feedrate)');
axis(R2_limits); % Zoom to R2
title('Region R2');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 3); % R3
plot(input_x_low, input_y_low, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(output_x_low, output_y_low, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low Feedrate)');
plot(output_x_high, output_y_high, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High Feedrate)');
axis(R3_limits); % Zoom to R3
title('Region R3');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

subplot(2, 2, 4); % R4
plot(input_x_low, input_y_low, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Toolpath');
hold on;
plot(output_x_low, output_y_low, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (Low Feedrate)');
plot(output_x_high, output_y_high, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Path (High Feedrate)');
axis(R4_limits); % Zoom to R4
title('Region R4');
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
grid on;

% Measure the Largest Contouring Error
% Contouring error is the maximum deviation from the reference toolpath
% Calculate contouring error for each bandwidth (distance between simulated and reference paths)

contour_error_low = sqrt((output_x_low - input_x_low).^2 + ...
                         (output_y_low - input_y_low).^2);
contour_error_high = sqrt((output_x_high - input_x_low).^2 + ...
                          (output_y_high - input_y_low).^2);

max_contour_error_low = max(contour_error_low); % Max contouring error for low BW
max_contour_error_high = max(contour_error_high); % Max contouring error for high BW

fprintf('Max Contouring Error (Low Bandwidth): %.2f mm\n', max_contour_error_low);
fprintf('Max Contouring Error (High Bandwidth): %.2f mm\n', max_contour_error_high);

% Comment on the effect of bandwidth:
% - Low bandwidth results in larger contouring error, especially in sharp turns.
% - High bandwidth minimizes contouring error but may introduce small oscillations or overshoot.
