%% Plot Graphs 
% Define folder containing the data files
dataFolder = "Lab2Data";

% Get all CSV and MAT files in the folder
csvFiles = dir(fullfile(dataFolder, "*.csv"));
matFiles = dir(fullfile(dataFolder, "*.mat"));

% Initialize structures for data storage
measuredData = struct();
simulatedData = struct();

% Process CSV files (Measured Data)
for i = 1:length(csvFiles)
    % Get file name and path
    filePath = fullfile(csvFiles(i).folder, csvFiles(i).name);
    [~, fileName, ~] = fileparts(csvFiles(i).name);
    
    % Read and process data
    tableData = readtable(filePath);
    measuredData.(fileName).Time = tableData.Time / 1000; % Convert time to seconds
    measuredData.(fileName).Response = tableData.Enc2_ActPos_mm_; % Adjust column name if necessary
end

% Process MAT files (Simulated Data)
for i = 1:length(matFiles)
    % Get file name and path
    filePath = fullfile(matFiles(i).folder, matFiles(i).name);
    [~, fileName, ~] = fileparts(matFiles(i).name);
    
    % Load and process data
    matData = load(filePath); % Assuming variables are named `Time` and `Response`
    simulatedData.(fileName).Time = matData.Time; % Adjust if variable names differ
    simulatedData.(fileName).Response = matData.Signal; % Adjust if variable names differ
end

% Plot Kp Data
figure;
subplot(2,1,1);
plot(measuredData.Kp_Step.Time, measuredData.Kp_Step.Response, 'r');
hold on;
plot(simulatedData.Kp_Step.Time, simulatedData.Kp_Step.Response, 'b');
xlim([0 1]);
ylabel('Step Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('Kp Controller Step Response of Experimental vs. Simulated Data');

subplot(2,1,2);
plot(measuredData.Kp_Ramp.Time, measuredData.Kp_Ramp.Response, 'r');
hold on;
plot(simulatedData.Kp_Ramp.Time, simulatedData.Kp_Ramp.Response, 'b');
xlim([0 1]);
ylabel('Ramp Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('Kp Controller Ramp Response of Experimental vs. Simulated Data');

% Plot LL Data
figure;
subplot(2,1,1);
plot(measuredData.LL_Step.Time, measuredData.LL_Step.Response, 'r');
hold on;
plot(simulatedData.LL_Step.Time, simulatedData.LL_Step.Response, 'b');
xlim([0 0.5]);
ylabel('Step Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('LL Controller Step Response of Experimental vs. Simulated Data');

subplot(2,1,2);
plot(measuredData.LL_Ramp.Time, measuredData.LL_Ramp.Response, 'r');
hold on;
plot(simulatedData.LL_Ramp.Time, simulatedData.LL_Ramp.Response, 'b');
xlim([0 0.5]);
ylabel('Ramp Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('LL Controller Ramp Response of Experimental vs. Simulated Data');

% Plot LLI Data
figure;
subplot(2,1,1);
plot(measuredData.LLI_Step.Time, measuredData.LLI_Step.Response, 'r');
hold on;
plot(simulatedData.LLI_Step.Time, simulatedData.LLI_Step.Response, 'b');
xlim([0 0.25]);
ylabel('Step Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('LLI Controller Step Response of Experimental vs. Simulated Data');

subplot(2,1,2);
plot(measuredData.LLI_Ramp.Time, measuredData.LLI_Ramp.Response, 'r');
hold on;
plot(simulatedData.LLI_Ramp.Time, simulatedData.LLI_Ramp.Response, 'b');
xlim([0 0.25]);
ylabel('Ramp Response (mm)');
xlabel('Time (s)');
legend('Experimental Results', 'Simulink Results');
title('LLI Controller Ramp Response of Experimental vs. Simulated Data');

%% Calculate Rise Time & Overshoot
% Initialize arrays to store rise times and overshoots
riseTimesMeasured = [];
riseTimesSimulated = [];
overshootsMeasured = [];
overshootsSimulated = [];

% Slicing for P Controller (Kp)
KpStartIdx = find(measuredData.Kp_Step.Time >= 0, 1);
KpEndIdx = find(measuredData.Kp_Step.Time <= 1, 1, 'last');
KpMeasuredTime = measuredData.Kp_Step.Time(KpStartIdx:KpEndIdx);
KpMeasuredResponse = measuredData.Kp_Step.Response(KpStartIdx:KpEndIdx);

% Calculate for P Controller (Kp)
expInfo = stepinfo(KpMeasuredResponse, KpMeasuredTime);
simInfo = stepinfo(simulatedData.Kp_Step.Response, simulatedData.Kp_Step.Time);
riseTimesMeasured = [riseTimesMeasured; expInfo.RiseTime];
riseTimesSimulated = [riseTimesSimulated; simInfo.RiseTime];
overshootsMeasured = [overshootsMeasured; expInfo.Overshoot];
overshootsSimulated = [overshootsSimulated; simInfo.Overshoot];

% Slicing for Lead Compensator (LL)
LLStepStartIdx = find(measuredData.LL_Step.Time >= 0, 1);
LLStepEndIdx = find(measuredData.LL_Step.Time <= 0.5, 1, 'last');
LLMeasuredTime = measuredData.LL_Step.Time(LLStepStartIdx:LLStepEndIdx);
LLMeasuredResponse = measuredData.LL_Step.Response(LLStepStartIdx:LLStepEndIdx);

% Calculate for Lead Compensator (LL)
expInfo = stepinfo(LLMeasuredResponse, LLMeasuredTime);
simInfo = stepinfo(simulatedData.LL_Step.Response, simulatedData.LL_Step.Time);
riseTimesMeasured = [riseTimesMeasured; expInfo.RiseTime];
riseTimesSimulated = [riseTimesSimulated; simInfo.RiseTime];
overshootsMeasured = [overshootsMeasured; expInfo.Overshoot];
overshootsSimulated = [overshootsSimulated; simInfo.Overshoot];

% Slicing for Lead Integral Compensator (LLI)
LLIStepStartIdx = find(measuredData.LLI_Step.Time >= 0, 1);
LLIStepEndIdx = find(measuredData.LLI_Step.Time <= 0.25, 1, 'last');
LLIMeasuredTime = measuredData.LLI_Step.Time(LLIStepStartIdx:LLIStepEndIdx);
LLIMeasuredResponse = measuredData.LLI_Step.Response(LLIStepStartIdx:LLIStepEndIdx);

% Calculate for Lead Integral Compensator (LLI)
expInfo = stepinfo(LLIMeasuredResponse, LLIMeasuredTime);
simInfo = stepinfo(simulatedData.LLI_Step.Response, simulatedData.LLI_Step.Time);
riseTimesMeasured = [riseTimesMeasured; expInfo.RiseTime];
riseTimesSimulated = [riseTimesSimulated; simInfo.RiseTime];
overshootsMeasured = [overshootsMeasured; expInfo.Overshoot];
overshootsSimulated = [overshootsSimulated; simInfo.Overshoot];

% Final results in arrays
riseTimesMeasured % Rise times from measured data
riseTimesSimulated % Rise times from simulated data
overshootsMeasured % Overshoots from measured data
overshootsSimulated % Overshoots from simulated data

%% Steady-state calculations
% Define ramp parameters
t = 0:0.01:2; % Time vector
slope = 5; % Ramp slope
expectedRamp = slope * t; % Expected ramp function

% Define the time range for steady-state error calculation
startTime = 1; % Start of steady state - Changes Per Controller
endTime = 2; % End of steady state - Changes Per Controller

% Initialize storage for steady-state errors
controllers = {'Kp', 'LL', 'LLI'}; % Controllers
steadyStateErrors = table(); % Table to store results

for i = 1:length(controllers)
    controller = controllers{i};

    % Experimental data
    measuredTime = measuredData.([controller '_Ramp']).Time;
    measuredResponse = measuredData.([controller '_Ramp']).Response;
    expIndices = measuredTime >= startTime & measuredTime <= endTime;
    expectedRampInterp = interp1(t, expectedRamp, measuredTime(expIndices));
    steadyStateErrorMeasured = mean(abs(measuredResponse(expIndices) - expectedRampInterp));

    % Simulated data
    simulatedTime = simulatedData.([controller '_Ramp']).Time;
    simulatedResponse = simulatedData.([controller '_Ramp']).Response;
    simIndices = simulatedTime >= startTime & simulatedTime <= endTime;
    expectedRampInterp = interp1(t, expectedRamp, simulatedTime(simIndices));
    steadyStateErrorSimulated = mean(abs(simulatedResponse(simIndices) - expectedRampInterp));

    % Append to table
    steadyStateErrors = [steadyStateErrors; table({controller}, steadyStateErrorMeasured, steadyStateErrorSimulated, ...
        'VariableNames', {'Controller', 'MeasuredError', 'SimulatedError'})];
end

% Display the results table
disp(steadyStateErrors);

% Plot comparison (optional)
figure;
bar(categorical(steadyStateErrors.Controller), [steadyStateErrors.MeasuredError, steadyStateErrors.SimulatedError]);
ylabel('Steady-State Error (mm)');
xlabel('Controller');
legend('Experimental', 'Simulated');
title('Comparison of Steady-State Errors');
grid on;
