%9.1 Experimental vs. Simulations Step and Ramp Response Results 
% STEP 1: PLOT STEP AND RAMP RESPONSE FOR P CONTROLLER--------------------- 

pStepMeasuredData = readtable("Lab2Data\Prop_Step.csv"); 
pStepMeasuredTime = pStepMeasuredData.Time / 1000; % Convert time to seconds 
pStepMeasuredResponse = pStepMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% pStepSimTime = out.scopeData.time; 
% pStepSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
figure; 
subplot(2,1,1); 
plot(pStepMeasuredTime, pStepMeasuredResponse, 'r'); 
hold on; 
plot(pStepSimTime, pStepSimResponse, 'b'); 
xlim([0 2]); 
ylabel('Step Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('P Controller Step Response of Experimental vs. Simulated Data'); 
% b) Get Ramp Response 
% Get measured data 
pRampMeasuredData = readtable("Lab2Data\Prop_Ramp.csv"); 
pRampMeasuredTime = pRampMeasuredData.Time / 1000; % Convert time to seconds 
pRampMeasuredResponse = pRampMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% pRampSimTime = out.scopeData.time; 
% pRampSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
subplot(2,1,2); 
plot(pRampMeasuredTime, pRampMeasuredResponse, 'r'); 
hold on; 
plot(pRampSimTime, pRampSimResponse, 'b'); 
xlim([0 2]); 
ylabel('Ramp Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('P Controller Ramp Response of Experimental vs. Simulated Data'); 
hold off; 
%%  
% STEP 2: PLOT STEP AND RAMP RESPONSE FOR LL CONTROLLER-------------------- 
% a) Get Step Response 
% Get measured data 
LLStepMeasuredData = readtable("Lab2Data\LL_Step.csv"); 
LLStepMeasuredTime = LLStepMeasuredData.Time / 1000; 
LLStepMeasuredResponse = LLStepMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% LLStepSimTime = out.scopeData.time; 
% LLStepSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
figure; 
subplot(2,1,1); 
plot(LLStepMeasuredTime, LLStepMeasuredResponse, 'r'); 
hold on; 
plot(LLStepSimTime, LLStepSimResponse, 'b'); 
ylim([0 1.5]); 
xlim([0 3]); 
ylabel('Step Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('LL Controller Step Response of Experimental vs. Simulated Data'); 
% b) Get Ramp Response 
% Get measured data 
LLRampMeasuredData = readtable("Lab2Data\LL_Ramp.csv"); 
LLRampMeasuredTime = LLRampMeasuredData.Time / 1000; % Convert time to seconds 
LLRampMeasuredResponse = LLRampMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% LLRampSimTime = out.scopeData.time; 
% LLRampSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
subplot(2,1,2); 
plot(LLRampMeasuredTime, LLRampMeasuredResponse, 'r'); 
hold on; 
plot(LLRampSimTime, LLRampSimResponse, 'b'); 
xlim([0 2]); 
ylabel('Ramp Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('LL Controller Ramp Response of Experimental vs. Simulated Data'); 
hold off; 
%%  
% STEP 3: PLOT STEP AND RAMP RESPONSE FOR LLI CONTROLLER------------------- 
% a) Get Step Response 
% Get measured data 
LLIStepMeasuredData = readtable("Lab2Data\LLI_Step.csv"); 
LLIStepMeasuredTime = LLIStepMeasuredData.Time / 1000; % Convert time to seconds 
LLIStepMeasuredResponse = LLIStepMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% LLIStepSimTime = out.scopeData.time; 
% LLIStepSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
figure; 
subplot(2,1,1); 
plot(LLIStepMeasuredTime, LLIStepMeasuredResponse, 'r'); 
hold on; 
plot(LLIStepSimTime, LLIStepSimResponse, 'b'); 
xlim([0 3]); 
ylabel('Step Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('LLI Controller Step Response of Experimental vs. Simulated Data'); 
% b) Get Ramp Response 
% Get measured data 
LLIRampMeasuredData = readtable("Lab2Data\LLI_Ramp.csv"); 
LLIRampMeasuredTime = LLIRampMeasuredData.Time / 1000; % Convert time to seconds 
LLIRampMeasuredResponse = LLIRampMeasuredData.Enc2_ActPos_mm_; 
% Get simulated data 
% --------------------------------------------------- 
% LLIRampSimTime = out.scopeData.time; 
% LLIRampSimResponse = out.scopeData.signals.values; 
% --------------------------------------------------- 
% Plot measured response and simulated response 
subplot(2,1,2); 
plot(LLIRampMeasuredTime, LLIRampMeasuredResponse, 'r'); 
hold on; 
plot(LLIRampSimTime, LLIRampSimResponse, 'b'); 
xlim([0 2]); 
ylabel('Ramp Response (mm)'); 
xlabel('Time (s)'); 
legend('Experimental Results', 'Simulink Results'); 
title('LLI Controller Ramp Response of Experimental vs. Simulated Data'); 
hold off; 


%9.2 Calculating Rise Times and Overshoots 
% STEP 1: GET RISE TIME AND OVERSHOOT OF STEP RESPONSE FOR P CONTROLLER---- 
% Get experimental and simulated info 
expInfo = stepinfo(pStepMeasuredResponse(1:10001), pStepMeasuredTime(1:10001)); % Measured rise time 
simInfo = stepinfo(pStepSimResponse, pStepSimTime); 
% Display the rise time and overshoot 
fprintf('P: Measured Rise Time= %.4fs\nP: Simulated Rise Time= %.4fs\n', expInfo.RiseTime, simInfo.RiseTime); 
fprintf('P: Measured Overshoot= %.2f%% (Relative to ss) OR 21.43%% (Relative to 1)\nP: Simulated Overshoot= %.2f%%\n', expInfo.Overshoot, simInfo.Overshoot); 
fprintf('\n'); 
% STEP 2: GET RISE TIME AND OVERSHOOT OF STEP RESPONSE FOR LL CONTROLLER--- 
% Get experimental and simulated info 
expInfo = stepinfo(LLStepMeasuredResponse(1:15001), LLStepMeasuredTime(1:15001)); % Measured rise time 
simInfo = stepinfo(LLStepSimResponse, LLStepSimTime); 
% Display the rise time and overshoot 
fprintf('LL: Measured Rise Time= %.4fs\nLL: Simulated Rise Time= %.4fs\n', expInfo.RiseTime, simInfo.RiseTime); 
fprintf('LL: Measured Overshoot= %.2f%% (Relative to ss) OR 0%% (Relative to 1)\nLL: Simulated Overshoot= %.2f%%\n', expInfo.Overshoot, simInfo.Overshoot); 
fprintf('\n'); 
% STEP 3: GET RISE TIME AND OVERSHOOT OF STEP RESPONSE FOR LLI CONTROLLER-- 
% Get experimental and simulated info 
expInfo = stepinfo(LLIStepMeasuredResponse(1:15001), LLIStepMeasuredTime(1:15001)); % Measured rise time 
simInfo = stepinfo(LLIStepSimResponse, LLIStepSimTime); 
% Display the rise time and overshoot 
fprintf('LLI: Measured Rise Time= %.4fs\nLLI: Simulated Rise Time= %.4fs\n', expInfo.RiseTime, simInfo.RiseTime); 
fprintf('LLI: Measured Overshoot= %.2f%% (Relative to ss)\nLLI: Simulated Overshoot= %.2f%%\n', expInfo.Overshoot, simInfo.Overshoot); 


%9.3 Calculating Steady State Error 
% Get expected ramp function to extract steady state error 
t = 0:0.01:2; 
slope = 5; 
expectedRamp = slope*t; 
% STEP 1: STEADY STATE ERROR FOR RAMP P CONTROLLER------------------------- 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
expIndices = pRampMeasuredTime >= startTime & pRampMeasuredTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, pRampMeasuredTime(expIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(pRampMeasuredResponse(expIndices) - expectedRampInterp)); 
% Display result 
fprintf('P: Average Steady-State Error for Measured Ramp Response: %.4f mm\n', steadyStateError); 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
simIndices = pRampSimTime >= startTime & pRampSimTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, pRampSimTime(simIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(pRampSimResponse(simIndices) - expectedRampInterp)); 
% Display result 
fprintf('P: Average Steady-State Error for Simulated Ramp Response: %.4f mm\n', steadyStateError); 
% STEP 2: STEADY STATE ERROR FOR RAMP LL CONTROLLER------------------------ 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
expIndices = LLRampMeasuredTime >= startTime & LLRampMeasuredTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, LLRampMeasuredTime(expIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(LLRampMeasuredResponse(expIndices) - expectedRampInterp)); 
% Display result 
fprintf('LL: Average Steady-State Error for Measured Ramp Response: %.4f mm\n', steadyStateError); 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
simIndices = LLRampSimTime >= startTime & LLRampSimTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, LLRampSimTime(simIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(LLRampSimResponse(simIndices) - expectedRampInterp)); 
% Display result 
fprintf('LL: Average Steady-State Error for Simulated Ramp Response: %.4f mm\n', steadyStateError); 
% STEP 2: STEADY STATE ERROR FOR RAMP LLI CONTROLLER----------------------- 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
expIndices = LLIRampMeasuredTime >= startTime & LLIRampMeasuredTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, LLIRampMeasuredTime(expIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(LLIRampMeasuredResponse(expIndices) - expectedRampInterp)); 
% Display result 
fprintf('LLI: Average Steady-State Error for Measured Ramp Response: %.4f mm\n', steadyStateError); 
% Define the time range for steady-state error calculation 
startTime = 1; 
endTime = 2; 
% Find indices for the time range in simulated response 
simIndices = LLIRampSimTime >= startTime & LLIRampSimTime <= endTime; 
% Interpolate expected ramp values at the specific time points of pRampSimTime 
expectedRampInterp = interp1(t, expectedRamp, LLIRampSimTime(simIndices)); 
% Calculate the steady-state error as the average absolute difference 
steadyStateError = mean(abs(LLIRampSimResponse(simIndices) - expectedRampInterp)); 
% Display result 
fprintf('LLI: Average Steady-State Error for Simulated Ramp Response: %.4f mm\n', steadyStateError); 