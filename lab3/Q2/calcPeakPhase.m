function [peakPhase] = calcPeakPhase(currentPhase)
%Calculate Peak Phase for Lead compensator
targetPhaseMargin = 60; %[deg]
peakPhase = targetPhaseMargin - currentPhase - 180;
end