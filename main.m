close all;
clear
clc
iteration = 1e4;

preSimulation(iteration); % include the preSimulation and ToA functions

% Measure time for optimal_Lpf_ToA
tic;
optimal_Lpf_ToA(iteration);
optimalLpfToATime = toc;

% Measure time for optimal_Predict_ToA
tic;
optimal_Predict_ToA(iteration);
optimalPredictToATime = toc;

% Measure time for KF_ToA
tic;
KF_ToA(iteration);
KFToATime = toc;

% Measure time for KF_ToA_designQ1
tic;
KF_ToA_designQ1(iteration);
KFToADesignQ1Time = toc;

% Measure time for KF_ToA_designQ2
tic;
KF_ToA_designQ2(iteration);
KFToADesignQ2Time = toc;

% Measure time for KF_ToA_DiagR
tic;
KF_ToA_DiagR(iteration);
KFToADiagRTime = toc;

tic
KF_ToA_designQ3(iteration);
KF_ToA_designQ3 = toc;

ResultofSimulation(iteration);

% Plot execution times
executionTimes = [optimalLpfToATime, optimalPredictToATime, KFToATime, KFToADesignQ1Time, KFToADesignQ2Time, KFToADiagRTime, KF_ToA_designQ3];
labels = {'optimal\_Lpf\_ToA', 'optimal\_Predict\_ToA', 'KF\_ToA', 'KF\_ToA\_designQ1', 'KF\_ToA\_designQ2', 'KF\_ToA\_DiagR', 'KF\_ToA\_designQ3'};

figure;
bar(executionTimes);
set(gca, 'XTickLabel', labels);
ylabel('Time (s)');
title('Execution Time Comparison');
xtickangle(45);
grid on;

