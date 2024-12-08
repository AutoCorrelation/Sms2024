clear
clc
close all

iteration=1e3;
tic

preSimulation(iteration);
findLPF_ToA(iteration);
findPredict_ToA(iteration);
KF_ToA(iteration);
KF_ToA_DiagQ(iteration);
findKF_ToA_AdaptiveQ(iteration);
RMSE(iteration);

toc