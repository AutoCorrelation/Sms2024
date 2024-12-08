function [prediction_state] =...
    kalmanFilter_prediction(prev_estimate_state,prev_velocity,meanSysnoise)
% Kalman Filter
% Input: predict_state, predict_cov
% Output: estimate_state, estimate_covariance, kalman_gain
dt = 0.1;
F = [1 0;0 1];

% predict
prediction_state = F * prev_estimate_state + prev_velocity*dt;
end