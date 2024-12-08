function [estimate_state,estimate_covariance,kalman_gain] = kalmanFilter(prev_estimate_state,prev_estimate_cov,prev_velocity,Q,R,measurement,meanSysnoise)
    persistent H dt F firstRun
    if isempty(firstRun)
        firstRun = 1;
        d1=10; d2=d1;
        H = [0, -2*d2; 2*d1, -2*d2; 2*d1, 0; 2*d1, 0; 2*d1, 2*d2; 0, 2*d2];
        dt = 0.1;
        F = eye(2);
    end

    % Predict
    predict_state = F * prev_estimate_state + prev_velocity*dt + meanSysnoise;
    predict_covariance = F * prev_estimate_cov * F' + Q;

    % Update
    S = (H * predict_covariance * H' + R);
    % kalman_gain = (predict_covariance * H') / S;
    kalman_gain = (predict_covariance * H') *pinv(S);
    
    % Estimate
    estimate_state = predict_state + kalman_gain * (measurement - H * predict_state);
    % I_KH = (eye(2) - kalman_gain * H);
    % estimate_covariance = I_KH * predict_covariance * I_KH' + kalman_gain * R * kalman_gain';
    estimate_covariance = predict_covariance - kalman_gain * H * predict_covariance;
end