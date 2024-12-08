function findOptimalGamma(iteration)
% variables
num_sample = 11;
dt = 0.1;
alphamax = 9;
load('LSE.mat');
load('Q.mat');
load('P.mat');
load('Z.mat');
load('Rmean.mat');
load('meanSysnoise.mat');
% Q.var001 = diag(diag(Q.var001));
% Q.var01 = diag(diag(Q.var01));
% Q.var1 = diag(diag(Q.var1));
% Q.var10 = diag(diag(Q.var10));
% Q.var100 = diag(diag(Q.var100));
% 
% P.var001 = diag(diag(P.var001));
% P.var01 = diag(diag(P.var01));
% P.var1 = diag(diag(P.var1));
% P.var10 = diag(diag(P.var10));
% P.var100 = diag(diag(P.var100));
Qbuf=Q;
AdaptiveQ_est_state = struct('var001', zeros(2,iteration,num_sample,alphamax),...
    'var01', zeros(2,iteration,num_sample,alphamax),...
    'var1', zeros(2,iteration,num_sample,alphamax),...
    'var10', zeros(2,iteration,num_sample,alphamax),...
    'var100', zeros(2,iteration,num_sample,alphamax)...
    );

AdaptiveQ_est_covariance = struct('var001', zeros(2,2,iteration,num_sample,alphamax),...
    'var01', zeros(2,2,iteration,num_sample,alphamax),...
    'var1', zeros(2,2,iteration,num_sample,alphamax),...
    'var10', zeros(2,2,iteration,num_sample,alphamax),...
    'var100', zeros(2,2,iteration,num_sample,alphamax)...
    );

AdaptiveQ_KalmanGain = struct('var001', zeros(2,6,iteration,num_sample,alphamax),...
    'var01', zeros(2,6,iteration,num_sample,alphamax),...
    'var1', zeros(2,6,iteration,num_sample,alphamax),...
    'var10', zeros(2,6,iteration,num_sample,alphamax),...
    'var100', zeros(2,6,iteration,num_sample,alphamax)...
    );
%0.06~0.1400 0.01 iter1e4
%77762=[0.12 0.12 0.12 0.11 0.07]
%88762=[0.13 0.13 0.12 0.11 0.07]

%Diag Q test
%88762=[0.13 0.13 0.12 0.11 0.07]
% 51 gamma 6e-2+(a-1)*1e-2/5;
% 37 34 34 33 7 [0.1320 0.1260 0.1260 0.1240 0.0720]
% [0.14 0.14 0.14 0.14 0.10]

% linearly decreasing alpha
% 66668 = [0.6 0.6 0.6 0.6 0.8]
%alpha = 0.5+0.04*(a-1);
%44447 = [0.62 0.62 0.62 0.62 0.74]
% alpha = 0.6+0.02*(a-1);
%22238 = [0.6000    0.6000    0.6200    0.6400    0.7400]  // [0.55 0.55 0.55 0.57 0.65]

% diag exp Q R test
% 2 1 1 1 1
%[0.16 0.15 0.15 0.12 0.11];

% KF 직전 Q 줄이기
% [0.098 0.098 0.090 0.082 0.058]

for a = 1:alphamax
    alpha = (a-1)*0.02;
    % alpha = 0.59+(a-1)*0.01;
    % alpha = 0.52+0.01*(a-1);
    % alpha = 0.07+0.01*(a-1);
    % alpha = 0.1+0.01*(a-1);
    for iter = 1:iteration
        Q=Qbuf;
        for num = 1:num_sample
            switch num
                case 1
                    AdaptiveQ_est_state.var001(:,iter,num,a) = [0;0];
                    AdaptiveQ_est_covariance.var001(:,:,iter,num,a) = P.var001;
                    velocity_var001 = [0;0];

                    AdaptiveQ_est_state.var01(:,iter,num,a) = [0;0];
                    AdaptiveQ_est_covariance.var01(:,:,iter,num,a) = P.var01;
                    velocity_var01 = [0;0];

                    AdaptiveQ_est_state.var1(:,iter,num,a) = [0;0];
                    AdaptiveQ_est_covariance.var1(:,:,iter,num,a) = P.var1;
                    velocity_var1 = [0;0];

                    AdaptiveQ_est_state.var10(:,iter,num,a) = [0;0];
                    AdaptiveQ_est_covariance.var10(:,:,iter,num,a) = P.var10;
                    velocity_var10 = [0;0];

                    AdaptiveQ_est_state.var100(:,iter,num,a) = [0;0];
                    AdaptiveQ_est_covariance.var100(:,:,iter,num,a) = P.var100;
                    velocity_var100 = [0;0];
                case 2
                    AdaptiveQ_est_covariance.var001(:,:,iter,num,a) = AdaptiveQ_est_covariance.var001(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var01(:,:,iter,num,a) = AdaptiveQ_est_covariance.var01(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var1(:,:,iter,num,a) = AdaptiveQ_est_covariance.var1(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var10(:,:,iter,num,a) = AdaptiveQ_est_covariance.var10(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var100(:,:,iter,num,a) = AdaptiveQ_est_covariance.var100(:,:,iter,num-1,a);

                    AdaptiveQ_est_state.var001(:,iter,num,a) =LSE.var001(:,iter,num);
                    AdaptiveQ_est_state.var01(:,iter,num,a) =  LSE.var01(:,iter,num);
                    AdaptiveQ_est_state.var1(:,iter,num,a) =    LSE.var1(:,iter,num);
                    AdaptiveQ_est_state.var10(:,iter,num,a) =  LSE.var10(:,iter,num);
                    AdaptiveQ_est_state.var100(:,iter,num,a) =LSE.var100(:,iter,num);
                case 3
                    AdaptiveQ_est_covariance.var001(:,:,iter,num,a) = AdaptiveQ_est_covariance.var001(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var01(:,:,iter,num,a) = AdaptiveQ_est_covariance.var01(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var1(:,:,iter,num,a) = AdaptiveQ_est_covariance.var1(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var10(:,:,iter,num,a) = AdaptiveQ_est_covariance.var10(:,:,iter,num-1,a);
                    AdaptiveQ_est_covariance.var100(:,:,iter,num,a) = AdaptiveQ_est_covariance.var100(:,:,iter,num-1,a);

                    AdaptiveQ_est_state.var001(:,iter,num,a) =LSE.var001(:,iter,num);
                    AdaptiveQ_est_state.var01(:,iter,num,a) =  LSE.var01(:,iter,num);
                    AdaptiveQ_est_state.var1(:,iter,num,a) =    LSE.var1(:,iter,num);
                    AdaptiveQ_est_state.var10(:,iter,num,a) =  LSE.var10(:,iter,num);
                    AdaptiveQ_est_state.var100(:,iter,num,a) =LSE.var100(:,iter,num);

                    velocity_var001 = (AdaptiveQ_est_state.var001(:,iter,num,a) - AdaptiveQ_est_state.var001(:,iter,num-1,a))./dt;
                    velocity_var01 = (AdaptiveQ_est_state.var01(:,iter,num,a) - AdaptiveQ_est_state.var01(:,iter,num-1,a))./dt;
                    velocity_var1 = (AdaptiveQ_est_state.var1(:,iter,num,a) - AdaptiveQ_est_state.var1(:,iter,num-1,a))./dt;
                    velocity_var10 = (AdaptiveQ_est_state.var10(:,iter,num,a) - AdaptiveQ_est_state.var10(:,iter,num-1,a))./dt;
                    velocity_var100 = (AdaptiveQ_est_state.var100(:,iter,num,a) - AdaptiveQ_est_state.var100(:,iter,num-1,a))./dt;
                otherwise
                    % [est_state_var001, est_covariance_var001, kalman_gain_001] =...
                    %     kalmanFilter(AdaptiveQ_est_state.var001(:,iter,num-1,a),AdaptiveQ_est_covariance.var001(:,:,iter,num-1,a),velocity_var001,Q.var001,diag(diag(Rmean.var001(:,:,1,num))),Z.var001(:,1,iter,num),meanSysnoise.var001);
                    % [est_state_var01, est_covariance_var01, kalman_gain_01] =...
                    %     kalmanFilter(AdaptiveQ_est_state.var01(:,iter,num-1,a),AdaptiveQ_est_covariance.var01(:,:,iter,num-1,a),velocity_var01,Q.var01,diag(diag(Rmean.var01(:,:,1,num))),Z.var01(:,1,iter,num),meanSysnoise.var01);
                    % [est_state_var1, est_covariance_var1, kalman_gain_1] =...
                    %     kalmanFilter(AdaptiveQ_est_state.var1(:,iter,num-1,a),AdaptiveQ_est_covariance.var1(:,:,iter,num-1,a),velocity_var1,Q.var1,diag(diag(Rmean.var1(:,:,1,num))),Z.var1(:,1,iter,num),meanSysnoise.var1);
                    % [est_state_var10, est_covariance_var10, kalman_gain_10] =...
                    %     kalmanFilter(AdaptiveQ_est_state.var10(:,iter,num-1,a),AdaptiveQ_est_covariance.var10(:,:,iter,num-1,a),velocity_var10,Q.var10,diag(diag(Rmean.var10(:,:,1,num))),Z.var10(:,1,iter,num),meanSysnoise.var10);
                    % [est_state_var100, est_covariance_var100, kalman_gain_100] =...
                    %     kalmanFilter(AdaptiveQ_est_state.var100(:,iter,num-1,a),AdaptiveQ_est_covariance.var100(:,:,iter,num-1,a),velocity_var100,Q.var100,diag(diag(Rmean.var100(:,:,1,num))),Z.var100(:,1,iter,num),meanSysnoise.var100);

                    [est_state_var001, est_covariance_var001, kalman_gain_001] =...
                        kalmanFilter(AdaptiveQ_est_state.var001(:,iter,num-1,a),AdaptiveQ_est_covariance.var001(:,:,iter,num-1,a),velocity_var001,Q.var001,((Rmean.var001(:,:,1,num))),Z.var001(:,1,iter,num),meanSysnoise.var001);
                    [est_state_var01, est_covariance_var01, kalman_gain_01] =...
                        kalmanFilter(AdaptiveQ_est_state.var01(:,iter,num-1,a),AdaptiveQ_est_covariance.var01(:,:,iter,num-1,a),velocity_var01,Q.var01,((Rmean.var01(:,:,1,num))),Z.var01(:,1,iter,num),meanSysnoise.var01);
                    [est_state_var1, est_covariance_var1, kalman_gain_1] =...
                        kalmanFilter(AdaptiveQ_est_state.var1(:,iter,num-1,a),AdaptiveQ_est_covariance.var1(:,:,iter,num-1,a),velocity_var1,Q.var1,((Rmean.var1(:,:,1,num))),Z.var1(:,1,iter,num),meanSysnoise.var1);
                    [est_state_var10, est_covariance_var10, kalman_gain_10] =...
                        kalmanFilter(AdaptiveQ_est_state.var10(:,iter,num-1,a),AdaptiveQ_est_covariance.var10(:,:,iter,num-1,a),velocity_var10,Q.var10,((Rmean.var10(:,:,1,num))),Z.var10(:,1,iter,num),meanSysnoise.var10);
                    [est_state_var100, est_covariance_var100, kalman_gain_100] =...
                        kalmanFilter(AdaptiveQ_est_state.var100(:,iter,num-1,a),AdaptiveQ_est_covariance.var100(:,:,iter,num-1,a),velocity_var100,Q.var100,((Rmean.var100(:,:,1,num))),Z.var100(:,1,iter,num),meanSysnoise.var100);

                    AdaptiveQ_est_state.var001(:,iter,num,a) = est_state_var001;
                    AdaptiveQ_est_state.var01(:,iter,num,a) = est_state_var01;
                    AdaptiveQ_est_state.var1(:,iter,num,a) = est_state_var1;
                    AdaptiveQ_est_state.var10(:,iter,num,a) = est_state_var10;
                    AdaptiveQ_est_state.var100(:,iter,num,a) = est_state_var100;

                    AdaptiveQ_est_covariance.var001(:,:,iter,num,a) = est_covariance_var001;
                    AdaptiveQ_est_covariance.var01(:,:,iter,num,a) = est_covariance_var01;
                    AdaptiveQ_est_covariance.var1(:,:,iter,num,a) = est_covariance_var1;
                    AdaptiveQ_est_covariance.var10(:,:,iter,num,a) = est_covariance_var10;
                    AdaptiveQ_est_covariance.var100(:,:,iter,num,a) = est_covariance_var100;

                    AdaptiveQ_KalmanGain.var001(:,:,iter,num,a) = kalman_gain_001;
                    AdaptiveQ_KalmanGain.var01(:,:,iter,num,a) = kalman_gain_01;
                    AdaptiveQ_KalmanGain.var1(:,:,iter,num,a) = kalman_gain_1;
                    AdaptiveQ_KalmanGain.var10(:,:,iter,num,a) = kalman_gain_10;
                    AdaptiveQ_KalmanGain.var100(:,:,iter,num,a) = kalman_gain_100;

                    velocity_var001 = (AdaptiveQ_est_state.var001(:,iter,num,a) - AdaptiveQ_est_state.var001(:,iter,num-1,a))./dt;
                    velocity_var01 = (AdaptiveQ_est_state.var01(:,iter,num,a) - AdaptiveQ_est_state.var01(:,iter,num-1,a))./dt;
                    velocity_var1 = (AdaptiveQ_est_state.var1(:,iter,num,a) - AdaptiveQ_est_state.var1(:,iter,num-1,a))./dt;
                    velocity_var10 = (AdaptiveQ_est_state.var10(:,iter,num,a) - AdaptiveQ_est_state.var10(:,iter,num-1,a))./dt;
                    velocity_var100 = (AdaptiveQ_est_state.var100(:,iter,num,a) - AdaptiveQ_est_state.var100(:,iter,num-1,a))./dt;



                    Q.var001 = Q.var001*exp(-alpha*(num-3));
                    Q.var01 = Q.var01*exp(-alpha*(num-3));
                    Q.var1 = Q.var1*exp(-alpha*(num-3));
                    Q.var10 = Q.var10*exp(-alpha*(num-3));
                    Q.var100 = Q.var100*exp(-alpha*(num-3));
                    % 
                    % Q.var001 = Q.var001*alpha;
                    % Q.var01 = Q.var01*alpha;
                    % Q.var1 = Q.var1*alpha;
                    % Q.var10 = Q.var10*alpha;
                    % Q.var100 = Q.var100*alpha;
            end
        end
    end
end

% save('AdaptiveQ_est_state.mat','AdaptiveQ_est_state');

% load('AdaptiveQ_est_state.mat');

n_variance = [0.01; 0.1; 1; 10; 100];
gammamax = size(AdaptiveQ_est_state.var001, 4);
% iteration = 1e4;
% num_sample = 11;
KF_mseQ_buf = zeros(length(n_variance), gammamax);

for iter = 1:iteration
    for num = 1:num_sample
        exactPos = [num-1; num-1];
        for g = 1:gammamax
            KF_mseQ_buf(:, g) = KF_mseQ_buf(:, g) + [
                norm(AdaptiveQ_est_state.var001(:, iter, num, g) - exactPos);
                norm(AdaptiveQ_est_state.var01(:, iter, num, g) - exactPos);
                norm(AdaptiveQ_est_state.var1(:, iter, num, g) - exactPos);
                norm(AdaptiveQ_est_state.var10(:, iter, num, g) - exactPos);
                norm(AdaptiveQ_est_state.var100(:, iter, num, g) - exactPos)
                ];
        end
    end
end

[KF_mseQ_min, optimal_gamma] = min(KF_mseQ_buf, [], 2);
KF_mseQ = KF_mseQ_min ./ (iteration * num_sample);
semilogx(n_variance,KF_mseQ);
disp('optimal gamma: ');
disp(optimal_gamma);

end