function KF_ToA_DiagR(iteration)
% variables
num_sample = 11;
dt = 0.1;
load('LSE.mat');
load('Q.mat');
load('P.mat');
load('Z.mat');
load('Rmean.mat');
load('meanSysnoise.mat');

diagR_est_state = struct('var001', zeros(2,iteration,num_sample),...
    'var01', zeros(2,iteration,num_sample),...
    'var1', zeros(2,iteration,num_sample),...
    'var10', zeros(2,iteration,num_sample),...
    'var100', zeros(2,iteration,num_sample)...
    );

diagR_est_covariance = struct('var001', zeros(2,2,iteration,num_sample),...
    'var01', zeros(2,2,iteration,num_sample),...
    'var1', zeros(2,2,iteration,num_sample),...
    'var10', zeros(2,2,iteration,num_sample),...
    'var100', zeros(2,2,iteration,num_sample)...
    );

diagR_est_KalmanGain = struct('var001', zeros(2,6,iteration,num_sample),...
    'var01', zeros(2,6,iteration,num_sample),...
    'var1', zeros(2,6,iteration,num_sample),...
    'var10', zeros(2,6,iteration,num_sample),...
    'var100', zeros(2,6,iteration,num_sample)...
    );

Q.var001 = diag(diag(Q.var001));
Q.var01 = diag(diag(Q.var01));
Q.var1 = diag(diag(Q.var1));
Q.var10 = diag(diag(Q.var10));
Q.var100 = diag(diag(Q.var100));

P.var001 = diag(diag(P.var001));
P.var01 = diag(diag(P.var01));
P.var1 = diag(diag(P.var1));
P.var10 = diag(diag(P.var10));  
P.var100 = diag(diag(P.var100));
Qbuf = Q;

% optimal_gamma = [0.59 0.61 0.62 0.65 0.74]; %linear form
optimal_gamma = [0.55 0.55 0.55 0.57 0.65]; %exp form
% optimal_gamma=[0.14 0.14 0.14 0.14 0.10];
for iter = 1:iteration
    Q=Qbuf;
    for num = 1:num_sample
        switch num
            case 1
                diagR_est_state.var001(:,iter,num) = [0;0];
                diagR_est_covariance.var001(:,:,iter,num) = P.var001;
                velocity_var001 = [0;0];

                diagR_est_state.var01(:,iter,num) = [0;0];
                diagR_est_covariance.var01(:,:,iter,num) = P.var01;
                velocity_var01 = [0;0];

                diagR_est_state.var1(:,iter,num) = [0;0];
                diagR_est_covariance.var1(:,:,iter,num) = P.var1;
                velocity_var1 = [0;0];

                diagR_est_state.var10(:,iter,num) = [0;0];
                diagR_est_covariance.var10(:,:,iter,num) = P.var10;
                velocity_var10 = [0;0];

                diagR_est_state.var100(:,iter,num) = [0;0];
                diagR_est_covariance.var100(:,:,iter,num) = P.var100;
                velocity_var100 = [0;0];
            case 2
                diagR_est_covariance.var001(:,:,iter,num) = diagR_est_covariance.var001(:,:,iter,num-1);
                diagR_est_covariance.var01(:,:,iter,num) = diagR_est_covariance.var01(:,:,iter,num-1);
                diagR_est_covariance.var1(:,:,iter,num) = diagR_est_covariance.var1(:,:,iter,num-1);
                diagR_est_covariance.var10(:,:,iter,num) = diagR_est_covariance.var10(:,:,iter,num-1);
                diagR_est_covariance.var100(:,:,iter,num) = diagR_est_covariance.var100(:,:,iter,num-1);

                diagR_est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                diagR_est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                diagR_est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                diagR_est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                diagR_est_state.var100(:,iter,num) = LSE.var100(:,iter,num);
            case 3
                diagR_est_covariance.var001(:,:,iter,num) = diagR_est_covariance.var001(:,:,iter,num-1);
                diagR_est_covariance.var01(:,:,iter,num) = diagR_est_covariance.var01(:,:,iter,num-1);
                diagR_est_covariance.var1(:,:,iter,num) = diagR_est_covariance.var1(:,:,iter,num-1);
                diagR_est_covariance.var10(:,:,iter,num) = diagR_est_covariance.var10(:,:,iter,num-1);
                diagR_est_covariance.var100(:,:,iter,num) = diagR_est_covariance.var100(:,:,iter,num-1);

                diagR_est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                diagR_est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                diagR_est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                diagR_est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                diagR_est_state.var100(:,iter,num) = LSE.var100(:,iter,num);

                velocity_var001 = (diagR_est_state.var001(:,iter,num) - diagR_est_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (diagR_est_state.var01(:,iter,num) - diagR_est_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (diagR_est_state.var1(:,iter,num) - diagR_est_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (diagR_est_state.var10(:,iter,num) - diagR_est_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (diagR_est_state.var100(:,iter,num) - diagR_est_state.var100(:,iter,num-1))./dt;
            otherwise
                [diagR_est_state_var001, diagR_est_covariance_var001, diagR_kalman_gain_001] =...
                    kalmanFilter(diagR_est_state.var001(:,iter,num-1),diagR_est_covariance.var001(:,:,iter,num-1),velocity_var001,Q.var001,diag(diag(Rmean.var001(:,:,1,num))),Z.var001(:,1,iter,num),meanSysnoise.var001);
                [diagR_est_state_var01, diagR_est_covariance_var01, diagR_kalman_gain_01] =...
                    kalmanFilter(diagR_est_state.var01(:,iter,num-1),diagR_est_covariance.var01(:,:,iter,num-1),velocity_var01,Q.var01,diag(diag(Rmean.var01(:,:,1,num))),Z.var01(:,1,iter,num),meanSysnoise.var01);
                [diagR_est_state_var1, diagR_est_covariance_var1, diagR_kalman_gain_1] =...
                    kalmanFilter(diagR_est_state.var1(:,iter,num-1),diagR_est_covariance.var1(:,:,iter,num-1),velocity_var1,Q.var1,diag(diag(Rmean.var1(:,:,1,num))),Z.var1(:,1,iter,num),meanSysnoise.var1);
                [diagR_est_state_var10, diagR_est_covariance_var10, diagR_kalman_gain_10] =...
                    kalmanFilter(diagR_est_state.var10(:,iter,num-1),diagR_est_covariance.var10(:,:,iter,num-1),velocity_var10,Q.var10,diag(diag(Rmean.var10(:,:,1,num))),Z.var10(:,1,iter,num),meanSysnoise.var10);
                [diagR_est_state_var100, diagR_est_covariance_var100, diagR_kalman_gain_100] =...
                    kalmanFilter(diagR_est_state.var100(:,iter,num-1),diagR_est_covariance.var100(:,:,iter,num-1),velocity_var100,Q.var100,diag(diag(Rmean.var100(:,:,1,num))),Z.var100(:,1,iter,num),meanSysnoise.var100);

                diagR_est_state.var001(:,iter,num) = diagR_est_state_var001;
                diagR_est_state.var01(:,iter,num) = diagR_est_state_var01;
                diagR_est_state.var1(:,iter,num) = diagR_est_state_var1;
                diagR_est_state.var10(:,iter,num) = diagR_est_state_var10;
                diagR_est_state.var100(:,iter,num) = diagR_est_state_var100;

                diagR_est_covariance.var001(:,:,iter,num) = diagR_est_covariance_var001;
                diagR_est_covariance.var01(:,:,iter,num) = diagR_est_covariance_var01;
                diagR_est_covariance.var1(:,:,iter,num) = diagR_est_covariance_var1;
                diagR_est_covariance.var10(:,:,iter,num) = diagR_est_covariance_var10;
                diagR_est_covariance.var100(:,:,iter,num) = diagR_est_covariance_var100;

                diagR_est_KalmanGain.var001(:,:,iter,num) = diagR_kalman_gain_001;
                diagR_est_KalmanGain.var01(:,:,iter,num) = diagR_kalman_gain_01;
                diagR_est_KalmanGain.var1(:,:,iter,num) = diagR_kalman_gain_1;
                diagR_est_KalmanGain.var10(:,:,iter,num) = diagR_kalman_gain_10;
                diagR_est_KalmanGain.var100(:,:,iter,num) = diagR_kalman_gain_100;

                velocity_var001 = (diagR_est_state.var001(:,iter,num) - diagR_est_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (diagR_est_state.var01(:,iter,num) - diagR_est_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (diagR_est_state.var1(:,iter,num) - diagR_est_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (diagR_est_state.var10(:,iter,num) - diagR_est_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (diagR_est_state.var100(:,iter,num) - diagR_est_state.var100(:,iter,num-1))./dt;

                Q.var001 = Q.var001*optimal_gamma(1);
                Q.var01 = Q.var01*optimal_gamma(2);
                Q.var1 = Q.var1*optimal_gamma(3);
                Q.var10 = Q.var10*optimal_gamma(4);
                Q.var100 = Q.var100*optimal_gamma(5);

                % Q.var001 = Q.var001*exp(-optimal_gamma(1)*(num-3));
                % Q.var01 = Q.var01*exp(-optimal_gamma(2)*(num-3));
                % Q.var1 = Q.var1*exp(-optimal_gamma(3)*(num-3));
                % Q.var10 = Q.var10*exp(-optimal_gamma(4)*(num-3));
                % Q.var100 = Q.var100*exp(-optimal_gamma(5)*(num-3));
        end
    end
end

save('diagR_est_state.mat','diagR_est_state');
% save('diagR_est_covariance.mat','diagR_est_covariance');
% save('diagR_est_KalmanGain.mat','diagR_est_KalmanGain');
% n_variance = [0.01; 0.1; 1; 10; 100];
a = mean(diagR_est_covariance.var1,3);
for i = 1:num_sample
    buf(i,1) = trace(a(:,:,1,i));
end
figure;
stem((1:num_sample),buf);
title('trace(P) KF diagR')
xlabel('step')
ylabel('trace(P_k)')
end