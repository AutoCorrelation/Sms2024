function KF_ToA(iteration)
    % variables
    num_sample = 11;
    dt = 0.1;
    load('LSE.mat');
    load('Q.mat');
    load('P.mat');
    load('Z.mat');
    load('Rmean.mat');
    load('meanSysnoise.mat');

    est_state = struct('var001', zeros(2,iteration,num_sample),...
        'var01', zeros(2,iteration,num_sample),...
        'var1', zeros(2,iteration,num_sample),...
        'var10', zeros(2,iteration,num_sample),...
        'var100', zeros(2,iteration,num_sample)...
        );

    est_covariance = struct('var001', zeros(2,2,iteration,num_sample),...
        'var01', zeros(2,2,iteration,num_sample),...
        'var1', zeros(2,2,iteration,num_sample),...
        'var10', zeros(2,2,iteration,num_sample),...
        'var100', zeros(2,2,iteration,num_sample)...
        );

    est_KalmanGain = struct('var001', zeros(2,6,iteration,num_sample),...
        'var01', zeros(2,6,iteration,num_sample),...
        'var1', zeros(2,6,iteration,num_sample),...
        'var10', zeros(2,6,iteration,num_sample),...
        'var100', zeros(2,6,iteration,num_sample)...
        );

    for iter = 1:iteration
        for num = 1:num_sample
            switch num
                case 1
                    est_state.var001(:,iter,num) = [0;0];
                    est_covariance.var001(:,:,iter,num) = P.var001;
                    velocity_var001 = [0 ;0];

                    est_state.var01(:,iter,num) = [0;0];
                    est_covariance.var01(:,:,iter,num) = P.var01;
                    velocity_var01 = [0;0];

                    est_state.var1(:,iter,num) = [0;0];
                    est_covariance.var1(:,:,iter,num) = P.var1;
                    velocity_var1 = [0;0];

                    est_state.var10(:,iter,num) = [0;0];
                    est_covariance.var10(:,:,iter,num) = P.var10;
                    velocity_var10 = [0;0];

                    est_state.var100(:,iter,num) = [0;0];
                    est_covariance.var100(:,:,iter,num) = P.var100;
                    velocity_var100 = [0;0];
                case 2
                    est_covariance.var001(:,:,iter,num) = est_covariance.var001(:,:,iter,num-1);
                    est_covariance.var01(:,:,iter,num) = est_covariance.var01(:,:,iter,num-1);
                    est_covariance.var1(:,:,iter,num) = est_covariance.var1(:,:,iter,num-1);
                    est_covariance.var10(:,:,iter,num) = est_covariance.var10(:,:,iter,num-1);
                    est_covariance.var100(:,:,iter,num) = est_covariance.var100(:,:,iter,num-1);

                    est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                    est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                    est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                    est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                    est_state.var100(:,iter,num) = LSE.var100(:,iter,num);
                case 3
                    est_covariance.var001(:,:,iter,num) = est_covariance.var001(:,:,iter,num-1);
                    est_covariance.var01(:,:,iter,num) = est_covariance.var01(:,:,iter,num-1);
                    est_covariance.var1(:,:,iter,num) = est_covariance.var1(:,:,iter,num-1);
                    est_covariance.var10(:,:,iter,num) = est_covariance.var10(:,:,iter,num-1);
                    est_covariance.var100(:,:,iter,num) = est_covariance.var100(:,:,iter,num-1);

                    est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                    est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                    est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                    est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                    est_state.var100(:,iter,num) = LSE.var100(:,iter,num);

                    velocity_var001 = (est_state.var001(:,iter,num) - est_state.var001(:,iter,num-1))./dt;
                    velocity_var01 = (est_state.var01(:,iter,num) - est_state.var01(:,iter,num-1))./dt;
                    velocity_var1 = (est_state.var1(:,iter,num) - est_state.var1(:,iter,num-1))./dt;
                    velocity_var10 = (est_state.var10(:,iter,num) - est_state.var10(:,iter,num-1))./dt;
                    velocity_var100 = (est_state.var100(:,iter,num) - est_state.var100(:,iter,num-1))./dt;
                otherwise
                    [est_state_var001, est_covariance_var001, kalman_gain_001] =...
                        kalmanFilter(est_state.var001(:,iter,num-1),est_covariance.var001(:,:,iter,num-1),velocity_var001,Q.var001,Rmean.var001(:,:,1,num),Z.var001(:,1,iter,num),meanSysnoise.var001);
                    [est_state_var01, est_covariance_var01, kalman_gain_01] =...
                        kalmanFilter(est_state.var01(:,iter,num-1),est_covariance.var01(:,:,iter,num-1),velocity_var01,Q.var01,Rmean.var01(:,:,1,num),Z.var01(:,1,iter,num),meanSysnoise.var01);
                    [est_state_var1, est_covariance_var1, kalman_gain_1] =...
                        kalmanFilter(est_state.var1(:,iter,num-1),est_covariance.var1(:,:,iter,num-1),velocity_var1,Q.var1,Rmean.var1(:,:,1,num),Z.var1(:,1,iter,num),meanSysnoise.var1);
                    [est_state_var10, est_covariance_var10, kalman_gain_10] =...
                        kalmanFilter(est_state.var10(:,iter,num-1),est_covariance.var10(:,:,iter,num-1),velocity_var10,Q.var10,Rmean.var10(:,:,1,num),Z.var10(:,1,iter,num),meanSysnoise.var10);
                    [est_state_var100, est_covariance_var100, kalman_gain_100] =...
                        kalmanFilter(est_state.var100(:,iter,num-1),est_covariance.var100(:,:,iter,num-1),velocity_var100,Q.var100,Rmean.var100(:,:,1,num),Z.var100(:,1,iter,num),meanSysnoise.var100);

                    est_state.var001(:,iter,num) = est_state_var001;
                    est_state.var01(:,iter,num) = est_state_var01;
                    est_state.var1(:,iter,num) = est_state_var1;
                    est_state.var10(:,iter,num) = est_state_var10;
                    est_state.var100(:,iter,num) = est_state_var100;

                    est_covariance.var001(:,:,iter,num) = est_covariance_var001;
                    est_covariance.var01(:,:,iter,num) = est_covariance_var01;
                    est_covariance.var1(:,:,iter,num) = est_covariance_var1;
                    est_covariance.var10(:,:,iter,num) = est_covariance_var10;
                    est_covariance.var100(:,:,iter,num) = est_covariance_var100;

                    est_KalmanGain.var001(:,:,iter,num) = kalman_gain_001;
                    est_KalmanGain.var01(:,:,iter,num) = kalman_gain_01;
                    est_KalmanGain.var1(:,:,iter,num) = kalman_gain_1;
                    est_KalmanGain.var10(:,:,iter,num) = kalman_gain_10;
                    est_KalmanGain.var100(:,:,iter,num) = kalman_gain_100;

                    velocity_var001 = (est_state.var001(:,iter,num) - est_state.var001(:,iter,num-1))./dt;
                    velocity_var01 = (est_state.var01(:,iter,num) - est_state.var01(:,iter,num-1))./dt;
                    velocity_var1 = (est_state.var1(:,iter,num) - est_state.var1(:,iter,num-1))./dt;
                    velocity_var10 = (est_state.var10(:,iter,num) - est_state.var10(:,iter,num-1))./dt;
                    velocity_var100 = (est_state.var100(:,iter,num) - est_state.var100(:,iter,num-1))./dt;
            end
        end
    end

    save('est_state.mat','est_state');
    % save('est_covariance.mat','est_covariance');
    save('est_KalmanGain.mat','est_KalmanGain');
    % n_variance = [0.01; 0.1; 1; 10; 100];
    a = mean(est_covariance.var1,3);
    for i = 1:num_sample
        buf(i,1) = trace(a(:,:,1,i));
    end
    figure;
    stem((1:num_sample),buf);
    title('trace(P) KF')
    xlabel('step')
    ylabel('trace(P_k)')
end