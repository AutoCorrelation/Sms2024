function KF_ToA_designQ3(iteration)
% variables
num_sample = 11;
dt = 0.1;
load('LSE.mat');
load('Q.mat');
load('P.mat');
load('Z.mat');
load('Rmean.mat');
load('meanSysnoise.mat');

designQ3_est_state = struct('var001', zeros(2,iteration,num_sample),...
    'var01', zeros(2,iteration,num_sample),...
    'var1', zeros(2,iteration,num_sample),...
    'var10', zeros(2,iteration,num_sample),...
    'var100', zeros(2,iteration,num_sample)...
    );

designQ3_est_covariance = struct('var001', zeros(2,2,iteration,num_sample),...
    'var01', zeros(2,2,iteration,num_sample),...
    'var1', zeros(2,2,iteration,num_sample),...
    'var10', zeros(2,2,iteration,num_sample),...
    'var100', zeros(2,2,iteration,num_sample)...
    );

designQ3_est_KalmanGain = struct('var001', zeros(2,6,iteration,num_sample),...
    'var01', zeros(2,6,iteration,num_sample),...
    'var1', zeros(2,6,iteration,num_sample),...
    'var10', zeros(2,6,iteration,num_sample),...
    'var100', zeros(2,6,iteration,num_sample)...
    );
Qbuf = Q;
alpha = 0.75;
for iter = 1:iteration
    Q = Qbuf;
    for num = 1:num_sample
        switch num
            case 1
                designQ3_est_state.var001(:,iter,num) = [0;0];
                designQ3_est_covariance.var001(:,:,iter,num) = P.var001;
                velocity_var001 = [0 ;0];

                designQ3_est_state.var01(:,iter,num) = [0;0];
                designQ3_est_covariance.var01(:,:,iter,num) = P.var01;
                velocity_var01 = [0;0];

                designQ3_est_state.var1(:,iter,num) = [0;0];
                designQ3_est_covariance.var1(:,:,iter,num) = P.var1;
                velocity_var1 = [0;0];

                designQ3_est_state.var10(:,iter,num) = [0;0];
                designQ3_est_covariance.var10(:,:,iter,num) = P.var10;
                velocity_var10 = [0;0];

                designQ3_est_state.var100(:,iter,num) = [0;0];
                designQ3_est_covariance.var100(:,:,iter,num) = P.var100;
                velocity_var100 = [0;0];
            case 2
                designQ3_est_covariance.var001(:,:,iter,num) = designQ3_est_covariance.var001(:,:,iter,num-1);
                designQ3_est_covariance.var01(:,:,iter,num) = designQ3_est_covariance.var01(:,:,iter,num-1);
                designQ3_est_covariance.var1(:,:,iter,num) = designQ3_est_covariance.var1(:,:,iter,num-1);
                designQ3_est_covariance.var10(:,:,iter,num) = designQ3_est_covariance.var10(:,:,iter,num-1);
                designQ3_est_covariance.var100(:,:,iter,num) = designQ3_est_covariance.var100(:,:,iter,num-1);

                designQ3_est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                designQ3_est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                designQ3_est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                designQ3_est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                designQ3_est_state.var100(:,iter,num) = LSE.var100(:,iter,num);
            case 3
                designQ3_est_covariance.var001(:,:,iter,num) = designQ3_est_covariance.var001(:,:,iter,num-1);
                designQ3_est_covariance.var01(:,:,iter,num) = designQ3_est_covariance.var01(:,:,iter,num-1);
                designQ3_est_covariance.var1(:,:,iter,num) = designQ3_est_covariance.var1(:,:,iter,num-1);
                designQ3_est_covariance.var10(:,:,iter,num) = designQ3_est_covariance.var10(:,:,iter,num-1);
                designQ3_est_covariance.var100(:,:,iter,num) = designQ3_est_covariance.var100(:,:,iter,num-1);

                designQ3_est_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                designQ3_est_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                designQ3_est_state.var1(:,iter,num) = LSE.var1(:,iter,num);
                designQ3_est_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                designQ3_est_state.var100(:,iter,num) = LSE.var100(:,iter,num);

                velocity_var001 = (designQ3_est_state.var001(:,iter,num) - designQ3_est_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (designQ3_est_state.var01(:,iter,num) - designQ3_est_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (designQ3_est_state.var1(:,iter,num) - designQ3_est_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (designQ3_est_state.var10(:,iter,num) - designQ3_est_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (designQ3_est_state.var100(:,iter,num) - designQ3_est_state.var100(:,iter,num-1))./dt;
            otherwise % //TODO: check the Z matrix.
                [designQ3_est_state_var001, designQ3_est_covariance_var001, kalmanGain_001, innov_001] =...
                    kalmanFilter2(designQ3_est_state.var001(:,iter,num-1),designQ3_est_covariance.var001(:,:,iter,num-1),velocity_var001,Q.var001,Rmean.var001(:,:,1,num),Z.var001(:,1,iter,num),meanSysnoise.var001);
                [designQ3_est_state_var01, designQ3_est_covariance_var01, kalmanGain_01, innov_01] =...
                    kalmanFilter2(designQ3_est_state.var01(:,iter,num-1),designQ3_est_covariance.var01(:,:,iter,num-1),velocity_var01,Q.var01,Rmean.var01(:,:,1,num),Z.var01(:,1,iter,num),meanSysnoise.var01);
                [designQ3_est_state_var1, designQ3_est_covariance_var1, kalmanGain_1, innov_1] =...
                    kalmanFilter2(designQ3_est_state.var1(:,iter,num-1),designQ3_est_covariance.var1(:,:,iter,num-1),velocity_var1,Q.var1,Rmean.var1(:,:,1,num),Z.var1(:,1,iter,num),meanSysnoise.var1);
                [designQ3_est_state_var10, designQ3_est_covariance_var10, kalmanGain_10, innov_10] =...
                    kalmanFilter2(designQ3_est_state.var10(:,iter,num-1),designQ3_est_covariance.var10(:,:,iter,num-1),velocity_var10,Q.var10,Rmean.var10(:,:,1,num),Z.var10(:,1,iter,num),meanSysnoise.var10);
                [designQ3_est_state_var100, designQ3_est_covariance_var100, kalmanGain_100,innov_100] =...
                    kalmanFilter2(designQ3_est_state.var100(:,iter,num-1),designQ3_est_covariance.var100(:,:,iter,num-1),velocity_var100,Q.var100,Rmean.var100(:,:,1,num),Z.var100(:,1,iter,num),meanSysnoise.var100);

                designQ3_est_state.var001(:,iter,num) = designQ3_est_state_var001;
                designQ3_est_state.var01(:,iter,num) = designQ3_est_state_var01;
                designQ3_est_state.var1(:,iter,num) = designQ3_est_state_var1;
                designQ3_est_state.var10(:,iter,num) = designQ3_est_state_var10;
                designQ3_est_state.var100(:,iter,num) = designQ3_est_state_var100;

                designQ3_est_covariance.var001(:,:,iter,num) = designQ3_est_covariance_var001;
                designQ3_est_covariance.var01(:,:,iter,num) = designQ3_est_covariance_var01;
                designQ3_est_covariance.var1(:,:,iter,num) = designQ3_est_covariance_var1;
                designQ3_est_covariance.var10(:,:,iter,num) = designQ3_est_covariance_var10;
                designQ3_est_covariance.var100(:,:,iter,num) = designQ3_est_covariance_var100;

                designQ3_est_KalmanGain.var001(:,:,iter,num) = kalmanGain_001;
                designQ3_est_KalmanGain.var01(:,:,iter,num) = kalmanGain_01;
                designQ3_est_KalmanGain.var1(:,:,iter,num) = kalmanGain_1;
                designQ3_est_KalmanGain.var10(:,:,iter,num) = kalmanGain_10;
                designQ3_est_KalmanGain.var100(:,:,iter,num) = kalmanGain_100;

                velocity_var001 = (designQ3_est_state.var001(:,iter,num) - designQ3_est_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (designQ3_est_state.var01(:,iter,num) - designQ3_est_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (designQ3_est_state.var1(:,iter,num) - designQ3_est_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (designQ3_est_state.var10(:,iter,num) - designQ3_est_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (designQ3_est_state.var100(:,iter,num) - designQ3_est_state.var100(:,iter,num-1))./dt;

                Q.var001 = (1-alpha)*Q.var001 + alpha*(kalmanGain_001*(innov_001*innov_001')*kalmanGain_001');
                Q.var01 = (1-alpha)*Q.var01 + alpha*(kalmanGain_01*(innov_01*innov_01')*kalmanGain_01');
                Q.var1 = (1-alpha)*Q.var1 + alpha*(kalmanGain_1*(innov_1*innov_1')*kalmanGain_1');
                Q.var10 = (1-alpha)*Q.var10 + alpha*(kalmanGain_10*(innov_10*innov_10')*kalmanGain_10');
                Q.var100 = (1-alpha)*Q.var100 + alpha*(kalmanGain_100*(innov_100*innov_100')*kalmanGain_100');
        end
    end
end



save('designQ3_est_state.mat','designQ3_est_state');
% save('designQ3_est_covariance.mat','designQ3_est_covariance');
% save('designQ3_est_KalmanGain.mat','designQ3_est_KalmanGain');
% n_variance = [0.01; 0.1; 1; 10; 100];
a = mean(designQ3_est_covariance.var1,3);
for i = 1:num_sample
    buf(i,1) = trace(a(:,:,1,i));
end
figure;
stem((1:num_sample),buf);
title('trace(P) KFQ3')
xlabel('step')
ylabel('trace(P_k)')


end