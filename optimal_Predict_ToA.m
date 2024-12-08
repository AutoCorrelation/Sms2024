function optimal_Predict_ToA(iteration)
load('LSE.mat');
load('meanSysnoise.mat');

dt = 0.1;
num_sample = 11;
optimal_predict_state = struct('var001', zeros(2,iteration,num_sample),...
    'var01', zeros(2,iteration,num_sample),...
    'var1', zeros(2,iteration,num_sample),...
    'var10', zeros(2,iteration,num_sample),...
    'var100', zeros(2,iteration,num_sample)...
    );

% OPTIMAL ALPHA VALUE IS [0.35 0.35 0.34 0.33 0.29]
alpha = [0.36 0.36 0.36 0.35 0.31];
for iter = 1:iteration
    for num = 1:num_sample
        switch num
            case 1
                optimal_predict_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                optimal_predict_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                optimal_predict_state.var1(:,iter,num)  = LSE.var1(:,iter,num);
                optimal_predict_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                optimal_predict_state.var100(:,iter,num) = LSE.var100(:,iter,num);
            case 2
                optimal_predict_state.var001(:,iter,num) =  LSE.var001(:,iter,num);
                optimal_predict_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                optimal_predict_state.var1(:,iter,num)  = LSE.var1(:,iter,num);
                optimal_predict_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                optimal_predict_state.var100(:,iter,num) =  LSE.var100(:,iter,num);
            case 3
                optimal_predict_state.var001(:,iter,num) =  LSE.var001(:,iter,num);
                optimal_predict_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                optimal_predict_state.var1(:,iter,num)  = LSE.var1(:,iter,num);
                optimal_predict_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                optimal_predict_state.var100(:,iter,num) =  LSE.var100(:,iter,num);
                velocity_var001 = (optimal_predict_state.var001(:,iter,num) - optimal_predict_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (optimal_predict_state.var01(:,iter,num) - optimal_predict_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (optimal_predict_state.var1(:,iter,num) - optimal_predict_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (optimal_predict_state.var10(:,iter,num) - optimal_predict_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (optimal_predict_state.var100(:,iter,num) - optimal_predict_state.var100(:,iter,num-1))./dt;
            otherwise
                prev_est_state_var001 = kalmanFilter_prediction(optimal_predict_state.var001(:,iter,num-1),velocity_var001,meanSysnoise.var001);
                prev_est_state_var01 = kalmanFilter_prediction(optimal_predict_state.var01(:,iter,num-1),velocity_var01,meanSysnoise.var01);
                prev_est_state_var1 = kalmanFilter_prediction(optimal_predict_state.var1(:,iter,num-1),velocity_var1,meanSysnoise.var1);
                prev_est_state_var10 = kalmanFilter_prediction(optimal_predict_state.var10(:,iter,num-1),velocity_var10,meanSysnoise.var10);
                prev_est_state_var100 = kalmanFilter_prediction(optimal_predict_state.var100(:,iter,num-1),velocity_var100,meanSysnoise.var100);

                optimal_predict_state.var001(:,iter,num) = lowpassFilter(prev_est_state_var001,LSE.var001(:,iter,num),alpha(1));
                optimal_predict_state.var01(:,iter,num) = lowpassFilter(prev_est_state_var01,LSE.var01(:,iter,num),alpha(2));
                optimal_predict_state.var1(:,iter,num) = lowpassFilter(prev_est_state_var1,LSE.var1(:,iter,num),alpha(3));
                optimal_predict_state.var10(:,iter,num) = lowpassFilter(prev_est_state_var10,LSE.var10(:,iter,num),alpha(4));
                optimal_predict_state.var100(:,iter,num) = lowpassFilter(prev_est_state_var100,LSE.var100(:,iter,num),alpha(5));

                velocity_var001 = (optimal_predict_state.var001(:,iter,num) - optimal_predict_state.var001(:,iter,num-1))./dt;
                velocity_var01 = (optimal_predict_state.var01(:,iter,num) - optimal_predict_state.var01(:,iter,num-1))./dt;
                velocity_var1 = (optimal_predict_state.var1(:,iter,num) - optimal_predict_state.var1(:,iter,num-1))./dt;
                velocity_var10 = (optimal_predict_state.var10(:,iter,num) - optimal_predict_state.var10(:,iter,num-1))./dt;
                velocity_var100 = (optimal_predict_state.var100(:,iter,num) - optimal_predict_state.var100(:,iter,num-1))./dt;

        end
    end
end
save('optimal_predict_state.mat','optimal_predict_state');
end