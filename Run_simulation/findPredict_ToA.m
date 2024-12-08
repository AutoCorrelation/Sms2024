function findPredict_ToA(iteration)
load('LSE.mat');
load('meanSysnoise.mat');
load('Q.mat');
dt = 0.1;
num_sample = 11;
num_alpha = 9;
KFpredict_state = struct('var001', zeros(2,iteration,num_sample,num_alpha),...
    'var01', zeros(2,iteration,num_sample,num_alpha),...
    'var1', zeros(2,iteration,num_sample,num_alpha),...
    'var10', zeros(2,iteration,num_sample,num_alpha),...
    'var100', zeros(2,iteration,num_sample,num_alpha)...
    );

% OPTIMAL ALPHA VALUE IS [0.36 0.36 0.36 0.35 0.31]
for a = 1:9
    alpha = 0.29 + 0.01*(a-1);
    % alpha = 1/a;
    for iter = 1:iteration
        for num = 1:num_sample
            switch num
                case 1
                    KFpredict_state.var001(:,iter,num,a) = LSE.var001(:,iter,num);
                    KFpredict_state.var01(:,iter,num,a) = LSE.var01(:,iter,num);
                    KFpredict_state.var1(:,iter,num,a)  = LSE.var1(:,iter,num);
                    KFpredict_state.var10(:,iter,num,a) = LSE.var10(:,iter,num);
                    KFpredict_state.var100(:,iter,num,a) = LSE.var100(:,iter,num);
                case 2
                    KFpredict_state.var001(:,iter,num,a) =  LSE.var001(:,iter,num);
                    KFpredict_state.var01(:,iter,num,a) = LSE.var01(:,iter,num);
                    KFpredict_state.var1(:,iter,num,a)  = LSE.var1(:,iter,num);
                    KFpredict_state.var10(:,iter,num,a) = LSE.var10(:,iter,num);
                    KFpredict_state.var100(:,iter,num,a) =  LSE.var100(:,iter,num);
                case 3
                    KFpredict_state.var001(:,iter,num,a) =  LSE.var001(:,iter,num);
                    KFpredict_state.var01(:,iter,num,a) = LSE.var01(:,iter,num);
                    KFpredict_state.var1(:,iter,num,a)  = LSE.var1(:,iter,num);
                    KFpredict_state.var10(:,iter,num,a) = LSE.var10(:,iter,num);
                    KFpredict_state.var100(:,iter,num,a) =  LSE.var100(:,iter,num);
                    velocity_var001 = (KFpredict_state.var001(:,iter,num,a) - KFpredict_state.var001(:,iter,num-1,a))./dt;
                    velocity_var01 = (KFpredict_state.var01(:,iter,num,a) - KFpredict_state.var01(:,iter,num-1,a))./dt;
                    velocity_var1 = (KFpredict_state.var1(:,iter,num,a) - KFpredict_state.var1(:,iter,num-1,a))./dt;
                    velocity_var10 = (KFpredict_state.var10(:,iter,num,a) - KFpredict_state.var10(:,iter,num-1,a))./dt;
                    velocity_var100 = (KFpredict_state.var100(:,iter,num,a) - KFpredict_state.var100(:,iter,num-1,a))./dt;
                otherwise
                    prev_est_state_var001 = kalmanFilter_prediction(KFpredict_state.var001(:,iter,num-1,a),velocity_var001,meanSysnoise.var001);
                    prev_est_state_var01 = kalmanFilter_prediction(KFpredict_state.var01(:,iter,num-1,a),velocity_var01,meanSysnoise.var01);
                    prev_est_state_var1 = kalmanFilter_prediction(KFpredict_state.var1(:,iter,num-1,a),velocity_var1,meanSysnoise.var1);
                    prev_est_state_var10 = kalmanFilter_prediction(KFpredict_state.var10(:,iter,num-1,a),velocity_var10,meanSysnoise.var10);
                    prev_est_state_var100 = kalmanFilter_prediction(KFpredict_state.var100(:,iter,num-1,a),velocity_var100,meanSysnoise.var100);

                    KFpredict_state.var001(:,iter,num,a) = lowpassFilter(prev_est_state_var001,LSE.var001(:,iter,num),alpha);
                    KFpredict_state.var01(:,iter,num,a) = lowpassFilter(prev_est_state_var01,LSE.var01(:,iter,num),alpha);
                    KFpredict_state.var1(:,iter,num,a) = lowpassFilter(prev_est_state_var1,LSE.var1(:,iter,num),alpha);
                    KFpredict_state.var10(:,iter,num,a) = lowpassFilter(prev_est_state_var10,LSE.var10(:,iter,num),alpha);
                    KFpredict_state.var100(:,iter,num,a) = lowpassFilter(prev_est_state_var100,LSE.var100(:,iter,num),alpha);

                    velocity_var001 = (KFpredict_state.var001(:,iter,num,a) - KFpredict_state.var001(:,iter,num-1,a))./dt;
                    velocity_var01 = (KFpredict_state.var01(:,iter,num,a) - KFpredict_state.var01(:,iter,num-1,a))./dt;
                    velocity_var1 = (KFpredict_state.var1(:,iter,num,a) - KFpredict_state.var1(:,iter,num-1,a))./dt;
                    velocity_var10 = (KFpredict_state.var10(:,iter,num,a) - KFpredict_state.var10(:,iter,num-1,a))./dt;
                    velocity_var100 = (KFpredict_state.var100(:,iter,num,a) - KFpredict_state.var100(:,iter,num-1,a))./dt;
            end
        end
    end
end
save('KFpredict_state.mat','KFpredict_state');

n_variance = [0.01; 0.1; 1; 10; 100];
num_sample = 11;
KFpredict_buf = zeros(length(n_variance), 9);

for iter = 1:iteration
    for num = 1:num_sample
        exactPos = [num-1; num-1];
        for a = 1:9
            KFpredict_buf(:, a) = KFpredict_buf(:, a) + [
                norm(KFpredict_state.var001(:, iter, num, a) - exactPos);
                norm(KFpredict_state.var01(:, iter, num, a) - exactPos);
                norm(KFpredict_state.var1(:, iter, num, a) - exactPos);
                norm(KFpredict_state.var10(:, iter, num, a) - exactPos);
                norm(KFpredict_state.var100(:, iter, num, a) - exactPos)
                ];
        end
    end
end

[KFpredict_min, optimal_alpha_predict] = min(KFpredict_buf, [], 2);
KFpredict_mse = KFpredict_min ./ (iteration * num_sample);

disp('optimal alpha_predict: ');
disp(optimal_alpha_predict);
end