function optimal_Lpf_ToA(iteration)
load('LSE.mat');

num_sample = 11;
optimal_lpf_state = struct('var001', zeros(2,iteration,num_sample),...
    'var01', zeros(2,iteration,num_sample),...
    'var1', zeros(2,iteration,num_sample),...
    'var10', zeros(2,iteration,num_sample),...
    'var100', zeros(2,iteration,num_sample)...
    );

optimal_alpha = [0.1; 0.1; 0.2; 0.5; 0.5];
for iter = 1:iteration
    for num = 1:num_sample
        switch num
            case 1
                optimal_lpf_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                optimal_lpf_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                optimal_lpf_state.var1(:,iter,num)  = LSE.var1(:,iter,num);
                optimal_lpf_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                optimal_lpf_state.var100(:,iter,num) =  LSE.var100(:,iter,num);

            case 2
                optimal_lpf_state.var001(:,iter,num) = LSE.var001(:,iter,num);
                optimal_lpf_state.var01(:,iter,num) = LSE.var01(:,iter,num);
                optimal_lpf_state.var1(:,iter,num)  = LSE.var1(:,iter,num);
                optimal_lpf_state.var10(:,iter,num) = LSE.var10(:,iter,num);
                optimal_lpf_state.var100(:,iter,num) =  LSE.var100(:,iter,num);

            otherwise
                optimal_lpf_state.var001(:,iter,num) = lowpassFilter(optimal_lpf_state.var001(:,iter,num-1),LSE.var001(:,iter,num),optimal_alpha(1));
                optimal_lpf_state.var01(:,iter,num) = lowpassFilter(optimal_lpf_state.var01(:,iter,num-1),LSE.var01(:,iter,num),optimal_alpha(2));
                optimal_lpf_state.var1(:,iter,num) = lowpassFilter(optimal_lpf_state.var1(:,iter,num-1),LSE.var1(:,iter,num),optimal_alpha(3));
                optimal_lpf_state.var10(:,iter,num) = lowpassFilter(optimal_lpf_state.var10(:,iter,num-1),LSE.var10(:,iter,num),optimal_alpha(4));
                optimal_lpf_state.var100(:,iter,num) = lowpassFilter(optimal_lpf_state.var100(:,iter,num-1),LSE.var100(:,iter,num),optimal_alpha(5));
        end
    end
end
save('optimal_lpf_state.mat','optimal_lpf_state');
end