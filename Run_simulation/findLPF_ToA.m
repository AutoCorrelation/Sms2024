function findLPF_ToA(iteration)
load('LSE.mat');
% iteration = 1e5;
num_sample = 11;
num_alpha = 9;
lpf_state = struct('var001', zeros(2,iteration,num_sample,num_alpha),...
    'var01', zeros(2,iteration,num_sample,num_alpha),...
    'var1', zeros(2,iteration,num_sample,num_alpha),...
    'var10', zeros(2,iteration,num_sample,num_alpha),...
    'var100', zeros(2,iteration,num_sample,num_alpha)...
    );

for a = 1:9
    alpha = a/10;
    for iter = 1:iteration
        for num = 1:num_sample
            switch num
                case 1
                    lpf_state.var001(:,iter,num,a) = LSE.var001(:,iter,num);
                    lpf_state.var01(:,iter,num,a) = LSE.var01(:,iter,num);
                    lpf_state.var1(:,iter,num,a)  = LSE.var1(:,iter,num);
                    lpf_state.var10(:,iter,num,a) = LSE.var10(:,iter,num);
                    lpf_state.var100(:,iter,num,a) =  LSE.var100(:,iter,num);

                case 2
                    lpf_state.var001(:,iter,num,a) = LSE.var001(:,iter,num);
                    lpf_state.var01(:,iter,num,a) = LSE.var01(:,iter,num);
                    lpf_state.var1(:,iter,num,a)  = LSE.var1(:,iter,num);
                    lpf_state.var10(:,iter,num,a) = LSE.var10(:,iter,num);
                    lpf_state.var100(:,iter,num,a) =  LSE.var100(:,iter,num);

                otherwise
                    lpf_state.var001(:,iter,num,a) = lowpassFilter(lpf_state.var001(:,iter,num-1,a),LSE.var001(:,iter,num),alpha);
                    lpf_state.var01(:,iter,num,a) = lowpassFilter(lpf_state.var01(:,iter,num-1,a),LSE.var01(:,iter,num),alpha);
                    lpf_state.var1(:,iter,num,a) = lowpassFilter(lpf_state.var1(:,iter,num-1,a),LSE.var1(:,iter,num),alpha);
                    lpf_state.var10(:,iter,num,a) = lowpassFilter(lpf_state.var10(:,iter,num-1,a),LSE.var10(:,iter,num),alpha);
                    lpf_state.var100(:,iter,num,a) = lowpassFilter(lpf_state.var100(:,iter,num-1,a),LSE.var100(:,iter,num),alpha);
            end
        end
    end
end
save('lpf_state.mat','lpf_state');
end