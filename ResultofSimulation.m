function ResultofSimulation(iteration)
load('LSE.mat');
load('optimal_lpf_state.mat');
load('optimal_predict_state.mat');
load('est_state.mat');

load('DesignQ1_est_state.mat')
load('DesignQ2_est_state.mat')
load('diagR_est_state.mat')
load('designQ3_est_state.mat')

n_variance = [0.01; 0.1; 1; 10; 100];
num_sample = 11;
toa_rmse = zeros(length(n_variance),1);
lpf_rmse = zeros(length(n_variance),1);
pred_rmse= zeros(length(n_variance),1);
kf_rmse  = zeros(length(n_variance),1);
designQ1_kf_rmse = zeros(length(n_variance),1);
designQ2_kf_rmse = zeros(length(n_variance),1);
diagR_kf_rmse = zeros(length(n_variance),1);
designQ3_kf_rmse = zeros(length(n_variance),1);

for iter = 1:iteration
    for num = 1:num_sample
        exactPos = [num-1;num-1];
        % ToA RMSE
        toa_rmse = toa_rmse + [...
            norm(LSE.var001(:,iter,num)-exactPos);...
            norm(LSE.var01(:,iter,num)-exactPos);...
            norm(LSE.var1(:,iter,num)-exactPos);...
            norm(LSE.var10(:,iter,num)-exactPos);...
            norm(LSE.var100(:,iter,num)-exactPos)...
            ];
        % LPF RMSE
        lpf_rmse = lpf_rmse + [...
            norm(optimal_lpf_state.var001(:,iter,num)-exactPos);...
            norm(optimal_lpf_state.var01(:,iter,num)-exactPos);...
            norm(optimal_lpf_state.var1(:,iter,num)-exactPos);...
            norm(optimal_lpf_state.var10(:,iter,num)-exactPos);...
            norm(optimal_lpf_state.var100(:,iter,num)-exactPos)...
            ];

        pred_rmse = pred_rmse + [...
            norm(optimal_predict_state.var001(:,iter,num)-exactPos);...
            norm(optimal_predict_state.var01(:,iter,num)-exactPos);...
            norm(optimal_predict_state.var1(:,iter,num)-exactPos);...
            norm(optimal_predict_state.var10(:,iter,num)-exactPos);...
            norm(optimal_predict_state.var100(:,iter,num)-exactPos)...
            ];

        kf_rmse = kf_rmse + [...
            norm(est_state.var001(:,iter,num)-exactPos);...
            norm(est_state.var01(:,iter,num)-exactPos);...
            norm(est_state.var1(:,iter,num)-exactPos);...
            norm(est_state.var10(:,iter,num)-exactPos);...
            norm(est_state.var100(:,iter,num)-exactPos)...
            ];

        designQ1_kf_rmse = designQ1_kf_rmse + [...
            norm(DesignQ1_est_state.var001(:,iter,num)-exactPos);...
            norm(DesignQ1_est_state.var01(:,iter,num)-exactPos);...
            norm(DesignQ1_est_state.var1(:,iter,num)-exactPos);...
            norm(DesignQ1_est_state.var10(:,iter,num)-exactPos);...
            norm(DesignQ1_est_state.var100(:,iter,num)-exactPos)...
            ];

        designQ2_kf_rmse = designQ2_kf_rmse + [...
            norm(DesignQ2_est_state.var001(:,iter,num)-exactPos);...
            norm(DesignQ2_est_state.var01(:,iter,num)-exactPos);...
            norm(DesignQ2_est_state.var1(:,iter,num)-exactPos);...
            norm(DesignQ2_est_state.var10(:,iter,num)-exactPos);...
            norm(DesignQ2_est_state.var100(:,iter,num)-exactPos)...
            ];

        diagR_kf_rmse = diagR_kf_rmse + [...
            norm(diagR_est_state.var001(:,iter,num)-exactPos);...
            norm(diagR_est_state.var01(:,iter,num)-exactPos);...
            norm(diagR_est_state.var1(:,iter,num)-exactPos);...
            norm(diagR_est_state.var10(:,iter,num)-exactPos);...
            norm(diagR_est_state.var100(:,iter,num)-exactPos)...
            ];

        designQ3_kf_rmse = designQ3_kf_rmse + [...
            norm(designQ3_est_state.var001(:,iter,num)-exactPos);...
            norm(designQ3_est_state.var01(:,iter,num)-exactPos);...
            norm(designQ3_est_state.var1(:,iter,num)-exactPos);...
            norm(designQ3_est_state.var10(:,iter,num)-exactPos);...
            norm(designQ3_est_state.var100(:,iter,num)-exactPos)...
            ];

    end
end
toa_rmse = toa_rmse/(iteration*num_sample);
lpf_rmse = lpf_rmse/(iteration*num_sample);
pred_rmse = pred_rmse/(iteration*num_sample);
kf_rmse = kf_rmse/(iteration*num_sample);
designQ1_kf_rmse = designQ1_kf_rmse/(iteration*num_sample);
designQ2_kf_rmse = designQ2_kf_rmse/(iteration*num_sample);
diagR_kf_rmse = diagR_kf_rmse/(iteration*num_sample);
designQ3_kf_rmse = designQ3_kf_rmse/(iteration*num_sample);

figure;
semilogx(n_variance(:),toa_rmse(:),'-o','DisplayName','ToA');
hold on
semilogx(n_variance(:),lpf_rmse(:),'-o','DisplayName','optimal LPF');
semilogx(n_variance(:),pred_rmse(:),'-o','DisplayName','optimal Predict');
semilogx(n_variance(:),kf_rmse(:),'-o','DisplayName','KF');
semilogx(n_variance(:),designQ1_kf_rmse(:),'-o','DisplayName','KF DesignQ1');
semilogx(n_variance(:),designQ2_kf_rmse(:),'-o','DisplayName','KF DesignQ2');
semilogx(n_variance(:),diagR_kf_rmse(:),'-o','DisplayName','KF DiagR');
semilogx(n_variance(:),designQ3_kf_rmse(:),'-o','DisplayName','KF DesignQ3');
title('RMSE of each method');
xlabel('n variance');
ylabel('RMSE');

hold off
grid on
legend;
end