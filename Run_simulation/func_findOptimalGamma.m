function optimal_gamma = func_findOptimalGamma(iteration)
    load('AdaptiveQ_est_state.mat');

    n_variance = [0.01; 0.1; 1; 10; 100];
    gammamax = size(AdaptiveQ_est_state.var001, 4);
    num_sample = 11;
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

    disp('optimal gamma: ');
    disp(optimal_gamma);
end