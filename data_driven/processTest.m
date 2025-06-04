
iter = numGenerations;

Memory_RMSE_ESM_t = zeros(iter, 1);
Memory_RMSE_ESM_V_t = zeros(iter, 1);

for i = 1:iter                                                                                               

    fprintf('第 %d 代种群结果如下: \n', i);

    population_string = memory_population_string{i};

    % ESINDy
    population_data = generate_theta(vars_train_data, population_string, str2funcHead);
    Theta_iter = population_data;
    % Theta_iter = [vars_train_data population_data];
    
%     Xi_iter = regression_lasso_new(Theta_iter, Ef);
%     Xi_iter = regression_sls(Theta_iter, Ef, 0.000001);
    % STRidge 参数
    lambda = 3;  % 正则化参数
    tau = 0.001;    % 稀疏化阈值
    max_iter = 10; % 最大迭代次数
    Xi_iter = regression_stridge(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);
    
    Xi_ESINDy = Xi_iter;
    
    % 多步预测
%     XM_ESINDy = Maaav_improved_ESINDy_discrep(Y, U, T, Trq, Xi_ESINDy, avp, sep, ...
%                   vars_mean, vars_std, population_string, str2funcHead);
    XM_t_ESINDy = Maaav_improved_ESINDy_discrep(TestY, TestU, TestT, TestTrq, Xi_ESINDy, avp, sepT, ...
                  vars_mean, vars_std, population_string, str2funcHead);
    
    %% data
    
    Error_ESM_T = XM_t_ESINDy - TestY;
    Error_ESM_T(:, 3) = (mod(Error_ESM_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
    Error_ESM_T(:, 4) = (mod(Error_ESM_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
    Error_ESM_T(:, 5) = (mod(Error_ESM_T(:, 5) + pi, 2*pi) - pi) * deg_trans;
    Error_ESM_T(:, 8) = (mod(Error_ESM_T(:, 8) + pi, 2*pi) - pi) * deg_trans;
    Error_ESM_T(:, 9) = (mod(Error_ESM_T(:, 9) + pi, 2*pi) - pi) * deg_trans;
    Error_ESM_T(:, 10) = (mod(Error_ESM_T(:, 10) + pi, 2*pi) - pi) * deg_trans;
    
    RMSE_ESM_XY_T = sqrt(mean(hypot(Error_ESM_T(:, 1), Error_ESM_T(:, 2)).^2));
    RMSE_ESM_X_T = sqrt(mean((Error_ESM_T(:, 1)).^2));
    RMSE_ESM_Y_T = sqrt(mean((Error_ESM_T(:, 2)).^2));
    RMSE_ESM_YAW1_T = sqrt(mean((Error_ESM_T(:, 3)).^2));
    RMSE_ESM_YAW2_T = sqrt(mean((Error_ESM_T(:, 4)).^2));
    RMSE_ESM_YAW3_T = sqrt(mean((Error_ESM_T(:, 5)).^2));
    RMSE_ESM_T = [RMSE_ESM_XY_T RMSE_ESM_X_T RMSE_ESM_Y_T RMSE_ESM_YAW1_T RMSE_ESM_YAW2_T RMSE_ESM_YAW3_T]
    
    RMSE_ESM_VXY_T = sqrt(mean(hypot(Error_ESM_T(:, 6), Error_ESM_T(:, 7)).^2));
    RMSE_ESM_VX_T = sqrt(mean((Error_ESM_T(:, 6)).^2));
    RMSE_ESM_VY_T = sqrt(mean((Error_ESM_T(:, 7)).^2));
    RMSE_ESM_YAWRATE1_T = sqrt(mean((Error_ESM_T(:, 8)).^2));
    RMSE_ESM_YAWRATE2_T = sqrt(mean((Error_ESM_T(:, 9)).^2));
    RMSE_ESM_YAWRATE3_T = sqrt(mean((Error_ESM_T(:, 10)).^2));
    RMSE_ESM_V_T = [RMSE_ESM_VXY_T RMSE_ESM_VX_T RMSE_ESM_VY_T RMSE_ESM_YAWRATE1_T RMSE_ESM_YAWRATE2_T RMSE_ESM_YAWRATE3_T]

    Memory_RMSE_ESM_t(i) = mean(RMSE_ESM_T);
    Memory_RMSE_ESM_V_t(i) = mean(RMSE_ESM_V_T);

end

%% plot 

iter = 23;
figure,
plot(1:1:iter, Memory_RMSE_ESM_t(1:iter));
title('Predict RMSE');

figure,
plot(1:1:iter, Memory_RMSE_ESM_V_t(1:iter));
title('Predict velocity RMSE');

figure,
plot(1:1:iter, memory_population_trainError(1:iter,:));
title('Train RMSE');

figure,
% subplot(2,2,1);
plot(1:1:iter, memory_mi_sum(1:iter,:));
title('Relevant sum');

figure,
% subplot(2,2,2);
plot(1:1:iter, memory_population_complexity(1:iter,:));
title('Complexity');

% subplot(2,2,3);
% plot(1:1:iter, memory_population_meancorr(1:iter,:));
% title('mean corr');
% 
% subplot(2,2,4);
% plot(1:1:iter, memory_population_stdcorr(1:iter,:));
% title('std corr');
% 
% figure,
% plot(1:1:iter, -(memory_mi_sum - 0.01 * memory_population_complexity - 10 * memory_population_stdcorr))
% title('memory_mi_sum - complexity - stdcorr');
