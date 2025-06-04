close all

addpath('./function'); 
addpath('./regression/'); 

deg_trans = 180 / pi;
polyorder = 3;
functionMode = 4;
regressionMode = 5;

%% train 

% organize and standardize
state = Y(:, 3:10);
vars = [state U];

% 数据归一化

% 归一化实际数据
% vars_mean = mean(vars);
% vars_std = std(vars);
% vars_normalized = (vars - vars_mean) ./ vars_std; 

vars_normalized = vars;
vars_mean = 0;
vars_std = 1;

% 候选函数选择
if functionMode == 1
    % basic 基础形式：全随机 变量纯线性组合
    Theta = basic_function(Y, U, polyorder);
end
if functionMode == 2
    % sine 基础改进形式：引入sin，但无脑
    Theta = sine_function(Y, U, polyorder);
end
if functionMode == 3
    % mechanic 机理引导的选择
    Theta = selected_function(state, U, polyorder);
end
if functionMode == 4
    % standard
    Theta = standard_function(vars_normalized, polyorder);
end

Xintercept = zeros(5, 1);
if regressionMode == 1
    lambda = 0.001;
    Xi = regression_sls(Theta, Ef, lambda);
end
if regressionMode == 2
    Xi = regression_lasso(Theta, Ef);
end
if regressionMode == 3
    [Xi, Xintercept] = regression_ridge(Theta, Ef);
end
if regressionMode == 4
    [Xi, Xintercept] = regression_elastic(Theta, Ef);
end
if regressionMode == 5
    % STRidge 参数
    lambda = 1;     % 正则化参数
    tau = 0.001;    % 稀疏化阈值
    max_iter = 10;  % 最大迭代次数
    Xi = regression_stridge(Theta, Ef(:, 6:10), lambda, tau, max_iter);
end

if functionMode == 1
    XM = Maaav_SINDy_discrep_basic(Y, U, T, Xi, Xintercept, avp, polyorder, sep);
end
if functionMode == 2
    XM = Maaav_SINDy_discrep_sine(Y, U, T, Xi, Xintercept, avp, polyorder, sep);
end
if functionMode == 3
    XM = Maaav_SINDy_discrep_selected(Y, U, T, Xi, Xintercept, avp, polyorder, sep);
end
if functionMode == 4
    XM = Maaav_SINDy_discrep_standard(Y, U, T, Xi, Xintercept, avp, polyorder, sep, vars_mean, vars_std);
end

figure,
for i = 1:length(sep(:, 1))
    subplot(2,4,i);
    plot(Y(sep(i,1):sep(i,2),1), Y(sep(i,1):sep(i,2),2), 'k','LineWidth',1), hold on
    plot(XM(sep(i,1):sep(i,2),1), XM(sep(i,1):sep(i,2),2), 'b','LineWidth',1), hold on
    legend('True','SINDy');
end

Error_SM = XM - Y;
Error_SM(:, 3) = (mod(Error_SM(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_SM(:, 4) = (mod(Error_SM(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_SM(:, 5) = (mod(Error_SM(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_SM_XY = sqrt(mean(hypot(Error_SM(:, 1), Error_SM(:, 2)).^2));
RMSE_SM_X = sqrt(mean((Error_SM(:, 1)).^2));
RMSE_SM_Y = sqrt(mean((Error_SM(:, 2)).^2));
RMSE_SM_YAW1 = sqrt(mean((Error_SM(:, 3)).^2));
RMSE_SM_YAW2 = sqrt(mean((Error_SM(:, 4)).^2));
RMSE_SM_YAW3 = sqrt(mean((Error_SM(:, 5)).^2));
RMSE_SM = [RMSE_SM_XY RMSE_SM_X RMSE_SM_Y RMSE_SM_YAW1 RMSE_SM_YAW2 RMSE_SM_YAW3]

RMSE_SM_VXY = sqrt(mean(hypot(Error_SM(:, 6), Error_SM(:, 7)).^2));
RMSE_SM_VX = sqrt(mean((Error_SM(:, 6)).^2));
RMSE_SM_VY = sqrt(mean((Error_SM(:, 7)).^2));
RMSE_SM_YAWRATE1 = sqrt(mean((Error_SM(:, 8)).^2));
RMSE_SM_YAWRATE2 = sqrt(mean((Error_SM(:, 9)).^2));
RMSE_SM_YAWRATE3 = sqrt(mean((Error_SM(:, 10)).^2));
RMSE_SM_V = [RMSE_SM_VXY RMSE_SM_VX RMSE_SM_VY RMSE_SM_YAWRATE1 RMSE_SM_YAWRATE2 RMSE_SM_YAWRATE3]

if functionMode == 1
    XM_t = Maaav_SINDy_discrep_basic(TestY, TestU, TestT, Xi, Xintercept, avp, polyorder, sepT);
end
if functionMode == 2
    XM_t = Maaav_SINDy_discrep_sine(TestY, TestU, TestT, Xi, Xintercept, avp, polyorder, sepT);
end
if functionMode == 3
    XM_t = Maaav_SINDy_discrep_selected(TestY, TestU, TestT, Xi, Xintercept, avp, polyorder, sepT);
end
if functionMode == 4
    XM_t = Maaav_SINDy_discrep_standard(TestY, TestU, TestT, Xi, Xintercept, avp, polyorder, sepT, vars_mean, vars_std);
end

Error_SM_T = XM_t - TestY;
Error_SM_T(:, 3) = (mod(Error_SM_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_SM_T(:, 4) = (mod(Error_SM_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_SM_T(:, 5) = (mod(Error_SM_T(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_SM_XY_T = sqrt(mean(hypot(Error_SM_T(:, 1), Error_SM_T(:, 2)).^2));
RMSE_SM_X_T = sqrt(mean((Error_SM_T(:, 1)).^2));
RMSE_SM_Y_T = sqrt(mean((Error_SM_T(:, 2)).^2));
RMSE_SM_YAW1_T = sqrt(mean((Error_SM_T(:, 3)).^2));
RMSE_SM_YAW2_T = sqrt(mean((Error_SM_T(:, 4)).^2));
RMSE_SM_YAW3_T = sqrt(mean((Error_SM_T(:, 5)).^2));
RMSE_SM_T = [RMSE_SM_XY_T RMSE_SM_X_T RMSE_SM_Y_T RMSE_SM_YAW1_T RMSE_SM_YAW2_T RMSE_SM_YAW3_T]

RMSE_SM_VXY_T = sqrt(mean(hypot(Error_SM_T(:, 6), Error_SM_T(:, 7)).^2));
RMSE_SM_VX_T = sqrt(mean((Error_SM_T(:, 6)).^2));
RMSE_SM_VY_T = sqrt(mean((Error_SM_T(:, 7)).^2));
RMSE_SM_YAWRATE1_T = sqrt(mean((Error_SM_T(:, 8)).^2));
RMSE_SM_YAWRATE2_T = sqrt(mean((Error_SM_T(:, 9)).^2));
RMSE_SM_YAWRATE3_T = sqrt(mean((Error_SM_T(:, 10)).^2));
RMSE_SM_V_T = [RMSE_SM_VXY_T RMSE_SM_VX_T RMSE_SM_VY_T RMSE_SM_YAWRATE1_T RMSE_SM_YAWRATE2_T RMSE_SM_YAWRATE3_T]

figure,
for i = 1:length(sepT(:, 1))
    subplot(2,2,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XM_t(sepT(i,1):sepT(i,2),1), XM_t(sepT(i,1):sepT(i,2),2), 'b','LineWidth',1), hold on
    legend('True','SINDy');
end
