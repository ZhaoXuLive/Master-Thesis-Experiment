% 进化 SINDy 

close all

%% basic environment

addpath('./minepy\matlab');
addpath('./evolve'); 
addpath('./judge');
addpath('./tree');
addpath('./function'); 
addpath('./regression/'); 

%% organize and standardize

deg_trans = 180 / pi;

state = zeros(size(Y, 1), 11);
state(:, 1) = sin(Y(:, 3));
state(:, 2) = sin(Y(:, 4));
state(:, 3) = sin(Y(:, 5));
state(:, 4) = cos(Y(:, 3));
state(:, 5) = cos(Y(:, 4));
state(:, 6) = cos(Y(:, 5));
state(:, 7:11) = Y(:, 6:10);
vars = [state U Trq(:, 1:2) Trq(:, 7:8)];

% 数据归一化
% 归一化实际数据
vars_mean = mean(vars);
vars_std = std(vars);
vars_normalized = (vars - vars_mean) ./ vars_std; 

vars_normalized = vars;
vars_mean = 0;
vars_std = 1;

%% evolve

% 进化算法参数
population_size = 200;           % 固定种群大小
numGenerations = 15;             % 代数
crossoverRate = 0.7;             % 交叉概率
thresholdx = 0.5;                % 最大相关性（与原库的相似性） 
thresholdy = 0.1;               % 最大相关性（与因变量的相似性） 
thresholdfit = 5;                % 最小适应度（认为迭代可终止）
mutationSize = 10;               % 每次进化所需变异次数

% 初始化种群 注意：最后的括号省略会导致出现函数使用不完整，问题在于strcat的使用
str2funcHead = ['@(psi1, psi2, psi3, vx, vy, w1, w2, w3, ' ...
                's1, s2, s3, s4, s5, s6, s7, s8, ' ...
                't1, t2, t7, t8) '];

% 初始化表达式基础信息
maxDepth = 3;                   % 最大深度
operators = {'+', '-', '.*', './', 'sin', 'cos'};
binaryOps = {'+', '-', '.*', './'};
unaryOps = {'sin', 'cos'};
variables = {'psi1', 'psi2', 'psi3', 'vx', 'vy', 'w1', 'w2', 'w3' ...
             's1', 's2', 's3', 's4', 's5', 's6', 's7', 's8', ...
             't1', 't2', 't7', 't8'};

theta_all = vars_normalized;
vars_train_data = vars_normalized;

fprintf('种群生成开始\n');
tic;
population = cell(population_size, 1);
population_string = cell(population_size, 1);
for i = 1:population_size
    population_iter = generateRandomTree(variables, randi([2, maxDepth]), binaryOps, unaryOps);
    population_string_iter = toString(population_iter);
    while ~enrich(population_string_iter, theta_all, vars_train_data, thresholdx, str2funcHead) ...
            || ~useful(population_string_iter, vars_train_data, Ef, thresholdy, str2funcHead)  
        population_iter = generateRandomTree(variables, randi([2, maxDepth]), binaryOps, unaryOps);
        population_string_iter = toString(population_iter);
    end
    disp(i);
    population{i} = population_iter;
    population_string{i} = population_string_iter;
    population_data = generate_theta_single(vars_train_data, population_string_iter, str2funcHead);
    theta_all = [theta_all population_data];
end
toc;
fprintf('生成种群耗时: %f 秒\n', toc);
fprintf('初始种群生成完毕\n');

%% test

% ESINDy
population_data = generate_theta(vars_train_data, population_string, str2funcHead);
Theta_iter = population_data;

% Xi_iter = regression_lasso(Theta_iter, Ef);
% Xi_iter = regression_lasso_new(Theta_iter, Ef);
% lambda = 1;
% Xi_iter = regression_sls(Theta_iter, Ef, lambda);
% STRidge 参数
lambda = 3;  % 正则化参数
tau = 0.001;    % 稀疏化阈值
max_iter = 10; % 最大迭代次数
% Xi_iter = regression_stridge_new(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);
Xi_iter = regression_stridge(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);

Xi_ESINDy = Xi_iter;

% 多步预测
% XM_ESINDy = Maaav_improved_ESINDy_discrep(Y, U, T, Trq, Xi_ESINDy, avp, sep, ...
%               vars_mean, vars_std, population_string, str2funcHead);
XM_t_ESINDy = Maaav_improved_ESINDy_discrep(TestY, TestU, TestT, TestTrq, Xi_ESINDy, avp, sepT, ...
              vars_mean, vars_std, population_string, str2funcHead);

%% data

% Error_ESM = XM_ESINDy - Y;
% Error_ESM(:, 3) = (mod(Error_ESM(:, 3) + pi, 2*pi) - pi) * deg_trans;
% Error_ESM(:, 4) = (mod(Error_ESM(:, 4) + pi, 2*pi) - pi) * deg_trans;
% Error_ESM(:, 5) = (mod(Error_ESM(:, 5) + pi, 2*pi) - pi) * deg_trans;
% 
% RMSE_ESM_XY = sqrt(mean(hypot(Error_ESM(:, 1), Error_ESM(:, 2)).^2));
% RMSE_ESM_X = sqrt(mean((Error_ESM(:, 1)).^2));
% RMSE_ESM_Y = sqrt(mean((Error_ESM(:, 2)).^2));
% RMSE_ESM_YAW1 = sqrt(mean((Error_ESM(:, 3)).^2));
% RMSE_ESM_YAW2 = sqrt(mean((Error_ESM(:, 4)).^2));
% RMSE_ESM_YAW3 = sqrt(mean((Error_ESM(:, 5)).^2));
% RMSE_ESM = [RMSE_ESM_XY RMSE_ESM_X RMSE_ESM_Y RMSE_ESM_YAW1 RMSE_ESM_YAW2 RMSE_ESM_YAW3]
% 
% RMSE_ESM_VXY = sqrt(mean(hypot(Error_ESM(:, 6), Error_ESM(:, 7)).^2));
% RMSE_ESM_VX = sqrt(mean((Error_ESM(:, 6)).^2));
% RMSE_ESM_VY = sqrt(mean((Error_ESM(:, 7)).^2));
% RMSE_ESM_YAWRATE1 = sqrt(mean((Error_ESM(:, 8)).^2));
% RMSE_ESM_YAWRATE2 = sqrt(mean((Error_ESM(:, 9)).^2));
% RMSE_ESM_YAWRATE3 = sqrt(mean((Error_ESM(:, 10)).^2));
% RMSE_ESM_V = [RMSE_ESM_VXY RMSE_ESM_VX RMSE_ESM_VY RMSE_ESM_YAWRATE1 RMSE_ESM_YAWRATE2 RMSE_ESM_YAWRATE3]

Error_ESM_T = XM_t_ESINDy - TestY;
Error_ESM_T(:, 3) = (mod(Error_ESM_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_ESM_T(:, 4) = (mod(Error_ESM_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_ESM_T(:, 5) = (mod(Error_ESM_T(:, 5) + pi, 2*pi) - pi) * deg_trans;

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

% figure,
% for i = 1:length(sep(:, 1))
%     subplot(3,5,i);
%     plot(Y(sep(i,1):sep(i,2),1), Y(sep(i,1):sep(i,2),2), 'k','LineWidth',1), hold on
%     plot(XM_ESINDy(sep(i,1):sep(i,2),1), XM_ESINDy(sep(i,1):sep(i,2),2), 'g','LineWidth',1), hold on
%     legend('True','ESINDy');
% end

figure,
for i = 1:length(sepT(:, 1))
    subplot(1,3,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XM_t_ESINDy(sepT(i,1):sepT(i,2),1), XM_t_ESINDy(sepT(i,1):sepT(i,2),2), 'g','LineWidth',1), hold on
    legend('True','ESINDy');
end

% processTest;

