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
vars = [state U Trq];

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
numGenerations = 25;             % 代数

crossoverRate = 0.7;             % 交叉概率
thresholdx = 0.7;                % 最大相关性（与原库的相似性） 
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

%% 进化
memory_population_string = cell(numGenerations, 1);
memory_mi_sum = zeros(numGenerations, 1);
memory_population_complexity = zeros(numGenerations, 1);
memory_population_all_fit = zeros(numGenerations, 1);
memory_population_meancorr = zeros(numGenerations, 1);
memory_population_stdcorr = zeros(numGenerations, 1);
memory_population_trainError = zeros(numGenerations, 1);

for gen = 1:numGenerations

    fprintf('第 %d 轮开始进化\n', gen);
    memory_population_string{gen} = population_string;
    
    % 将函数库转换成数据
    population_data = generate_theta(vars_train_data, population_string, str2funcHead);
    Theta_iter = population_data;

    lambda = 3;     % 正则化参数
    tau = 0.001;    % 稀疏化阈值
    max_iter = 10;  % 最大迭代次数
    Xi_iter = regression_stridge(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);

    % 互信息
    population_mi = calculate_mi(population_data, Ef);
    % 复杂性
    population_complex = calculate_complex(population);
    % 稀疏系数
    population_sparse = calculate_sparse(Xi_iter);
    
    % 种群个体适应度定义
    % 相关性 + 复杂性 + 稀疏性 max(population_mi, [], 2) 
    population_fitness = sum(abs(population_mi), 2) - 0.0001 * population_complex + 0.4 * population_sparse;

    % 种群整体适应度定义
    % 训练集误差
    XM_iter = Maaav_improved_ESINDy_discrep(Y, U, T, Trq, Xi_iter, avp, sep, ...
              vars_mean, vars_std, population_string, str2funcHead);

    RMSE_SM_iter = sqrt(mean((XM_iter - Y).^2));

    Error_SM_iter = XM_iter - Y;
    Error_SM_iter(:, 3) = (mod(Error_SM_iter(:, 3) + pi, 2*pi) - pi) * deg_trans;
    Error_SM_iter(:, 4) = (mod(Error_SM_iter(:, 4) + pi, 2*pi) - pi) * deg_trans;
    Error_SM_iter(:, 5) = (mod(Error_SM_iter(:, 5) + pi, 2*pi) - pi) * deg_trans;
    RMSE_SM_iter_XY = sqrt(mean(hypot(Error_SM_iter(:, 1), Error_SM_iter(:, 2)).^2));
    RMSE_SM_iter_YAW1 = sqrt(mean((Error_SM_iter(:, 3)).^2));
    RMSE_SM_iter_YAW2 = sqrt(mean((Error_SM_iter(:, 4)).^2));
    RMSE_SM_iter_YAW3 = sqrt(mean((Error_SM_iter(:, 5)).^2));

    error = RMSE_SM_iter_XY + RMSE_SM_iter_YAW1 + RMSE_SM_iter_YAW2 + RMSE_SM_iter_YAW3;
    fprintf('第 %d 轮进化, 训练集误差 %f \n', gen, error);
    fprintf('位置误差：%f, 偏航角1误差：%f, 偏航角2误差：%f, 偏航角3误差：%f\n' ...
            , RMSE_SM_iter_XY, RMSE_SM_iter_YAW1, RMSE_SM_iter_YAW2, RMSE_SM_iter_YAW3);
    memory_population_trainError(gen) = error;

    % 复杂性
    complexity = nnz(Xi_iter);  
    memory_population_complexity(gen) = complexity;
    fprintf('非零系数个数: %d\n', complexity);

    % 相关性总和
    mi_total_sum = sum(abs(population_mi(:)));
    memory_mi_sum(gen) = mi_total_sum;

    % 相关性 - 复杂性
    population_all_fit = mi_total_sum - 0.01 * complexity;
    memory_population_all_fit(gen) = population_all_fit;
    fprintf('第 %d 轮进化, 整体种群相关性 %f \n', gen, population_all_fit);

    % 多样性和冗余性
    % 计算候选函数组的相关性矩阵
    correlationMatrix = corr(population_data);  % X 是候选函数矩阵，每一列是一个候选函数
    % 计算相关性矩阵的均值和标准差
    meanCorrelation = mean(abs(correlationMatrix(:)));  % 所有候选函数对之间的平均绝对相关性
    stdCorrelation = std(abs(correlationMatrix(:)));    % 相关性标准差
    memory_population_meancorr(gen) = meanCorrelation;
    memory_population_stdcorr(gen) = stdCorrelation;

    % if fitness_iter <= thresholdfit
    %     fprintf('种群达到进化要求，退出迭代进化\n');
    % end
    
    % 进化流程
    
    fprintf('第 %d 轮交叉变异开始\n', gen);

    % SGA-PDE
    % 降序排序
    [~, sortedIdx] = sort(population_fitness, 'descend');

    new_population = population(sortedIdx);
    new_population = new_population(1:population_size*crossoverRate);

    % 使用 randperm 打乱顺序
    shuffledIndices = randperm(numel(new_population));   
    shuffled_population1 = population(shuffledIndices); 
    shuffledIndices = randperm(numel(new_population));   
    shuffled_population2 = population(shuffledIndices); 

    tentative_population = cell(population_size, 1);
    for i = 1:population_size * crossoverRate
        parent1 = shuffled_population1{i};
        parent2 = shuffled_population2{i};
        [child1, child2] = crossover(parent1, parent2);
        tentative_population{2*i-1} = child1;
        tentative_population{2*i} = child2;
    end

    % 从交叉结果中筛选出能用的
    tic;
    [add_population, add_string, theta_all] = select_population(tentative_population, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead);
    toc;

    add_population_size = size(add_population, 1);
    multi_population = cell(population_size + add_population_size, 1);
    multi_population(1:population_size) = population;
    multi_population(population_size+1:population_size+add_population_size) = add_population;
    multi_string = cell(population_size + add_population_size, 1);
    multi_string(1:population_size) = population_string;
    multi_string(population_size+1:population_size+add_population_size) = add_string;

    % 将函数库转换成数据
    multi_population_data = generate_theta(vars_train_data, multi_string, str2funcHead);

    % 对函数库计算适应度
    Xi_iter_new = regression_stridge(multi_population_data, Ef(:, 6:10), lambda, tau, max_iter);
%     Xi_iter_new = regression_lasso_new(multi_population_data, Ef);

    multi_population_mi = calculate_mi(multi_population_data, Ef);
    multi_population_complex = calculate_complex(multi_population);
    multi_population_sparse = calculate_sparse(Xi_iter_new);

    multi_population_fitness = max(multi_population_mi, [], 2) - 0.0001 * multi_population_complex + 0.4 * multi_population_sparse;

    [~, sortedIdx] = sort(multi_population_fitness, 'descend');
    population = multi_population(sortedIdx);
    population_string = multi_string(sortedIdx);
    population_fitness = multi_population_fitness(sortedIdx);

    population = population(1:population_size);
    population_string = population_string(1:population_size);
    population_fitness = population_fitness(1:population_size);

    tic;
    for j = 1:population_size

        expr = population{j};
        weights = [0.04; 0.04; 0.04; 0.08; 0.8];
        
        % according to type choose different opreation
        while true
            % choose type i
            % 创建累积概率分布
            cumulativeWeights = cumsum(weights);
            % 生成随机数
            randomValue = rand();
            % 找到随机数对应的索引
            selectedIndex = find(randomValue <= cumulativeWeights, 1);
    
            newExpr = expr;
            switch selectedIndex
                case 1          % mutate opreator
                    newExpr = modifyRandomOperator(expr, operators);
                case 2          % add node
                    newExpr = insertRandomNode(expr, operators, variables);
                case 3          % delete subtree
                    newExpr = replaceSubtreeWithConstantOrVariable(expr, variables);
                case 4          % new tree entirely
                    newExpr = generateRandomTree(variables, randi([2, maxDepth]), binaryOps, unaryOps);
                case 5          % no mutation
                    newExpr = expr;
            end
            if selectedIndex == 5
                break;
            end
            if satisfies_constraints(newExpr, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead)
                break;
            end
        end

        population{j} = newExpr;
        population_string{j} = toString(newExpr);
        expr_data = generate_theta_single(vars_train_data, toString(newExpr), str2funcHead);
        theta_all = [theta_all expr_data];
    end
    toc;

    % 更新种群
    fprintf('第 %d 轮进化结束\n', gen);
end

%% test

% ESINDy
population_data = generate_theta(vars_train_data, population_string, str2funcHead);
Theta_iter = population_data;

% Xi_iter = regression_lasso(Theta_iter, Ef);
Xi_iter = regression_stridge(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);
% % STRidge 参数
% lambda = 3;  % 正则化参数
% tau = 0.001;    % 稀疏化阈值
% max_iter = 10; % 最大迭代次数
% Xi_iter = regression_stridge_new(Theta_iter, Ef(:, 6:10), lambda, tau, max_iter);

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

processTest;
