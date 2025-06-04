
% function [add_population, add_string, theta_all] = select_population(tentative_population, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead)
% 
%     % 初始化并行池（如果尚未开启）
%     if isempty(gcp('nocreate'))
%         parpool; % 启动默认并行池
%     end
% 
%     % 预分配存储空间（cell arrays）
%     test_population = cell(size(tentative_population, 1), 1);
%     test_string = cell(size(tentative_population, 1), 1);
%     updated_theta_all = cell(size(tentative_population, 1), 1); % 存储局部更新的 theta_all
% 
%     % 并行化处理
%     parfor i = 1:size(tentative_population, 1)
%         exprTree = tentative_population{i};
%         expr_string = toString(exprTree);
% 
%         % 约束 1: 限制树的深度
%         maxDepth = 5; % 最大允许深度
%         [~, ~, depth] = computeComplexity(exprTree);
%         if depth > maxDepth
%             continue;
%         end
% 
%         % 约束 2: 限制树的节点数量
%         maxNodes = 20; % 最大允许节点数量
%         [~, nodeCount, ~] = computeComplexity(exprTree);
%         if nodeCount > maxNodes
%             continue;
%         end
% 
%         % 约束 5: 输出限制（根据具体需求添加）
%         % 可基于表达式的实际值进一步约束，例如限制特定范围 [-100, 100]
%         % 具体实现略，需结合数值评估表达式。
% 
%         if ~enrich(expr_string, theta_all, vars_train_data, thresholdx, str2funcHead)
%             continue;
%         end
% 
%         if ~useful(expr_string, vars_train_data, Ef, thresholdy, str2funcHead)
%             continue;
%         end
% 
%         % 将符合条件的表达式保存
%         test_population{i} = exprTree;
%         test_string{i} = expr_string;
% 
%         % 局部更新 theta_all
%         expr_data = generate_theta_single(vars_train_data, expr_string, str2funcHead);
%         updated_theta_all{i} = expr_data;
%     end
% 
%     % 汇总非空结果
%     nonEmptyMask = cellfun(@(x) ~isempty(x), test_population);
%     add_population = test_population(nonEmptyMask);
%     add_string = test_string(nonEmptyMask);
% 
%     % 合并所有局部更新的 theta_all
%     local_theta = [updated_theta_all{~cellfun('isempty', updated_theta_all)}];
%     theta_all = [theta_all, local_theta]; % 如果需要返回更新后的 theta_all，可输出它
% 
% end

function [add_population, add_string, theta_all] = select_population(tentative_population, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead)

    test_population = cell(size(tentative_population, 1), 1);
    test_string = cell(size(tentative_population, 1), 1);

    for i = 1:size(tentative_population, 1)
    
        exprTree = tentative_population{i};
        expr_string = toString(exprTree);

        % 约束 1: 限制树的深度
        maxDepth = 5; % 最大允许深度
        [~, ~, depth] = computeComplexity(exprTree);
        if depth > maxDepth
            continue;
        end
        
        % 约束 2: 限制树的节点数量
        maxNodes = 20; % 最大允许节点数量
        [~, nodeCount, ~] = computeComplexity(exprTree);
        if nodeCount > maxNodes
            continue;
        end 
    
        % 约束 5: 输出限制（根据具体需求添加）
        % 可基于表达式的实际值进一步约束，例如限制特定范围 [-100, 100]
        % 具体实现略，需结合数值评估表达式。
    
        if ~enrich(expr_string, theta_all, vars_train_data, thresholdx, str2funcHead)
            continue;
        end   
    
%         if ~useful(expr_string, vars_train_data, Ef, thresholdy, str2funcHead)
%             continue;
%         end

        test_population{i} = exprTree;
        test_string{i} = toString(exprTree);

        expr_data = generate_theta_single(vars_train_data, expr_string, str2funcHead);
        theta_all = [theta_all expr_data];

    end

    % 自定义判断非空的函数
    nonEmptyMask = cellfun(@(x) ~isempty(x), test_population);

    % 去除空元素
    add_population = test_population(nonEmptyMask);
    add_string = test_string(nonEmptyMask);

end