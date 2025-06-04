function isValid = satisfies_constraints(exprTree, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead)
    % satisfies_constraints: 判断表达式树是否满足特定约束条件
    % 输入:
    %   exprTree - 表达式树的根节点 (ExprNode)
    % 输出:
    %   isValid - 布尔值，true 表示满足约束，false 表示不满足
    
    % 初始化结果
    isValid = true;
    
    % 约束 1: 限制树的深度
    maxDepth = 5; % 最大允许深度
    [~, ~, depth] = computeComplexity(exprTree);
    if depth > maxDepth
        isValid = false;
        return;
    end
    
    % 约束 2: 限制树的节点数量
    maxNodes = 20; % 最大允许节点数量
    [~, nodeCount, ~] = computeComplexity(exprTree);
    if nodeCount > maxNodes
        isValid = false;
        return;
    end
    
%     % 约束 3: 限制特定操作符的使用
%     % 禁止使用特定操作符，例如 "log" 或 "exp"
%     forbiddenOperators = {'log', 'exp'};
%     [~, operators] = exprTree.collectNodesAndOperators();
%     if any(ismember(forbiddenOperators, operators))
%         isValid = false;
%         return;
%     end
    
%     % 约束 4: 检查是否有数学异常（如除以零）
%     try
%         testValue = 1; % 测试变量值
%         eval(exprTree.toString());
%     catch
%         isValid = false;
%         return;
%     end
    
    % 约束 5: 输出限制（根据具体需求添加）
    % 可基于表达式的实际值进一步约束，例如限制特定范围 [-100, 100]
    % 具体实现略，需结合数值评估表达式。

    if ~enrich(toString(exprTree), theta_all, vars_train_data, thresholdx, str2funcHead)
        isValid = false;
        return;
    end   

%     if ~useful(toString(exprTree), vars_train_data, Ef, thresholdy, str2funcHead)
%         isValid = false;
%         return;
%     end

end
