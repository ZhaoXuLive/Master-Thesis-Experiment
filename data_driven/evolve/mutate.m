function newExpr = mutate(expr, temperature, operators, variables, binaryOps, unaryOps, maxDepth, theta_all, vars_train_data, str2funcHead, Ef, thresholdx, thresholdy)

    % parameters
%     f = 1;                  % constant perturbation scale
%     epison = 0.1;           % minimum perturbation
    alpha = 0.1;            % temperature scale
    % the probability weight of mutation type i
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
                newExpr = generateRandomTree(variables, randi([3, maxDepth]), binaryOps, unaryOps);
            case 5          % no mutation
                newExpr = expr;
        end
        if satisfies_constraints(newExpr, theta_all, vars_train_data, Ef, thresholdx, thresholdy, str2funcHead)
            break;
        end
%         fprintf('test\n');
    end

    % calculate fitness 
    fitness = calculate_fit(expr, vars_train_data, str2funcHead, Ef);
    newfitness = calculate_fit(newExpr, vars_train_data, str2funcHead, Ef);

    if newfitness < fitness
        p = exp((newfitness - fitness) ./ (alpha * temperature));
        if rand > p
            newExpr = expr;
        end
    end
    
end