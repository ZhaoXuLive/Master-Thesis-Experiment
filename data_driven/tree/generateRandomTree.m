function root = generateRandomTree(variables, maxDepth, operators, unaryOps)
    % 随机生成表达式树，确保至少包含一个变量
    % variables: 可用变量列表 (e.g., {'x', 'y', 'z'})
    % maxDepth: 树的最大深度
    
%     if nargin < 2
%         maxDepth = 3; % 默认最大深度
%     end
    
    while true
        % 生成随机表达式树
        root = generateSubTree(variables, maxDepth, operators, unaryOps);
        
        % 确保树中至少包含一个变量节点
        if ~root.isConstant()
            break;
        end

        if strcmp(root.Operator, '+') || strcmp(root.Operator, '-')
            break;
        end

    end
end

function node = generateSubTree(variables, depth, operators, unaryOps)
    % 递归生成子树
    if depth == 0 || rand() < 0.3 % 终止条件：达到最大深度或随机停止
        % 随机生成叶子节点
        if rand() < 0.5
            % 随机数值
            node = ExprNode('', {}, randi(10)); % 随机整数 [1, 10]
        else
            % 随机变量
            var = variables{randi(numel(variables))};
            node = ExprNode('', {}, var);
        end
    else
        if rand() < 0.3
            % 一元操作符节点
            op = unaryOps{randi(numel(unaryOps))};
            child = generateSubTree(variables, depth - 1, operators, unaryOps);
            node = ExprNode(op, {child}, []);
        else
            % 二元操作符节点
            op = operators{randi(numel(operators))};
            leftChild = generateSubTree(variables, depth - 1, operators, unaryOps);
            rightChild = generateSubTree(variables, depth - 1, operators, unaryOps);
            node = ExprNode(op, {leftChild, rightChild}, []);
        end
    end
end