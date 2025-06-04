classdef ExprNode
    properties
        Operator  % 操作符，例如 '+', '*', 'sin'
        Children  % 子节点（列表，最大为 2 个）
        Value     % 常量值或变量名（仅叶子节点有）
    end
    
    methods
        function obj = ExprNode(op, children, value)
            % 构造函数
            obj.Operator = op;
            obj.Children = children;
            obj.Value = value;
        end

        function exprStr = toString(obj)
            % 递归转为字符串
            if isempty(obj.Operator)
                if iscell(obj.Value)
                    obj.Value = obj.Value{1};
                end
                if isnumeric(obj.Value)
                    exprStr = num2str(obj.Value); % 数值转换为字符串
                elseif ischar(obj.Value) || isstring(obj.Value)
                    exprStr = char(obj.Value); % 字符串变量名直接转换
                else
                    fprintf('test\n');
                    disp(obj.Value)
                    error('Unsupported Value type: obj.Value must be numeric or a string.');
                end
            else
                if numel(obj.Children) == 1
                    exprStr = sprintf('%s(%s)', obj.Operator, obj.Children{1}.toString());
                else
                    exprStr = sprintf('(%s %s %s)', ...
                        obj.Children{1}.toString(), obj.Operator, obj.Children{2}.toString());
                end
            end
        end

        function isConst = isConstant(obj)
            % 判断当前树是否是常数（仅包含数值叶子节点）
            if isempty(obj.Operator)
                isConst = isnumeric(obj.Value); % 叶子节点，数值为常数
            else
                isConst = all(cellfun(@(child) child.isConstant(), obj.Children));
            end
        end

        function obj = modifyRandomOperator(obj, availableOperators)
            % 随机选择并修改操作符
            
            % 收集所有节点及其操作符
            [nodes, operators] = obj.collectNodesAndOperators();
            
            if isempty(operators)
%                 error('表达式中没有操作符可供修改。');
                return
            end
            
            % 随机选择一个操作符
            idx = randi(length(operators));
            selectedNode = nodes{idx};
            
            % 随机选择一个与当前操作符不同的新操作符
            newOperator = selectedNode.Operator;

            if strcmp(selectedNode.Operator, 'sin') || strcmp(selectedNode.Operator, 'cos') ...
               || strcmp(selectedNode.Operator, 'log') || strcmp(selectedNode.Operator, 'exp')
                opmode = 1;
            else
                opmode = 2;
            end
            newopmode = opmode;

            while strcmp(newOperator, selectedNode.Operator) || newopmode ~= opmode
                newOperator = availableOperators{randi(length(availableOperators))};
                if strcmp(newOperator, 'sin') || strcmp(newOperator, 'cos') ...
                   || strcmp(newOperator, 'log') || strcmp(newOperator, 'exp')
                    newopmode = 1;
                else
                    newopmode = 2;
                end
            end
            
            % 修改操作符
%             fprintf('将操作符 "%s" 修改为 "%s"\n', selectedNode.Operator, newOperator);
            selectedNode.Operator = newOperator;
        end
        
        function [nodes, operators] = collectNodesAndOperators(obj)
            % 递归收集所有包含操作符的节点及其操作符
            nodes = {};
            operators = {};
            
            if ~isempty(obj.Operator)
                nodes{end+1} = obj; 
                operators{end+1} = obj.Operator; 
            end
            
            if ~isempty(obj.Children)
                for i = 1:length(obj.Children)
                    [childNodes, childOperators] = obj.Children{i}.collectNodesAndOperators();
                    nodes = [nodes, childNodes]; 
                    operators = [operators, childOperators]; 
                end
            end
        end

        function obj = insertRandomNode(obj, availableOperators, vars)
            % 随机插入一个新节点
            
            % 随机选择操作符
            newOperator = availableOperators{randi(length(availableOperators))};

            if strcmp(newOperator, 'sin') || strcmp(newOperator, 'cos') ...
               || strcmp(newOperator, 'log') || strcmp(newOperator, 'exp')
                numChildren = 1;
            else
                numChildren = 2;
            end
            
            % 创建新节点
            newNode = ExprNode(newOperator, {}, []);
            
            % 收集所有节点
            allNodes = obj.collectAllNodes();
            
            % 随机选择一个插入位置
            targetNode = allNodes{randi(length(allNodes))};
            
            % 插入逻辑
            if numChildren == 1
                % 一元操作符
%                 fprintf('在节点插入一元操作符 "%s"\n', newOperator);
                newNode.Children = {targetNode}; % 新节点的唯一子节点是目标节点
                obj = replaceNode(obj, targetNode, newNode);
            elseif numChildren == 2
                % 二元操作符
%                 fprintf('在节点插入二元操作符 "%s"\n', newOperator);
                
                % 新节点需要两个子节点，目标节点作为一个子节点，另一个随机生成
                if rand() > 0.5
                    newChild = ExprNode([], {}, randi(10)); % 随机生成常量值节点
                else
                    newChild = ExprNode([], {}, vars(randi(length(vars))));
                end

                newNode.Children = {targetNode, newChild};
                obj = replaceNode(obj, targetNode, newNode);
            end
        end


        function obj = replaceSubtreeWithConstantOrVariable(obj, vars)
            % 随机用变量或常数替换一棵子树
            
            % 收集所有可替换的子树节点
            allNodes = collectAllNodes(obj);
            replaceableNodes = allNodes(cellfun(@(node) isa(node, 'ExprNode') && ~isempty(node.Children), allNodes)); % 只保留有子节点的 ExprNode
            
            if isempty(replaceableNodes)
%                 error('没有可替换的子树。');
                return
            end
            
            % 随机选择一个子树
            targetNode = replaceableNodes{randi(length(replaceableNodes))};
            
            % 创建替换节点
            if rand() > 0.5
                newNode = ExprNode([], {}, randi(10)); % 随机生成常量值节点
            else
                if iscell(vars)
                    newNode = ExprNode([], {}, vars{randi(length(vars))});
                else
                    newNode = ExprNode([], {}, vars(randi(length(vars))));
                end
            end
        
            % 替换目标子树
            obj = obj.replaceNode(targetNode, newNode);
        end


        function nodes = collectAllNodes(obj)
            % 收集所有节点，并处理空值或非 ExprNode 类型
            nodes = {obj};
            if ~isempty(obj.Children)
                for i = 1:length(obj.Children)
                    if ~isempty(obj.Children{i}) && isa(obj.Children{i}, 'ExprNode')
                        childNodes = obj.Children{i}.collectAllNodes();
                        nodes = [nodes, childNodes];
                    end
                end
            end
        end


        
        function obj = replaceNode(obj, oldNode, newNode)
            % 替换树中的某个节点
            if isequal(obj, oldNode)
                obj = newNode;
                return;
            end
            if ~isempty(obj.Children)
                for i = 1:length(obj.Children)
                    obj.Children{i} = obj.Children{i}.replaceNode(oldNode, newNode);
                end
            end
        end

        function [complexity, nodeCount, depth] = computeComplexity(obj)
            % 计算表达式树的复杂度
            % complexity: 表达式的综合复杂度
            % nodeCount: 节点数量
            % depth: 树的深度
            
            if isempty(obj.Children)
                % 叶子节点
                complexity = 1; % 叶子节点的复杂度权重
                nodeCount = 1;
                depth = 1;
            else
                % 非叶子节点
                childComplexities = zeros(1, length(obj.Children));
                childNodeCounts = zeros(1, length(obj.Children));
                childDepths = zeros(1, length(obj.Children));
                
                for i = 1:length(obj.Children)
                    [childComplexities(i), childNodeCounts(i), childDepths(i)] = ...
                        obj.Children{i}.computeComplexity();
                end
                
                % 当前节点复杂度
                if any(strcmp(obj.Operator, {'+', '-', '*', '/'}))
                    nodeComplexity = 2; % 二元操作符的复杂度权重
                elseif any(strcmp(obj.Operator, {'sin', 'cos', 'exp', 'log'}))
                    nodeComplexity = 3; % 一元操作符的复杂度权重
                else
                    nodeComplexity = 1; % 默认复杂度
                end
                
                % 计算综合复杂度
                complexity = nodeComplexity + sum(childComplexities);
                nodeCount = 1 + sum(childNodeCounts);
                depth = 1 + max(childDepths);
            end
        end

    end
end
