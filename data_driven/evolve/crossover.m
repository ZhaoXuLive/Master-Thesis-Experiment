function [child1, child2] = crossover(parent1, parent2)
    % 随机选择交叉点
    subTree1 = selectRandomSubtree(parent1);
    subTree2 = selectRandomSubtree(parent2);
    
    % 替换子树
    child1 = replaceSubtree(parent1, subTree1, subTree2);
    child2 = replaceSubtree(parent2, subTree2, subTree1);
end

function subtree = selectRandomSubtree(tree)
    % 随机选择一个子树
    if isempty(tree.Children) || rand < 0.3
        subtree = tree; % 选择当前节点
    else
        childIdx = randi(numel(tree.Children));
        subtree = selectRandomSubtree(tree.Children{childIdx});
    end
end

function newTree = replaceSubtree(tree, oldSubtree, newSubtree)
    % 替换子树
    if isequal(tree, oldSubtree)
        newTree = newSubtree;
    else
        newChildren = cell(size(tree.Children));
        for i = 1:numel(tree.Children)
            newChildren{i} = replaceSubtree(tree.Children{i}, oldSubtree, newSubtree);
        end
        newTree = ExprNode(tree.Operator, newChildren, tree.Value);
    end
end
