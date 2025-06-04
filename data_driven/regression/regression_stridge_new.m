function Xi = regression_stridge_new(Theta, dXdt, lambda, tau, max_iter)
% STRidge: Sparse Thresholding Ridge Regression for multi-output
% 修正说明：
%   1. 各输出独立进行特征选择
%   2. 添加截距项处理
%   3. 支持特征标准化

X = Theta;
Y = dXdt;

% 添加截距项（若未包含）
if ~all(X(:,1) == 1)
    X = [ones(size(X,1),1), X]; % 添加全1列作为截距
end

% 标准化特征（岭回归需标准化）
[n, d] = size(X);
X_mean = mean(X(:,2:end), 1); % 截距列不标准化
X_std = std(X(:,2:end), 0, 1);
X(:,2:end) = (X(:,2:end) - X_mean) ./ X_std;

% 初始化Xi矩阵（包含截距项）
m = size(Y, 2);
Xi = zeros(d, m);

% 对每个输出独立进行STRidge
for output_idx = 1:m
    y = Y(:, output_idx);
    
    % 初始岭回归解
    xi = (X' * X + lambda * eye(d)) \ (X' * y);
    
    % 迭代稀疏阈值
    for iter = 1:max_iter
        % 阈值处理（保留截距项）
        mask = [true; abs(xi(2:end)) >= tau]; % 截距始终保留
        xi(~mask) = 0;
        
        % 缩减特征子集（包含截距）
        X_reduced = X(:, mask);
        if sum(mask) == 1 % 仅剩截距项
            xi_reduced = X_reduced \ y;
        else
            xi_reduced = (X_reduced' * X_reduced + lambda * eye(sum(mask))) \ (X_reduced' * y);
        end
        
        % 更新系数
        xi = zeros(d, 1);
        xi(mask) = xi_reduced;
        
        % 提前终止条件：系数不再变化
        if all(mask == [true; abs(xi(2:end)) >= tau])
            break;
        end
    end
    
    % 存储结果并还原标准化
    xi(2:end) = xi(2:end) ./ X_std'; % 系数反标准化
    xi(1) = xi(1) - sum(xi(2:end) .* X_mean'); % 调整截距
    Xi(:, output_idx) = xi;
end

% 移除添加的截距项（若原始数据未包含）
if ~all(Theta(:,1) == 1)
    Xi = Xi(2:end, :);
end
end