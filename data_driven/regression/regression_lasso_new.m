function Xi = regression_lasso_new(Theta, dXdt)

%% Peform sparse regression
Xi = zeros(size(Theta,2), 5);

for i = 1:5
    lambda = 0.00001;
    [B, FitInfo] = lasso(Theta, dXdt(:, i+5), 'Lambda', lambda);
    % 'Standardize', false 'CV', 10
    % 选择一个合适的lambda值，例如使用交叉验证的最优值
%     idxLambdaMinMSE = FitInfo.IndexMinMSE;
%     MaxLambda = FitInfo.Lambda1SE
%     Xi(:, i) = B(:, idxLambdaMinMSE);
    Xi(:, i) = B;
end

end