function Xi = regression_lasso(Theta, dXdt)

% % 数据标准化（可选）
% Theta_mean = mean(Theta);
% Theta_std = std(Theta);
% Theta_std(Theta_std == 0) = 1; % 避免除以零
% Theta = (Theta - Theta_mean) ./ Theta_std;
% dXdt_mean = mean(dXdt);
% dXdt = dXdt - dXdt_mean;

% %% Normalize the library data
% if NormalizeLib==1
%     for norm_k=1:size(Theta,2)
%         normLib(norm_k) = norm(Theta(:,norm_k));
%         Theta(:,norm_k) = Theta(:,norm_k)/normLib(norm_k);
%     end
% end

%% Peform sparse regression
Xi = zeros(size(Theta,2), 5);
% for i = 1:5
%     [B, ~] = lasso(Theta, dXdt(:, i+5), 'Lambda', 1e-5);
%     Xi(:, i) = B;
% end

for i = 1:5
    [B, FitInfo] = lasso(Theta, dXdt(:, i+5), 'CV', 10);
    % 'Standardize', false 'CV', 10
    % 选择一个合适的lambda值，例如使用交叉验证的最优值
    idxLambdaMinMSE = FitInfo.IndexMinMSE;
    MaxLambda = FitInfo.Lambda1SE
    Xi(:, i) = B(:, idxLambdaMinMSE);
end

% %% Now output the SINDy Identified ODEs
% % Now retrive the parameters
% if NormalizeLib==1
%     for norm_k=1:length(Xi)
%         Xi(norm_k,:) = Xi(norm_k,:)/normLib(norm_k);
%     end
% end

end