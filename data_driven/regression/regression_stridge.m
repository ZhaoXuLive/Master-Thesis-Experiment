function Xi = regression_stridge(Theta, dXdt, lambda, tau, max_iter)

X = Theta;
Y = dXdt;

% STRidge: Sparse Thresholding Ridge Regression for multi-output
% Inputs:
%   X        - Design matrix (n x d)
%   Y        - Response matrix (n x m)
%   lambda   - Ridge regression regularization parameter
%   tau      - Sparsity threshold
%   max_iter - Maximum number of iterations (default = 10)
% Output:
%   Xi     - Regression coefficients (d x m)

% Set default maximum iterations if not provided
if nargin < 5
    max_iter = 10;
end

% Initialize Xi using Ridge regression
[n, d] = size(X);
m = size(Y, 2); % Number of output dimensions
Xi = (X' * X + lambda * eye(d)) \ (X' * Y); % d x m matrix

% Iteratively apply sparsity thresholding
for iter = 1:max_iter
    % Thresholding: set small coefficients to zero (element-wise)
    Xi(abs(Xi) < tau) = 0;

    % Recompute Ridge regression on remaining terms
    non_zero_idx = any(abs(Xi) >= tau, 2); % Keep rows with any non-zero coefficients
    if sum(non_zero_idx) == 0
        break; % If all coefficients are zero, exit early
    end
    X_reduced = X(:, non_zero_idx); % Reduce X
    Xi_reduced = (X_reduced' * X_reduced + lambda * eye(sum(non_zero_idx))) \ (X_reduced' * Y);

    % Update Xi with reduced solution
    Xi = zeros(d, m);
    Xi(non_zero_idx, :) = Xi_reduced;
end

end