function out = Maaav_SINDy_discrep_standard(Y, U, T, Xi, Xintercept, avp, polyorder, sep, vars_mean, vars_std) %, Xi_sparse_indices)

% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

dt = T(2) - T(1);
out = zeros(length(T), 10);

% 用于记录差异数据和候选函数数据，辅助测试
eff = zeros(length(T), 5);
poolff = zeros(length(T), length(Xi(:, 1)));

for iter = 1:length(sep(:, 1))
    out(sep(iter, 1), :) = Y(sep(iter, 1), :);
    
    for i = sep(iter, 1) + 1:sep(iter,2)
        
        qdd = Maaav(out(i - 1, :), avp, U(i - 1, :))';

        vars = [out(i - 1, 3:10) U(i - 1, :)];
        vars_norm = (vars - vars_mean) ./ vars_std;

        yPool = standard_function(vars_norm, polyorder);

        disc = yPool * Xi + Xintercept'; 

        eff(i,:) = disc;
        poolff(i,:) = yPool;
        qdd = disc + qdd;

        out(i, 6:10) = out(i-1, 6:10) + qdd * dt;
        out(i, 1:5) = out(i-1, 1:5) + out(i-1, 6:10) * dt;

        % when predict over and over, do some limit
        if(hypot(out(i, 1) - out(i-1, 1), out(i, 2) - out(i-1, 2)) > 0.1)
            break;
        end
    end
end
end

