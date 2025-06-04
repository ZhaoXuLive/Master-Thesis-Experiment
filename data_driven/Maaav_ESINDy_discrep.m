function out = Maaav_ESINDy_discrep(Y, U, T, Xi_iter, avp, polyorder, sep, ...
                  vars_mean, vars_std, Ef_mean, Ef_std, population_string, str2funcHead)

% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

dt = T(2) - T(1);
out = zeros(length(T), 10);

% 用于记录差异数据和候选函数数据，辅助测试
eff = zeros(length(T), 5);
poolff = zeros(length(T), length(Xi_iter(:, 1)));

m = length(population_string);
functions = cell(m, 1);
for i = 1:m
    functions{i} = str2func(strcat(str2funcHead, population_string{i}));
end

for iter = 1:length(sep(:, 1))
    out(sep(iter, 1), :) = Y(sep(iter, 1), :);
    
    for i = sep(iter, 1) + 1:sep(iter,2)
        
        qdd = Maaav(out(i - 1, :), avp, U(i - 1, :))';

        vars = [sin(out(i - 1, 3)) sin(out(i - 1, 4)) sin(out(i - 1, 5)) ...
                cos(out(i - 1, 3)) cos(out(i - 1, 4)) cos(out(i - 1, 5)) ...
                out(i - 1, 6:10) U(i - 1, :)];

        vars_norm = (vars - vars_mean) ./ vars_std;

        yPool = zeros(1, size(Xi_iter, 1));
%         yPool(:, 1:1540) = poolData_standard_vars(vars_norm, polyorder);
        for k = 1:m
            yPool(:, k) = functions{k}(vars_norm(1), vars_norm(2), vars_norm(3), ...
                      vars_norm(4), vars_norm(5), vars_norm(6), vars_norm(7), vars_norm(8), ...
                      vars_norm(9), vars_norm(10), vars_norm(11), vars_norm(12), ...
                      vars_norm(13), vars_norm(14), vars_norm(15), vars_norm(16));
        end

%         disc = (yPool * Xi_iter .* Ef_std(:, 6:10)) + Ef_mean(:, 6:10);
        disc = yPool * Xi_iter;
    
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

