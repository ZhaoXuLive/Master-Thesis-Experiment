function out = Maaav_improved_NN_discrep(Y, U, T, Trq, avp, sep, net, vars_mean, vars_std)

% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

out = zeros(length(T), 10);

for iter = 1:length(sep(:, 1))
    out(sep(iter, 1), :) = Y(sep(iter, 1), :);
    
    for i = sep(iter, 1) + 1:sep(iter,2)

    	qdd = Maaav_improved(out(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :))';
        
%         vars = [out(i - 1, 3:10) U(i - 1, :)];
        vars = [sin(out(i - 1, 3)) sin(out(i - 1, 4)) sin(out(i - 1, 5)) ...
                cos(out(i - 1, 3)) cos(out(i - 1, 4)) cos(out(i - 1, 5)) ...
                out(i - 1, 6:10) U(i - 1, :) Trq(i - 1, :)];
        vars_norm = (vars - vars_mean) ./ vars_std;

        % 使用训练好的网络进行预测
        yPred = predict(net, vars_norm);
        qdd = yPred + qdd;

        dt = T(i) - T(i-1);
        out(i, 6:10) = out(i-1, 6:10) + qdd * dt;
        out(i, 1:5) = out(i-1, 1:5) + out(i-1, 6:10) * dt;

    end
end
