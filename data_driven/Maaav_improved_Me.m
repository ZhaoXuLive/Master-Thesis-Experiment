function out = Maaav_improved_Me(Y, U, T, Trq, avp, sep)

% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

out = zeros(length(T), 10);

for iter = 1:length(sep(:, 1))
    out(sep(iter, 1), :) = Y(sep(iter, 1), :);
    
    for i = sep(iter, 1) + 1:sep(iter,2)

    	qdd = Maaav_improved(out(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :))';
        
        dt = T(i) - T(i-1);
        out(i, 6:10) = out(i-1, 6:10) + qdd * dt;
        out(i, 1:5) = out(i-1, 1:5) + out(i-1, 6:10) * dt;

    end
end
