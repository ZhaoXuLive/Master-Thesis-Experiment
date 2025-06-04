 function out = Maaav_improved_single(t, y, avp, u, trq, n)
% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang


x0 = y(1, :);  
out = zeros(length(t), n);
out(1, :) = x0;

for i = 2:length(t)
    dt = t(i) - t(i-1);
    out(i, 6:10) = y(i-1, 6:10) + Maaav_improved(y(i - 1, :), avp, u(i - 1, :), trq(i - 1, :))' * dt;
    out(i, 1:5) = y(i-1, 1:5) + out(i-1, 6:10) * dt;
end