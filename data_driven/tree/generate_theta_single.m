%% 生成候选特征矩阵的函数
function Theta = generate_theta_single(data, expression, str2funcHead)
    n = size(data, 1);

    funcs = str2func(strcat(str2funcHead, expression));

    % 特殊情况的处理是有问题的，为什么需要对每一次都进行处理呢？
    % 按理说是对符号进行处理，不用代入值

    Theta = funcs(data(:,1), data(:,2), data(:,3), data(:,4), data(:,5), ...
                      data(:,6), data(:,7), data(:,8), data(:,9), data(:,10), ...
                      data(:,11), data(:,12), data(:,13), data(:,14), data(:,15), ...
                      data(:,16), data(:,17), data(:,18), data(:,19), data(:,20));

    if length(Theta) == 1
        Theta = Theta * ones(n, 1);
    end
    
end