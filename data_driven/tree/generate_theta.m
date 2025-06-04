%% 生成候选特征矩阵的函数
function Theta = generate_theta(data, expression, str2funcHead)
    n = size(data, 1);
    m = length(expression);
    Theta = zeros(n, m);
    threshold = 0.001;

    functions = cell(m, 1);
    for i = 1:m
        functions{i} = str2func(strcat(str2funcHead, expression{i}));
    end

    % 特殊情况的处理是有问题的，为什么需要对每一次都进行处理呢？
    % 按理说是对符号进行处理，不用代入值
    for i = 1:m
        Theta(:, i) = functions{i}(data(:,1), data(:,2), data(:,3), data(:,4), data(:,5), ...
                      data(:,6), data(:,7), data(:,8), data(:,9), data(:,10), ...
                      data(:,11), data(:,12), data(:,13), data(:,14), data(:,15), ...
                      data(:,16), data(:,17), data(:,18), data(:,19), data(:,20));
    end
    
end