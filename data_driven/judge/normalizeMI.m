

function nmi = normalizeMI(x, y, num_bins)
    % 计算归一化互信息 (Normalized Mutual Information, NMI)
    % x 和 y 是输入向量
    % num_bins 是直方图的分箱数，默认使用 sqrt(length(x)) 个分箱
    
    if nargin < 3
        num_bins = round(sqrt(length(x))); % 默认分箱数
    end
    
    % 计算互信息
    mi = mutual_info(x, y, num_bins);
    
    % 计算熵 H(X) 和 H(Y)
    hx = entropy(x, num_bins);
    hy = entropy(y, num_bins);
    
    % 归一化互信息
    nmi = 2 * mi / (hx + hy);
end

function mi = mutual_info(x, y, num_bins)
    % 计算互信息 (Mutual Information)
    % x 和 y 是输入向量
    % num_bins 是直方图的分箱数，默认使用 sqrt(length(x)) 个分箱
    
    if nargin < 3
        num_bins = round(sqrt(length(x))); % 默认分箱数
    end
    
    % 计算联合直方图
    jointHist = histcounts2(x, y, num_bins, 'Normalization', 'probability');
    
    % 计算边缘分布
    px = sum(jointHist, 2); % x 的边缘概率
    py = sum(jointHist, 1); % y 的边缘概率
    
    % 避免 log(0) 引发的 NaN
    jointHist(jointHist == 0) = NaN; 
    
    % 计算互信息
    [X, Y] = meshgrid(py, px);
    mi = nansum(jointHist(:) .* log(jointHist(:) ./ (X(:) .* Y(:))));
end

function h = entropy(x, num_bins)
    % 计算熵 H(X)
    % x 是输入向量
    % num_bins 是分箱数
    
    if nargin < 2
        num_bins = round(sqrt(length(x))); % 默认分箱数
    end
    
    % 计算直方图
    hist_data = histcounts(x, num_bins, 'Normalization', 'probability');
    
    % 计算熵
    hist_data = hist_data(hist_data > 0); % 排除零值
    h = -sum(hist_data .* log(hist_data));
end
