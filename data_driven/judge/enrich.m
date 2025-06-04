function result = enrich(population_string_iter, Theta_all, varsdata, threshold, str2funcHead)

    % 将字符串转为数值
    population_data_iter = generate_theta_single(varsdata, population_string_iter, str2funcHead);

    pearson = corr(population_data_iter, Theta_all(:, 2:end), 'Type', 'Pearson');
    hasLargerValue = any(abs(pearson) > threshold);
    hasNaN = any(isnan(pearson));
    allEqual = all(population_data_iter == population_data_iter(1));

%     disp(population_string_iter);

    if hasLargerValue || hasNaN || allEqual
        result = 0;
%         fprintf('与先前函数相似性过高，重新生成\n')
    else
        result = 1;
%         fprintf('与先前函数相似性较低，可继续\n')
    end
end