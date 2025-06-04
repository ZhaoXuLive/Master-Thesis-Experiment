%% 评估种群的适应度函数
function complexity = calculate_complex(population)

    n = size(population, 1);
    complexity = zeros(n, 1);
    for i = 1:n
        [complexity(i), ~, ~] = computeComplexity(population{i});
    end

    complexity = abs(complexity);
    max_complexity = max(complexity);
    min_complexity = min(complexity(complexity ~= 0));
    complexity(complexity == 0) = min_complexity / 2;
    complexity = complexity / max_complexity;

end