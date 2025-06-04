%% 评估种群的适应度函数
function sparse = calculate_sparse(Xi)

    Xi = abs(Xi);
    
    sum_xi_row = sum(Xi, 2);

    max_xi = max(sum_xi_row);
    min_xi = min(sum_xi_row(sum_xi_row ~= 0));

    sum_xi_row(sum_xi_row == 0) = min_xi / 2;

    sparse = sum_xi_row / max_xi;

end