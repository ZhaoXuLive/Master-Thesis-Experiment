%% 评估种群的适应度函数
function mi = calculate_mi(population_data, datay)

    n = size(population_data, 2);
    m = 5;
    mi = zeros(n, m);
    
    for k = 1:m
        for i = 1:n
            mi(i, k) = normalizeMI(population_data(:, i), datay(:, k+5));
        end
    end

%     m = size(population_data, 1);
%     if m > 1000
%         selected_indices = randperm(m, 1000);
%         for i = 1:n
%             fitness(i) = mine(population_data(selected_indices, i)', datay(selected_indices)').mic;
%             fprintf('对于第%d个变量，在第%d个种群中：\n', j, i);
%     %         fprintf('特征组合如下: \n');
%     %         disp(population{i});
%             fprintf('互信息系数： %f\n ', fitness(i));
%         end
%     else
%        for i = 1:n
%             fitness(i) = mine(population_data(:, i)', datay').mic;
%             fprintf('对于第%d个变量，在第%d个种群中：\n', j, i);
%     %         fprintf('特征组合如下: \n');
%     %         disp(population{i});
%             fprintf('互信息系数： %f\n ', fitness(i));
%        end
%     end
end