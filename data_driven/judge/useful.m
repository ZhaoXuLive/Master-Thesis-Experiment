function result = useful(theta_iter, x, dX, threshold, str2funcHead)

    % 将字符串转为数值
    theta_data_iter = generate_theta_single(x, theta_iter, str2funcHead);

%     m = size(theta_data_iter, 1);
%     if m > 1000
%         selected_indices = randperm(m, 500);
%         mic = mine(theta_data_iter(selected_indices)', dX(selected_indices)').mic;
%     else
%         mic = mine(theta_data_iter', dX').mic;
%     end

    for i = 1:5
        mi_all(i) = normalizeMI(theta_data_iter, dX(:,i+5));
    end

%     mi = sum(abs(mi_all));
    mi = max(abs(mi_all));

%     disp(mi);

    if mi > threshold
        result = 1;
%         fprintf('对目标数据有效\n');
    else
        result = 0;
%         fprintf('对目标数据无效\n');
    end
end