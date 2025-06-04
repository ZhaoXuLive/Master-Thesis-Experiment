%% mechanism test

close all

deg_trans = 180 / pi;

% data memory
% sin(yaw) cos(yaw) vx vy yawrate normalized
% train: 6.2061    4.7884    3.9481    7.2070    7.5754    7.5554
%        1.0036    0.5395    0.8463    0.0174    0.0233    0.0167
% test:  160.7870  134.8410   87.5806   88.1093   88.5277   87.8271
%        9.9816    7.2867    6.8218    0.1598    0.1622    0.1624

% 训练集：末尾偏离问题
% 测试集：速度问题？角度问题？


XM = Maaav_improved_Me(Y, U, T, Trq, avp, sep);

figure,
for i = 1:length(sep(:, 1))
    subplot(3,5,i);
    plot(Y(sep(i,1):sep(i,2),1), Y(sep(i,1):sep(i,2),2), 'k','LineWidth',1), hold on
    plot(XM(sep(i,1):sep(i,2),1), XM(sep(i,1):sep(i,2),2), 'b','LineWidth',1), hold on
    legend('True','Mechanism');
end

Error_NM = XM - Y;
Error_NM(:, 3) = (mod(Error_NM(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_NM(:, 4) = (mod(Error_NM(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_NM(:, 5) = (mod(Error_NM(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_NM_XY = sqrt(mean(hypot(Error_NM(:, 1), Error_NM(:, 2)).^2));
RMSE_NM_X = sqrt(mean((Error_NM(:, 1)).^2));
RMSE_NM_Y = sqrt(mean((Error_NM(:, 2)).^2));
RMSE_NM_YAW1 = sqrt(mean((Error_NM(:, 3)).^2));
RMSE_NM_YAW2 = sqrt(mean((Error_NM(:, 4)).^2));
RMSE_NM_YAW3 = sqrt(mean((Error_NM(:, 5)).^2));
RMSE_NM = [RMSE_NM_XY RMSE_NM_X RMSE_NM_Y RMSE_NM_YAW1 RMSE_NM_YAW2 RMSE_NM_YAW3]

RMSE_NM_VXY = sqrt(mean(hypot(Error_NM(:, 6), Error_NM(:, 7)).^2));
RMSE_NM_VX = sqrt(mean((Error_NM(:, 6)).^2));
RMSE_NM_VY = sqrt(mean((Error_NM(:, 7)).^2));
RMSE_NM_YAWRATE1 = sqrt(mean((Error_NM(:, 8)).^2));
RMSE_NM_YAWRATE2 = sqrt(mean((Error_NM(:, 9)).^2));
RMSE_NM_YAWRATE3 = sqrt(mean((Error_NM(:, 10)).^2));
RMSE_NM_V = [RMSE_NM_VXY RMSE_NM_VX RMSE_NM_VY RMSE_NM_YAWRATE1 RMSE_NM_YAWRATE2 RMSE_NM_YAWRATE3]


XM_t = Maaav_improved_Me(TestY, TestU, TestT, TestTrq, avp, sepT);

Error_NM_T = XM_t - TestY;
Error_NM_T(:, 3) = (mod(Error_NM_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_NM_T(:, 4) = (mod(Error_NM_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_NM_T(:, 5) = (mod(Error_NM_T(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_NM_XY_T = sqrt(mean(hypot(Error_NM_T(:, 1), Error_NM_T(:, 2)).^2));
RMSE_NM_X_T = sqrt(mean((Error_NM_T(:, 1)).^2));
RMSE_NM_Y_T = sqrt(mean((Error_NM_T(:, 2)).^2));
RMSE_NM_YAW1_T = sqrt(mean((Error_NM_T(:, 3)).^2));
RMSE_NM_YAW2_T = sqrt(mean((Error_NM_T(:, 4)).^2));
RMSE_NM_YAW3_T = sqrt(mean((Error_NM_T(:, 5)).^2));
RMSE_NM_T = [RMSE_NM_XY_T RMSE_NM_X_T RMSE_NM_Y_T RMSE_NM_YAW1_T RMSE_NM_YAW2_T RMSE_NM_YAW3_T]

RMSE_NM_VXY_T = sqrt(mean(hypot(Error_NM_T(:, 6), Error_NM_T(:, 7)).^2));
RMSE_NM_VX_T = sqrt(mean((Error_NM_T(:, 6)).^2));
RMSE_NM_VY_T = sqrt(mean((Error_NM_T(:, 7)).^2));
RMSE_NM_YAWRATE1_T = sqrt(mean((Error_NM_T(:, 8)).^2));
RMSE_NM_YAWRATE2_T = sqrt(mean((Error_NM_T(:, 9)).^2));
RMSE_NM_YAWRATE3_T = sqrt(mean((Error_NM_T(:, 10)).^2));
RMSE_NM_V_T = [RMSE_NM_VXY_T RMSE_NM_VX_T RMSE_NM_VY_T RMSE_NM_YAWRATE1_T RMSE_NM_YAWRATE2_T RMSE_NM_YAWRATE3_T]

figure,
for i = 1:length(sepT(:, 1))
    subplot(1,3,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XM_t(sepT(i,1):sepT(i,2),1), XM_t(sepT(i,1):sepT(i,2),2), 'b','LineWidth',1), hold on
    legend('True','Mechanism');
end

