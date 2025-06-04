%% mechanism test

close all

deg_trans = 180 / pi;

XB_t = Maaav_improved_Me(TestY, TestU, TestT, TestTrq, avp, sepT);
Error_XB_t = XB_t - TestY;
Error_XB_t(:, 3) = (mod(Error_XB_t(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_XB_t(:, 4) = (mod(Error_XB_t(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_XB_t(:, 5) = (mod(Error_XB_t(:, 5) + pi, 2*pi) - pi) * deg_trans;
LatError_XB_t = sqrt(hypot(Error_XB_t(:, 1), Error_XB_t(:, 2)).^2);

load('sindy_xmt.mat')
Error_XM_t = XM_t - TestY;
Error_XM_t(:, 3) = (mod(Error_XM_t(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_XM_t(:, 4) = (mod(Error_XM_t(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_XM_t(:, 5) = (mod(Error_XM_t(:, 5) + pi, 2*pi) - pi) * deg_trans;
LatError_XM_t = sqrt(hypot(Error_XM_t(:, 1), Error_XM_t(:, 2)).^2);

load('nn_xnt.mat')
Error_XN_t = XN_t - TestY;
Error_XN_t(:, 3) = (mod(Error_XN_t(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_XN_t(:, 4) = (mod(Error_XN_t(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_XN_t(:, 5) = (mod(Error_XN_t(:, 5) + pi, 2*pi) - pi) * deg_trans;
LatError_XN_t = sqrt(hypot(Error_XN_t(:, 1), Error_XN_t(:, 2)).^2);

load('gpt_xgt.mat')
Error_XG_t = XG_t - TestY;
Error_XG_t(:, 3) = (mod(Error_XG_t(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_XG_t(:, 4) = (mod(Error_XG_t(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_XG_t(:, 5) = (mod(Error_XG_t(:, 5) + pi, 2*pi) - pi) * deg_trans;
LatError_XG_t = hypot(Error_XG_t(:, 1), Error_XG_t(:, 2));

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
set(0,'DefaultAxesFontsize',16);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

time_t = (0.01:0.01:30)';

figure,
plot(time_t, LatError_XB_t(sepT(3,1):sepT(3,2)), 'LineWidth',1), hold on
plot(time_t, LatError_XM_t(sepT(3,1):sepT(3,2)), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, LatError_XN_t(sepT(3,1):sepT(3,2)), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, LatError_XG_t(sepT(3,1):sepT(3,2)), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}位置误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','northwest');
set(gcf, 'Position', [600, 400, 500, 300]); 

figure,
plot(time_t, Error_XB_t(sepT(3,1):sepT(3,2), 6), 'LineWidth',1), hold on
plot(time_t, Error_XM_t(sepT(3,1):sepT(3,2), 6), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XN_t(sepT(3,1):sepT(3,2), 6), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XG_t(sepT(3,1):sepT(3,2), 6), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}纵向速度误差 \fontname{Times New Roman}(m/s)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','southwest');
set(gcf, 'Position', [600, 400, 500, 300]); 

figure,
plot(time_t, Error_XB_t(sepT(3,1):sepT(3,2), 7), 'LineWidth',1), hold on
plot(time_t, Error_XM_t(sepT(3,1):sepT(3,2), 7), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XN_t(sepT(3,1):sepT(3,2), 7), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XG_t(sepT(3,1):sepT(3,2), 7), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横向速度误差 \fontname{Times New Roman}(m/s)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','southwest');
set(gcf, 'Position', [600, 400, 500, 300]); 

figure,
plot(time_t, Error_XB_t(sepT(3,1):sepT(3,2), 3), 'LineWidth',1), hold on
plot(time_t, Error_XM_t(sepT(3,1):sepT(3,2), 3), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XN_t(sepT(3,1):sepT(3,2), 3), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XG_t(sepT(3,1):sepT(3,2), 3), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}头车横摆角误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','northwest');
set(gcf, 'Position', [600, 400, 500, 300]); 

figure,
plot(time_t, Error_XB_t(sepT(3,1):sepT(3,2), 4), 'LineWidth',1), hold on
plot(time_t, Error_XM_t(sepT(3,1):sepT(3,2), 4), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XN_t(sepT(3,1):sepT(3,2), 4), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XG_t(sepT(3,1):sepT(3,2), 4), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}中间车横摆角误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','northwest');
set(gcf, 'Position', [600, 400, 500, 300]); 

figure,
plot(time_t, Error_XB_t(sepT(3,1):sepT(3,2), 5), 'LineWidth',1), hold on
plot(time_t, Error_XM_t(sepT(3,1):sepT(3,2), 5), '--','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XN_t(sepT(3,1):sepT(3,2), 5), '-.','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
plot(time_t, Error_XG_t(sepT(3,1):sepT(3,2), 5), '-+','LineWidth',1, 'MarkerIndices', 1:100:length(time_t)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}尾车横摆角误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}机理模型','\fontname{宋体}进化稀疏建模','\fontname{宋体}神经网络','\fontname{宋体}高斯过程回归', 'Location','northwest');
set(gcf, 'Position', [600, 400, 500, 300]); 
