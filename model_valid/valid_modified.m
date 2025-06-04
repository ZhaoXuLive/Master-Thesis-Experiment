close all;
addpath('DataRecord');

%% parameters
deg_trans = 180 / pi;

% articulated vehicle parameter
avp.m = [11142; 7651; 11142;];
avp.iz = [143570; 37800; 143570;];
avp.lj = [11.4; 9.4; 11.4;];
avp.la = [2.5; 4; 9.9; 1.5; 7.9; 1.5; 7.4; 8.9];
avp.lg = [5.862; 4.643; 5.538;];
avp.h = [0.9625; 0.9625; 0.98; 0.98; 0.98; 0.98; 0.9625; 0.9625;];
avp.d = [10000; 10000];
avp.ca = 184000;
avp.sat = 15;
avp.hc = [1.855; 1.719; 1.855];

% longtidinual model parameer
avp.g = 9.8034;
avp.pho = 1.225;  % 空气密度
avp.Crr = 0.0094;   % 滚动阻力系数
avp.KinRollRadius = 0.49;   % 运动滚动半径
avp.NomRadius = 0.4995; % 静态滚动半径
avp.A1 = 2.0;
avp.A2 = 5.5;
avp.A3 = 9.5;
avp.Cd1 = 0.2;
avp.Cd2 = 0.525;
avp.Cd3 = 0.525;

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
set(0,'DefaultAxesFontsize',16);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

sty={'r-','g-','b-','r--','g--','b--'};
axle_str={'Axle 1','Axle 2','Axle 3','Axle 4','Axle 5','Axle 6','Axle 7','Axle 8'};
unit_str={'\fontname{宋体}头车（机理）','\fontname{宋体}中间车（机理）','\fontname{宋体}尾车（机理）', ...
          '\fontname{宋体}头车（仿真）','\fontname{宋体}中间车（仿真）','\fontname{宋体}尾车（仿真）'};

%% load data

load('15m5mscircle.mat');
startpoint = 3621;
endpoint = length(Expr_time) - 5000;

T = Expr_time(startpoint:endpoint, :) - Expr_time(startpoint-1);
Y = Expr_state(startpoint:endpoint, :);
U = Expr_steer(startpoint:endpoint, :);
% Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));
Trq = Expr_drive(startpoint:endpoint, :);
FxL = Expr_FxL(startpoint:endpoint, :);
FxR = Expr_FxR(startpoint:endpoint, :);
slipAngleL = Expr_LatSlipL(startpoint:endpoint, :);
slipAngleR = Expr_LatSlipR(startpoint:endpoint, :);
FyL = Expr_FyL(startpoint:endpoint, :);
FyR = Expr_FyR(startpoint:endpoint, :);

%% iter predict

XO = zeros(length(T), 10);
FxL_m = zeros(length(T), 8);
FxR_m = zeros(length(T), 8);
slipAngleL_m = zeros(length(T), 8);
slipAngleR_m = zeros(length(T), 8);
FyL_m = zeros(length(T), 8);
FyR_m = zeros(length(T), 8);
FzL_m = zeros(length(T), 8);
FzR_m = zeros(length(T), 8);
XO(1, :) = Y(1, :);
for i = 2:length(T)
    dt = T(i) - T(i-1);
    [qdd, FxL_m(i, :), FxR_m(i, :), slipAngleL_m(i, :), slipAngleR_m(i, :), FyL_m(i, :), FyR_m(i, :), FzL_m(i, :), FzR_m(i, :)] = ...
        Maaav_up(XO(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :));
    XO(i, 6:10) = XO(i-1, 6:10) + qdd * dt;
    XO(i, 1:5) = XO(i-1, 1:5) + XO(i-1, 6:10) * dt;
end
FxL_m(1, :) = FxL_m(2, :);
FxR_m(1, :) = FxR_m(2, :);
slipAngleL_m(1, :) = slipAngleL_m(2, :);
slipAngleR_m(1, :) = slipAngleR_m(2, :);
FyL_m(1, :) = FyL_m(2, :);
FyR_m(1, :) = FyR_m(2, :);
FzL_m(1, :) = FzL_m(2, :);
FzR_m(1, :) = FzR_m(2, :);

vx_tm = Y(:, 6) .* cos(Y(:, 3)) + Y(:, 7) .* sin(Y(:, 3));
vy_tm = -Y(:, 6) .* sin(Y(:, 3)) + Y(:, 7) .* cos(Y(:, 3));
vx_math = XO(:, 6) .* cos(XO(:, 3)) + XO(:, 7) .* sin(XO(:, 3));
vy_math = -XO(:, 6) .* sin(XO(:, 3)) + XO(:, 7) .* cos(XO(:, 3));

h7 = figure;
plot(Y(:,1), Y(:,2), 'LineWidth',1), hold on
plot(XO(:,1), XO(:,2), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型');
grid on
xlabel('\fontname{宋体}横向位置 \fontname{Times New Roman}(m)');
ylabel('\fontname{宋体}纵向位置 \fontname{Times New Roman}(m)');
set(gcf, 'Position', [600, 400, 700, 250]); 
axis([150 310 -10 40]);
print(h7,'-depsc2','-loose',[FigurePath,'traj_modified.eps']); 
savefig(h7, [FigurePath,'/traj_modified.fig']);

set(0,'DefaultAxesFontsize',15);

h8 = figure;
plot(T, vx_tm, 'LineWidth',1), hold on
plot(T, vx_math, '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型','Location','southeast');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}纵向速度 \fontname{Times New Roman}(m/s)');
set(gcf, 'Position', [600, 400, 500, 300]); 
axis([0 45 1.5 6.5]);
print(h8,'-depsc2','-loose',[FigurePath,'vx_modified.eps']); 
savefig(h8, [FigurePath,'/vx_modified.fig']);

h9 = figure;
plot(T, vy_tm, 'LineWidth',1), hold on
plot(T, vy_math, '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横向速度 \fontname{Times New Roman}(m/s)');
set(gcf, 'Position', [600, 400, 500, 300]); 
axis([0 45 -0.5 2.5]);
print(h9,'-depsc2','-loose',[FigurePath,'vy_modified.eps']); 
savefig(h9, [FigurePath,'/vy_modified.fig']);

h10 = figure;
plot(T, (mod(XO(:, 3) + pi, 2*pi) - pi) * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, (mod(XO(:, 4) + pi, 2*pi) - pi) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, (mod(XO(:, 5) + pi, 2*pi) - pi) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, Y(:, 3) * deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:500:length(T)), hold on
plot(T, Y(:, 4) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:500:length(T)), hold on
plot(T, Y(:, 5) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:500:length(T)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横摆角 \fontname{Times New Roman}(deg)');
set(gcf, 'Position', [600, 400, 700, 400]); 
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 45 -220 300]);
print(h10,'-depsc2','-loose',[FigurePath,'yaw_modified.eps']); 
savefig(h10, [FigurePath,'/yaw_modified.fig']);

h11 = figure;
plot(T, (mod(XO(:, 8) + pi, 2*pi) - pi) * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, (mod(XO(:, 9) + pi, 2*pi) - pi) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, (mod(XO(:, 10) + pi, 2*pi) - pi) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, Y(:, 8) * deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:500:length(T)), hold on
plot(T, Y(:, 9) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:500:length(T)), hold on
plot(T, Y(:, 10) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:500:length(T)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横摆角速度 \fontname{Times New Roman}(deg/s)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 45 -5 30]);
print(h11,'-depsc2','-loose',[FigurePath,'yawrate_modified.eps']); 
savefig(h11, [FigurePath,'/yawrate_modified.fig']);

