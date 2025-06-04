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

%% load data

load('valid2_20kph_tm.mat');
startpoint = 15001;
endpoint = length(Expr_time) - 5000;
T = Expr_time(startpoint:10:endpoint, :) - Expr_time(startpoint-1);
Y = Expr_state(startpoint:10:endpoint, :);
U = Expr_steer(startpoint:10:endpoint, :);
steer = Expr_steer(startpoint:10:endpoint, :);

load('valid2_20kph_math.mat');
startpoint = 4001;
endpoint = length(out.tout) - 5000;
T_m = out.tout(startpoint:endpoint, :) - out.tout(startpoint-1);
q_m = out.q_out(startpoint:endpoint, :);
qd_m = out.qd_out(startpoint:endpoint, :);
steer_m = out.steer(startpoint:endpoint, :);

%% data process

vx_tm = Y(:, 6) .* cos(Y(:, 3)) + Y(:, 7) .* sin(Y(:, 3));
vy_tm = -Y(:, 6) .* sin(Y(:, 3)) + Y(:, 7) .* cos(Y(:, 3));
vx_math = qd_m(:, 1) .* cos(q_m(:, 3)) + qd_m(:, 2) .* sin(q_m(:, 3));
vy_math = -qd_m(:, 1) .* sin(q_m(:, 3)) + qd_m(:, 2) .* cos(q_m(:, 3));

%% plot

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
set(0,'DefaultAxesFontsize',16);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

sty={'r-','g-','b-','r--','g--','b--'};
% axle_str={'Axle 1','Axle 2','Axle 3','Axle 4','Axle 5','Axle 6','Axle 7','Axle 8'};
axle_tm_str={'Axle 1 (IPG)','Axle 2 (IPG)','Axle 3 (IPG)','Axle 4 (IPG)','Axle 5 (IPG)','Axle 6 (IPG)','Axle 7 (IPG)','Axle 8 (IPG)'};
axle_math_str={'Axle 1 (Math)','Axle 2 (Math)','Axle 3 (Math)','Axle 4 (Math)','Axle 5 (Math)','Axle 6 (Math)','Axle 7 (Math)','Axle 8 (Math)'};

unit_str={'\fontname{宋体}头车（机理）','\fontname{宋体}中间车（机理）','\fontname{宋体}尾车（机理）', ...
          '\fontname{宋体}头车（仿真）','\fontname{宋体}中间车（仿真）','\fontname{宋体}尾车（仿真）'};
axle_str={'\fontname{宋体}1轴','\fontname{宋体}2轴','\fontname{宋体}3轴','\fontname{宋体}4轴', ...
             '\fontname{宋体}5轴','\fontname{宋体}6轴','\fontname{宋体}7轴','\fontname{宋体}8轴'};


h1 = figure;
plot(Y(:,1), Y(:,2), 'LineWidth',1), hold on
plot(q_m(:,1), q_m(:,2), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型');
grid on
xlabel('\fontname{宋体}横向位置 \fontname{Times New Roman}(m)');
ylabel('\fontname{宋体}纵向位置 \fontname{Times New Roman}(m)');
set(gcf, 'Position', [400, 400, 800, 250]); 
axis([180 480 -5 65]);
print(h1,'-depsc2','-loose',[FigurePath,'traj_valid2.eps']); 
savefig(h1, [FigurePath,'/traj_valid2.fig']);

% h2 = figure;
% plot(T, vx_tm, 'k','LineWidth',1), hold on
% plot(T_m, vx_math, '--', 'LineWidth', 1, 'Color', [0.8, 0, 0]), hold on
% legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型');
% grid on
% xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
% xlabel('\fontname{宋体}纵向速度 \fontname{Times New Roman}(m/s)');
% set(gcf, 'Position', [400, 400, 800, 300]); 
% axis([-5 120 5.4 5.7]);
% print(h2,'-depsc2','-loose',[FigurePath,'vx_valid2.eps']); 
% savefig(h2, [FigurePath,'/vx_valid2.fig']);

h3 = figure;
plot(T, vy_tm, 'LineWidth',1), hold on
plot(T_m, vy_math, '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}仿真模型','\fontname{宋体}机理模型');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横向速度 \fontname{Times New Roman}(m/s)');
set(gcf, 'Position', [400, 400, 800, 300]); 
axis([0 130 -1.8 1.8]);
print(h3,'-depsc2','-loose',[FigurePath,'vy_valid2.eps']); 
savefig(h3, [FigurePath,'/vy_valid2.fig']);

set(0,'DefaultAxesFontsize',16);

h4 = figure;
plot(T_m, (mod(q_m(:, 3) + pi, 2*pi) - pi) * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(q_m(:, 4) + pi, 2*pi) - pi) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(q_m(:, 5) + pi, 2*pi) - pi) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, Y(:, 3) * deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 4) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 5) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:100:length(T)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横摆角 \fontname{Times New Roman}(deg)');
set(gcf, 'Position', [600, 400, 700, 400]); 
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 110 -240 320]);
print(h4,'-depsc2','-loose',[FigurePath,'yaw_valid2.eps']); 
savefig(h4, [FigurePath,'/yaw_valid2.fig']);

h5 = figure;
plot(T_m, (mod(qd_m(:, 3) + pi, 2*pi) - pi) * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(qd_m(:, 4) + pi, 2*pi) - pi) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(qd_m(:, 5) + pi, 2*pi) - pi) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, Y(:, 8) * deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 9) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 10) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:100:length(T)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横摆角速度 \fontname{Times New Roman}(deg/s)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 110 -15 20]);
print(h5,'-depsc2','-loose',[FigurePath,'yawrate_valid2.eps']); 
savefig(h5, [FigurePath,'/yawrate_valid2.fig']);

% h6 = figure;
% plot(T, steer(:, 1) * deg_trans, '-', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 2) * deg_trans, '--', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 3) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 4) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.8, 0.8, 0], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 5) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410.8], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 6) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980.8], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 7) * deg_trans, '-x', 'LineWidth', 1, 'Color', [0.2, 0.2, 0], 'MarkerIndices', 1:100:length(T)), hold on
% plot(T, steer(:, 8) * deg_trans, '-p', 'LineWidth', 1, 'Color', [0, 0.2, 0.2], 'MarkerIndices', 1:100:length(T)), hold on
% grid on
% xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
% ylabel('\fontname{宋体}转向角 \fontname{Times New Roman}(deg)');
% set(gcf, 'Position', [400, 400, 800, 300]); % 设置Figure窗口的位置和大小
% legend(axle_str, 'Location','northwest');
% lgd = legend(axle_str); % 获取图例句柄
% lgd.Orientation = 'horizontal'; % 设置图例水平排列
% lgd.Position = [0.24, 0.78, 0.8, 0.12]; % [左, 下, 宽, 高]
% lgd.NumColumns = 4; % 设置每行显示的列数
% axis([-5 120 -15 15]);
% print(h6,'-depsc2','-loose',[FigurePath,'steer_valid2.eps']); 
% savefig(h6, [FigurePath,'/steer_valid2.fig']);
% 
% h7 = figure;
% plot(T_m, steer_m(:, 1) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 2) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 3) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 4) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 5) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 6) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 7) * deg_trans, 'LineWidth',1), hold on
% plot(T_m, steer_m(:, 8) * deg_trans, 'LineWidth',1), hold on
% grid on
% xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
% ylabel('\fontname{宋体}转向角 \fontname{Times New Roman}(deg)');
% set(gcf, 'Position', [400, 400, 800, 300]); % 设置Figure窗口的位置和大小
% legend(axle_math_str, 'Location','EastOutside');
% axis([-5 120 -15 15]);
% print(h7,'-depsc2','-loose',[FigurePath,'steer_m_valid2.eps']); 
% savefig(h7, [FigurePath,'/steer_m_valid2.fig']);
