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

load('valid1_sine3_3s.mat');
startpoint = 13001;
endpoint = startpoint - 1 + 30000;

T = Expr_time(startpoint:10:endpoint, :) - Expr_time(startpoint-1);
Y = Expr_state(startpoint:10:endpoint, :);
U = Expr_steer(startpoint:10:endpoint, :);
Trq = (Expr_drive(startpoint:10:endpoint, :) + Expr_brake(startpoint:10:endpoint, :));

%% iter predict

XO = zeros(length(T), 10);
XO(1, :) = Y(1, :);
for i = 2:length(T)
    dt = T(i) - T(i-1);
    qdd = Maaav_trq(XO(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :));
    XO(i, 6:10) = XO(i-1, 6:10) + qdd * dt;
    XO(i, 1:5) = XO(i-1, 1:5) + XO(i-1, 6:10) * dt;
end

%% data process

YawRate1_math = XO(:, 8);
YawRate2_math = XO(:, 9);
YawRate3_math = XO(:, 10);
YawRate1_tm = Y(:, 8);
YawRate2_tm = Y(:, 9);
YawRate3_tm = Y(:, 10);

vxg_1_math = XO(:, 6) + avp.lg(1) .* sin(XO(:, 3)) .* XO(:, 8);
vyg_1_math = XO(:, 7) - avp.lg(1) .* cos(XO(:, 3)) .* XO(:, 8);
LateralVelocity1_math = -vxg_1_math .* sin(XO(:, 3)) + vyg_1_math .* cos(XO(:, 3));
vxg_2_math = XO(:, 6) + avp.lj(1) .* sin(XO(:, 3)) .* XO(:, 8) + avp.lg(2) .* sin(XO(:, 4)) .* XO(:, 9);
vyg_2_math = XO(:, 7) - avp.lj(1) .* cos(XO(:, 3)) .* XO(:, 8) - avp.lg(2) .* cos(XO(:, 4)) .* XO(:, 9);
LateralVelocity2_math = -vxg_2_math .* sin(XO(:, 4)) + vyg_2_math .* cos(XO(:, 4));
vxg_3_math = XO(:, 6) + avp.lj(1) .* sin(XO(:, 3)) .* XO(:, 8) + avp.lj(2) .* sin(XO(:, 4)) .* XO(:, 9) + avp.lg(3) .* sin(XO(:, 5)) .* XO(:, 10);
vyg_3_math = XO(:, 7) - avp.lj(1) .* cos(XO(:, 3)) .* XO(:, 8) - avp.lj(2) .* cos(XO(:, 4)) .* XO(:, 9) - avp.lg(3) .* cos(XO(:, 5)) .* XO(:, 10);
LateralVelocity3_math = -vxg_3_math .* sin(XO(:, 5)) + vyg_3_math .* cos(XO(:, 5));
LatAcc1_math = (LateralVelocity1_math(2:end) - LateralVelocity1_math(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc1_math = [LatAcc1_math; LatAcc1_math(end)];
LatAcc2_math = (LateralVelocity2_math(2:end) - LateralVelocity2_math(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc2_math = [LatAcc2_math; LatAcc2_math(end)];
LatAcc3_math = (LateralVelocity3_math(2:end) - LateralVelocity3_math(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc3_math = [LatAcc3_math; LatAcc3_math(end)];

vxg_1_tm = Y(:, 6) + avp.lg(1) .* sin(Y(:, 3)) .* Y(:, 8);
vyg_1_tm = Y(:, 7) - avp.lg(1) .* cos(Y(:, 3)) .* Y(:, 8);
LateralVelocity1_tm = -vxg_1_tm .* sin(Y(:, 3)) + vyg_1_tm .* cos(Y(:, 3));
vxg_2_tm = Y(:, 6) + avp.lj(1) .* sin(Y(:, 3)) .* Y(:, 8) + avp.lg(2) .* sin(Y(:, 4)) .* Y(:, 9);
vyg_2_tm = Y(:, 7) - avp.lj(1) .* cos(Y(:, 3)) .* Y(:, 8) - avp.lg(2) .* cos(Y(:, 4)) .* Y(:, 9);
LateralVelocity2_tm = -vxg_2_tm .* sin(Y(:, 4)) + vyg_2_tm .* cos(Y(:, 4));
vxg_3_tm = Y(:, 6) + avp.lj(1) .* sin(Y(:, 3)) .* Y(:, 8) + avp.lj(2) .* sin(Y(:, 4)) .* Y(:, 9) + avp.lg(3) .* sin(Y(:, 5)) .* Y(:, 10);
vyg_3_tm = Y(:, 7) - avp.lj(1) .* cos(Y(:, 3)) .* Y(:, 8) - avp.lj(2) .* cos(Y(:, 4)) .* Y(:, 9) - avp.lg(3) .* cos(Y(:, 5)) .* Y(:, 10);
LateralVelocity3_tm = -vxg_3_tm .* sin(Y(:, 5)) + vyg_3_tm .* cos(Y(:, 5));
LatAcc1_tm = (LateralVelocity1_tm(2:end) - LateralVelocity1_tm(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc1_tm = [LatAcc1_tm; LatAcc1_tm(end)];
LatAcc2_tm = (LateralVelocity2_tm(2:end) - LateralVelocity2_tm(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc2_tm = [LatAcc2_tm; LatAcc2_tm(end)];
LatAcc3_tm = (LateralVelocity3_tm(2:end) - LateralVelocity3_tm(1:end-1)) ./ (T(2:end)- T(1:end-1));
LatAcc3_tm = [LatAcc3_tm; LatAcc3_tm(end)];

%% plot

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
% set(0,'DefaultAxesFontname','宋体');
set(0,'DefaultAxesFontsize',16);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

sty={'r-','g-','b-','r--','g--','b--'};
axle_str={'Axle 1','Axle 2','Axle 3','Axle 4','Axle 5','Axle 6','Axle 7','Axle 8'};
unit_str={'Mechanism  U1','Mechanism  U2','Mechanism  U3', ...
          'Simulation U1','Simulation U2','Simulation U3'};

% unit_str={'\fontname{宋体}头车（机理）','\fontname{宋体}头车（仿真）', '\fontname{宋体}中间车（机理）', ...
%           '\fontname{宋体}中间车（仿真）','\fontname{宋体}尾车（机理）','\fontname{宋体}尾车（仿真）'};

% yaw rates
h1 = figure;
plot(T, YawRate1_math * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, YawRate2_math * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, YawRate3_math * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, YawRate1_tm *   deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, YawRate2_tm *   deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, YawRate3_tm *   deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
grid on
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 15 -3 4]);
saveas(gcf, [FigurePath,'simu11.png']);
% print(h1,'-depsc2','-loose',[FigurePath,'simu11.png']); 
% savefig(h1, [FigurePath,'/yawrate_ms_sine33.fig']);
    
% lateral velocity
h2 = figure;
plot(T, LatAcc1_math, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc2_math, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc3_math, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc1_tm  , '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc2_tm  , '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc3_tm  , '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
grid on
xlabel('Time (s)');
ylabel('Lateral Acc (m/s^2)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 15 -0.45 0.65]);
saveas(gcf, [FigurePath,'simu12.png']);
% print(h2,'-depsc2','-loose',[FigurePath,'simu12.emf']); 
% savefig(h2, [FigurePath,'/latacc_ms_sine33.fig']);

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

h3 = figure;
plot(Y(:,1), Y(:,2), 'LineWidth',1), hold on
plot(q_m(:,1), q_m(:,2), '--', 'LineWidth', 1), hold on
legend('Simulation','Mechanism');
grid on
xlabel('Position X (m)');
ylabel('Position Y (m)');
set(gcf, 'Position', [600, 400, 700, 400]); 
axis([200 480 -30 90]);
saveas(gcf, [FigurePath,'simu21.png']);
% print(h1,'-depsc2','-loose',[FigurePath,'traj_valid2.eps']); 
% savefig(h1, [FigurePath,'/traj_valid2.fig']);

h4 = figure;
plot(T_m, (mod(q_m(:, 3) + pi, 2*pi) - pi) * deg_trans, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(q_m(:, 4) + pi, 2*pi) - pi) * deg_trans, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:300:length(T)), hold on
plot(T_m, (mod(q_m(:, 5) + pi, 2*pi) - pi) * deg_trans, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:300:length(T)), hold on
plot(T, Y(:, 3) * deg_trans, '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 4) * deg_trans, '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:100:length(T)), hold on
plot(T, Y(:, 5) * deg_trans, '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:100:length(T)), hold on
grid on
xlabel('Time (s)');
ylabel('Yaw (deg)');
set(gcf, 'Position', [600, 400, 700, 400]); 
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
axis([0 110 -240 320]);
saveas(gcf, [FigurePath,'simu22.png']);
% print(h4,'-depsc2','-loose',[FigurePath,'yaw_valid2.eps']); 
% savefig(h4, [FigurePath,'/yaw_valid2.fig']);