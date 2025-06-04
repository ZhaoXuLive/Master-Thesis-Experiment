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

%% load data 2789

for index = 9:9
    mode = index;

if mode == 1
    load('valid1_step1.mat');
    startpoint = 15001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 2
    load('valid1_step3.mat');
    startpoint = 15001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 3
    load('valid1_step5.mat');
    startpoint = 15001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 4
    load('valid1_sine1_1s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 5
    load('valid1_sine1_2s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 6
    load('valid1_sine1_3s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 7
    load('valid1_sine3_1s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 8
    load('valid1_sine3_2s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 9
    load('valid1_sine3_3s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 10
    load('valid1_sine5_1s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 11
    load('valid1_sine5_2s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end
if mode == 12
    load('valid1_sine5_3s.mat');
    startpoint = 13001;
    endpoint = startpoint - 1 + 30000;
end

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
unit_str={'\fontname{宋体}头车（机理）','\fontname{宋体}中间车（机理）','\fontname{宋体}尾车（机理）', ...
          '\fontname{宋体}头车（仿真）','\fontname{宋体}中间车（仿真）','\fontname{宋体}尾车（仿真）'};

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
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横摆角速度 \fontname{Times New Roman}(deg/s)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数

if mode == 1
    axis([0 20 -1.2 2.4]);
    set(gcf, 'PaperPositionMode', 'auto');
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_step1.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_step1.fig']);
end
if mode == 2
    axis([0 20 -2.5 8]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_step3.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_step3.fig']);
end
if mode == 3
    axis([0 20 -8 13]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_step5.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_step5.fig']);
end
if mode == 4
    axis([0 7 -1 1]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine11.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine11.fig']);
end
if mode == 5
    axis([0 10 -1.2 1.2]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine12.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine12.fig']);
end
if mode == 6
    axis([0 15 -1.2 1.2]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine13.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine13.fig']);
end
if mode == 7
    axis([0 7 -2.3 3.3]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine31.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine31.fig']);
end
if mode == 8
    axis([0 10 -2.7 3.5]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine32.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine32.fig']);
end
if mode == 9
    axis([0 15 -3 4]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine33.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine33.fig']);
end
if mode == 10
    axis([0 7 -5.4 5.4]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine51.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine51.fig']);
end
if mode == 11
    axis([0 10 -6 6]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine51.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine51.fig']);
end
if mode == 12
    axis([0 15 -6.6 6.6]);
    print(h1,'-depsc2','-loose',[FigurePath,'yawrate_ms_sine51.eps']); 
    savefig(h1, [FigurePath,'/yawrate_ms_sine51.fig']);
end

% lateral velocity
h2 = figure;
plot(T, LatAcc1_math, '-',  'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc2_math, '-+', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc3_math, '-*', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc1_tm  , '--', 'LineWidth', 1, 'Color', [0, 0.4470, 0.7410], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc2_tm  , '-^', 'LineWidth', 1, 'Color', [0.8500, 0.3250, 0.0980], 'MarkerIndices', 1:50:length(T)), hold on
plot(T, LatAcc3_tm  , '-.', 'LineWidth', 1, 'Color', [0.4660, 0.6740, 0.1880], 'MarkerIndices', 1:50:length(T)), hold on
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}横向加速度 \fontname{Times New Roman}(m/s^2)');
set(gcf, 'Position', [600, 400, 700, 400]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
if mode == 1
    axis([0 20 -0.06 0.06]);
%     set(gcf, 'PaperPositionMode', 'auto');
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_step1.eps']); 
%     print(h2,'-dpdf',[FigurePath,'latacc_ms_step1.pdf']); 
    savefig(h2, [FigurePath,'/latacc_ms_step1.fig']);
end
if mode == 2
    axis([0 20 -0.12 0.2]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_step3.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_step3.fig']);
end
if mode == 3
    axis([0 20 -0.28 0.28]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_step5.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_step5.fig']);
end
if mode == 4
    axis([0 7 -0.5 0.5]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine11.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine11.fig']);
end
if mode == 5
    axis([0 10 -0.3 0.3]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine12.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine12.fig']);
end
if mode == 6
    axis([0 15 -0.25 0.25]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine13.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine13.fig']);
end
if mode == 7
    axis([0 7 -1.2 1.7]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine31.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine31.fig']);
end
if mode == 8
    axis([0 10 -0.6 0.9]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine32.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine32.fig']);
end
if mode == 9
    axis([0 15 -0.45 0.65]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine33.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine33.fig']);
end
if mode == 10
    axis([0 7 -2.5 2.5]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine51.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine51.fig']);
end
if mode == 11
    axis([0 10 -1.6 1.6]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine52.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine52.fig']);
end
if mode == 12
    axis([0 15 -1 1]);
    print(h2,'-depsc2','-loose',[FigurePath,'latacc_ms_sine53.eps']); 
    savefig(h2, [FigurePath,'/latacc_ms_sine53.fig']);
end

end