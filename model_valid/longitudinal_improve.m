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
unit_str={'Unit 1 (Math)','Unit 2 (Math)','Unit 3 (Math)', 'Unit 1 (IPG)', 'Unit 2 (IPG)', 'Unit 3 (IPG)'};

%% load data

load('15m5mscircle.mat');
startpoint = 15001;
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
        Maaav_roll(XO(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :));
    XO(i, 6:10) = XO(i-1, 6:10) + qdd * dt;
    XO(i, 1:5) = XO(i-1, 1:5) + XO(i-1, 6:10) * dt;
end
FxL_m(1, :) = FxL_m(2, :);
FxR_m(1, :) = FxR_m(2, :);
slipAngleL_m(1, :) = slipAngleL_m(2, :);
slipAngleL_m(1, :) = slipAngleL_m(2, :);
FyL_m(1, :) = FyL_m(2, :);
FyR_m(1, :) = FyR_m(2, :);
FzL_m(1, :) = FzL_m(2, :);
FzR_m(1, :) = FzR_m(2, :);

XO_basic = zeros(length(T), 10);
FxL_basic_m = zeros(length(T), 8);
FxR_basic_m = zeros(length(T), 8);
XO_basic(1, :) = Y(1, :);
for i = 2:length(T)
    dt = T(i) - T(i-1);
    [qdd, FxL_basic_m(i, :), FxR_basic_m(i, :), ~, ~, ~, ~] = Maaav_analy(XO_basic(i - 1, :), avp, U(i - 1, :), Trq(i - 1, :));
    XO_basic(i, 6:10) = XO_basic(i-1, 6:10) + qdd * dt;
    XO_basic(i, 1:5) = XO_basic(i-1, 1:5) + XO_basic(i-1, 6:10) * dt;
end
FxL_basic_m(1, :) = FxL_basic_m(2, :);
FxR_basic_m(1, :) = FxR_basic_m(2, :);

%% plot

FxL_true = FxL(1:10:30000, :);
FxL_basic = FxL_basic_m(1:10:30000, :);
FxL_improve = FxL_m(1:10:30000, :);
time = T(1:10:30000, :);
FxR_true = FxR(1:10:30000, :);
FxR_basic = FxR_basic_m(1:10:30000, :);
FxR_improve = FxR_m(1:10:30000, :);

h1 = figure;
plot(time, (FxL_true(:, 2) - FxL_improve(:, 2)), 'LineWidth', 1), hold on
plot(time, (FxL_true(:, 2) - FxL_basic(:, 2)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -300 250]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h1,'-depsc2','-loose',[FigurePath,'secondaxle_left_longf_error.eps']); 
savefig(h1, [FigurePath,'/secondaxle_left_longf_error.fig']);

h2 = figure;
plot(time, (FxR_true(:, 2) - FxR_improve(:, 2)), 'LineWidth', 1), hold on
plot(time, (FxR_true(:, 2) - FxR_basic(:, 2)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -400 250]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h2,'-depsc2','-loose',[FigurePath,'secondaxle_right_longf_error.eps']); 
savefig(h2, [FigurePath,'/secondaxle_right_longf_error.fig']);

h3 = figure;
plot(time, (FxL_true(:, 4) - FxL_improve(:, 4)), 'LineWidth', 1), hold on
plot(time, (FxL_true(:, 4) - FxL_basic(:, 4)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -300 200]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h3,'-depsc2','-loose',[FigurePath,'fourthaxle_left_longf_error.eps']); 
savefig(h3, [FigurePath,'/fourthaxle_left_longf_error.fig']);

h4 = figure;
plot(time, (FxR_true(:, 4) - FxR_improve(:, 4)), 'LineWidth', 1), hold on
plot(time, (FxR_true(:, 4) - FxR_basic(:, 4)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -400 200]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h4,'-depsc2','-loose',[FigurePath,'fourthaxle_right_longf_error.eps']); 
savefig(h4, [FigurePath,'/fourthaxle_right_longf_error.fig']);

h5 = figure;
plot(time, (FxL_true(:, 6) - FxL_improve(:, 6)), 'LineWidth', 1), hold on
plot(time, (FxL_true(:, 6) - FxL_basic(:, 6)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -400 200]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h5,'-depsc2','-loose',[FigurePath,'sixthaxle_left_longf_error.eps']); 
savefig(h5, [FigurePath,'/sixthaxle_left_longf_error.fig']);

h6 = figure;
plot(time, (FxR_true(:, 6) - FxR_improve(:, 6)), 'LineWidth', 1), hold on
plot(time, (FxR_true(:, 6) - FxR_basic(:, 6)), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}改进模型','\fontname{宋体}初始模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎纵向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -400 250]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h6,'-depsc2','-loose',[FigurePath,'sixthaxle_right_longf_error.eps']); 
savefig(h6, [FigurePath,'/sixthaxle_right_longf_error.fig']);
