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

T = Expr_time(startpoint:endpoint, :) - Expr_time(startpoint);
Y = Expr_state(startpoint:endpoint, :);
U = Expr_steer(startpoint:endpoint, :);
Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));

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

vx_tm = Y(:, 6) .* cos(Y(:, 3)) + Y(:, 7) .* sin(Y(:, 3));
vy_tm = -Y(:, 6) .* sin(Y(:, 3)) + Y(:, 7) .* cos(Y(:, 3));
vx_math = XO(:, 6) .* cos(XO(:, 3)) + XO(:, 7) .* sin(XO(:, 3));
vy_math = -XO(:, 6) .* sin(XO(:, 3)) + XO(:, 7) .* cos(XO(:, 3));

%% plot

h1 = figure;
plot(Y(:,1), Y(:,2), 'k','LineWidth',1), hold on
plot(XO(:,1), XO(:,2), 'r','LineWidth',1), hold on
legend('IPG','Math');
grid on
xlabel('X / m');
ylabel('Y / m');
set(gcf, 'Position', [600, 400, 700, 300]); 
axis([200 320 -10 40]);
print(h1,'-depsc2','-loose',[FigurePath,'traj_valid3.eps']); 
savefig(h1, [FigurePath,'/traj_valid3.fig']);

h2 = figure;
plot(T, vx_tm, 'k','LineWidth',1), hold on
plot(T, vx_math, 'r','LineWidth',1), hold on
legend('IPG','Math');
grid on
xlabel('Time / s');
ylabel('Vx / (m/s)');
set(gcf, 'Position', [600, 400, 500, 250]); 
axis([0 35 -1 9]);
print(h2,'-depsc2','-loose',[FigurePath,'vx_valid3.eps']); 
savefig(h2, [FigurePath,'/vx_valid3.fig']);

h3 = figure;
plot(T, vy_tm, 'k','LineWidth',1), hold on
plot(T, vy_math, 'r','LineWidth',1), hold on
legend('IPG','Math');
grid on
xlabel('Time / s');
ylabel('Vy / (m/s)');
set(gcf, 'Position', [600, 400, 500, 250]); 
axis([0 35 -0.5 4]);
print(h3,'-depsc2','-loose',[FigurePath,'vy_valid3.eps']); 
savefig(h3, [FigurePath,'/vy_valid3.fig']);

h4 = figure;
plot(T, (mod(XO(:, 3) + pi, 2*pi) - pi) * deg_trans, sty{1},'LineWidth',1), hold on
plot(T, (mod(XO(:, 4) + pi, 2*pi) - pi) * deg_trans, sty{2},'LineWidth',1), hold on
plot(T, (mod(XO(:, 5) + pi, 2*pi) - pi) * deg_trans, sty{3},'LineWidth',1), hold on
plot(T, Y(:, 3) * deg_trans, sty{4},'LineWidth',1), hold on
plot(T, Y(:, 4) * deg_trans, sty{5},'LineWidth',1), hold on
plot(T, Y(:, 5) * deg_trans, sty{6},'LineWidth',1), hold on
grid on
xlabel('Time / s');
ylabel('Yaw / deg');
set(gcf, 'Position', [400, 400, 800, 300]); 
legend(unit_str, 'Location','EastOutside');
axis([0 35 -220 220]);
print(h4,'-depsc2','-loose',[FigurePath,'yaw_valid3.eps']); 
savefig(h4, [FigurePath,'/yaw_valid3.fig']);

h5 = figure;
plot(T, (mod(XO(:, 8) + pi, 2*pi) - pi) * deg_trans, sty{1},'LineWidth',1), hold on
plot(T, (mod(XO(:, 9) + pi, 2*pi) - pi) * deg_trans, sty{2},'LineWidth',1), hold on
plot(T, (mod(XO(:, 10) + pi, 2*pi) - pi) * deg_trans, sty{3},'LineWidth',1), hold on
plot(T, Y(:, 8) * deg_trans, sty{4},'LineWidth',1), hold on
plot(T, Y(:, 9) * deg_trans, sty{5},'LineWidth',1), hold on
plot(T, Y(:, 10) * deg_trans, sty{6},'LineWidth',1), hold on
grid on
xlabel('Time / s');
ylabel('Yaw rate / (deg/s)');
set(gcf, 'Position', [400, 400, 800, 300]); % 设置Figure窗口的位置和大小
legend(unit_str, 'Location','EastOutside');
axis([0 35 -10 35]);
print(h5,'-depsc2','-loose',[FigurePath,'yawrate_valid3.eps']); 
savefig(h5, [FigurePath,'/yawrate_valid3.fig']);

%% reload
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% load data

load('15m5mscircle.mat');
startpoint = 3621;
endpoint = length(Expr_time) - 5000;

T = Expr_time(startpoint:endpoint, :) - Expr_time(startpoint);
Y = Expr_state(startpoint:endpoint, :);
U = Expr_steer(startpoint:endpoint, :);
Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));

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

vx_tm = Y(:, 6) .* cos(Y(:, 3)) + Y(:, 7) .* sin(Y(:, 3));
vy_tm = -Y(:, 6) .* sin(Y(:, 3)) + Y(:, 7) .* cos(Y(:, 3));
vx_math = XO(:, 6) .* cos(XO(:, 3)) + XO(:, 7) .* sin(XO(:, 3));
vy_math = -XO(:, 6) .* sin(XO(:, 3)) + XO(:, 7) .* cos(XO(:, 3));

%% plot

h6 = figure;
plot(Y(:,1), Y(:,2), 'k','LineWidth',1), hold on
plot(XO(:,1), XO(:,2), 'r','LineWidth',1), hold on
legend('IPG','Math','Location','northwest');
grid on
xlabel('X / m');
ylabel('Y / m');
set(gcf, 'Position', [600, 400, 700, 230]); 
axis([150 320 -10 40]);
print(h6,'-depsc2','-loose',[FigurePath,'traj_valid3_zero.eps']); 
savefig(h6, [FigurePath,'/traj_valid3_zero.fig']);

h7 = figure;
plot(T, vx_tm, 'k','LineWidth',1), hold on
plot(T, vx_math, 'r','LineWidth',1), hold on
legend('IPG','Math');
grid on
xlabel('Time / s');
ylabel('Vx / (m/s)');
set(gcf, 'Position', [600, 400, 500, 250]); 
axis([0 50 -1 9]);
print(h7,'-depsc2','-loose',[FigurePath,'vx_valid3_zero.eps']); 
savefig(h7, [FigurePath,'/vx_valid3_zero.fig']);

h8 = figure;
plot(T, vy_tm, 'k','LineWidth',1), hold on
plot(T, vy_math, 'r','LineWidth',1), hold on
legend('IPG','Math');
grid on
xlabel('Time / s');
ylabel('Vy / (m/s)');
set(gcf, 'Position', [600, 400, 500, 250]); 
axis([0 50 -0.5 4]);
print(h8,'-depsc2','-loose',[FigurePath,'vy_valid3_zero.eps']); 
savefig(h8, [FigurePath,'/vy_valid3_zero.fig']);

