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
avp.sat = 15;
avp.hc = [1.855; 1.719; 1.855];
avp.ca = 184000;

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

% load('15m5mscircle.mat');
% startpoint = 5001;
% endpoint = length(Expr_time) - 5000;
% Y = Expr_state(startpoint:endpoint, :);
% U = Expr_steer(startpoint:endpoint, :);
% Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));
% FyL = Expr_FyL(startpoint:endpoint, :);
% FyR = Expr_FyR(startpoint:endpoint, :);
% FzL = Expr_FzL(startpoint:endpoint, :);
% FzR = Expr_FzR(startpoint:endpoint, :);
% slipAngleL = Expr_LatSlipL(startpoint:endpoint, :);
% slipAngleR = Expr_LatSlipR(startpoint:endpoint, :);

% load('15m3mscircle.mat');
% startpoint = 5001;
% endpoint = length(Expr_time) - 5000;
% Y = Expr_state(startpoint:endpoint, :);
% U = Expr_steer(startpoint:endpoint, :);
% Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));
% FyL = Expr_FyL(startpoint:endpoint, :);
% FyR = Expr_FyR(startpoint:endpoint, :);
% FzL = Expr_FzL(startpoint:endpoint, :);
% FzR = Expr_FzR(startpoint:endpoint, :);
% slipAngleL = Expr_LatSlipL(startpoint:endpoint, :);
% slipAngleR = Expr_LatSlipR(startpoint:endpoint, :);

load('15m7mscircle.mat');
startpoint = 5001;
endpoint = length(Expr_time) - 5000;
Y = Expr_state(startpoint:endpoint, :);
U = Expr_steer(startpoint:endpoint, :);
T = Expr_time(startpoint:endpoint, :) - Expr_time(startpoint-1);
Trq = (Expr_drive(startpoint:endpoint, :) + Expr_brake(startpoint:endpoint, :));
FyL = Expr_FyL(startpoint:endpoint, :);
FyR = Expr_FyR(startpoint:endpoint, :);
FzL = Expr_FzL(startpoint:endpoint, :);
FzR = Expr_FzR(startpoint:endpoint, :);
slipAngleL = Expr_LatSlipL(startpoint:endpoint, :);
slipAngleR = Expr_LatSlipR(startpoint:endpoint, :);

%% 各轮胎横纵向速度

vx_j1 = Y(:, 6);
vy_j1 = Y(:, 7);
vx_j2 = vx_j1 + avp.lj(1) * sin(Y(:, 3)) .* Y(:, 8);
vy_j2 = vy_j1 - avp.lj(1) * cos(Y(:, 3)) .* Y(:, 8);
vx_j3 = vx_j2 + avp.lj(2) * sin(Y(:, 4)) .* Y(:, 9);
vy_j3 = vy_j2 - avp.lj(2) * cos(Y(:, 4)) .* Y(:, 9);
vxL_a1 = vx_j1 + avp.la(1) * sin(Y(:, 3)) .* Y(:, 8) - avp.h(1) * cos(Y(:, 3)) .* Y(:, 8);
vyL_a1 = vy_j1 - avp.la(1) * cos(Y(:, 3)) .* Y(:, 8) - avp.h(1) * sin(Y(:, 3)) .* Y(:, 8);
vxL_a1_t = vxL_a1 .* cos(U(:, 1) + Y(:, 3)) + vyL_a1 .* sin(U(:, 1) + Y(:, 3));
vyL_a1_t = -vxL_a1 .* sin(U(:, 1) + Y(:, 3)) + vyL_a1 .* cos(U(:, 1) + Y(:, 3));
vxL_a2 = vx_j1 + avp.la(2) * sin(Y(:, 3)) .* Y(:, 8) - avp.h(2) * cos(Y(:, 3)) .* Y(:, 8);
vyL_a2 = vy_j1 - avp.la(2) * cos(Y(:, 3)) .* Y(:, 8) - avp.h(2) * sin(Y(:, 3)) .* Y(:, 8);
vxL_a2_t = vxL_a2 .* cos(U(:, 2) + Y(:, 3)) + vyL_a2 .* sin(U(:, 2) + Y(:, 3));
vyL_a2_t = -vxL_a2 .* sin(U(:, 2) + Y(:, 3)) + vyL_a2 .* cos(U(:, 2) + Y(:, 3));
vxL_a3 = vx_j1 + avp.la(3) * sin(Y(:, 3)) .* Y(:, 8) - avp.h(3) * cos(Y(:, 3)) .* Y(:, 8);
vyL_a3 = vy_j1 - avp.la(3) * cos(Y(:, 3)) .* Y(:, 8) - avp.h(3) * sin(Y(:, 3)) .* Y(:, 8);
vxL_a3_t = vxL_a3 .* cos(U(:, 3) + Y(:, 3)) + vyL_a3 .* sin(U(:, 3) + Y(:, 3));
vyL_a3_t = -vxL_a3 .* sin(U(:, 3) + Y(:, 3)) + vyL_a3 .* cos(U(:, 3) + Y(:, 3));
vxL_a4 = vx_j2 + avp.la(4) * sin(Y(:, 4)) .* Y(:, 9) - avp.h(4) * cos(Y(:, 4)) .* Y(:, 9);
vyL_a4 = vy_j2 - avp.la(4) * cos(Y(:, 4)) .* Y(:, 9) - avp.h(4) * sin(Y(:, 4)) .* Y(:, 9);
vxL_a4_t = vxL_a4 .* cos(U(:, 4) + Y(:, 4)) + vyL_a4 .* sin(U(:, 4) + Y(:, 4));
vyL_a4_t = -vxL_a4 .* sin(U(:, 4) + Y(:, 4)) + vyL_a4 .* cos(U(:, 4) + Y(:, 4));
vxL_a5 = vx_j2 + avp.la(5) * sin(Y(:, 4)) .* Y(:, 9) - avp.h(5) * cos(Y(:, 4)) .* Y(:, 9);
vyL_a5 = vy_j2 - avp.la(5) * cos(Y(:, 4)) .* Y(:, 9) - avp.h(5) * sin(Y(:, 4)) .* Y(:, 9);
vxL_a5_t = vxL_a5 .* cos(U(:, 5) + Y(:, 4)) + vyL_a5 .* sin(U(:, 5) + Y(:, 4));
vyL_a5_t = -vxL_a5 .* sin(U(:, 5) + Y(:, 4)) + vyL_a5 .* cos(U(:, 5) + Y(:, 4));
vxL_a6 = vx_j3 + avp.la(6) * sin(Y(:, 5)) .* Y(:, 10) - avp.h(6) * cos(Y(:, 5)) .* Y(:, 10);
vyL_a6 = vy_j3 - avp.la(6) * cos(Y(:, 5)) .* Y(:, 10) - avp.h(6) * sin(Y(:, 5)) .* Y(:, 10);
vxL_a6_t = vxL_a6 .* cos(U(:, 6) + Y(:, 5)) + vyL_a6 .* sin(U(:, 6) + Y(:, 5));
vyL_a6_t = -vxL_a6 .* sin(U(:, 6) + Y(:, 5)) + vyL_a6 .* cos(U(:, 6) + Y(:, 5));
vxL_a7 = vx_j3 + avp.la(7) * sin(Y(:, 5)) .* Y(:, 10) - avp.h(7) * cos(Y(:, 5)) .* Y(:, 10);
vyL_a7 = vy_j3 - avp.la(7) * cos(Y(:, 5)) .* Y(:, 10) - avp.h(7) * sin(Y(:, 5)) .* Y(:, 10);
vxL_a7_t = vxL_a7 .* cos(U(:, 7) + Y(:, 5)) + vyL_a7 .* sin(U(:, 7) + Y(:, 5));
vyL_a7_t = -vxL_a7 .* sin(U(:, 7) + Y(:, 5)) + vyL_a7 .* cos(U(:, 7) + Y(:, 5));
vxL_a8 = vx_j3 + avp.la(8) * sin(Y(:, 5)) .* Y(:, 10) - avp.h(8) * cos(Y(:, 5)) .* Y(:, 10);
vyL_a8 = vy_j3 - avp.la(8) * cos(Y(:, 5)) .* Y(:, 10) - avp.h(8) * sin(Y(:, 5)) .* Y(:, 10);
vxL_a8_t = vxL_a8 .* cos(U(:, 8) + Y(:, 5)) + vyL_a8 .* sin(U(:, 8) + Y(:, 5));
vyL_a8_t = -vxL_a8 .* sin(U(:, 8) + Y(:, 5)) + vyL_a8 .* cos(U(:, 8) + Y(:, 5));

vxR_a1 = vx_j1 + avp.la(1) * sin(Y(:, 3)) .* Y(:, 8) + avp.h(1) * cos(Y(:, 3)) .* Y(:, 8);
vyR_a1 = vy_j1 - avp.la(1) * cos(Y(:, 3)) .* Y(:, 8) + avp.h(1) * sin(Y(:, 3)) .* Y(:, 8);
vxR_a1_t = vxR_a1 .* cos(U(:, 1) + Y(:, 3)) + vyR_a1 .* sin(U(:, 1) + Y(:, 3));
vyR_a1_t = -vxR_a1 .* sin(U(:, 1) + Y(:, 3)) + vyR_a1 .* cos(U(:, 1) + Y(:, 3));
vxR_a2 = vx_j1 + avp.la(2) * sin(Y(:, 3)) .* Y(:, 8) + avp.h(2) * cos(Y(:, 3)) .* Y(:, 8);
vyR_a2 = vy_j1 - avp.la(2) * cos(Y(:, 3)) .* Y(:, 8) + avp.h(2) * sin(Y(:, 3)) .* Y(:, 8);
vxR_a2_t = vxR_a2 .* cos(U(:, 2) + Y(:, 3)) + vyR_a2 .* sin(U(:, 2) + Y(:, 3));
vyR_a2_t = -vxR_a2 .* sin(U(:, 2) + Y(:, 3)) + vyR_a2 .* cos(U(:, 2) + Y(:, 3));
vxR_a3 = vx_j1 + avp.la(3) * sin(Y(:, 3)) .* Y(:, 8) + avp.h(3) * cos(Y(:, 3)) .* Y(:, 8);
vyR_a3 = vy_j1 - avp.la(3) * cos(Y(:, 3)) .* Y(:, 8) + avp.h(3) * sin(Y(:, 3)) .* Y(:, 8);
vxR_a3_t = vxR_a3 .* cos(U(:, 3) + Y(:, 3)) + vyR_a3 .* sin(U(:, 3) + Y(:, 3));
vyR_a3_t = -vxR_a3 .* sin(U(:, 3) + Y(:, 3)) + vyR_a3 .* cos(U(:, 3) + Y(:, 3));
vxR_a4 = vx_j2 + avp.la(4) * sin(Y(:, 4)) .* Y(:, 9) + avp.h(4) * cos(Y(:, 4)) .* Y(:, 9);
vyR_a4 = vy_j2 - avp.la(4) * cos(Y(:, 4)) .* Y(:, 9) + avp.h(4) * sin(Y(:, 4)) .* Y(:, 9);
vxR_a4_t = vxR_a4 .* cos(U(:, 4) + Y(:, 4)) + vyR_a4 .* sin(U(:, 4) + Y(:, 4));
vyR_a4_t = -vxR_a4 .* sin(U(:, 4) + Y(:, 4)) + vyR_a4 .* cos(U(:, 4) + Y(:, 4));
vxR_a5 = vx_j2 + avp.la(5) * sin(Y(:, 4)) .* Y(:, 9) + avp.h(5) * cos(Y(:, 4)) .* Y(:, 9);
vyR_a5 = vy_j2 - avp.la(5) * cos(Y(:, 4)) .* Y(:, 9) + avp.h(5) * sin(Y(:, 4)) .* Y(:, 9);
vxR_a5_t = vxR_a5 .* cos(U(:, 5) + Y(:, 4)) + vyR_a5 .* sin(U(:, 5) + Y(:, 4));
vyR_a5_t = -vxR_a5 .* sin(U(:, 5) + Y(:, 4)) + vyR_a5 .* cos(U(:, 5) + Y(:, 4));
vxR_a6 = vx_j3 + avp.la(6) * sin(Y(:, 5)) .* Y(:, 10) + avp.h(6) * cos(Y(:, 5)) .* Y(:, 10);
vyR_a6 = vy_j3 - avp.la(6) * cos(Y(:, 5)) .* Y(:, 10) + avp.h(6) * sin(Y(:, 5)) .* Y(:, 10);
vxR_a6_t = vxR_a6 .* cos(U(:, 6) + Y(:, 5)) + vyR_a6 .* sin(U(:, 6) + Y(:, 5));
vyR_a6_t = -vxR_a6 .* sin(U(:, 6) + Y(:, 5)) + vyR_a6 .* cos(U(:, 6) + Y(:, 5));
vxR_a7 = vx_j3 + avp.la(7) * sin(Y(:, 5)) .* Y(:, 10) + avp.h(7) * cos(Y(:, 5)) .* Y(:, 10);
vyR_a7 = vy_j3 - avp.la(7) * cos(Y(:, 5)) .* Y(:, 10) + avp.h(7) * sin(Y(:, 5)) .* Y(:, 10);
vxR_a7_t = vxR_a7 .* cos(U(:, 7) + Y(:, 5)) + vyR_a7 .* sin(U(:, 7) + Y(:, 5));
vyR_a7_t = -vxR_a7 .* sin(U(:, 7) + Y(:, 5)) + vyR_a7 .* cos(U(:, 7) + Y(:, 5));
vxR_a8 = vx_j3 + avp.la(8) * sin(Y(:, 5)) .* Y(:, 10) + avp.h(8) * cos(Y(:, 5)) .* Y(:, 10);
vyR_a8 = vy_j3 - avp.la(8) * cos(Y(:, 5)) .* Y(:, 10) + avp.h(8) * sin(Y(:, 5)) .* Y(:, 10);
vxR_a8_t = vxR_a8 .* cos(U(:, 8) + Y(:, 5)) + vyR_a8 .* sin(U(:, 8) + Y(:, 5));
vyR_a8_t = -vxR_a8 .* sin(U(:, 8) + Y(:, 5)) + vyR_a8 .* cos(U(:, 8) + Y(:, 5));

alpha1L = atan2(vyL_a1_t, vxL_a1_t);
alpha2L = atan2(vyL_a2_t, vxL_a2_t);
alpha3L = atan2(vyL_a3_t, vxL_a3_t);
alpha4L = atan2(vyL_a4_t, vxL_a4_t);
alpha5L = atan2(vyL_a5_t, vxL_a5_t);
alpha6L = atan2(vyL_a6_t, vxL_a6_t);
alpha7L = atan2(vyL_a7_t, vxL_a7_t);
alpha8L = atan2(vyL_a8_t, vxL_a8_t);
alpha1R = atan2(vyR_a1_t, vxR_a1_t);
alpha2R = atan2(vyR_a2_t, vxR_a2_t);
alpha3R = atan2(vyR_a3_t, vxR_a3_t);
alpha4R = atan2(vyR_a4_t, vxR_a4_t);
alpha5R = atan2(vyR_a5_t, vxR_a5_t);
alpha6R = atan2(vyR_a6_t, vxR_a6_t);
alpha7R = atan2(vyR_a7_t, vxR_a7_t);
alpha8R = atan2(vyR_a8_t, vxR_a8_t);

alphaL = [alpha1L alpha2L alpha3L alpha4L alpha5L alpha6L alpha7L alpha8L];
alphaR = [alpha1R alpha2R alpha3R alpha4R alpha5R alpha6R alpha7R alpha8R];

sat = avp.sat;
for i = 1:8
    if abs(alphaL(i)) > sat * pi / 180
        alphaL(i) = sign(alphaL(i)) * sat * pi / 180;
    end
    if abs(alphaR(i)) > sat * pi / 180
        alphaR(i) = sign(alphaR(i)) * sat * pi / 180;
    end
end

slipAngleL_m = alphaL;
slipAngleR_m = alphaR;

%% lateral

% 定义魔术公式 D*sin(C*arctan(B*(x+Sx)-E*(B*(x+Sx)-arctan(B*(x+Sx)))))+Sv
magic_formula = @(params, xdata) params(3) * xdata(:, 2) .* sin( params(2) * ...
    atan( params(1) * (xdata(:, 1) + params(5)) - params(4) * (params(1) * ...
    (xdata(:, 1) + params(5)) - atan(params(1) * (xdata(:, 1) + params(5)))))) + params(6);
options = optimoptions('lsqcurvefit', ...
    'TolFun', 1e-6, ...                                % 函数容忍度
    'TolX', 1e-6, ...                                  % 参数变化容忍度
    'OptimalityTolerance', 1e-6, ...                    % 参数变化容忍度
    'MaxIterations', 20000, ...                          % 最大迭代次数
    'MaxFunctionEvaluations', 100000, ...                 % 最大函数调用次数
    'Algorithm', 'trust-region-reflective');            % 使用信任域反射算法，默认，支持边界约束

FyL = - FyL;
FyR = - FyR;
optimal_params_latL = zeros(8, 6);
optimal_params_latR = zeros(8, 6);
FyL_fit = zeros(length(Y), 8);
FyR_fit = zeros(length(Y), 8);

for i = 1:8
    % 初始参数 [B, C, D, E, Sx, Sv]
    initial_params = [15, 1.3, 1, -1, 0, 0]; 
    optimal_params = lsqcurvefit(magic_formula, initial_params, [slipAngleL(:, i) FzL(:, i)], FyL(:, i), ...
        [0, 0, 0.2, -5, -1, -300], [30, 3.5, 2.5, 0, 1, 300], options);
    % 计算拟合曲线
    Fy_fitafter = magic_formula(optimal_params, [slipAngleL(:, i) FzL(:, i)]);
    FyL_fit(:, i) = Fy_fitafter;
    optimal_params_latL(i, :) = optimal_params;
    fprintf('B: %2f, C: %2f, D: %2f, E: %2f, Sx: %4f, Sv: %2f\n', ...
           optimal_params(1), optimal_params(2), optimal_params(3), ...
           optimal_params(4), optimal_params(5), optimal_params(6));

    % 初始参数 [B, C, D, E, Sx, Sv]
    initial_params = [15, 1.3, 1, -1, 0, 0]; 
    optimal_params = lsqcurvefit(magic_formula, initial_params, [slipAngleR(:, i) FzR(:, i)], FyR(:, i), ...
        [0, 0, 0.2, -5, -1, -300], [30, 3.5, 2.5, 0, 1, 300], options);
    % 计算拟合曲线
    Fy_fitafter = magic_formula(optimal_params, [slipAngleR(:, i) FzR(:, i)]);
    FyR_fit(:, i) = Fy_fitafter;
    optimal_params_latR(i, :) = optimal_params;
    fprintf('B: %2f, C: %2f, D: %2f, E: %2f, Sx: %4f, Sv: %2f\n', ...
           optimal_params(1), optimal_params(2), optimal_params(3), ...
           optimal_params(4), optimal_params(5), optimal_params(6));
end

FyLlinear = slipAngleL_m * avp.ca;
FyRlinear = slipAngleR_m * avp.ca;

%% plot

close all;

set(0,'DefaultAxesFontsize',16);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

sty={'r-','g-','b-','r--','g--','b--'};
axle_str={'Axle 1','Axle 2','Axle 3','Axle 4','Axle 5','Axle 6','Axle 7','Axle 8'};
unit_str={'Unit 1 (Math)','Unit 2 (Math)','Unit 3 (Math)', 'Unit 1 (IPG)', 'Unit 2 (IPG)', 'Unit 3 (IPG)'};


% plot(FyL(:, i) - FyL_fit(:, i), '-^', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerIndices', 1:5:length(FyL(:, i)), 'LineWidth', 1), hold on
% plot(FyL(:, i) - FyL_linear(:, i), '-x', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerIndices', 1:5:length(FyL(:, i)), 'LineWidth', 1), hold on

FyL_true = FyL(1:10:30000, :);
FyL_linear = FyLlinear(1:10:30000, :);
FyL_nonlinear = FyL_fit(1:10:30000, :);
time = T(1:10:30000, :);
FyR_true = FyR(1:10:30000, :);
FyR_linear = FyRlinear(1:10:30000, :);
FyR_nonlinear = FyR_fit(1:10:30000, :);

h1 = figure;
plot(time, FyL_true(:, 2) - FyL_nonlinear(:, 2), 'LineWidth', 1), hold on
plot(time, FyL_true(:, 2) - FyL_linear(:, 2), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -5000 10000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h1,'-depsc2','-loose',[FigurePath,'secondaxle_left_latf_error.eps']); 
savefig(h1, [FigurePath,'/secondaxle_left_latf_error.fig']);

h2 = figure;
plot(time, FyR_true(:, 2) - FyR_nonlinear(:, 2), 'LineWidth', 1), hold on
plot(time, FyR_true(:, 2) - FyR_linear(:, 2), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -8000 5000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h2,'-depsc2','-loose',[FigurePath,'secondaxle_right_latf_error.eps']); 
savefig(h2, [FigurePath,'/secondaxle_right_latf_error.fig']);

h3 = figure;
plot(time, FyL_true(:, 4) - FyL_nonlinear(:, 4), 'LineWidth', 1), hold on
plot(time, FyL_true(:, 4) - FyL_linear(:, 4), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -5000 7000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h3,'-depsc2','-loose',[FigurePath,'fourthaxle_left_latf_error.eps']); 
savefig(h3, [FigurePath,'/fourthaxle_left_latf_error.fig']);

h4 = figure;
plot(time, FyR_true(:, 4) - FyR_nonlinear(:, 4), 'LineWidth', 1), hold on
plot(time, FyR_true(:, 4) - FyR_linear(:, 4), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -7000 6000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h4,'-depsc2','-loose',[FigurePath,'fourthaxle_right_latf_error.eps']); 
savefig(h4, [FigurePath,'/fourthaxle_right_latf_error.fig']);

h5 = figure;
plot(time, FyL_true(:, 6) - FyL_nonlinear(:, 6), 'LineWidth', 1), hold on
plot(time, FyL_true(:, 6) - FyL_linear(:, 6), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -5000 7000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h5,'-depsc2','-loose',[FigurePath,'sixthaxle_left_latf_error.eps']); 
savefig(h5, [FigurePath,'/sixthaxle_left_latf_error.fig']);

h6 = figure;
plot(time, FyR_true(:, 6) - FyR_nonlinear(:, 6), 'LineWidth', 1), hold on
plot(time, FyR_true(:, 6) - FyR_linear(:, 6), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -7000 6000]);
set(gcf, 'Position', [600, 400, 500, 300]); 
print(h6,'-depsc2','-loose',[FigurePath,'sixthaxle_right_latf_error.eps']); 
savefig(h6, [FigurePath,'/sixthaxle_right_latf_error.fig']);

figure;
plot(time, FyL_true(:, 8) - FyL_nonlinear(:, 8), 'LineWidth', 1), hold on
plot(time, FyL_true(:, 8) - FyL_linear(:, 8), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -5000 15000]);
set(gcf, 'Position', [600, 400, 500, 300]); 

figure;
plot(time, FyR_true(:, 8) - FyR_nonlinear(:, 8), 'LineWidth', 1), hold on
plot(time, FyR_true(:, 8) - FyR_linear(:, 8), '--', 'LineWidth', 1), hold on
legend('\fontname{宋体}非线性模型','\fontname{宋体}线性模型','Location','northwest');
grid on
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}轮胎侧向力误差 \fontname{Times New Roman}(N)');
axis([0 30 -7000 6000]);
set(gcf, 'Position', [600, 400, 500, 300]); 

% figure;
% for i = 1:8
%     subplot(2, 4, i);
%     plot(FyL(:, i), 'k','LineWidth',1), hold on
%     plot(FyLlinear(:, i), 'r','LineWidth',1), hold on
%     legend('IPG','Math','Location','southwest');
%     grid on
%     xlabel('Time / s');
%     ylabel('F lat / N');
% %     axis([0 35 -1 2]);
%     title([num2str(i), '-th axle left tyre']);
% end
% set(gcf, 'Position', [50, 200, 1600, 600]); 
% 
% figure;
% for i = 1:8
%     subplot(2, 4, i);
%     plot(FyR(:, i), 'k','LineWidth',1), hold on
%     plot(FyRlinear(:, i), 'r','LineWidth',1), hold on
%     legend('IPG','Math','Location','southwest');
%     grid on
%     xlabel('Time / s');
%     ylabel('F lat / N');
% %     axis([0 35 -1 2]);
%     title([num2str(i), '-th axle right tyre']);
% end
% set(gcf, 'Position', [50, 200, 1600, 600]); 


