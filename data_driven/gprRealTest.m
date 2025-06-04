%% GPR Learn Physics

close all

% organize and standardize
state = zeros(size(Y, 1), 11);
state(:, 1) = sin(Y(:, 3));
state(:, 2) = sin(Y(:, 4));
state(:, 3) = sin(Y(:, 5));
state(:, 4) = cos(Y(:, 3));
state(:, 5) = cos(Y(:, 4));
state(:, 6) = cos(Y(:, 5));
state(:, 7:11) = Y(:, 6:10);
vars = [state U];

% state = Y(:, 3:10);
% vars = [state U];

% normalize
vars_mean = mean(vars);
vars_std = std(vars);
vars_normalized = (vars - vars_mean) ./ vars_std; 

vars_normalized = vars;
vars_mean = 0;
vars_std = 1;

TrainX = vars_normalized;
TrainT = Ef(:, 6:10);

% data memory
% sin(yaw) cos(yaw) vx vy yawrate no_normalized/normalized 100 sr fic matern32
% train: 3.0791    2.1058    2.2465    5.4069    6.5838    6.4430
%        0.2796    0.1952    0.2002    0.0141    0.0266    0.0150
% test:  1.6293    0.8619    1.3827    3.3546    3.6739    3.4851
%        0.1789    0.1210    0.1318    0.0113    0.0201    0.0124
% sin(yaw) cos(yaw) vx vy yawrate no_normalized 200 sr fic matern32 300 more bad
% train: 6.7082    3.6285    5.6422   12.7260   14.8264   15.2920
%        0.5215    0.3487    0.3877    0.0176    0.0255    0.0214
% test:  3.9212    1.7948    3.4864    6.3815    8.6476    8.5025
%        0.2903    0.1642    0.2394    0.0108    0.0158    0.0168
% sin(yaw) cos(yaw) vx vy yawrate normalized 200 sr fic matern32
% train: 7.0192    3.7829    5.9126   12.0331   15.0452   15.3857
%        0.5168    0.3378    0.3911    0.0159    0.0248    0.0210
% test:  4.1075    2.0596    3.5539    7.7871    9.9176    9.6976
%        0.3103    0.1745    0.2565    0.0107    0.0165    0.0174
% yaw vx vy yawrate no_normalized 100 sr fic matern32 no result
% sin(yaw) cos(yaw) vx vy yawrate no_normalized exponential 100 sr fic
% train: 3.0797    2.2418    2.1117    5.3788    6.4749    6.2510
%        0.2798    0.1958    0.1998    0.0142    0.0271    0.0150
% test:  2.0038    1.2747    1.5461    5.1270    5.2244    5.3054
%        0.1999    0.1382    0.1445    0.0111    0.0208    0.0121
% sin(yaw) cos(yaw) vx vy yawrate no_normalized exponential 200 sr fic
% train: 2.9685    2.2809    1.8999    6.3067    6.9924    7.0653
%        0.2801    0.1916    0.2043    0.0138    0.0270    0.0142
% test:  1.9806    1.3059    1.4890    5.3807    5.4849    5.5572
%        0.2008    0.1355    0.1482    0.0109    0.0210    0.0119
% sin(yaw) cos(yaw) vx vy yawrate no_normalized exponential 300 sr fic 500 more bad
% train: 3.5083    2.3550    2.6004    5.4886    6.7961    6.8197
%        0.2993    0.2032    0.2198    0.0140    0.0264    0.0153
% test:  1.9647    1.3870    1.3915    5.6659    5.8083    5.9617
%        0.2011    0.1362    0.1480    0.0110    0.0211    0.0122
% sin(yaw) cos(yaw) vx vy yawrate no_normalized matern52 100 sr fic 200 300 more bad
% train: 3.0827    2.0640    2.2897    5.3129    6.4466    6.3218
%        0.2785    0.1940    0.1998    0.0140    0.0265    0.0150
% test:  1.7499    1.0227    1.4200    4.2111    4.3434    4.2915
%        0.1820    0.1230    0.1342    0.0109    0.0204    0.0121
% sin(yaw) cos(yaw) vx vy yawrate no_normalized rationalquadratic 100 sr fic 200 300 more bad
% train: 3.1072    2.2512    2.1416    5.4145    6.5435    6.3384
%        0.2814    0.1976    0.2004    0.0143    0.0272    0.0151
% test:  1.9321    1.2026    1.5122    4.7981    4.8780    4.9501
%        0.1964    0.1363    0.1415    0.0112    0.0208    0.0122

%% GPR main

rng('default'); 
cv = cvpartition(size(X, 1), 'HoldOut', 0.2); 
XTrain = TrainX(training(cv), :);
YTrain = TrainT(training(cv), :);
XValidation = TrainX(test(cv), :);
YValidation = TrainT(test(cv), :);

% % Reduce dataset size for training
% subsampleRatio = 0.1; % Use 10% of the data
% idx = randperm(nTrain, floor(subsampleRatio * nTrain));
% X_train_sub = X_train(idx, :);
% Y_train_sub = Y_train(idx, :);

% Initialize GPR models for each output variable
gprModels = cell(1, 5); 

% Train a separate GPR model for each output variable
% matern32 matern52 exponential rationalquadratic ard + ...
% Exponential, ARDExponential,SquaredExponential, ARDSquaredExponential, 
% Matern32, ARDMatern32, Matern52, ARDMatern52, RationalQuadratic, ARDRationalQuadratic

disp('Training GPR models...');
parpool('local'); 
parfor i = 1:5
    tic;
    fprintf('Training GPR model for output variable %d...\n', i);
    gprModels{i} = fitrgp(XTrain, YTrain(:, i), ...
        'KernelFunction', 'rationalquadratic', ...
        'ActiveSetSize', 200, ...       
        'FitMethod', 'sr', ...          % Use Sparse Regression (SR) method
        'PredictMethod', 'fic', ...     % Fully Independent Conditional (FIC)
        ... % 'Optimizer', 'lbfgs', ...       
        'Standardize', true, ...
        'OptimizerOptions', struct('MaxIterations', 500)); 
    toc;
end
delete(gcp('nocreate'));

% 验证集预测误差
YPred = zeros(size(YValidation));
for i = 1:5
    YPred(:, i) = predict(gprModels{i}, XValidation);
end
rmse = sqrt(mean((YPred - YValidation).^2, 'all'));
fprintf('Valid RMSE: %.4f\n', rmse);

%% test

deg_trans = 180 / pi;
XG = Maaav_GPR_discrep(Y, U, T, avp, sep, gprModels, vars_mean, vars_std);

figure,
for i = 1:length(sep(:, 1))
    subplot(3,3,i);
    plot(Y(sep(i,1):sep(i,2),1), Y(sep(i,1):sep(i,2),2), 'k','LineWidth',1), hold on
    plot(XG(sep(i,1):sep(i,2),1), XG(sep(i,1):sep(i,2),2), 'b','LineWidth',1), hold on
    legend('True','GaussProcess');
end

Error_GR = XG - Y;
Error_GR(:, 3) = (mod(Error_GR(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_GR(:, 4) = (mod(Error_GR(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_GR(:, 5) = (mod(Error_GR(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_GR_XY = sqrt(mean(hypot(Error_GR(:, 1), Error_GR(:, 2)).^2));
RMSE_GR_X = sqrt(mean((Error_GR(:, 1)).^2));
RMSE_GR_Y = sqrt(mean((Error_GR(:, 2)).^2));
RMSE_GR_YAW1 = sqrt(mean((Error_GR(:, 3)).^2));
RMSE_GR_YAW2 = sqrt(mean((Error_GR(:, 4)).^2));
RMSE_GR_YAW3 = sqrt(mean((Error_GR(:, 5)).^2));
RMSE_GR = [RMSE_GR_XY RMSE_GR_X RMSE_GR_Y RMSE_GR_YAW1 RMSE_GR_YAW2 RMSE_GR_YAW3]

RMSE_GR_VXY = sqrt(mean(hypot(Error_GR(:, 6), Error_GR(:, 7)).^2));
RMSE_GR_VX = sqrt(mean((Error_GR(:, 6)).^2));
RMSE_GR_VY = sqrt(mean((Error_GR(:, 7)).^2));
RMSE_GR_YAWRATE1 = sqrt(mean((Error_GR(:, 8)).^2));
RMSE_GR_YAWRATE2 = sqrt(mean((Error_GR(:, 9)).^2));
RMSE_GR_YAWRATE3 = sqrt(mean((Error_GR(:, 10)).^2));
RMSE_GR_V = [RMSE_GR_VXY RMSE_GR_VX RMSE_GR_VY RMSE_GR_YAWRATE1 RMSE_GR_YAWRATE2 RMSE_GR_YAWRATE3]

XG_t = Maaav_GPR_discrep(TestY, TestU, TestT, avp, sepT, gprModels, vars_mean, vars_std);

Error_GR_T = XG_t - TestY;
Error_GR_T(:, 3) = (mod(Error_GR_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_GR_T(:, 4) = (mod(Error_GR_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_GR_T(:, 5) = (mod(Error_GR_T(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_GR_XY_T = sqrt(mean(hypot(Error_GR_T(:, 1), Error_GR_T(:, 2)).^2));
RMSE_GR_X_T = sqrt(mean((Error_GR_T(:, 1)).^2));
RMSE_GR_Y_T = sqrt(mean((Error_GR_T(:, 2)).^2));
RMSE_GR_YAW1_T = sqrt(mean((Error_GR_T(:, 3)).^2));
RMSE_GR_YAW2_T = sqrt(mean((Error_GR_T(:, 4)).^2));
RMSE_GR_YAW3_T = sqrt(mean((Error_GR_T(:, 5)).^2));
RMSE_GR_T = [RMSE_GR_XY_T RMSE_GR_X_T RMSE_GR_Y_T RMSE_GR_YAW1_T RMSE_GR_YAW2_T RMSE_GR_YAW3_T]

RMSE_GR_VXY_T = sqrt(mean(hypot(Error_GR_T(:, 6), Error_GR_T(:, 7)).^2));
RMSE_GR_VX_T = sqrt(mean((Error_GR_T(:, 6)).^2));
RMSE_GR_VY_T = sqrt(mean((Error_GR_T(:, 7)).^2));
RMSE_GR_YAWRATE1_T = sqrt(mean((Error_GR_T(:, 8)).^2));
RMSE_GR_YAWRATE2_T = sqrt(mean((Error_GR_T(:, 9)).^2));
RMSE_GR_YAWRATE3_T = sqrt(mean((Error_GR_T(:, 10)).^2));
RMSE_GR_V_T = [RMSE_GR_VXY_T RMSE_GR_VX_T RMSE_GR_VY_T RMSE_GR_YAWRATE1_T RMSE_GR_YAWRATE2_T RMSE_GR_YAWRATE3_T]

figure,
for i = 1:length(sepT(:, 1))
    subplot(2,2,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XG_t(sepT(i,1):sepT(i,2),1), XG_t(sepT(i,1):sepT(i,2),2), 'b','LineWidth',1), hold on
    legend('True','GaussProcess');
end
