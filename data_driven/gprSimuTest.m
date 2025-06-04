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

% vars_normalized = vars;
% vars_mean = 0;
% vars_std = 1;

TrainX = vars_normalized;
TrainT = Ef(:, 6:10);

% data memory
% sin(yaw) cos(yaw) vx vy yawrate no_normalized/normalized 100 sr fic matern32
% train: 3.0791    2.1058    2.2465    5.4069    6.5838    6.4430
%        0.2796    0.1952    0.2002    0.0141    0.0266    0.0150
% test:  1.6293    0.8619    1.3827    3.3546    3.6739    3.4851
%        0.1789    0.1210    0.1318    0.0113    0.0201    0.0124


%% GPR main

rng('default'); 
cv = cvpartition(size(X, 1), 'HoldOut', 0.2); 
XTrain = TrainX(training(cv), :);
YTrain = TrainT(training(cv), :);
XValidation = TrainX(test(cv), :);
YValidation = TrainT(test(cv), :);

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
        'KernelFunction', 'matern52', ...
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
XG = Maaav_improved_GPR_discrep(Y, U, T, Trq, avp, sep, gprModels, vars_mean, vars_std);

figure,
for i = 1:length(sep(:, 1))
    subplot(3,5,i);
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

XG_t = Maaav_improved_GPR_discrep(TestY, TestU, TestT, TestTrq, avp, sepT, gprModels, vars_mean, vars_std);

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
    subplot(1,3,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XG_t(sepT(i,1):sepT(i,2),1), XG_t(sepT(i,1):sepT(i,2),2), 'b','LineWidth',1), hold on
    legend('True','GaussProcess');
end
