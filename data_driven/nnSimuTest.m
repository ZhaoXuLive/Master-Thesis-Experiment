%% NN Learn Physics

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
vars = [state U Trq];

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
% sin(yaw) cos(yaw) vx vy yawrate normalized
% train: 6.2868    4.9171    3.9175    7.3601    7.9774    8.5036
%        1.0289    0.5877    0.8445    0.0193    0.0257    0.0191
% test:  137.4448  105.8599   87.6627   88.1255   95.1352  105.8228
%        22.2603   16.9493   14.4306    2.1212    2.5795    2.3303

%% Train Neural Network Parameter

rng('default'); 
cv = cvpartition(size(X, 1), 'HoldOut', 0.2); 
XTrain = TrainX(training(cv), :);
YTrain = TrainT(training(cv), :);
XValidation = TrainX(test(cv), :);
YValidation = TrainT(test(cv), :);

% define layers
% layers = [
%     featureInputLayer(27, 'Name', 'input')                     
%     fullyConnectedLayer(256, 'Name', 'fc1')                    
%     leakyReluLayer(0.01, 'Name', 'leakyrelu1')                 
%     fullyConnectedLayer(128, 'Name', 'fc2')                    
%     leakyReluLayer(0.01, 'Name', 'leakyrelu2')                 
%     fullyConnectedLayer(64, "Name", "fc3")                    
%     swishLayer("Name", "Swish")                                
%     fullyConnectedLayer(5, 'Name', 'fc5')                      
%     regressionLayer('Name', 'regression')                      
% ];

layers = [
    featureInputLayer(27, 'Name', 'input')   % 输入层（20维特征）
    batchNormalizationLayer('Name', 'bn1')   % 批标准化
    fullyConnectedLayer(32, 'Name', 'fc1')  % 全连接层128神经元
    leakyReluLayer(0.01, 'Name', 'leaky1')   % 自定义LeakyReLU
    dropoutLayer(0.2, 'Name', 'drop1')       % Dropout层
    fullyConnectedLayer(32, 'Name', 'fc2')   % 全连接层64神经元
    leakyReluLayer(0.01, 'Name', 'leaky2')   % 自定义LeakyReLU
    fullyConnectedLayer(16, "Name", "fc3")                    
    swishLayer("Name", "Swish")    
%     fullyConnectedLayer(32, "Name", "fc4")                    
%     swishLayer("Name", "Swish")  
    fullyConnectedLayer(5, 'Name', 'output') % 输出层（5维残差）
    regressionLayer('Name', 'regression')    % 回归任务层（默认MSE损失）
];

options = trainingOptions('adam', ...
    'ExecutionEnvironment', 'gpu', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 64, ...
    'InitialLearnRate', 3e-4, ...
    'ValidationData', {XValidation, YValidation}, ...
    'ValidationFrequency', 10, ...
    'ValidationPatience', 20, ...     % 允许10次验证无改善
    'Shuffle', 'every-epoch', ...
    'Verbose', false, ...
    'Plots', 'training-progress');

%     'LearnRateSchedule', 'piecewise', ...
%     'LearnRateDropPeriod', 15, ...
%     'LearnRateDropFactor', 0.5, ...

% options = trainingOptions('adam', ...
%     'InitialLearnRate', 0.001, ...
%     'MaxEpochs', 100, ...
%     'MiniBatchSize', 64, ...
%     'Shuffle', 'every-epoch', ...
%     'ValidationData', {XValidation, YValidation}, ...
%     'ValidationFrequency', 10, ... %     
%     'ValidationPatience', 20, ...
%     'Plots', 'training-progress', ...
%     'Verbose', false);

% 网络训练
net = trainNetwork(XTrain, YTrain, layers, options);

% 验证集预测误差
YPred = predict(net, XValidation);
rmse = sqrt(mean((YPred - YValidation).^2, 'all'));
fprintf('Valid RMSE: %.4f\n', rmse);

%% test

deg_trans = 180 / pi;

XN = Maaav_improved_NN_discrep(Y, U, T, Trq, avp, sep, net, vars_mean, vars_std);

figure,
for i = 1:length(sep(:, 1))
    subplot(3,5,i);
    plot(Y(sep(i,1):sep(i,2),1), Y(sep(i,1):sep(i,2),2), 'k','LineWidth',1), hold on
    plot(XN(sep(i,1):sep(i,2),1), XN(sep(i,1):sep(i,2),2), 'b','LineWidth',1), hold on
    legend('True','NerualNet');
end

Error_NN = XN - Y;
Error_NN(:, 3) = (mod(Error_NN(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_NN(:, 4) = (mod(Error_NN(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_NN(:, 5) = (mod(Error_NN(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_NN_XY = sqrt(mean(hypot(Error_NN(:, 1), Error_NN(:, 2)).^2));
RMSE_NN_X = sqrt(mean((Error_NN(:, 1)).^2));
RMSE_NN_Y = sqrt(mean((Error_NN(:, 2)).^2));
RMSE_NN_YAW1 = sqrt(mean((Error_NN(:, 3)).^2));
RMSE_NN_YAW2 = sqrt(mean((Error_NN(:, 4)).^2));
RMSE_NN_YAW3 = sqrt(mean((Error_NN(:, 5)).^2));
RMSE_NN = [RMSE_NN_XY RMSE_NN_X RMSE_NN_Y RMSE_NN_YAW1 RMSE_NN_YAW2 RMSE_NN_YAW3]

RMSE_NN_VXY = sqrt(mean(hypot(Error_NN(:, 6), Error_NN(:, 7)).^2));
RMSE_NN_VX = sqrt(mean((Error_NN(:, 6)).^2));
RMSE_NN_VY = sqrt(mean((Error_NN(:, 7)).^2));
RMSE_NN_YAWRATE1 = sqrt(mean((Error_NN(:, 8)).^2));
RMSE_NN_YAWRATE2 = sqrt(mean((Error_NN(:, 9)).^2));
RMSE_NN_YAWRATE3 = sqrt(mean((Error_NN(:, 10)).^2));
RMSE_NN_V = [RMSE_NN_VXY RMSE_NN_VX RMSE_NN_VY RMSE_NN_YAWRATE1 RMSE_NN_YAWRATE2 RMSE_NN_YAWRATE3]

XN_t = Maaav_improved_NN_discrep(TestY, TestU, TestT, TestTrq, avp, sepT, net, vars_mean, vars_std);

Error_NN_T = XN_t - TestY;
Error_NN_T(:, 3) = (mod(Error_NN_T(:, 3) + pi, 2*pi) - pi) * deg_trans;
Error_NN_T(:, 4) = (mod(Error_NN_T(:, 4) + pi, 2*pi) - pi) * deg_trans;
Error_NN_T(:, 5) = (mod(Error_NN_T(:, 5) + pi, 2*pi) - pi) * deg_trans;

RMSE_NN_XY_T = sqrt(mean(hypot(Error_NN_T(:, 1), Error_NN_T(:, 2)).^2));
RMSE_NN_X_T = sqrt(mean((Error_NN_T(:, 1)).^2));
RMSE_NN_Y_T = sqrt(mean((Error_NN_T(:, 2)).^2));
RMSE_NN_YAW1_T = sqrt(mean((Error_NN_T(:, 3)).^2));
RMSE_NN_YAW2_T = sqrt(mean((Error_NN_T(:, 4)).^2));
RMSE_NN_YAW3_T = sqrt(mean((Error_NN_T(:, 5)).^2));
RMSE_NN_T = [RMSE_NN_XY_T RMSE_NN_X_T RMSE_NN_Y_T RMSE_NN_YAW1_T RMSE_NN_YAW2_T RMSE_NN_YAW3_T]

RMSE_NN_VXY_T = sqrt(mean(hypot(Error_NN_T(:, 6), Error_NN_T(:, 7)).^2));
RMSE_NN_VX_T = sqrt(mean((Error_NN_T(:, 6)).^2));
RMSE_NN_VY_T = sqrt(mean((Error_NN_T(:, 7)).^2));
RMSE_NN_YAWRATE1_T = sqrt(mean((Error_NN_T(:, 8)).^2));
RMSE_NN_YAWRATE2_T = sqrt(mean((Error_NN_T(:, 9)).^2));
RMSE_NN_YAWRATE3_T = sqrt(mean((Error_NN_T(:, 10)).^2));
RMSE_NN_V_T = [RMSE_NN_VXY_T RMSE_NN_VX_T RMSE_NN_VY_T RMSE_NN_YAWRATE1_T RMSE_NN_YAWRATE2_T RMSE_NN_YAWRATE3_T]

figure,
for i = 1:length(sepT(:, 1))
    subplot(1,3,i);
    plot(TestY(sepT(i,1):sepT(i,2),1), TestY(sepT(i,1):sepT(i,2),2), 'k','LineWidth',1), hold on
    plot(XN_t(sepT(i,1):sepT(i,2),1), XN_t(sepT(i,1):sepT(i,2),2), 'b','LineWidth',1), hold on
    legend('True','NerualNet');
end

