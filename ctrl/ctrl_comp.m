close all;

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
set(0,'DefaultAxesFontsize',18);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];
mkdir(FigurePath)

sty={'r-','g-','b-','r--','g--','b--'};
axle_str={'Axle 1','Axle 2','Axle 3','Axle 4','Axle 5','Axle 6','Axle 7','Axle 8'};
unit_str={'Unit 1 (Math)','Unit 2 (Math)','Unit 3 (Math)', 'Unit 1 (IPG)', 'Unit 2 (IPG)', 'Unit 3 (IPG)'};

load('ctrl_ltvmpc.mat');
t1 = Ctrl_time;
r1 = Ctrl_ref;
s1 = Ctrl_state;
d1 = Ctrl_steer;

load('ctrl_ackm.mat');
t2 = Ctrl_time;
r2 = Ctrl_ref;
s2 = Ctrl_state;
d2 = Ctrl_steer;

h1 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,1,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,1,:)), '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第一控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.5 3]);
print(h1,'-depsc2','-loose',[FigurePath,'ctrlpoint1_error_ca.eps']); 
savefig(h1, [FigurePath,'/ctrlpoint1_error_ca.fig']);

h2 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,2,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,2,:)), '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第二控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -1.2 1.6]);
print(h2,'-depsc2','-loose',[FigurePath,'ctrlpoint2_error_ca.eps']); 
savefig(h2, [FigurePath,'/ctrlpoint2_error_ca.fig']);

h3 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,3,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,3,:)), '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第三控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.5 5]);
print(h3,'-depsc2','-loose',[FigurePath,'ctrlpoint3_error_ca.eps']); 
savefig(h3, [FigurePath,'/ctrlpoint3_error_ca.fig']);

h4 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,4,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,4,:)), '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第四控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -1 8]);
print(h4,'-depsc2','-loose',[FigurePath,'ctrlpoint4_error_ca.eps']); 
savefig(h4, [FigurePath,'/ctrlpoint4_error_ca.fig']);

%% Controller Compare Heading Error

h5 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,1,:)) - s1(:,3)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,1,:)) - s2(:,3)) / DEG2RAD, '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}头车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -5 12]);
print(h5,'-depsc2','-loose',[FigurePath,'yaw1_error_ca.eps']); 
savefig(h5, [FigurePath,'/yaw1_error_ca.fig']);

h6 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,2,:)) - s1(:,4)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,2,:)) - s2(:,4)) / DEG2RAD, '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}中间车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -20 7]);
print(h6,'-depsc2','-loose',[FigurePath,'yaw2_error_ca.eps']); 
savefig(h6, [FigurePath,'/yaw2_error_ca.fig']);

h7 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,3,:)) - s1(:,5)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,3,:)) - s2(:,5)) / DEG2RAD, '--', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}尾车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}模型预测控制', '\fontname{宋体}扩展阿克曼方法', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -20 20]);
print(h7,'-depsc2','-loose',[FigurePath,'yaw3_error_ca.eps']); 
savefig(h7, [FigurePath,'/yaw3_error_ca.fig']);

%%
h8 = figure;
hold on;
plot(t1, wrap_angle(d1(:, 1)) / DEG2RAD, 'LineWidth', 1);
plot(t1, wrap_angle(d1(:, 2)) / DEG2RAD, '--', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t1));
plot(t1, wrap_angle(d1(:, 3)) / DEG2RAD, '-.', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t1));
plot(t1, wrap_angle(d1(:, 4)) / DEG2RAD, '-+', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t1));
plot(t1, wrap_angle(d1(:, 5)) / DEG2RAD, '-*', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t1));
plot(t1, wrap_angle(d1(:, 6)) / DEG2RAD, '-^', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t1));
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}各轴转向角度 \fontname{Times New Roman}(deg)');
legend('\fontname{Times New Roman}1\fontname{宋体}轴', '\fontname{Times New Roman}3\fontname{宋体}轴',...
       '\fontname{Times New Roman}4\fontname{宋体}轴', '\fontname{Times New Roman}5\fontname{宋体}轴',...
       '\fontname{Times New Roman}6\fontname{宋体}轴', '\fontname{Times New Roman}8\fontname{宋体}轴','Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([-3 65 -6 10]);
print(h8,'-depsc2','-loose',[FigurePath,'steer_ca_mpc.eps']); 
savefig(h8, [FigurePath,'/steer_ca_mpc.fig']);

h9 = figure;
hold on;
plot(t2, wrap_angle(d2(:, 1)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(d2(:, 2)) / DEG2RAD, '--', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t2));
plot(t2, wrap_angle(d2(:, 3)) / DEG2RAD, '-.', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t2));
plot(t2, wrap_angle(d2(:, 4)) / DEG2RAD, '-+', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t2));
plot(t2, wrap_angle(d2(:, 5)) / DEG2RAD, '-*', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t2));
plot(t2, wrap_angle(d2(:, 6)) / DEG2RAD, '-^', 'LineWidth', 1, 'MarkerIndices', 1:2000:length(t2));
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}各轴转向角度 \fontname{Times New Roman}(deg)');
legend('\fontname{Times New Roman}1\fontname{宋体}轴', '\fontname{Times New Roman}3\fontname{宋体}轴',...
       '\fontname{Times New Roman}4\fontname{宋体}轴', '\fontname{Times New Roman}5\fontname{宋体}轴',...
       '\fontname{Times New Roman}6\fontname{宋体}轴', '\fontname{Times New Roman}8\fontname{宋体}轴','Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([-3 75 -25 25]);
print(h9,'-depsc2','-loose',[FigurePath,'steer_ca_ackm.eps']); 
savefig(h9, [FigurePath,'/steer_ca_ackm.fig']);
