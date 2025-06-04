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


load('ctrl_model_basic.mat');
t1 = Ctrl_time;
r1 = Ctrl_ref;
s1 = Ctrl_state;

load('ctrl_model_improved.mat');
t2 = Ctrl_time;
r2 = Ctrl_ref;
s2 = Ctrl_state;

load('ctrl_model_hybrid.mat');
t3 = Ctrl_time;
r3 = Ctrl_ref;
s3 = Ctrl_state;

h1 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,1,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,1,:)), '--', 'LineWidth', 1);
plot(t3, squeeze(r3.ref_laterr.Data(1,1,:)), '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第一控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.01 0.06]);
print(h1,'-depsc2','-loose',[FigurePath,'ctrlpoint1_error.eps']); 
savefig(h1, [FigurePath,'/ctrlpoint1_error.fig']);

h2 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,2,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,2,:)), '--', 'LineWidth', 1);
plot(t3, squeeze(r3.ref_laterr.Data(1,2,:)), '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第二控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.1 0.2]);
print(h2,'-depsc2','-loose',[FigurePath,'ctrlpoint2_error.eps']); 
savefig(h2, [FigurePath,'/ctrlpoint2_error.fig']);

h3 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,3,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,3,:)), '--', 'LineWidth', 1);
plot(t3, squeeze(r3.ref_laterr.Data(1,3,:)), '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第三控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.1 0.25]);
print(h3,'-depsc2','-loose',[FigurePath,'ctrlpoint3_error.eps']); 
savefig(h3, [FigurePath,'/ctrlpoint3_error.fig']);

h4 = figure;
hold on;
plot(t1, squeeze(r1.ref_laterr.Data(1,4,:)), 'LineWidth', 1);
plot(t2, squeeze(r2.ref_laterr.Data(1,4,:)), '--', 'LineWidth', 1);
plot(t3, squeeze(r3.ref_laterr.Data(1,4,:)), '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}第四控制点横向误差 \fontname{Times New Roman}(m)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northwest');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.1 0.15]);
print(h4,'-depsc2','-loose',[FigurePath,'ctrlpoint4_error.eps']); 
savefig(h4, [FigurePath,'/ctrlpoint4_error.fig']);

%% Controller Compare Heading Error

h5 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,1,:)) - s1(:,3)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,1,:)) - s2(:,3)) / DEG2RAD, '--', 'LineWidth', 1);
plot(t3, wrap_angle(squeeze(r3.ref_yaws.Data(1,1,:)) - s3(:,3)) / DEG2RAD, '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}头车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.8 0.6]);
print(h5,'-depsc2','-loose',[FigurePath,'yaw1_error.eps']); 
savefig(h5, [FigurePath,'/yaw1_error.fig']);

h6 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,2,:)) - s1(:,4)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,2,:)) - s2(:,4)) / DEG2RAD, '--', 'LineWidth', 1);
plot(t3, wrap_angle(squeeze(r3.ref_yaws.Data(1,2,:)) - s3(:,4)) / DEG2RAD, '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}中间车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.9 0.6]);
print(h6,'-depsc2','-loose',[FigurePath,'yaw2_error.eps']); 
savefig(h6, [FigurePath,'/yaw2_error.fig']);

h7 = figure;
hold on;
plot(t1, wrap_angle(squeeze(r1.ref_yaws.Data(1,3,:)) - s1(:,5)) / DEG2RAD, 'LineWidth', 1);
plot(t2, wrap_angle(squeeze(r2.ref_yaws.Data(1,3,:)) - s2(:,5)) / DEG2RAD, '--', 'LineWidth', 1);
plot(t3, wrap_angle(squeeze(r3.ref_yaws.Data(1,3,:)) - s3(:,5)) / DEG2RAD, '-.', 'LineWidth', 1);
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}尾车横摆角度误差 \fontname{Times New Roman}(deg)');
legend('\fontname{宋体}基础模型', '\fontname{宋体}改进模型', '\fontname{宋体}混合模型', 'Location', 'northeast');
set(gcf, 'Position', [600, 400, 600, 400]); 
axis([0 58 -0.1 0.7]);
print(h7,'-depsc2','-loose',[FigurePath,'yaw3_error.eps']); 
savefig(h7, [FigurePath,'/yaw3_error.fig']);
