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
point_str={'Unit 1 (Math)','Unit 2 (Math)','Unit 3 (Math)', 'Unit 1 (IPG)', 'Unit 2 (IPG)', 'Unit 3 (IPG)'};


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
plot(t1, abs(squeeze(r1.ref_laterr.Data(1,2,:))), 'LineWidth', 1);
plot(t3, abs(squeeze(r3.ref_laterr.Data(1,2,:))), '--', 'LineWidth', 1);
plot(t1, abs(squeeze(r1.ref_laterr.Data(1,3,:))), 'LineWidth', 1);
plot(t3, abs(squeeze(r3.ref_laterr.Data(1,3,:))), '--', 'LineWidth', 1);
xlabel('\fontname{Times New Roman}Time (s)');
ylabel('\fontname{Times New Roman}Lateral error (m)');
legend('Mechanism J1', 'Proposed J1','Mechanism J2', 'Proposed J2', 'Location','northwest');
set(gcf, 'Position', [600, 400, 700, 400]); 
axis([-10 60 -0.01 0.25]);
saveas(gcf, [FigurePath,'simu41.png']);
% print(h1,'-depsc2','-loose',[FigurePath,'ctrlpoint_all_error.eps']); 
% savefig(h1, [FigurePath,'/ctrlpoint_all_error.fig']);

%% Controller Compare Heading Error

set(0,'DefaultAxesFontsize',16);
h2 = figure;
hold on;
plot(t1, abs(wrap_angle(squeeze(r1.ref_yaws.Data(1,1,:)) - s1(:,3)) / DEG2RAD), 'LineWidth', 1);
plot(t3, abs(wrap_angle(squeeze(r3.ref_yaws.Data(1,1,:)) - s3(:,3)) / DEG2RAD), '--', 'LineWidth', 1);
plot(t1, abs(wrap_angle(squeeze(r1.ref_yaws.Data(1,2,:)) - s1(:,4)) / DEG2RAD), 'LineWidth', 1);
plot(t3, abs(wrap_angle(squeeze(r3.ref_yaws.Data(1,2,:)) - s3(:,4)) / DEG2RAD), '--', 'LineWidth', 1);
plot(t1, abs(wrap_angle(squeeze(r1.ref_yaws.Data(1,3,:)) - s1(:,5)) / DEG2RAD), 'LineWidth', 1);
plot(t3, abs(wrap_angle(squeeze(r3.ref_yaws.Data(1,3,:)) - s3(:,5)) / DEG2RAD), '--', 'LineWidth', 1);
xlabel('\fontname{Times New Roman}Time (s)');
ylabel('\fontname{Times New Roman}Yaw error (deg)');
unit_str = {'Mechanism U1', 'Proposed U1','Mechanism U2', 'Proposed U2','Mechanism U3', 'Proposed U3'};
legend(unit_str, 'Location','northwest');
lgd = legend(unit_str); % 获取图例句柄
lgd.Orientation = 'horizontal'; % 设置图例水平排列
lgd.Position = [0.24, 0.78, 0.55, 0.12]; % [左, 下, 宽, 高]
lgd.NumColumns = 3; % 设置每行显示的列数
set(gcf, 'Position', [600, 400, 700, 400]); 
axis([0 58 -0.05 1]);
saveas(gcf, [FigurePath,'simu42.png']);
% print(h5,'-depsc2','-loose',[FigurePath,'yaw1_error.eps']); 
% savefig(h5, [FigurePath,'/yaw1_error.fig']);

