%% Circle Track Generate

close all;

% 设定图片中的统一字体和字号。单幅图片可以另行设定。设定20号字主要考虑图片缩小后放在论文中的效果。
% set(0,'DefaultAxesFontname','TimesSimSun');
set(0,'DefaultAxesFontname','Times New Roman');
set(0,'DefaultAxesFontsize',14);

% 建立一个专用的文件夹，用于存放数据和图片。可根据实际情况设定。
FigurePath = ['./SimuData/figures/', datestr(now,'yyyymmdd')];

% valid2
part_1 = straight_gen(1,[0,0],0,0,350);
part_2 = curve_gen_new(1,part_1(end,:),1/30,225,1);
part_3 = straight_gen(1,part_2(end,2:3),part_2(end,1),225,60);
part_4 = curve_gen_new(1,part_3(end,:),1/30,270,2);
part_5 = straight_gen(1,part_4(end,2:3),part_4(end,1),315,60);
part_6 = curve_gen_new(1,part_5(end,:),1/30,45,1);
part_7 = straight_gen(1,part_6(end,2:3),part_6(end,1),0,150);
track = [part_1(1:end-1,:); part_2(1:end-1,:); part_3(1:end-1,:); part_4(1:end-1,:); ...
         part_5(1:end-1,:); part_6(1:end-1,:); part_7];
     
h1 = figure;
plot(track(150:850, 2), track(150:850, 3), 'k', 'LineWidth', 1);
legend('\fontname{宋体}参考轨迹','Location','northwest');
grid on
xlabel('\fontname{宋体}横向位置 \fontname{Times New Roman}(m)');
ylabel('\fontname{宋体}纵向位置 \fontname{Times New Roman}(m)');
axis([130 470 -10 70]);
set(gcf, 'Position', [500, 400, 700, 200]); 
print(h1,'-depsc2','-loose',[FigurePath,'track_valid2.eps']); 
savefig(h1, [FigurePath,'/track_valid2.fig']);

% valid3
part_1 = straight_gen(1,[0,0],0,0,250);
part_2 = curve_gen(1,part_1(end,:),1/15,360);
part_3 = straight_gen(1,part_2(end,2:3),part_2(end,1),0,100);
track = [part_1(1:end-1,:); part_2(1:end-1,:); part_3];

h2 = figure;
plot(track(200:420, 2), track(200:420, 3), 'k', 'LineWidth', 1);
legend('\fontname{宋体}参考轨迹','Location','northwest');
grid on
xlabel('\fontname{宋体}横向位置 \fontname{Times New Roman}(m)');
ylabel('\fontname{宋体}纵向位置 \fontname{Times New Roman}(m)');
axis([180 340 -5 35]);
set(gcf, 'Position', [500, 400, 700, 200]); 
print(h2,'-depsc2','-loose',[FigurePath,'track_valid3.eps']); 
savefig(h2, [FigurePath,'/track_valid3.fig']);

function track = straight_gen(delta, start_p, start_s, dir_ang, length)
    phi = dir_ang*pi/180;
    s = start_s + (0:delta:length);
    s = s';
    index_max = size(s,1);
    x = start_p(1) + (0:delta:length)*cos(phi);
    y = start_p(2) + (0:delta:length)*sin(phi);
    x = x';
    y = y';
    t = [ones(index_max,1)*cos(phi) ones(index_max,1)*sin(phi)];
    n = [-t(:,2) t(:,1)];
    cur = zeros(index_max,1);
    track = [s x y t n cur];

end

function track = curve_gen(delta, start_row, curve, totalAng)
    totalAng = totalAng*pi/180;
    R = 1/curve;
    start_s = start_row(1);
    start_p = start_row(2:3);
    start_n = start_row(6:7);
    center = start_p + R*start_n;
    s = start_s + (0:delta:totalAng*abs(R));
    s = s';
    index_max = size(s,1);
    start_phi = atan2(start_p(2) - center(2),start_p(1) - center(1));
    angles = start_phi + (0:(index_max-1))*delta*curve;
    angles = angles';
    x = center(1) + abs(R)*cos(angles);
    y = center(2) + abs(R)*sin(angles);
    n = -sign(curve)*[cos(angles) sin(angles)];
    t = [n(:,2),-n(:,1)];
    cur = ones(index_max,1)*curve;
    track = [s x y t n cur];
end

function track = curve_gen_new(delta, start_row, curve, totalAng, direction)
    totalAng = totalAng*pi/180;
    R = 1/curve;
    start_s = start_row(1);
    start_p = start_row(2:3);
    start_n = start_row(6:7);
    if direction == 1
        center = start_p + R*start_n;
    else
        center = start_p - R*start_n;
    end
    s = start_s + (0:delta:totalAng*abs(R));
    s = s';
    index_max = size(s,1);
    start_phi = atan2(start_p(2) - center(2),start_p(1) - center(1));
    if direction == 1
        angles = start_phi + (0:(index_max-1))*delta*curve;
    else
        angles = start_phi - (0:(index_max-1))*delta*curve;
    end
    angles = angles';
    x = center(1) + abs(R)*cos(angles);
    y = center(2) + abs(R)*sin(angles);
    if direction == 1
        n = -sign(curve)*[cos(angles) sin(angles)];
    else
        n = sign(curve)*[cos(angles) sin(angles)];
    end
    t = [n(:,2),-n(:,1)];
    cur = ones(index_max,1)*curve;
    track = [s x y t n cur];
end
