%% get simulation vehicle train data

% load data
addpath('D:\TruckMaker\simulation\dataProcess\RealData')

cutoff = 4;
n = 10;
deg = pi / 180;
dt = 0.01;

% articulated vehicle parameter
avp.m = [11142; 7651; 11142;];
avp.iz = [143570; 37800; 143570;];
avp.lj = [11.4; 9.4; 11.4;];
avp.la = [2.5; 4; 9.9; 1.5; 7.9; 1.5; 7.4; 8.9];
avp.lg = [5.862; 4.643; 5.538;];
avp.h = [0.9625; 0.9625; 0.98; 0.98; 0.98; 0.98; 0.9625; 0.9625;];
avp.d = [10000; 10000];
avp.ca = 25000;
avp.sat = 15;

% 15m circle
load('15mcircle1.mat')
startpoint1 = 7001;
endpoint1 = 12000;
data_len1 = size(q_debug(startpoint1:endpoint1,:),1);
time1 = linspace(0,(data_len1-1)*0.01,data_len1)';
qqd_debug = [q_debug qd_debug];
s1.qqd1 = qqd_debug(startpoint1:endpoint1, :);
s1.u1 = steer_m(startpoint1:endpoint1, :) * deg;
u1 = zeros(data_len1, 8);
u1(:, 1) = s1.u1(:,1);
u1(:, 2) = 0.7 * s1.u1(:,1);
u1(:, 3:6) = s1.u1(:,2:5);
u1(:, 7) = 0.7 * s1.u1(:,6);
u1(:, 8) = s1.u1(:,6);
qqd1 = s1.qqd1;
% low filter
qqd_tmp1 = [];
for i = 1:10
    qqd_tmp1(:,i) = butterfilt(s1.qqd1(:,i), cutoff, 4, 'Low', time1, 'off');
end
u_tmp1 = [];
for i = 1:8
    u_tmp1(:,i) = butterfilt(u1(:,i), cutoff, 4, 'Low', time1, 'off');
end
y1 = qqd_tmp1;
u1 = u_tmp1;

load('15mcircle2.mat')
startpoint2 = 21001;
endpoint2 = 30000;
data_len2 = size(q_debug(startpoint2:endpoint2,:),1);
time2 = linspace(0,(data_len2-1)*0.01,data_len2)';
qqd_debug = [q_debug qd_debug];
s1.qqd2 = qqd_debug(startpoint2:endpoint2, :);
s1.u2 = steer_m(startpoint2:endpoint2, :) * deg;
u2 = zeros(data_len2, 8);
u2(:, 1) = s1.u2(:,1);
u2(:, 2) = 0.7 * s1.u2(:,1);
u2(:, 3:6) = s1.u2(:,2:5);
u2(:, 7) = 0.7 * s1.u2(:,6);
u2(:, 8) = s1.u2(:,6);
qqd2 = s1.qqd2;
% low filter
qqd_tmp2 = [];
for i = 1:10
    qqd_tmp2(:,i) = butterfilt(s1.qqd2(:,i), cutoff, 4, 'Low', time2, 'off');
end
u_tmp2 = [];
for i = 1:8
    u_tmp2(:,i) = butterfilt(u2(:,i), cutoff, 4, 'Low', time2, 'off');
end
y2 = qqd_tmp2;
u2 = u_tmp2;

% 15m right angle bend
load('15mrightangle1.mat')
startpoint3 = 3001;
endpoint3 = 9000;
data_len3 = size(q_debug(startpoint3:endpoint3,:),1);
time3 = linspace(0,(data_len3-1)*0.01,data_len3)';
qqd_debug = [q_debug qd_debug];
s1.qqd3 = qqd_debug(startpoint3:endpoint3, :);
s1.u3 = steer_m(startpoint3:endpoint3, :) * deg;
u3 = zeros(data_len3, 8);
u3(:, 1) = s1.u3(:,1);
u3(:, 2) = 0.7 * s1.u3(:,1);
u3(:, 3:6) = s1.u3(:,2:5);
u3(:, 7) = 0.7 * s1.u3(:,6);
u3(:, 8) = s1.u3(:,6);
qqd3 = s1.qqd3;
% low filter
qqd_tmp3 = [];
for i = 1:10
    qqd_tmp3(:,i) = butterfilt(s1.qqd3(:,i), cutoff, 4, 'Low', time3, 'off');
end
u_tmp3 = [];
for i = 1:8
    u_tmp3(:,i) = butterfilt(u3(:,i), cutoff, 4, 'Low', time3, 'off');
end
y3 = qqd_tmp3;
u3 = u_tmp3;

load('15mrightangle2.mat')
startpoint4 = 4001;
endpoint4 = 11000;
data_len4 = size(q_debug(startpoint4:endpoint4,:),1);
time4 = linspace(0,(data_len4-1)*0.01,data_len4)';
qqd_debug = [q_debug qd_debug];
s1.qqd4 = qqd_debug(startpoint4:endpoint4, :);
s1.u4 = steer_m(startpoint4:endpoint4, :) * deg;
u4 = zeros(data_len4, 8);
u4(:, 1) = s1.u4(:,1);
u4(:, 2) = 0.7 * s1.u4(:,1);
u4(:, 3:6) = s1.u4(:,2:5);
u4(:, 7) = 0.7 * s1.u4(:,6);
u4(:, 8) = s1.u4(:,6);
qqd4 = s1.qqd4;
% low filter
qqd_tmp4 = [];
for i = 1:10
    qqd_tmp4(:,i) = butterfilt(s1.qqd4(:,i), cutoff, 4, 'Low', time4, 'off');
end
u_tmp4 = [];
for i = 1:8
    u_tmp4(:,i) = butterfilt(u4(:,i), cutoff, 4, 'Low', time4, 'off');
end
y4 = qqd_tmp4;
u4 = u_tmp4;

load('15mrightangle3.mat')
startpoint5 = 6001;
endpoint5 = 12000;
data_len5 = size(q_debug(startpoint5:endpoint5,:),1);
time5 = linspace(0,(data_len5-1)*0.01,data_len5)';
qqd_debug = [q_debug qd_debug];
s1.qqd5 = qqd_debug(startpoint5:endpoint5, :);
s1.u5 = steer_m(startpoint5:endpoint5, :) * deg;
u5 = zeros(data_len5, 8);
u5(:, 1) = s1.u5(:,1);
u5(:, 2) = 0.7 * s1.u5(:,1);
u5(:, 3:6) = s1.u5(:,2:5);
u5(:, 7) = 0.7 * s1.u5(:,6);
u5(:, 8) = s1.u5(:,6);
qqd5 = s1.qqd5;
% low filter
qqd_tmp5 = [];
for i = 1:10
    qqd_tmp5(:,i) = butterfilt(s1.qqd5(:,i), cutoff, 4, 'Low', time5, 'off');
end
u_tmp5 = [];
for i = 1:8
    u_tmp5(:,i) = butterfilt(u5(:,i), cutoff, 4, 'Low', time5, 'off');
end
y5 = qqd_tmp5;
u5 = u_tmp5;

% 25m right angle bend
load('25mrightangle1.mat')
startpoint6 = 6001;
endpoint6 = 11000;
data_len6 = size(q_debug(startpoint6:endpoint6,:),1);
time6 = linspace(0,(data_len6-1)*0.01,data_len6)';
qqd_debug = [q_debug qd_debug];
s1.qqd6 = qqd_debug(startpoint6:endpoint6, :);
s1.u6 = steer_m(startpoint6:endpoint6, :) * deg;
u6 = zeros(data_len6, 8);
u6(:, 1) = s1.u6(:,1);
u6(:, 2) = 0.7 * s1.u6(:,1);
u6(:, 3:6) = s1.u6(:,2:5);
u6(:, 7) = 0.7 * s1.u6(:,6);
u6(:, 8) = s1.u6(:,6);
qqd6 = s1.qqd6;
% low filter
qqd_tmp6 = [];
for i = 1:10
    qqd_tmp6(:,i) = butterfilt(s1.qqd6(:,i), cutoff, 4, 'Low', time6, 'off');
end
u_tmp6 = [];
for i = 1:8
    u_tmp6(:,i) = butterfilt(u6(:,i), cutoff, 4, 'Low', time6, 'off');
end
y6 = qqd_tmp6;
u6 = u_tmp6;

load('25mrightangle2.mat')
startpoint7 = 3001;
endpoint7 = 8000;
data_len7 = size(q_debug(startpoint7:endpoint7,:),1);
time7 = linspace(0,(data_len7-1)*0.01,data_len7)';
qqd_debug = [q_debug qd_debug];
s1.qqd7 = qqd_debug(startpoint7:endpoint7, :);
s1.u7 = steer_m(startpoint7:endpoint7, :) * deg;
u7 = zeros(data_len7, 8);
u7(:, 1) = s1.u7(:,1);
u7(:, 2) = 0.7 * s1.u7(:,1);
u7(:, 3:6) = s1.u7(:,2:5);
u7(:, 7) = 0.7 * s1.u7(:,6);
u7(:, 8) = s1.u7(:,6);
qqd7 = s1.qqd7;
% low filter
qqd_tmp7 = [];
for i = 1:10
    qqd_tmp7(:,i) = butterfilt(s1.qqd7(:,i), cutoff, 4, 'Low', time7, 'off');
end
u_tmp7 = [];
for i = 1:8
    u_tmp7(:,i) = butterfilt(u7(:,i), cutoff, 4, 'Low', time7, 'off');
end
y7 = qqd_tmp7;
u7 = u_tmp7;

load('25mrightangle3.mat')
startpoint8 = 3001;
endpoint8 = 8000;
data_len8 = size(q_debug(startpoint8:endpoint8,:),1);
time8 = linspace(0,(data_len8-1)*0.01,data_len8)';
qqd_debug = [q_debug qd_debug];
s1.qqd8 = qqd_debug(startpoint8:endpoint8, :);
s1.u8 = steer_m(startpoint8:endpoint8, :) * deg;
u8 = zeros(data_len8, 8);
u8(:, 1) = s1.u8(:,1);
u8(:, 2) = 0.7 * s1.u8(:,1);
u8(:, 3:6) = s1.u8(:,2:5);
u8(:, 7) = 0.7 * s1.u8(:,6);
u8(:, 8) = s1.u8(:,6);
qqd8 = s1.qqd8;
% low filter
qqd_tmp8 = [];
for i = 1:10
    qqd_tmp8(:,i) = butterfilt(s1.qqd8(:,i), cutoff, 4, 'Low', time8, 'off');
end
u_tmp8 = [];
for i = 1:8
    u_tmp8(:,i) = butterfilt(u8(:,i), cutoff, 4, 'Low', time8, 'off');
end
y8 = qqd_tmp8;
u8 = u_tmp8;

Y = [y1; y2; y3; y4; y5; y6; y7; y8;];
U = [u1; u2; u3; u4; u5; u6; u7; u8;];
data_len_all = data_len1 + data_len2 + data_len3 + data_len4 + data_len5 + ...
    data_len6 + data_len7 + data_len8;
T = linspace(0,(data_len_all-1)*0.01,data_len_all)';

% 记录起始点
sep = [1 5000; 5001 14000; 14001 20000; 20001 27000; 27001 33000; ...
    33001 38000; 38001 43000; 43001 48000;];

% Multi acticulated all axle vehicle system, lagrange

x1 = Maaav_single(time1, y1, avp, u1, n);
x2 = Maaav_single(time2, y2, avp, u2, n);
x3 = Maaav_single(time3, y3, avp, u3, n);
x4 = Maaav_single(time4, y4, avp, u4, n);
x5 = Maaav_single(time5, y5, avp, u5, n);
x6 = Maaav_single(time6, y6, avp, u6, n);
x7 = Maaav_single(time7, y7, avp, u7, n);
x8 = Maaav_single(time8, y8, avp, u8, n);
X = [x1; x2; x3; x4; x5; x6; x7; x8;];

% compute Derivative

clear dy dx ef 

for i = 1:n
    dy1(:,i) = gradient(y1(:,i), dt);
    dy2(:,i) = gradient(y2(:,i), dt);
    dy3(:,i) = gradient(y3(:,i), dt);
    dy4(:,i) = gradient(y4(:,i), dt);
    dy5(:,i) = gradient(y5(:,i), dt);
    dy6(:,i) = gradient(y6(:,i), dt);
    dy7(:,i) = gradient(y7(:,i), dt);
    dy8(:,i) = gradient(y8(:,i), dt);
end
dY = [dy1; dy2; dy3; dy4; dy5; dy6; dy7; dy8;];

for i = 1:n
    dx1(:,i) = gradient(x1(:,i), dt);
    dx2(:,i) = gradient(x2(:,i), dt);
    dx3(:,i) = gradient(x3(:,i), dt);
    dx4(:,i) = gradient(x4(:,i), dt);
    dx5(:,i) = gradient(x5(:,i), dt);
    dx6(:,i) = gradient(x6(:,i), dt);
    dx7(:,i) = gradient(x7(:,i), dt);
    dx8(:,i) = gradient(x8(:,i), dt);
end

% Subtract signals to isolate discrepancy 
ef1 = dy1 - dx1;
ef2 = dy2 - dx2;
ef3 = dy3 - dx3;
ef4 = dy4 - dx4;
ef5 = dy5 - dx5;
ef6 = dy6 - dx6;
ef7 = dy7 - dx7;
ef8 = dy8 - dx8;
Ef = [ef1; ef2; ef3; ef4; ef5; ef6; ef7; ef8;];

%% get real vehicle test data

% Test1 
load('15mcircle3.mat')
startpoint_t1 = 2001;
endpoint_t1 = 5000;
qqd_debug = [q_debug qd_debug];
data_len_t1 = size(q_debug(startpoint_t1:endpoint_t1,:),1);
time_t1 = linspace(0,(data_len_t1-1)*0.01,data_len_t1)';
s1.qqd_t1 = qqd_debug(startpoint_t1:endpoint_t1, :);
s1.u_t1 = steer_m(startpoint_t1:endpoint_t1, :) * deg;
u_t1 = zeros(data_len_t1, 8);
u_t1(:, 1) = s1.u_t1(:,1);
u_t1(:, 2) = 0.7 * s1.u_t1(:,1);
u_t1(:, 3:6) = s1.u_t1(:,2:5);
u_t1(:, 7) = 0.7 * s1.u_t1(:,6);
u_t1(:, 8) = s1.u_t1(:,6);
qqd_t1 = s1.qqd_t1;
% low filter
qqd_tmp_t1 = [];
for i = 1:10
    qqd_tmp_t1(:,i) = butterfilt(s1.qqd_t1(:,i), cutoff, 4, 'Low', time_t1, 'off');
end
u_tmp_t1 = [];
for i = 1:8
    u_tmp_t1(:,i) = butterfilt(u_t1(:,i), cutoff, 4, 'Low', time_t1, 'off');
end
qqd_t1 = qqd_tmp_t1;
u_t1 = u_tmp_t1;

% Test2
load('25mrightangle7.mat')
startpoint_t2 = 2501;
endpoint_t2 = 5500;
qqd_debug = [q_debug qd_debug];
data_len_t2 = size(q_debug(startpoint_t2:endpoint_t2,:),1);
time_t2 = linspace(0,(data_len_t2-1)*0.01,data_len_t2)';
s1.qqd_t2 = qqd_debug(startpoint_t2:endpoint_t2, :);
s1.u_t2 = steer_m(startpoint_t2:endpoint_t2, :) * deg;
u_t2 = zeros(data_len_t2, 8);
u_t2(:, 1) = s1.u_t2(:,1);
u_t2(:, 2) = 0.7 * s1.u_t2(:,1);
u_t2(:, 3:6) = s1.u_t2(:,2:5);
u_t2(:, 7) = 0.7 * s1.u_t2(:,6);
u_t2(:, 8) = s1.u_t2(:,6);
qqd_t2 = s1.qqd_t2;
% low filter
qqd_tmp_t2 = [];
for i = 1:10
    qqd_tmp_t2(:,i) = butterfilt(s1.qqd_t2(:,i), cutoff, 4, 'Low', time_t2, 'off');
end
u_tmp_t2 = [];
for i = 1:8
    u_tmp_t2(:,i) = butterfilt(u_t2(:,i), cutoff, 4, 'Low', time_t2, 'off');
end
qqd_t2 = qqd_tmp_t2;
u_t2 = u_tmp_t2;

% Test3  
load('25mrightangle5.mat')
startpoint_t3 = 3001;
endpoint_t3 = 6000;
qqd_debug = [q_debug qd_debug];
data_len_t3 = size(q_debug(startpoint_t3:endpoint_t3,:),1);
time_t3 = linspace(0,(data_len_t3-1)*0.01,data_len_t3)';
s1.qqd_t3 = qqd_debug(startpoint_t3:endpoint_t3, :);
s1.u_t3 = steer_m(startpoint_t3:endpoint_t3, :) * deg;
u_t3 = zeros(data_len_t3, 8);
u_t3(:, 1) = s1.u_t3(:,1);
u_t3(:, 2) = 0.7 * s1.u_t3(:,1);
u_t3(:, 3:6) = s1.u_t3(:,2:5);
u_t3(:, 7) = 0.7 * s1.u_t3(:,6);
u_t3(:, 8) = s1.u_t3(:,6);
qqd_t3 = s1.qqd_t3;
% low filter
qqd_tmp_t3 = [];
for i = 1:10
    qqd_tmp_t3(:,i) = butterfilt(s1.qqd_t3(:,i), cutoff, 4, 'Low', time_t3, 'off');
end
u_tmp_t3 = [];
for i = 1:8
    u_tmp_t3(:,i) = butterfilt(u_t3(:,i), cutoff, 4, 'Low', time_t3, 'off');
end
qqd_t3 = qqd_tmp_t3;
u_t3 = u_tmp_t3;

% Test4  
load('15mrightangle4.mat')
startpoint_t4 = 4001;
endpoint_t4 = 7000;
qqd_debug = [q_debug qd_debug];
data_len_t4 = size(q_debug(startpoint_t4:endpoint_t4,:),1);
time_t4 = linspace(0,(data_len_t4-1)*0.01,data_len_t4)';
s1.qqd_t4 = qqd_debug(startpoint_t4:endpoint_t4, :);
s1.u_t4 = steer_m(startpoint_t4:endpoint_t4, :) * deg;
u_t4 = zeros(data_len_t4, 8);
u_t4(:, 1) = s1.u_t4(:,1);
u_t4(:, 2) = 0.7 * s1.u_t4(:,1);
u_t4(:, 3:6) = s1.u_t4(:,2:5);
u_t4(:, 7) = 0.7 * s1.u_t4(:,6);
u_t4(:, 8) = s1.u_t4(:,6);
qqd_t4 = s1.qqd_t4;
% low filter
qqd_tmp_t4 = [];
for i = 1:10
    qqd_tmp_t4(:,i) = butterfilt(s1.qqd_t4(:,i), cutoff, 4, 'Low', time_t4, 'off');
end
u_tmp_t4 = [];
for i = 1:8
    u_tmp_t4(:,i) = butterfilt(u_t4(:,i), cutoff, 4, 'Low', time_t4, 'off');
end
qqd_t4 = qqd_tmp_t4;
u_t4 = u_tmp_t4;


TestY = [qqd_t1; qqd_t2; qqd_t3; qqd_t4;];
TestU = [u_t1; u_t2; u_t3; u_t4;];
dt = time_t1(2) - time_t1(1);
TestT = [time_t1; time_t2+time_t1(end)+dt; time_t3+time_t1(end)+time_t2(end)+dt+dt; ...
         time_t4+time_t1(end)+time_t2(end)+time_t3(end)+dt+dt+dt;];
sepT = [1 endpoint_t1-startpoint_t1+1; ...
    endpoint_t1-startpoint_t1+2 endpoint_t1-startpoint_t1+2+endpoint_t2-startpoint_t2; ...
    endpoint_t1-startpoint_t1+3+endpoint_t2-startpoint_t2 ...
    endpoint_t1-startpoint_t1+3+endpoint_t2-startpoint_t2+endpoint_t3-startpoint_t3; ...
    endpoint_t1-startpoint_t1+4+endpoint_t2-startpoint_t2+endpoint_t3-startpoint_t3 ...
    endpoint_t1-startpoint_t1+4+endpoint_t2-startpoint_t2+endpoint_t3-startpoint_t3+endpoint_t4-startpoint_t4;];

Testx1 = Maaav_single(time_t1, qqd_t1, avp, u_t1, n);
Testx2 = Maaav_single(time_t2, qqd_t2, avp, u_t2, n);
Testx3 = Maaav_single(time_t3, qqd_t3, avp, u_t3, n);
Testx4 = Maaav_single(time_t4, qqd_t4, avp, u_t4, n);

TestX = [Testx1; Testx2; Testx3; Testx4;];
