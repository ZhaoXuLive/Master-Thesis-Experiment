%% get simulation vehicle train data

% load data
addpath('D:\TruckMaker\simulation\dataProcess\DataRecord')

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
avp.ca = 184000;
avp.sat = 15;
avp.hc = [1.855; 1.719; 1.855];

% longtidinual model parameter
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

% 15m circle
load('15m3mscircle.mat')
startpoint1 = 3001;
endpoint1 = 73000;
data_len1 = size(Expr_state(startpoint1:10:endpoint1, :), 1);
time1 = Expr_time(startpoint1:10:endpoint1, :);
y1 = Expr_state(startpoint1:10:endpoint1, :);
u1 = Expr_steer(startpoint1:10:endpoint1, :);
trq1 = (Expr_drive(startpoint1:10:endpoint1, :) + Expr_brake(startpoint1:10:endpoint1, :));
pitch1 = Expr_Pitch(startpoint1:10:endpoint1, :);
pitchrate1 = Expr_PitchRate(startpoint1:10:endpoint1, :);
roll1 = Expr_Roll(startpoint1:10:endpoint1, :);
rollrate1 = Expr_RollRate(startpoint1:10:endpoint1, :);
wheelspdL1 = Expr_wheelSpdL(startpoint1:10:endpoint1, :);
wheelspdR1 = Expr_wheelSpdR(startpoint1:10:endpoint1, :);

load('15m5mscircle.mat')
startpoint2 = 3001;
endpoint2 = 48000;
data_len2 = size(Expr_state(startpoint2:10:endpoint2, :), 1);
time2 = Expr_time(startpoint2:10:endpoint2, :);
y2 = Expr_state(startpoint2:10:endpoint2, :);
u2 = Expr_steer(startpoint2:10:endpoint2, :);
trq2 = (Expr_drive(startpoint2:10:endpoint2, :) + Expr_brake(startpoint2:10:endpoint2, :));
pitch2 = Expr_Pitch(startpoint2:10:endpoint2, :);
pitchrate2 = Expr_PitchRate(startpoint2:10:endpoint2, :);
roll2 = Expr_Roll(startpoint2:10:endpoint2, :);
rollrate2 = Expr_RollRate(startpoint2:10:endpoint2, :);
wheelspdL2 = Expr_wheelSpdL(startpoint2:10:endpoint2, :);
wheelspdR2 = Expr_wheelSpdR(startpoint2:10:endpoint2, :);

load('15m7mscircle.mat')
startpoint3 = 3001;
endpoint3 = 38000;
data_len3 = size(Expr_state(startpoint3:10:endpoint3, :), 1);
time3 = Expr_time(startpoint3:10:endpoint3, :);
y3 = Expr_state(startpoint3:10:endpoint3, :);
u3 = Expr_steer(startpoint3:10:endpoint3, :);
trq3 = (Expr_drive(startpoint3:10:endpoint3, :) + Expr_brake(startpoint3:10:endpoint3, :));
pitch3 = Expr_Pitch(startpoint3:10:endpoint3, :);
pitchrate3 = Expr_PitchRate(startpoint3:10:endpoint3, :);
roll3 = Expr_Roll(startpoint3:10:endpoint3, :);
rollrate3 = Expr_RollRate(startpoint3:10:endpoint3, :);
wheelspdL3 = Expr_wheelSpdL(startpoint3:10:endpoint3, :);
wheelspdR3 = Expr_wheelSpdR(startpoint3:10:endpoint3, :);

% 20m circle
load('20m3mscircle.mat')
startpoint4 = 3001;
endpoint4 = 83000;
data_len4 = size(Expr_state(startpoint4:10:endpoint4, :), 1);
time4 = Expr_time(startpoint4:10:endpoint4, :);
y4 = Expr_state(startpoint4:10:endpoint4, :);
u4 = Expr_steer(startpoint4:10:endpoint4, :);
trq4 = (Expr_drive(startpoint4:10:endpoint4, :) + Expr_brake(startpoint4:10:endpoint4, :));
pitch4 = Expr_Pitch(startpoint4:10:endpoint4, :);
pitchrate4 = Expr_PitchRate(startpoint4:10:endpoint4, :);
roll4 = Expr_Roll(startpoint4:10:endpoint4, :);
rollrate4 = Expr_RollRate(startpoint4:10:endpoint4, :);
wheelspdL4 = Expr_wheelSpdL(startpoint4:10:endpoint4, :);
wheelspdR4 = Expr_wheelSpdR(startpoint4:10:endpoint4, :);

load('20m5mscircle.mat')
startpoint5 = 3001;
endpoint5 = 53000;
data_len5 = size(Expr_state(startpoint5:10:endpoint5, :), 1);
time5 = Expr_time(startpoint5:10:endpoint5, :);
y5 = Expr_state(startpoint5:10:endpoint5, :);
u5 = Expr_steer(startpoint5:10:endpoint5, :);
trq5 = (Expr_drive(startpoint5:10:endpoint5, :) + Expr_brake(startpoint5:10:endpoint5, :));
pitch5 = Expr_Pitch(startpoint5:10:endpoint5, :);
pitchrate5 = Expr_PitchRate(startpoint5:10:endpoint5, :);
roll5 = Expr_Roll(startpoint5:10:endpoint5, :);
rollrate5 = Expr_RollRate(startpoint5:10:endpoint5, :);
wheelspdL5 = Expr_wheelSpdL(startpoint5:10:endpoint5, :);
wheelspdR5 = Expr_wheelSpdR(startpoint5:10:endpoint5, :);

load('20m7mscircle.mat')
startpoint6 = 3001;
endpoint6 = 43000;
data_len6 = size(Expr_state(startpoint6:10:endpoint6, :), 1);
time6 = Expr_time(startpoint6:10:endpoint6, :);
y6 = Expr_state(startpoint6:10:endpoint6, :);
u6 = Expr_steer(startpoint6:10:endpoint6, :);
trq6 = (Expr_drive(startpoint6:10:endpoint6, :) + Expr_brake(startpoint6:10:endpoint6, :));
pitch6 = Expr_Pitch(startpoint6:10:endpoint6, :);
pitchrate6 = Expr_PitchRate(startpoint6:10:endpoint6, :);
roll6 = Expr_Roll(startpoint6:10:endpoint6, :);
rollrate6 = Expr_RollRate(startpoint6:10:endpoint6, :);
wheelspdL6 = Expr_wheelSpdL(startpoint6:10:endpoint6, :);
wheelspdR6 = Expr_wheelSpdR(startpoint6:10:endpoint6, :);

% 30m circle
load('30m5mscircle.mat')
startpoint7 = 3001;
endpoint7 = 68000;
data_len7 = size(Expr_state(startpoint7:10:endpoint7, :), 1);
time7 = Expr_time(startpoint7:10:endpoint7, :);
y7 = Expr_state(startpoint7:10:endpoint7, :);
u7 = Expr_steer(startpoint7:10:endpoint7, :);
trq7 = (Expr_drive(startpoint7:10:endpoint7, :) + Expr_brake(startpoint7:10:endpoint7, :));
pitch7 = Expr_Pitch(startpoint7:10:endpoint7, :);
pitchrate7 = Expr_PitchRate(startpoint7:10:endpoint7, :);
roll7 = Expr_Roll(startpoint7:10:endpoint7, :);
rollrate7 = Expr_RollRate(startpoint7:10:endpoint7, :);
wheelspdL7 = Expr_wheelSpdL(startpoint7:10:endpoint7, :);
wheelspdR7 = Expr_wheelSpdR(startpoint7:10:endpoint7, :);

load('30m7mscircle.mat')
startpoint8 = 3001;
endpoint8 = 53000;
data_len8 = size(Expr_state(startpoint8:10:endpoint8, :), 1);
time8 = Expr_time(startpoint8:10:endpoint8, :);
y8 = Expr_state(startpoint8:10:endpoint8, :);
u8 = Expr_steer(startpoint8:10:endpoint8, :);
trq8 = (Expr_drive(startpoint8:10:endpoint8, :) + Expr_brake(startpoint8:10:endpoint8, :));
pitch8 = Expr_Pitch(startpoint8:10:endpoint8, :);
pitchrate8 = Expr_PitchRate(startpoint8:10:endpoint8, :);
roll8 = Expr_Roll(startpoint8:10:endpoint8, :);
rollrate8 = Expr_RollRate(startpoint8:10:endpoint8, :);
wheelspdL8 = Expr_wheelSpdL(startpoint8:10:endpoint8, :);
wheelspdR8 = Expr_wheelSpdR(startpoint8:10:endpoint8, :);

load('30m9mscircle.mat')
startpoint9 = 3001;
endpoint9 = 43000;
data_len9 = size(Expr_state(startpoint9:10:endpoint9, :), 1);
time9 = Expr_time(startpoint9:10:endpoint9, :);
y9 = Expr_state(startpoint9:10:endpoint9, :);
u9 = Expr_steer(startpoint9:10:endpoint9, :);
trq9 = (Expr_drive(startpoint9:10:endpoint9, :) + Expr_brake(startpoint9:10:endpoint9, :));
pitch9 = Expr_Pitch(startpoint9:10:endpoint9, :);
pitchrate9 = Expr_PitchRate(startpoint9:10:endpoint9, :);
roll9 = Expr_Roll(startpoint9:10:endpoint9, :);
rollrate9 = Expr_RollRate(startpoint9:10:endpoint9, :);
wheelspdL9 = Expr_wheelSpdL(startpoint9:10:endpoint9, :);
wheelspdR9 = Expr_wheelSpdR(startpoint9:10:endpoint9, :);

% 40m circle
load('40m5mscircle.mat')
startpoint10 = 3001;
endpoint10 = 78000;
data_len10 = size(Expr_state(startpoint10:10:endpoint10, :), 1);
time10 = Expr_time(startpoint10:10:endpoint10, :);
y10 = Expr_state(startpoint10:10:endpoint10, :);
u10 = Expr_steer(startpoint10:10:endpoint10, :);
trq10 = (Expr_drive(startpoint10:10:endpoint10, :) + Expr_brake(startpoint10:10:endpoint10, :));
pitch10 = Expr_Pitch(startpoint10:10:endpoint10, :);
pitchrate10 = Expr_PitchRate(startpoint10:10:endpoint10, :);
roll10 = Expr_Roll(startpoint10:10:endpoint10, :);
rollrate10 = Expr_RollRate(startpoint10:10:endpoint10, :);
wheelspdL10 = Expr_wheelSpdL(startpoint10:10:endpoint10, :);
wheelspdR10 = Expr_wheelSpdR(startpoint10:10:endpoint10, :);

load('40m7mscircle.mat')
startpoint11 = 3001;
endpoint11 = 63000;
data_len11 = size(Expr_state(startpoint11:10:endpoint11, :), 1);
time11 = Expr_time(startpoint11:10:endpoint11, :);
y11 = Expr_state(startpoint11:10:endpoint11, :);
u11 = Expr_steer(startpoint11:10:endpoint11, :);
trq11 = (Expr_drive(startpoint11:10:endpoint11, :) + Expr_brake(startpoint11:10:endpoint11, :));
pitch11 = Expr_Pitch(startpoint11:10:endpoint11, :);
pitchrate11 = Expr_PitchRate(startpoint11:10:endpoint11, :);
roll11 = Expr_Roll(startpoint11:10:endpoint11, :);
rollrate11 = Expr_RollRate(startpoint11:10:endpoint11, :);
wheelspdL11 = Expr_wheelSpdL(startpoint11:10:endpoint11, :);
wheelspdR11 = Expr_wheelSpdR(startpoint11:10:endpoint11, :);

load('40m9mscircle.mat')
startpoint12 = 3001;
endpoint12 = 53000;
data_len12 = size(Expr_state(startpoint12:10:endpoint12, :), 1);
time12 = Expr_time(startpoint12:10:endpoint12, :);
y12 = Expr_state(startpoint12:10:endpoint12, :);
u12 = Expr_steer(startpoint12:10:endpoint12, :);
trq12 = (Expr_drive(startpoint12:10:endpoint12, :) + Expr_brake(startpoint12:10:endpoint12, :));
pitch12 = Expr_Pitch(startpoint12:10:endpoint12, :);
pitchrate12 = Expr_PitchRate(startpoint12:10:endpoint12, :);
roll12 = Expr_Roll(startpoint12:10:endpoint12, :);
rollrate12 = Expr_RollRate(startpoint12:10:endpoint12, :);
wheelspdL12 = Expr_wheelSpdL(startpoint12:10:endpoint12, :);
wheelspdR12 = Expr_wheelSpdR(startpoint12:10:endpoint12, :);

% 50m circle
load('50m7mscircle.mat')
startpoint13 = 3001;
endpoint13 = 73000;
data_len13 = size(Expr_state(startpoint13:10:endpoint13, :), 1);
time13 = Expr_time(startpoint13:10:endpoint13, :);
y13 = Expr_state(startpoint13:10:endpoint13, :);
u13 = Expr_steer(startpoint13:10:endpoint13, :);
trq13 = (Expr_drive(startpoint13:10:endpoint13, :) + Expr_brake(startpoint13:10:endpoint13, :));
pitch13 = Expr_Pitch(startpoint13:10:endpoint13, :);
pitchrate13 = Expr_PitchRate(startpoint13:10:endpoint13, :);
roll13 = Expr_Roll(startpoint13:10:endpoint13, :);
rollrate13 = Expr_RollRate(startpoint13:10:endpoint13, :);
wheelspdL13 = Expr_wheelSpdL(startpoint13:10:endpoint13, :);
wheelspdR13 = Expr_wheelSpdR(startpoint13:10:endpoint13, :);

load('50m9mscircle.mat')
startpoint14 = 3001;
endpoint14 = 58000;
data_len14 = size(Expr_state(startpoint14:10:endpoint14, :), 1);
time14 = Expr_time(startpoint14:10:endpoint14, :);
y14 = Expr_state(startpoint14:10:endpoint14, :);
u14 = Expr_steer(startpoint14:10:endpoint14, :);
trq14 = (Expr_drive(startpoint14:10:endpoint14, :) + Expr_brake(startpoint14:10:endpoint14, :));
pitch14 = Expr_Pitch(startpoint14:10:endpoint14, :);
pitchrate14 = Expr_PitchRate(startpoint14:10:endpoint14, :);
roll14 = Expr_Roll(startpoint14:10:endpoint14, :);
rollrate14 = Expr_RollRate(startpoint14:10:endpoint14, :);
wheelspdL14 = Expr_wheelSpdL(startpoint14:10:endpoint14, :);
wheelspdR14 = Expr_wheelSpdR(startpoint14:10:endpoint14, :);

load('50m11mscircle.mat')
startpoint15 = 3001;
endpoint15 = 48000;
data_len15 = size(Expr_state(startpoint15:10:endpoint15, :), 1);
time15 = Expr_time(startpoint15:10:endpoint15, :);
y15 = Expr_state(startpoint15:10:endpoint15, :);
u15 = Expr_steer(startpoint15:10:endpoint15, :);
trq15 = (Expr_drive(startpoint15:10:endpoint15, :) + Expr_brake(startpoint15:10:endpoint15, :));
pitch15 = Expr_Pitch(startpoint15:10:endpoint15, :);
pitchrate15 = Expr_PitchRate(startpoint15:10:endpoint15, :);
roll15 = Expr_Roll(startpoint15:10:endpoint15, :);
rollrate15 = Expr_RollRate(startpoint15:10:endpoint15, :);
wheelspdL15 = Expr_wheelSpdL(startpoint15:10:endpoint15, :);
wheelspdR15 = Expr_wheelSpdR(startpoint15:10:endpoint15, :);

Y = [y1; y2; y3; y4; y5; y6; y7; y8; y9; y10; y11; y12; y13; y14; y15;];
U = [u1; u2; u3; u4; u5; u6; u7; u8; u9; u10; u11; u12; u13; u14; u15;];
T = [time1; time2; time3; time4; time5; time6; time7; time8; time9; ...
     time10; time11; time12; time13; time14; time15;];
Trq = [trq1; trq2; trq3; trq4; trq5; trq6; trq7; trq8; trq9; ...
       trq10; trq11; trq12; trq13; trq14; trq15;];
Pitch = [pitch1; pitch2; pitch3; pitch4; pitch5; pitch6; pitch7; pitch8; pitch9; ...
         pitch10; pitch11; pitch12; pitch13; pitch14; pitch15;];
PitchRate = [pitchrate1; pitchrate2; pitchrate3; pitchrate4; pitchrate5; pitchrate6; ...
             pitchrate7; pitchrate8; pitchrate9; pitchrate10; pitchrate11; pitchrate12; ...
             pitchrate13; pitchrate14; pitchrate15;];
Roll = [roll1; roll2; roll3; roll4; roll5; roll6; roll7; roll8; roll9; ...
        roll10; roll11; roll12; roll13; roll14; roll15;];
RollRate = [rollrate1; rollrate2; rollrate3; rollrate4; rollrate5; rollrate6; ...
            rollrate7; rollrate8; rollrate9; rollrate10; rollrate11; rollrate12; ...
            rollrate13; rollrate14; rollrate15;];
WheelSpdL = [wheelspdL1; wheelspdL2; wheelspdL3; wheelspdL4; wheelspdL5; wheelspdL6; ...
             wheelspdL7; wheelspdL8; wheelspdL9; wheelspdL10; wheelspdL11; wheelspdL12; ...
             wheelspdL13; wheelspdL14; wheelspdL15;];
WheelSpdR = [wheelspdR1; wheelspdR2; wheelspdR3; wheelspdR4; wheelspdR5; wheelspdR6; ...
             wheelspdR7; wheelspdR8; wheelspdR9; wheelspdR10; wheelspdR11; wheelspdR12; ...
             wheelspdR13; wheelspdR14; wheelspdR15;];
data_len_all = data_len1 + data_len2 + data_len3 + data_len4 + data_len5 + ...
               data_len6 + data_len7 + data_len8 + data_len9 + data_len10 + ...
               data_len11 + data_len12 + data_len13 + data_len14 + data_len15;

% 记录起始点
sep = [1 7000; 7001 11500; 11501 15000; 15001 23000; 23001 28000; ...
       28001 32000; 32001 38500; 38501 43500; 43501 47500; 47501 55000; ...
       55001 61000; 61001 66000; 66001 73000; 73001 78500; 78501 83000];

% Multi acticulated all axle vehicle system, lagrange

x1 = Maaav_improved_single(time1, y1, avp, u1, trq1, n);
x2 = Maaav_improved_single(time2, y2, avp, u2, trq2, n);
x3 = Maaav_improved_single(time3, y3, avp, u3, trq3, n);
x4 = Maaav_improved_single(time4, y4, avp, u4, trq4, n);
x5 = Maaav_improved_single(time5, y5, avp, u5, trq5, n);
x6 = Maaav_improved_single(time6, y6, avp, u6, trq6, n);
x7 = Maaav_improved_single(time7, y7, avp, u7, trq7, n);
x8 = Maaav_improved_single(time8, y8, avp, u8, trq8, n);
x9 = Maaav_improved_single(time9, y9, avp, u9, trq9, n);
x10 = Maaav_improved_single(time10, y10, avp, u10, trq10, n);
x11 = Maaav_improved_single(time11, y11, avp, u11, trq11, n);
x12 = Maaav_improved_single(time12, y12, avp, u12, trq12, n);
x13 = Maaav_improved_single(time13, y13, avp, u13, trq13, n);
x14 = Maaav_improved_single(time14, y14, avp, u14, trq14, n);
x15 = Maaav_improved_single(time15, y15, avp, u15, trq15, n);
X = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12; x13; x14; x15;];

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
    dy9(:,i) = gradient(y9(:,i), dt);
    dy10(:,i) = gradient(y10(:,i), dt);
    dy11(:,i) = gradient(y11(:,i), dt);
    dy12(:,i) = gradient(y12(:,i), dt);
    dy13(:,i) = gradient(y13(:,i), dt);
    dy14(:,i) = gradient(y14(:,i), dt);
    dy15(:,i) = gradient(y15(:,i), dt);
end
dY = [dy1; dy2; dy3; dy4; dy5; dy6; dy7; dy8; dy9; dy10; dy11; dy12; dy13; dy14; dy15;];

for i = 1:n
    dx1(:,i) = gradient(x1(:,i), dt);
    dx2(:,i) = gradient(x2(:,i), dt);
    dx3(:,i) = gradient(x3(:,i), dt);
    dx4(:,i) = gradient(x4(:,i), dt);
    dx5(:,i) = gradient(x5(:,i), dt);
    dx6(:,i) = gradient(x6(:,i), dt);
    dx7(:,i) = gradient(x7(:,i), dt);
    dx8(:,i) = gradient(x8(:,i), dt);
    dx9(:,i) = gradient(x9(:,i), dt);
    dx10(:,i) = gradient(x10(:,i), dt);
    dx11(:,i) = gradient(x11(:,i), dt);
    dx12(:,i) = gradient(x12(:,i), dt);
    dx13(:,i) = gradient(x13(:,i), dt);
    dx14(:,i) = gradient(x14(:,i), dt);
    dx15(:,i) = gradient(x15(:,i), dt);
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
ef9 = dy9 - dx9;
ef10 = dy10 - dx10;
ef11 = dy11 - dx11;
ef12 = dy12 - dx12;
ef13 = dy13 - dx13;
ef14 = dy14 - dx14;
ef15 = dy15 - dx15;
Ef = [ef1; ef2; ef3; ef4; ef5; ef6; ef7; ef8; ef9; ef10; ef11; ef12; ef13; ef14; ef15;];

%% get real vehicle test data

load('25m6mscircle.mat')
startpoint_t1 = 3001;
endpoint_t1 = 33000;
data_len_t1 = size(Expr_state(startpoint_t1:10:endpoint_t1, :), 1);
time_t1 = Expr_time(startpoint_t1:10:endpoint_t1, :);
y_t1 = Expr_state(startpoint_t1:10:endpoint_t1, :);
u_t1 = Expr_steer(startpoint_t1:10:endpoint_t1, :);
trq_t1 = (Expr_drive(startpoint_t1:10:endpoint_t1, :) + Expr_brake(startpoint_t1:10:endpoint_t1, :));
pitch_t1 = Expr_Pitch(startpoint_t1:10:endpoint_t1, :);
pitchrate_t1 = Expr_PitchRate(startpoint_t1:10:endpoint_t1, :);
roll_t1 = Expr_Roll(startpoint_t1:10:endpoint_t1, :);
rollrate_t1 = Expr_RollRate(startpoint_t1:10:endpoint_t1, :);
wheelspdL_t1 = Expr_wheelSpdL(startpoint_t1:10:endpoint_t1, :);
wheelspdR_t1 = Expr_wheelSpdR(startpoint_t1:10:endpoint_t1, :);

load('35m8mscircle.mat')
startpoint_t2 = 3001;
endpoint_t2 = 33000;
data_len_t2 = size(Expr_state(startpoint_t2:10:endpoint_t2, :), 1);
time_t2 = Expr_time(startpoint_t2:10:endpoint_t2, :);
y_t2 = Expr_state(startpoint_t2:10:endpoint_t2, :);
u_t2 = Expr_steer(startpoint_t2:10:endpoint_t2, :);
trq_t2 = (Expr_drive(startpoint_t2:10:endpoint_t2, :) + Expr_brake(startpoint_t2:10:endpoint_t2, :));
pitch_t2 = Expr_Pitch(startpoint_t2:10:endpoint_t2, :);
pitchrate_t2 = Expr_PitchRate(startpoint_t2:10:endpoint_t2, :);
roll_t2 = Expr_Roll(startpoint_t2:10:endpoint_t2, :);
rollrate_t2 = Expr_RollRate(startpoint_t2:10:endpoint_t2, :);
wheelspdL_t2 = Expr_wheelSpdL(startpoint_t2:10:endpoint_t2, :);
wheelspdR_t2 = Expr_wheelSpdR(startpoint_t2:10:endpoint_t2, :);

load('45m10mscircle.mat')
startpoint_t3 = 3001;
endpoint_t3 = 33000;
data_len_t3 = size(Expr_state(startpoint_t3:10:endpoint_t3, :), 1);
time_t3 = Expr_time(startpoint_t3:10:endpoint_t3, :);
y_t3 = Expr_state(startpoint_t3:10:endpoint_t3, :);
u_t3 = Expr_steer(startpoint_t3:10:endpoint_t3, :);
trq_t3 = (Expr_drive(startpoint_t3:10:endpoint_t3, :) + Expr_brake(startpoint_t3:10:endpoint_t3, :));
pitch_t3 = Expr_Pitch(startpoint_t3:10:endpoint_t3, :);
pitchrate_t3 = Expr_PitchRate(startpoint_t3:10:endpoint_t3, :);
roll_t3 = Expr_Roll(startpoint_t3:10:endpoint_t3, :);
rollrate_t3 = Expr_RollRate(startpoint_t3:10:endpoint_t3, :);
wheelspdL_t3 = Expr_wheelSpdL(startpoint_t3:10:endpoint_t3, :);
wheelspdR_t3 = Expr_wheelSpdR(startpoint_t3:10:endpoint_t3, :);

% load('75m12mscircle.mat')
% startpoint_t4 = 3001;
% endpoint_t4 = 61000;
% data_len_t4 = size(Expr_state(startpoint_t4:10:endpoint_t4, :), 1);
% time_t4 = Expr_time(startpoint_t4:10:endpoint_t4, :);
% y_t4 = Expr_state(startpoint_t4:10:endpoint_t4, :);
% u_t4 = Expr_steer(startpoint_t4:10:endpoint_t4, :);
% trq_t4 = (Expr_drive(startpoint_t4:10:endpoint_t4, :) + Expr_brake(startpoint_t4:10:endpoint_t4, :));
% pitch_t4 = Expr_Pitch(startpoint_t4:10:endpoint_t4, :);
% pitchrate_t4 = Expr_PitchRate(startpoint_t4:10:endpoint_t4, :);
% roll_t4 = Expr_Roll(startpoint_t4:10:endpoint_t4, :);
% rollrate_t4 = Expr_RollRate(startpoint_t4:10:endpoint_t4, :);
% wheelspdL_t4 = Expr_wheelSpdL(startpoint_t4:10:endpoint_t4, :);
% wheelspdR_t4 = Expr_wheelSpdR(startpoint_t4:10:endpoint_t4, :);
% 
% load('100m12mscircle.mat')
% startpoint_t5 = 3001;
% endpoint_t5 = 73000;
% data_len_t5 = size(Expr_state(startpoint_t5:10:endpoint_t5, :), 1);
% time_t5 = Expr_time(startpoint_t5:10:endpoint_t5, :);
% y_t5 = Expr_state(startpoint_t5:10:endpoint_t5, :);
% u_t5 = Expr_steer(startpoint_t5:10:endpoint_t5, :);
% trq_t5 = (Expr_drive(startpoint_t5:10:endpoint_t5, :) + Expr_brake(startpoint_t5:10:endpoint_t5, :));
% pitch_t5 = Expr_Pitch(startpoint_t5:10:endpoint_t5, :);
% pitchrate_t5 = Expr_PitchRate(startpoint_t5:10:endpoint_t5, :);
% roll_t5 = Expr_Roll(startpoint_t5:10:endpoint_t5, :);
% rollrate_t5 = Expr_RollRate(startpoint_t5:10:endpoint_t5, :);
% wheelspdL_t5 = Expr_wheelSpdL(startpoint_t5:10:endpoint_t5, :);
% wheelspdR_t5 = Expr_wheelSpdR(startpoint_t5:10:endpoint_t5, :);

TestY = [y_t1; y_t2; y_t3;];% y_t4; y_t5;];
TestU = [u_t1; u_t2; u_t3;];% u_t4; u_t5;];
TestTrq = [trq_t1; trq_t2; trq_t3;];% trq_t4; trq_t5;];
TestPitch = [pitch_t1; pitch_t2; pitch_t3;];% pitch_t4; pitch_t5;];
TestPitchRate = [pitchrate_t1; pitchrate_t2; pitchrate_t3;];% pitchrate_t4; pitchrate_t5;];
TestRoll = [roll_t1; roll_t2; roll_t3;];% roll_t4; roll_t5;];
TestRollRate = [rollrate_t1; rollrate_t2; rollrate_t3;];% rollrate_t4; rollrate_t5;];
TestWheelSpdL = [wheelspdL_t1; wheelspdL_t2; wheelspdL_t3;];% wheelspdL_t4; wheelspdL_t5;];
TestWheelSpdR = [wheelspdR_t1; wheelspdR_t2; wheelspdR_t3;];% wheelspdR_t4; wheelspdR_t5;];
TestT = [time_t1; time_t2; time_t3;];% time_t4; time_t5;];

sepT = [1 3000; 3001 6000; 6001 9000;];% 14801 20600; 20601 27600];
% [1 5000; 5001 10000; 10001 14800;];% 14801 20600; 20601 27600];


% load('test_varV.mat')
% % load('test_fixV30.mat')
% startpoint_t = 3001;
% endpoint_t = 188000;
% % endpoint_t = 223000;
% data_len_t = size(Expr_state(startpoint_t:10:endpoint_t, :), 1);
% time_t = Expr_time(startpoint_t:10:endpoint_t, :);
% y_t = Expr_state(startpoint_t:10:endpoint_t, :);
% u_t = Expr_steer(startpoint_t:10:endpoint_t, :);
% trq_t = (Expr_drive(startpoint_t:10:endpoint_t, :));% + Expr_brake(startpoint_t:10:endpoint_t, :));
% pitch_t = Expr_Pitch(startpoint_t:10:endpoint_t, :);
% pitchrate_t = Expr_PitchRate(startpoint_t:10:endpoint_t, :);
% roll_t = Expr_Roll(startpoint_t:10:endpoint_t, :);
% rollrate_t = Expr_RollRate(startpoint_t:10:endpoint_t, :);
% wheelspdL_t = Expr_wheelSpdL(startpoint_t:10:endpoint_t, :);
% wheelspdR_t = Expr_wheelSpdR(startpoint_t:10:endpoint_t, :);
% 
% TestY = y_t;
% TestU = u_t;
% TestTrq = trq_t;
% TestPitch = pitch_t;
% TestPitchRate = pitchrate_t;
% TestRoll = roll_t;
% TestRollRate = rollrate_t;
% TestWheelSpdL = wheelspdL_t;
% TestWheelSpdR = wheelspdR_t;
% TestT = time_t;
% 
% sepT = [1 data_len_t];
% 
% Testx = Maaav_improved_single(time_t, y_t, avp, u_t, trq_t, n);
% TestX = Testx;
