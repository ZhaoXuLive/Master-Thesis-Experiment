 function out = Maaav_improved(x, avp, u, trq)
% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

% x: q + qd dim: 1*10
% u: steer  dim: 1*8
% avp: parameter (ca -- lateral stiffness, sat -- tire model saturation,
% they are open to discussion)
% avp.m = [11142; 7651; 11142;];
% avp.iz = [143570; 37800; 143570;];
% avp.lj = [11.4; 9.4; 11.4;];
% avp.la = [2.5; 4; 9.9; 1.5; 7.9; 1.5; 7.4; 8.9];
% avp.lg = [5.862; 4.643; 5.538;];
% avp.h = [0.9625; 0.9625; 0.98; 0.98; 0.98; 0.98; 0.9625; 0.9625;];
% avp.d = [10000; 10000];
% avp.ca = 20000;
% avp.sat = 15;

%% get state and input
q = x(1:5)';
qd = x(6:10)';
psi1 = x(3);
psi2 = x(4);
psi3 = x(5);
xd = x(6);
yd = x(7);
psi1d = x(8);
psi2d = x(9);
psi3d = x(10);

delta = u;

%% get vehicle parameter
m1 = avp.m(1);
m2 = avp.m(2);
m3 = avp.m(3);
iz1 = avp.iz(1);
iz2 = avp.iz(2);
iz3 = avp.iz(3);
l_j1j2 = avp.lj(1);
l_j2j3 = avp.lj(2);
l_j3j4 = avp.lj(3);
l_j1a1 = avp.la(1);
l_j1a2 = avp.la(2);
l_j1a3 = avp.la(3);
l_j2a4 = avp.la(4);
l_j2a5 = avp.la(5);
l_j3a6 = avp.la(6);
l_j3a7 = avp.la(7);
l_j3a8 = avp.la(8);
l_j1g1 = avp.lg(1);
l_j2g2 = avp.lg(2);
l_j3g3 = avp.lg(3);
h11 = avp.h(1);
h12 = avp.h(2);
h13 = avp.h(3);
h21 = avp.h(4);
h22 = avp.h(5);
h31 = avp.h(6);
h32 = avp.h(7);
h33 = avp.h(8);
hc1 = avp.hc(1);
hc2 = avp.hc(2);
hc3 = avp.hc(3);
d1 = avp.d(1);
d2 = avp.d(2);
ca_T = avp.ca;

%% matrix

% M
M = [m1 + m2 + m3, 0, (l_j1g1*m1 + l_j1j2*(m2 + m3))*sin(psi1), (l_j2g2*m2 + l_j2j3*m3)*sin(psi2), l_j3g3*m3*sin(psi3);
     0, m1 + m2 + m3, -(l_j1g1*m1 + l_j1j2*(m2 + m3))*cos(psi1), -(l_j2g2*m2 + l_j2j3*m3)*cos(psi2), -l_j3g3*m3*cos(psi3);
     (l_j1g1*m1 + l_j1j2*(m2 + m3))*sin(psi1), -(l_j1g1*m1 + l_j1j2*(m2 + m3))*cos(psi1), iz1+m1*(l_j1g1^2)+(m2+m3)*(l_j1j2^2), (l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*cos(psi1-psi2), l_j3g3*l_j1j2*m3*cos(psi1-psi3);
     (l_j2g2*m2 + l_j2j3*m3)*sin(psi2), -(l_j2g2*m2 + l_j2j3*m3)*cos(psi2), (l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*cos(psi1-psi2), iz2+m2*(l_j2g2^2)+m3*(l_j2j3^2), l_j3g3*l_j2j3*m3*cos(psi2-psi3);
     l_j3g3*m3*sin(psi3), -l_j3g3*m3*cos(psi3), l_j3g3*l_j1j2*m3*cos(psi1-psi3), l_j3g3*l_j2j3*m3*cos(psi2-psi3), iz3+m3*(l_j3g3^2);];

% C
C = [0, 0, (l_j1g1*m1 + l_j1j2*(m2 + m3))*cos(psi1)*psi1d, (l_j2g2*m2 + l_j2j3*m3)*cos(psi2)*psi2d, l_j3g3*m3*cos(psi3)*psi3d;
     0, 0, (l_j1g1*m1 + l_j1j2*(m2 + m3))*sin(psi1)*psi1d, (l_j2g2*m2 + l_j2j3*m3)*sin(psi2)*psi2d, l_j3g3*m3*sin(psi3)*psi3d;
     1/2*(l_j1g1*m1 + l_j1j2*(m2 + m3))*cos(psi1)*psi1d, 1/2*(l_j1g1*m1 + l_j1j2*(m2 + m3))*sin(psi1)*psi1d, -1/2*(l_j1g1*m1 + l_j1j2*(m2 + m3))*(xd*cos(psi1)+yd*sin(psi1))+1/2*(l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*sin(psi1-psi2)*psi2d+1/2*l_j3g3*l_j1j2*m3*sin(psi1-psi3)*psi3d, (l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*sin(psi1-psi2)*(-1/2*psi1d+psi2d), l_j3g3*l_j1j2*m3*sin(psi1-psi3)*(-1/2*psi1d+psi3d);
     1/2*(l_j2g2*m2 + l_j2j3*m3)*cos(psi2)*psi2d, 1/2*(l_j2g2*m2 + l_j2j3*m3)*sin(psi2)*psi2d, (l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*sin(psi1-psi2)*(-psi1d+1/2*psi2d), -1/2*(l_j2g2*m2+l_j2j3*m3)*(xd*cos(psi2)+yd*sin(psi2))-1/2*(l_j2g2*l_j1j2*m2+l_j1j2*l_j2j3*m3)*sin(psi1-psi2)*psi1d+1/2*l_j3g3*l_j2j3*m3*sin(psi2-psi3)*psi3d, l_j3g3*l_j2j3*m3*sin(psi2-psi3)*(-1/2*psi2d+psi3d);
     1/2*l_j3g3*m3*cos(psi3)*psi3d, 1/2*l_j3g3*m3*sin(psi3)*psi3d, l_j3g3*l_j1j2*m3*sin(psi1-psi3)*(-psi1d+1/2*psi3d), l_j3g3*l_j2j3*m3*sin(psi2-psi3)*(-psi2d+1/2*psi3d), -1/2*l_j3g3*m3*(xd*cos(psi3)+yd*sin(psi3))-1/2*l_j3g3*l_j1j2*m3*sin(psi1-psi3)*psi1d-1/2*l_j3g3*l_j2j3*m3*sin(psi2-psi3)*psi2d;];

% D
D = [0, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, d1, -d1, 0;
     0, 0, -d1, d1+d2, -d2;
     0, 0, 0, -d2, d2;];

% Q
psij = [psi1, psi2, psi3]';
Rot_w2v = zeros(2, 2, 3);
for i = 1:3
    Rot_w2v(:, :, i) = [cos(psij(i)), -sin(psij(i));
                        sin(psij(i)), cos(psij(i));];
end

Rot_v2t = zeros(2, 2, 8);
for i = 1:8
    Rot_v2t(:, :, i) = [cos(delta(i)), -sin(delta(i));
                        sin(delta(i)), cos(delta(i));];
end

u = zeros(2, 3);
n = zeros(2, 3);
for i = 1:3
    u(:, i)=[cos(q(i + 2)), sin(q(i + 2))]';
    n(:, i)=[-sin(q(i + 2)), cos(q(i + 2))]';
end

IU = [1, 1, 1, 2, 2, 3, 3, 3]';
l_ja = [l_j1a1, l_j1a2, l_j1a3, l_j2a4, l_j2a5, l_j3a6, l_j3a7, l_j3a8]';
ha = [h11, h12, h13, h21, h22, h31, h32, h33]';

L_long1 = [cos(psi1), cos(psi1), cos(psi1), cos(psi2), cos(psi2), cos(psi3), cos(psi3), cos(psi3);
           sin(psi1), sin(psi1), sin(psi1), sin(psi2), sin(psi2), sin(psi3), sin(psi3), sin(psi3);
           -h11, -h12, -h13, l_j1j2*sin(psi1-psi2), l_j1j2*sin(psi1-psi2), l_j1j2*sin(psi1-psi3), l_j1j2*sin(psi1-psi3), l_j1j2*sin(psi1-psi3);
           0, 0, 0, -h21, -h22, l_j2j3*sin(psi2-psi3), l_j2j3*sin(psi2-psi3), l_j2j3*sin(psi2-psi3);
           0, 0, 0, 0, 0, -h31, -h32, -h33;];

L_long2 = [cos(psi1), cos(psi1), cos(psi1), cos(psi2), cos(psi2), cos(psi3), cos(psi3), cos(psi3);
           sin(psi1), sin(psi1), sin(psi1), sin(psi2), sin(psi2), sin(psi3), sin(psi3), sin(psi3);
           h11, h12, h13, l_j1j2*sin(psi1-psi2), l_j1j2*sin(psi1-psi2), l_j1j2*sin(psi1-psi3), l_j1j2*sin(psi1-psi3), l_j1j2*sin(psi1-psi3);
           0, 0, 0, h21, h22, l_j2j3*sin(psi2-psi3), l_j2j3*sin(psi2-psi3), l_j2j3*sin(psi2-psi3);
           0, 0, 0, 0, 0, h31, h32, h33;];

L_lat1 = [-sin(psi1), -sin(psi1), -sin(psi1), -sin(psi2), -sin(psi2), -sin(psi3), -sin(psi3), -sin(psi3);
          cos(psi1), cos(psi1), cos(psi1), cos(psi2), cos(psi2), cos(psi3), cos(psi3), cos(psi3);
          -l_j1a1, -l_j1a2, -l_j1a3, -l_j1j2*cos(psi1-psi2), -l_j1j2*cos(psi1-psi2), -l_j1j2*cos(psi1-psi3), -l_j1j2*cos(psi1-psi3), -l_j1j2*cos(psi1-psi3);
          0, 0, 0, -l_j2a4, -l_j2a5, -l_j2j3*cos(psi2-psi3), -l_j2j3*cos(psi2-psi3), -l_j2j3*cos(psi2-psi3);
          0, 0, 0, 0, 0, -l_j3a6, -l_j3a7, -l_j3a8;];

L_lat2 = [-sin(psi1), -sin(psi1), -sin(psi1), -sin(psi2), -sin(psi2), -sin(psi3), -sin(psi3), -sin(psi3);
          cos(psi1), cos(psi1), cos(psi1), cos(psi2), cos(psi2), cos(psi3), cos(psi3), cos(psi3);
          -l_j1a1, -l_j1a2, -l_j1a3, -l_j1j2*cos(psi1-psi2), -l_j1j2*cos(psi1-psi2), -l_j1j2*cos(psi1-psi3), -l_j1j2*cos(psi1-psi3), -l_j1j2*cos(psi1-psi3);
          0, 0, 0, -l_j2a4, -l_j2a5, -l_j2j3*cos(psi2-psi3), -l_j2j3*cos(psi2-psi3), -l_j2j3*cos(psi2-psi3);
          0, 0, 0, 0, 0, -l_j3a6, -l_j3a7, -l_j3a8;];

Delta_c = zeros(8, 8);
Delta_s = zeros(8, 8);
for i = 1:8
    Delta_c(i, i) = cos(delta(i));
    Delta_s(i, i) = sin(delta(i));
end

vxj = zeros(3, 1);
vyj = zeros(3, 1);
vxj(1) = xd;
vyj(1) = yd;
vxj(2) = xd + l_j1j2*sin(psi1)*psi1d;
vyj(2) = yd - l_j1j2*cos(psi1)*psi1d;
vxj(3) = xd + l_j1j2*sin(psi1)*psi1d + l_j2j3*sin(psi2)*psi2d;
vyj(3) = yd - l_j1j2*cos(psi1)*psi1d - l_j2j3*cos(psi2)*psi2d;
va1 = zeros(2, 8);
va2 = zeros(2, 8);
psidj = [psi1d, psi2d, psi3d]';
for i = 1:8
    va1(:, i) = [vxj(IU(i)); vyj(IU(i))] - n(:, IU(i))*l_ja(i)*psidj(IU(i)) + (-1)*u(:, IU(i))*ha(i)*psidj(IU(i));
    va2(:, i) = [vxj(IU(i)); vyj(IU(i))] - n(:, IU(i))*l_ja(i)*psidj(IU(i)) + u(:, IU(i))*ha(i)*psidj(IU(i));
end

va1_v = zeros(2, 8);
va2_v = zeros(2, 8);
for i = 1:8
    va1_v(:, i) = inv(Rot_w2v(:, :, IU(i)))*va1(:, i);
    va2_v(:, i) = inv(Rot_w2v(:, :, IU(i)))*va2(:, i);
end

beta1 = zeros(8, 1);
beta2 = zeros(8, 1);
for i = 1:8
    beta1(i) = atan2(va1_v(2, i), va1_v(1, i));
    beta2(i) = atan2(va2_v(2, i), va2_v(1, i));
end

ca1 = ca_T;
ca2 = ca_T;
ca3 = ca_T;
ca4 = ca_T;
ca5 = ca_T;
ca6 = ca_T;
ca7 = ca_T;
ca8 = ca_T;
ca = [ca1, ca2, ca3, ca4, ca5, ca6, ca7, ca8]';
Ca = diag(ca);

alpha1 = delta' - beta1;
alpha2 = delta' - beta2;
sat = avp.sat;
for i = 1:8
    if abs(alpha1(i)) > sat * pi / 180
        alpha1(i) = sign(alpha1(i)) * sat * pi / 180;
    end
    if abs(alpha2(i)) > sat * pi / 180
        alpha2(i) = sign(alpha2(i)) * sat * pi / 180;
    end
end

FyL = (Ca * alpha1)';
FyR = (Ca * alpha2)';

% 计算质心速度
vxg = zeros(3, 1);
vyg = zeros(3, 1);
vxg(1) = xd + l_j1g1*sin(psi1)*psi1d;
vyg(1) = yd - l_j1g1*cos(psi1)*psi1d;
vxg(2) = xd + l_j1j2*sin(psi1)*psi1d + l_j2g2*sin(psi2)*psi2d;
vyg(2) = yd - l_j1j2*cos(psi1)*psi1d - l_j2g2*cos(psi2)*psi2d;
vxg(3) = xd + l_j1j2*sin(psi1)*psi1d + l_j2j3*sin(psi2)*psi2d + l_j3g3*sin(psi3)*psi3d;
vyg(3) = yd - l_j1j2*cos(psi1)*psi1d - l_j2j3*cos(psi2)*psi2d - l_j3g3*cos(psi3)*psi3d;
vxg1_v = vxg(1) * cos(psi1) + vyg(1) * sin(psi1);
vxg2_v = vxg(2) * cos(psi2) + vyg(2) * sin(psi2);
vxg3_v = vxg(3) * cos(psi3) + vyg(3) * sin(psi3);
vyg1_v = -vxg(1) * sin(psi1) + vyg(1) * cos(psi1);
vyg2_v = -vxg(2) * sin(psi2) + vyg(2) * cos(psi2);
vyg3_v = -vxg(3) * sin(psi3) + vyg(3) * cos(psi3);

% 计算横纵向加速度
a_v = (sum(trq, 2) / avp.KinRollRadius - 0.5 * avp.pho * (avp.Cd1 + avp.Cd2 + avp.Cd3) ...
      * (avp.A1 + avp.A2 + avp.A3) .* vxg1_v .* vxg1_v - ...
      avp.Crr * (avp.m(1) + avp.m(2) + avp.m(3)) * avp.g) / (avp.m(1) + avp.m(2) + avp.m(3));
ay = ((FyL(1) + FyR(1)) * cos(delta(1)) + (FyL(2) + FyR(2)) * cos(delta(2)) + (FyL(3) + FyR(3)) * cos(delta(3)) + ...
     (FyL(4) + FyR(4)) * cos(delta(4)) + (FyL(5) + FyR(5)) * cos(delta(5)) + (FyL(6) + FyR(6)) * cos(delta(6)) + ...
     (FyL(7) + FyR(7)) * cos(delta(7)) + (FyL(8) + FyR(8)) * cos(delta(8))) / (m1 + m2 + m3);

% Fz
FzL = zeros(1, 8);
FzR = zeros(1, 8);

% 静态Fz 可用TM静态为准
basicFz = zeros(1, 8);
% first vehicle w1:w2 = 1:1.04 ? 这个比例并不固定，可以假设为悬架刚度的比例，假设悬架压缩量相同
% 3.362 * w1 + 1.862 * w2 = 4.038 * w3
% w1 + w2 + w3 = m1g
% w2 = 1.02w1
basicFz(1) = m1 * avp.g * 0.32;
basicFz(2) = m1 * avp.g * 0.327;
basicFz(3) = m1 * avp.g * 0.353;
basicFz(4) = m2 * avp.g * 0.494;
basicFz(5) = m2 * avp.g * 0.506;
basicFz(6) = m3 * avp.g * 0.328;
basicFz(7) = m3 * avp.g * 0.335;
basicFz(8) = m3 * avp.g * 0.337;

% 纵向载荷转移
longitudinal_trans = zeros(1, 8);
longitudinal_trans(1) = -m1 * hc1 * a_v * 1.5 / (l_j1a3 - l_j1a1) / 2;
longitudinal_trans(2) = -m1 * hc1 * a_v * 0.8 / (l_j1a3 - l_j1a1) / 2;
longitudinal_trans(3) = m1 * hc1 * a_v * 2.1 / (l_j1a3 - l_j1a1) / 2;
longitudinal_trans(4) = -m2 * hc2 * a_v / (l_j2a5 - l_j2a4) / 2;
longitudinal_trans(5) = m2 * hc2 * a_v / (l_j2a5 - l_j2a4) / 2;
longitudinal_trans(6) = -m3 * hc3 * a_v * 2.1 / (l_j3a8 - l_j3a6) / 2;
longitudinal_trans(7) = m3 * hc3 * a_v * 0.8 / (l_j3a8 - l_j3a6) / 2;
longitudinal_trans(8) = m3 * hc3 * a_v * 1.5 / (l_j3a8 - l_j3a6) / 2;

FzL = (basicFz + longitudinal_trans) / 2;
FzR = (basicFz + longitudinal_trans) / 2;

% 横向载荷转移
lateral_trans = zeros(1, 8);
lateral_trans(1) = m1 * ay * hc1 * 0.7 / (2 * h11) / 2;
lateral_trans(2) = m1 * ay * hc1 * 0.7 / (2 * h12) / 2;
lateral_trans(3) = m1 * ay * hc1 * 0.7 / (2 * h13) / 2;
lateral_trans(4) = m2 * ay * hc2 * 1.1 / (2 * h21) / 2;
lateral_trans(5) = m2 * ay * hc2 * 1.1 / (2 * h22) / 2;
lateral_trans(6) = m3 * ay * hc3 * 0.7 / (2 * h31) / 2;
lateral_trans(7) = m3 * ay * hc3 * 0.7 / (2 * h32) / 2;
lateral_trans(8) = m3 * ay * hc3 * 0.7 / (2 * h33) / 2;

FzL = FzL - lateral_trans;
FzR = FzR + lateral_trans;

% 计算Fx
FxL = trq / avp.KinRollRadius / 2 - avp.Crr * FzL;
FxR = trq / avp.KinRollRadius / 2 - avp.Crr * FzR;

Tau_tire = L_long1 * (Delta_c * FxL' - Delta_s * Ca * alpha1) ...
         + L_long2 * (Delta_c * FxR' - Delta_s * Ca * alpha2) ...
         + L_lat1 * (Delta_s * FxL' + Delta_c * Ca * alpha1) ...
         + L_lat2 * (Delta_s * FxR' + Delta_c * Ca * alpha2);
     
Q = Tau_tire;
qdd_output = inv(M)*(Q - C * qd - D * qd);
out = qdd_output;

