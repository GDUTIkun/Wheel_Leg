clc;
clear;

% 机器人结构参数
global r d L Lm l mw mp M Iw Ip h w Im g 
syms L0
r=0.05; 
%理想摆杆，只用来传动，没有质量
L=L0/2; %摆杆重心到机体重心
Lm=L0/2; %摆杆重心到轮轴
l=0; 
mw=0.34311; 
mp=0.87958; 
M=3.99077; 
Iw=0.5 * mw * r^2; 
Ip=(1/12)*mp*L0^2; 
h = 0.125;   % height of the body (m)
w = 0.2;   % width of the body (m)
Im=(1/12) * M * (h^2 + w^2);
g=9.81;

%状态方程
global x2_fun xb2_fun theta2_fun phi2_fun Ja Jb
[theta2_fun,x2_fun,xb2_fun,phi2_fun,Ja,Jb] = state_equation();

%controlle
global Q R
Q=diag([500 100 10 20 300 60]);
R=diag([.01 0.02]);

LQR_controller();

%EKF
global x_hat0 x_hat_minus0 P0 n sampletime Q_EKF R1_EKF R2_EKF
x_hat0 = [0;0;0;0;0;0];
x_hat_minus0 = [0;0;0;0;0;0];
P0 = diag([.1 .1 .1 .1 .1 .1]);
n = size(x_hat0,1);
sampletime = 0.01;
Q_EKF = diag([1e-3 1e-3 1e-3 1e-3 1e-3 1e-3]);
R1_EKF = diag([1e-2 1e-2]);
R2_EKF = diag([1e-1 1e-1]);

%支持力解算
FN_estimation();
global FN_min;
FN_min = 0.3;