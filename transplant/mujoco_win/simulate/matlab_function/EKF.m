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


function x_estimate = EKF(acc_x, z2, F, L)

global sampletime Q_EKF R1_EKF R2_EKF Ja 
global  n x_hat_minus x_hat P
syms theta theta1 x x1 phi phi1 L0 T Tp


%%%%%%%%%%%%%%%%%扩展卡尔曼滤波器%%%%%%%%%%%%%%%%%
z1 = zeros(2,1);
z1(2) = x_hat(4,1) + acc_x*sampletime;
z1(1) = x_hat(3,1) + z1(2)*sampletime;

% 计算先验状态估计
dx = Dynamics(x_hat, F, L);
x_hat_minus = x_hat + sampletime*dx;
% x_hat_minus = rk4_step(@(X) Dynamics(X, F, L0), x_hat, sampletime);

% 计算更新雅可比矩阵
A = double(subs(Ja,[theta theta1 x x1  phi phi1 L0 T Tp],[x_hat.' L F.']));
W = eye(6);
H_m1 = [0 0 1 0 0 0;
        0 0 0 1 0 0];
V1 = eye(2);
H_m2 = [0 0 1 0 0 0;
        0 0 0 1 0 0];
V2 = eye(2);

%% 计算先验估计误差协方差矩阵
P_minus = A*P*transpose(A) + transpose(W)*Q_EKF*W;

%第一次融合
%门限判断
gate_threshold = 4; %chi2inv(0.90,2)

S1 = H_m1*P_minus*H_m1.' + V1*R1_EKF*V1.';%测量预测协方差
inov1 = z1 - H_m1*x_hat_minus;%创新
d2_1   = inov1.'*inv(S1)*inov1;%马氏距离平方
if d2_1 <= gate_threshold
    %更新
    K =  P_minus*transpose(H_m1)/S1;
    x_hat = x_hat_minus + K*inov1;
    P = (eye(n)- K*H_m1)*P_minus;
else
    %跳过
    x_hat = x_hat_minus;
    P = P_minus;
end

% 第二次融合
S2 = H_m2*P*H_m2.' + V2*R2_EKF*V2.';
inov2 = z2 - H_m2*x_hat;
d2_2   = inov2.'*inv(S2)*inov2;
if d2_2 <= gate_threshold
    K =  P*transpose(H_m2)/S2;
    x_hat = x_hat + K*inov2;
    P = (eye(n)- K*H_m2)*P;
end

x_estimate = x_hat(3:4);
end


function dX = Dynamics(X, f, L0)
    global theta2_fun xb2_fun phi2_fun
    theta2 = theta2_fun(X(1),X(2),L0,f(1),f(2));
    x2 = xb2_fun(X(1),X(2),L0,f(1),f(2));
    phi2 = phi2_fun(f(2));
    dX = [X(2);theta2;X(4);x2;X(6);phi2];
end

function Ja = Ja(theta,theta1,x,x1,phi,phi1,L0,T,Tp)
    %后面会贴出计算式
end