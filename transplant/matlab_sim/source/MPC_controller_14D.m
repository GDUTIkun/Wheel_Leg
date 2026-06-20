% filepath: d:\Matlab depository\project\wheel_leg_robort\source\MPC_controller_14D.m
function out = MPC_controller_14D(states12, Xd, L0L, L0R, FN, w0, yaw0, dyaw0, dyaw_d)
global Q R FN_min r d
import casadi.*

step = 20; dt = 0.01;
nmpc = casadi.Opti();

X = nmpc.variable(12, step); 
W = nmpc.variable(2, step);
U = nmpc.variable(4, step-1);     
nmpc.subject_to(X(:,1) == states12(:));

Q12 = blkdiag(Q,Q);               % 先对前12维加权
R4  = blkdiag(R,R);

w_delta   = 400;
w_d_delta = 80;
delta_max = deg2rad(20);

yaw_pred = yaw0;   % 用代数递推预测yaw
dyaw_pred = dyaw0;
w_yaw = 600;
w_dyaw = 100;

if FN <= FN_min
    Q12(4,4)   = 0;   % 左腿位移速度降权（按你的策略）
    Q12(10,10) = 0;   % 右腿位移速度降权
    w_yaw = 0;
end
% 12维参考扩展
Xd0  = [Xd; Xd];        % 若Xd12是6x1，这里得到12x1
Ref12 = generateReference(step, states12, Xd0);
dyaw_ref = linspace(dyaw0, dyaw_d, step);
yaw_ref = zeros(step,1);
yaw_ref(1) = yaw_pred + dyaw_ref(1)*dt;
for i=2:step
    yaw_ref(i) = yaw_ref(i-1) +  dyaw_ref(i)*dt;
end
    

nmpc.subject_to ( X(:,1) == states12 )
nmpc.subject_to ( W(:,1) == w0 )

J = 0;
pre_delta = 0;
for k = 1:step-1
    XL = X(1:6,k); XR = X(7:12,k);
    uL = U(1:2,k); uR = U(3:4,k);

    [dXL, dwL] = Dynamics_leg7(XL, uL, L0L);
    [dXR, dwR] = Dynamics_leg7(XR, uR, L0R);

    nmpc.subject_to(X(1:6,k+1)  == XL + dt*dXL);
    nmpc.subject_to(X(7:12,k+1) == XR + dt*dXR);
    nmpc.subject_to(W(1,k+1) == W(1, k) + dt*dwL);
    nmpc.subject_to(W(2,k+1) == W(2, k) + dt*dwR);
    
    nmpc.subject_to(-deg2rad(40) <= X(1,k)  <= deg2rad(40));  % thetaL
    nmpc.subject_to(-deg2rad(40) <= X(6,k)  <= deg2rad(40));  % thetaR
    nmpc.subject_to(-deg2rad(10) <= X(5,k)  <= deg2rad(10));  % phiL
    nmpc.subject_to(-deg2rad(10) <= X(11,k) <= deg2rad(10));  % phiR
    nmpc.subject_to(-delta_max <= (X(1,k)-X(7,k)) <= delta_max);
    
    delta = X(1) - X(7);
    if k>1
        d_delta = (delta-pre_delta)/dt;
    else
        d_delta = 0;
    end
    pre_delta = delta;

    % yaw预测: dyaw = (Rw/d)*(wR - wL)
    dyaw = r/d*(W(1,k) - W(2,k));
    yaw_pred = yaw_pred + dt*dyaw;

    % 主代价（前12维）
    e12 = [X(1:6,k); X(7:12,k)] - Ref12(:,k);
    J = J + e12.'*Q12*e12 + U(:,k).'*R4*U(:,k) + ...
        w_yaw*(yaw_pred - yaw_ref(k))^2 + w_dyaw*(dyaw_pred - dyaw_ref(k))^2 ... 
        + w_delta*delta^2 + w_d_delta*d_delta^2;
end
SL = S_mat(L0L);
SR = S_mat(L0R);
ef = X(:,end) - Ref12(:,end);

Sy = 50*w_yaw;
Sdy = 50*w_dyaw;
dyawf = (r/d)*(W(1,end) - W(2,end));
edyf = dyawf - dyaw_ref(end);
eyf = yaw_pred + dyawf*dt - yaw_ref(end);

J = J + ef.'*blkdiag(SL,SR)*ef + edyf^2*Sdy + eyf^2*Sy + 80*w_delta*delta^2 + 80*w_d_delta*d_delta^2;

nmpc.minimize(J);
nmpc.solver('ipopt', struct('expand',true));
sol = nmpc.solve();

u_out = sol.value(U(:,1));
x_out = sol.value(X(1:6,1));
xd_out = Ref12(1:6,1);
out = [u_out;x_out;xd_out];
end

function [dX,dw] = Dynamics_leg7(X, u, L0)

global theta2_fun x2_fun xb2_fun phi2_fun r
theta2 = theta2_fun(X(1),X(2),L0,u(1),u(2));
xb2 = xb2_fun(X(1),X(2),L0,u(1),u(2));
x2     = x2_fun(X(1),X(2),L0,u(1),u(2));
phi2   = phi2_fun(u(2));

dw  = x2 / r;   % 关键：由动力学推出轮角加速度
dX = [X(2); theta2; X(4); xb2; X(6); phi2];
end

function [ref, yaw_ref] = generateReference(step, X, Xd)
    ref = zeros(12, step);
    for i = 1 : 12
        ref(i,1:step) = linspace(X(i), Xd(i), step);
    end
    % for i = 1:step-1
    %     ref(3,i+1) = ref(3,i) + ref(4,i)*0.01;
    % end
end