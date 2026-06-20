function [theta2_fun,x2_fun, xb2_fun, phi2_fun, Ja, Jb] = state_equation()

global r L Lm l mw mp M Iw Ip h w Im g 

syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
syms x x1 x2 xb xb1 xb2;
syms phi phi1 phi2;
syms T Tp N P Nm Pm Nf t;
syms L0


% 进行物理计算
Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
%二阶导数求解
equ1=x2-(T-N*r)/(Iw/r+mw*r);% =0求解
equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
[x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
xb2 = x2 + (L+Lm)*theta2*cos(theta) - (L+Lm)*theta1^2*sin(theta);

theta2_fun = matlabFunction(theta2, 'Vars', [theta, theta1, L0, T, Tp]);
x2_fun = matlabFunction(x2, 'Vars', [theta, theta1, L0, T, Tp]);
xb2_fun = matlabFunction(xb2, 'Vars',[theta, theta1, L0, T, Tp]);
phi2_fun = matlabFunction(phi2, 'Vars', Tp);



% 求得雅克比矩阵，然后得到状态空间方程
Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
matlabFunction(Ja, 'File','Ja', 'Vars', {theta,theta1,x,x1,phi,phi1,L0,T,Tp});
matlabFunction(Jb, 'File','Jb', 'Vars', {theta,theta1,x,x1,phi,phi1,L0,T,Tp});
end
