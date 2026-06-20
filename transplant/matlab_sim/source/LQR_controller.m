function LQR_controller()

global Ja Jb Q R
syms theta theta1 xb xb1 phi phi1 L0

L0s=0.12:0.01:0.35; % L0变化范围
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K
Ss = zeros(6,6,length(L0s));

for step=1:length(L0s)
    A=double(subs(Ja,[theta theta1 xb xb1  phi phi1 L0],[0 0 0 0 0 0 L0s(step)]))
    B=double(subs(Jb,[theta theta1 xb xb1  phi phi1 L0],[0 0 0 0 0 0 L0s(step)]))
    [A_d, B_d]=c2d(A,B,0.01);
   
    [Ss(:,:,step), ~, Ks(:,:,step)]=dare(A_d,B_d,Q,R);
end

K=sym('K',[2 6]);
for xb=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(xb,y,:),1,length(L0s)),3);
        K(xb,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

%MPC的终端代价权重矩阵
for xb=1:6
    for y=1:6
        p=polyfit(L0s,reshape(Ss(xb,y,:),1,length(L0s)),3);
        S(xb,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end
S=sym('K',[6 6]);

matlabFunction(K,'File','LQR_K');
matlabFunction(S,'File','S_mat');