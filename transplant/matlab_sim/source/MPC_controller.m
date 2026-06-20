function u = MPC_controller(states, Xd, L0, FN)

global  Q R FN_min

import casadi.*

%% states:
X_1 = states;

%% mpc parameters
step = 20; % # of horizons
dt_MPC = 0.01; % sec

%% desired reference:
ref = generateReference(step, X_1, Xd);

%% nmpc setup in CasADi:
nmpc = casadi.Opti();
X = nmpc.variable(6,step);
F = nmpc.variable(2,step-1); 

%% constraints:
nmpc.subject_to ( X(:,1) == X_1 ); %IC = current states

for k = 1 : step-1
    % dynamics:
    dX = Dynamics(X(:, k), F(:, k), L0);
    nmpc.subject_to( X(:, k+1) == X(:, k) + dt_MPC*dX );
    %动力学方程离散化
    %欧拉离散化   更精确的是四阶龙格库塔法（RK4）
    
    % nmpc.subject_to( -1 <= F(:,k) <= 1);
    nmpc.subject_to(-deg2rad(40)<=X(1,k)<=deg2rad(40));
    nmpc.subject_to(-deg2rad(20)<=X(5,k)<=deg2rad(20));
end

Qa = Q;
if FN <= FN_min
    Qa(3,3) = 0;
    Qa(4,4) = 0;
end

    J = 0;
for k = 1 : step-1
    e = X(:,k) - ref(:,k);
    J = J + e.'*Qa*e + F(:,k).'*R*F(:,k);
end
S = S_mat(L0);
ef = X(:,end) - ref(:,end);
J = J + ef.'*S*ef;

nmpc.minimize(J);

%% solve!
options = struct('expand',true);
nmpc.solver('ipopt', options);
solution = nmpc.solve();
Fx = solution.value(F);
u = Fx(:,1);

end


%% %%%%% functions %%%%%%
function ref = generateReference(step, X, Xd)
    ref = zeros(6, step);
    for i = 1 : 6
        ref(i,1:step) = linspace(X(i), Xd(i), step);
    end
    % for i = 1:step-1
    %     ref(3,i+1) = ref(3,i) + ref(4,i)*0.01;
    % end
end



function dX = Dynamics(X, f, L0)
    global theta2_fun xb2_fun phi2_fun
    theta2 = theta2_fun(X(1),X(2),L0,f(1),f(2));
    x2 = xb2_fun(X(1),X(2),L0,f(1),f(2));
    phi2 = phi2_fun(f(2));
    dX = [X(2);theta2;X(4);x2;X(6);phi2];
end

function X_next = rk4_step(f, X, dt)
    % 四阶RK单步
    k1 = f(X);
    k2 = f(X + 0.5*dt*k1);
    k3 = f(X + 0.5*dt*k2);
    k4 = f(X + dt*k3);
    X_next = X + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end