function Tp = solveT_from_ddtheta(theta,dtheta,L0,Tw,ddtheta_des)
A = 5800 + 160000*L0*cos(theta);
D = L0^2*(9*cos(theta)^2 + 29*sin(theta)^2);
B = -5800*Tw - 28449*L0*sin(theta) + 2000*L0^2*dtheta^2*cos(theta)*sin(theta);

epsA = 1e-3;
A = sign(A)*max(abs(A), epsA);   % 防奇异
Tp = (-100*ddtheta_des*D - B)/A;
end