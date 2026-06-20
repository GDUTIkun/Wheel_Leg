syms F T T1 T2 theta1 theta2 x y phi L0

l1 = 0.18;
l2 = 0.225;

x = -l1*sin(theta1-pi/2)+l2*cos(theta1+theta2-pi);
y = l1*cos(theta1-pi/2)+l2*sin(theta1+theta2-pi);

J = [-l1*sin(theta1)+l2*sin(theta2 + theta1)   l2*sin(theta2 + theta1)
     l1*cos(theta1)-l2*cos(theta2 + theta1)   -l2*cos(theta2 + theta1)];
R = [cos(phi) -sin(phi)
     sin(phi)   cos(phi)];

M = [-1 0;0 1/L0];

T12 = J.'*R*M*[F;T];
T1 = T12(1);
T2 = T12(2);

matlabFunction([T1 T2], 'File', 'serialVMC', 'Vars', {F,T,L0,phi,theta1,theta2});



