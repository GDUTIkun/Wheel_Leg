function [L0, phi] = phi_L0(theta1, theta2)
%phi_L0
l1 = 0.18;
l2 = 0.225;

% Clockwise is positive.
% theta1: angle between l1 and the horizontal line.
% theta2: included inner angle between l1 and l2.
%
% For a right-facing "<" two-link chain, the absolute direction of l2 is:
% theta1 + theta2 - pi.
theta_l2 = theta1 + theta2 - pi;

x = l1*cos(theta1) + l2*cos(theta_l2);
y_cw = l1*sin(theta1) + l2*sin(theta_l2);

L0 = sqrt(x^2 + y_cw^2);
phi = atan2(y_cw, x);
