% [state] t=9.120s
%   target phi=90.000deg, phi_rate=0.000deg/s, distance=0.000m, velocity=0.000m/s, pitch=0.000deg, pitch_rate=0.000deg/s
%   left   phi=98.117deg, phi_rate=0.401deg/s, distance=-3.086m, velocity=0.050m/s, pitch=1.037deg, pitch_rate=-2.036deg/s
%   right  phi=97.164deg, phi_rate=-0.831deg/s, distance=-3.086m, velocity=0.050m/s, pitch=1.037deg, pitch_rate=-2.036deg/s
%   leg_length left=0.234m, right=0.237m
%   LQR_L wheel_torque=0.050Nm, hip_torque=0.002Nm, torque_magnitude=0.050, fly_flag=0
%   LQR_R wheel_torque=-0.053Nm, hip_torque=0.002Nm, torque_magnitude=0.053, fly_flag=0

target = [deg2rad(90); 0; 0; 0; 0; 0];

left_leg_length = 0.234;
right_leg_length = 0.237;

left_state = [
    deg2rad(98.117);
    deg2rad(0.401);
    -3.086;
    0.050;
    deg2rad(1.037);
    deg2rad(-2.036)
];

right_state = [
    deg2rad(97.164);
    deg2rad(-0.831);
    -3.086;
    0.050;
    deg2rad(1.037);
    deg2rad(-2.036)
];

K_l = LQR_K(left_leg_length);
K_r = LQR_K(right_leg_length);

u_l = -K_l * (left_state - target)
u_r = -K_r * (right_state - target)

fprintf('left:  wheel_torque=%.6f Nm, hip_torque=%.6f Nm\n', u_l(1), u_l(2));
fprintf('right: wheel_torque=%.6f Nm, hip_torque=%.6f Nm\n', u_r(1), u_r(2));
