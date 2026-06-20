function FN_estimation
    syms F_N F Tp theta L0 L0_ddot L0_dot
    syms dtheta ddtheta ddz_M real
    mw=0.15; 
    g=9.81;
    % 定义 P 和 z_w 的表达式
    P = F * cos(theta) + (Tp * sin(theta)) / L0;
    
    % 定义 z_w 的二阶导数
    ddz_w = ddz_M - L0_ddot * cos(theta) + 2 * L0_dot * dtheta * sin(theta) + L0 * ddtheta * sin(theta) + L0 * dtheta^2 * cos(theta);

    % 定义力平衡方程并解 F_N
    F_N = mw * ddz_w + P + mw * g;
    
    % 显示结果
    matlabFunction(F_N, ddz_w, 'File','SupportDetection');