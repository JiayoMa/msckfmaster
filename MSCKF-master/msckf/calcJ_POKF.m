function J = calcJ_POKF(camera, imuState_k, camStates_k)
%POKF-specific Jacobian calculation
% Returns Jacobian of new camera state w.r.t. existing states
%
% For POKF with simplified state (position and rotation only):
% State vector: [imu_q (3), imu_p (3), cam_1_q (3), cam_1_p (3), ...]
% New camera state = [cam_p (3), cam_q (3)]

    C_CI = quatToRotMat(camera.q_CI);
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    % Jacobian of new camera state w.r.t. base IMU state and existing camera states
    % New camera state has 6 DOF, base state has 6 DOF, plus 6*N for N camera states
    J = zeros(6, 6 + 6*size(camStates_k,2));
    J(1:3,1:3) = C_CI;
    J(4:6,1:3) = crossMat(C_IG' * camera.p_C_I);
    J(4:6,4:6) = eye(3);

end
