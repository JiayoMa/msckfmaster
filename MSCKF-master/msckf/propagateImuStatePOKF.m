function imuState_prop = propagateImuStatePOKF(imuState_k, measurements_k, q_IG_true)
%Position-only IMU state propagation
%Orientation comes from ground truth, only position is estimated
%
%Output:
%      imuState_prop: propagated IMU state
%Input:
%      imuState_k: previous IMU state
%      measurements_k: IMU measurements
%      q_IG_true: ground truth orientation

    % Use ground truth orientation
    imuState_prop.q_IG = q_IG_true;
    
    % Velocity bias remains constant
    imuState_prop.b_v = imuState_k.b_v;
    
    % Propagate position using velocity measurement
    C_IG = quatToRotMat(q_IG_true);
    d = (measurements_k.v - imuState_k.b_v) * measurements_k.dT;
    imuState_prop.p_I_G = C_IG' * d + imuState_k.p_I_G;
    
    % Note: No gyro bias in position-only version
    imuState_prop.b_g = zeros(3,1);
end
