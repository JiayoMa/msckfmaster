function pokfState = augmentStatePOKF(pokfState, camera, state_k, q_IG_true)
%Augment position-only state with new camera pose
%Orientation from ground truth, position estimated
%
%Output:
%      pokfState: augmented state
%Input:
%      pokfState: current state
%      camera: camera parameters
%      state_k: current state index
%      q_IG_true: ground truth orientation

    % Get IMU orientation (from ground truth) and position (estimated)
    C_IG = quatToRotMat(q_IG_true);
    p_I_G = pokfState.imuState.p_I_G;
    
    % Calculate camera pose from IMU pose
    q_CG = quatLeftComp(camera.q_CI) * q_IG_true;
    p_C_G = p_I_G + C_IG' * camera.p_C_I;
    
    % Add new camera state
    newCamState.q_CG = q_CG;
    newCamState.p_C_G = p_C_G;
    newCamState.trackedFeatureIds = [];
    newCamState.state_k = state_k;
    pokfState.camStates{end+1} = newCamState;
    
    % Augment covariance
    % For position-only: camera state is 3D (position only)
    % J maps IMU position error to camera position error
    J = eye(3);  % Camera position error = IMU position error (in global frame)
    
    % Augment covariance matrix
    pokfState.imuCamCovar = [pokfState.imuCamCovar, pokfState.imuCovar * J'];
    
    tempMat = [J * pokfState.imuCovar, J * pokfState.imuCamCovar];
    pokfState.camCovar = [pokfState.camCovar, tempMat(1:3, 4:end)';
                          tempMat(1:3, 4:end), tempMat(1:3, 1:3)];
end
