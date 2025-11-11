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
    
    % Build current covariance matrix
    N = size(pokfState.camStates, 2) - 1;  % Number of camera states before augmentation
    
    if N == 0
        % First camera state augmentation
        P = pokfState.imuCovar;
    else
        % Subsequent augmentations
        P = [pokfState.imuCovar, pokfState.imuCamCovar;
             pokfState.imuCamCovar', pokfState.camCovar];
    end
    
    % Jacobian: new camera position = IMU position (in global frame)
    % J maps IMU state + existing camera states to new camera state
    % Size: 3 × (3 + 3*N) where N is number of existing cameras
    J = zeros(3, 3 + 3*N);
    J(1:3, 1:3) = eye(3);  % New camera position depends on IMU position
    
    % Augmentation matrix: [existing states; new camera state]
    tempMat = [eye(3 + 3*N); J];
    
    % Augment covariance
    P_aug = tempMat * P * tempMat';
    
    % Extract components
    pokfState.imuCovar = P_aug(1:3, 1:3);
    pokfState.camCovar = P_aug(4:end, 4:end);
    pokfState.imuCamCovar = P_aug(1:3, 4:end);
end
