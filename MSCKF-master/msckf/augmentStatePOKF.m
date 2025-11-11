function pokfState_aug = augmentStatePOKF(pokfState, camera, state_k, q_IG_true)
%POKF state augmentation function: Augments POKF state with new camera pose
%
% Inputs:
%      pokfState: Current POKF state
%      camera: Camera parameters and IMU-camera transformation
%      state_k: Frame ID
%      q_IG_true: True quaternion (optional, for ground truth)
%
% Outputs:
%      pokfState_aug: Augmented POKF state

    % Get rotation matrix from IMU quaternion
    C_IG = quatToRotMat(pokfState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    % Get global to camera quaternion transformation
    q_CG = quatLeftComp(camera.q_CI) * pokfState.imuState.q_IG;
    % Get camera position in global frame
    p_C_G = pokfState.imuState.p_I_G + C_IG' * camera.p_C_I;

    % Build POKF covariance matrix
    % Combine IMU-IMU, IMU-camera, and camera-camera covariances
    P = [pokfState.imuCovar, pokfState.imuCamCovar;
        pokfState.imuCamCovar', pokfState.camCovar];
    
    % Camera state Jacobian
    % Calculate Jacobian relating new camera state to existing states
    J = calcJ_POKF(camera, pokfState.imuState, pokfState.camStates);
    
    % Number of existing camera states
    N = size(pokfState.camStates,2);
    
    % Augment state transition matrix
    % Fixed: Changed from eye(3 + 3*N) to eye(6 + 6*N) to match state dimensions
    % POKF state includes: 6 DOF for base state (3 position + 3 rotation)
    %                      + 6*N DOF for N camera states
    tempMat = [eye(6 + 6*N); J];
    
    % Augment the POKF covariance matrix
    P_aug = tempMat * P * tempMat';
    
    % Build augmented state structure
    pokfState_aug = pokfState;
    pokfState_aug.camStates{N+1}.p_C_G = p_C_G;
    pokfState_aug.camStates{N+1}.q_CG = q_CG;
    pokfState_aug.camStates{N+1}.state_k = state_k;
    pokfState_aug.camStates{N+1}.trackedFeatureIds = [];
    
    % Update covariance matrices
    pokfState_aug.imuCovar = P_aug(1:6,1:6);
    pokfState_aug.camCovar = P_aug(7:end,7:end);
    pokfState_aug.imuCamCovar = P_aug(1:6, 7:end);
end
