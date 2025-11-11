function [pokfState, featureTracks, trackedFeatureIds] = initializePOKF(firstImuState, measurements, camera, state_k, noiseParams)
%Initialize position-only Kalman Filter
%Similar to MSCKF but with position-only state
%
%Output:
%      pokfState: initialized state
%      featureTracks: feature tracking structure
%      trackedFeatureIds: IDs of tracked features
%Input:
%      firstImuState: initial IMU state (with orientation from ground truth)
%      measurements: initial measurements
%      camera: camera parameters
%      state_k: current state index
%      noiseParams: noise parameters

    % Initialize IMU state (position-only, but keep orientation for compatibility)
    pokfState.imuState.q_IG = firstImuState.q_IG;  % From ground truth
    pokfState.imuState.p_I_G = firstImuState.p_I_G;
    pokfState.imuState.b_g = zeros(3,1);  % Not estimated in po-KF
    pokfState.imuState.b_v = zeros(3,1);  % Not estimated in po-KF
    
    % Position-only covariance (3x3 for position only)
    pokfState.imuCovar = noiseParams.initialIMUCovar(4:6, 4:6);  % Only position part
    
    % No camera states initially
    pokfState.camStates = {};
    pokfState.camCovar = [];
    pokfState.imuCamCovar = [];
    
    % Augment state with first camera pose (following MSCKF pattern)
    pokfState = augmentStatePOKF(pokfState, camera, state_k, firstImuState.q_IG);
    
    % Initialize feature tracks
    featureTracks = {};
    trackedFeatureIds = [];
    
    % Add initial observations to feature tracks
    for featureId = 1:size(measurements.y, 2)
        meas_k = measurements.y(:, featureId);
        if ~isnan(meas_k(1,1))
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;
            pokfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
    end
end
