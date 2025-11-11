function [pokfState, featureTracks, trackedFeatureIds] = initializePOKF(firstImuState, firstMeasurements, camera, state_k, noiseParams)
%POKF initialization function
% Initializes POKF state with first camera pose and tracked features
%
% Outputs:
%      pokfState: Initial POKF state (IMU and first camera state)
%      featureTracks: Tracked feature data structure
%      trackedFeatureIds: IDs of tracked features
% Inputs:
%      firstImuState: Initial IMU state (position and orientation)
%      firstMeasurements: Measurements at first frame
%      camera: Camera-IMU transformation parameters
%      state_k: Frame ID
%      noiseParams: Noise parameters for covariance initialization

% Initialize POKF state structure
% POKF has simplified state compared to MSCKF (no biases)
pokfState.imuState = firstImuState;
pokfState.imuCovar = noiseParams.initialIMUCovar;
pokfState.camCovar = [];
pokfState.imuCamCovar = [];
pokfState.camStates = {};

% Augment state with first camera pose
pokfState = augmentStatePOKF(pokfState, camera, state_k, firstImuState.q_IG);

% Initialize feature tracking
featureTracks = {};
trackedFeatureIds = [];

% Process all features in first frame
for featureId = 1:size(firstMeasurements.y,2)
    meas_k = firstMeasurements.y(:, featureId);
    % Only track valid measurements
    if ~isnan(meas_k(1,1))
        % Create new feature track
        track.featureId = featureId;
        track.observations = meas_k;
        featureTracks{end+1} = track;
        trackedFeatureIds(end+1) = featureId;
        % Add observation to current camera state
        pokfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
    end
end
 
end
