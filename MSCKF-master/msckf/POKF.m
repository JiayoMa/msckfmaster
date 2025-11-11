%% POKF - Partial Observability Kalman Filter
% This is a simplified implementation based on MSCKF structure
% POKF uses a reduced state representation compared to MSCKF

%% Setup
clear;
close all;
clc;
addpath('utils');

tic
dataDir = '../datasets';

% Use the same dataset as MSCKF for testing
fileName = '2011_09_26_drive_0001_sync_KLT'; kStart = 2; kEnd = 98;

% Load dataset
load(sprintf('%s/%s.mat',dataDir,fileName));

% Dataset window bounds
numLandmarks = size(y_k_j,3);

% Set up camera parameters
camera.c_u      = cu;                   
camera.c_v      = cv;                   
camera.f_u      = fu;                   
camera.f_v      = fv;                   
camera.b        = b;                    
camera.q_CI     = rotMatToQuat(C_c_v);  
camera.p_C_I    = rho_v_c_v;            

% Set up noise parameters
y_var = 11^2 * ones(1,4);               
noiseParams.u_var_prime = y_var(1)/camera.f_u^2;
noiseParams.v_var_prime = y_var(2)/camera.f_v^2;

% Initial covariance parameters
w_var = 4e-2 * ones(1,3);              
v_var = 4e-2 * ones(1,3);              
noiseParams.Q_imu = diag([w_var, v_var]);  % Simplified for POKF

q_var_init = 1e-6 * ones(1,3);         
p_var_init = 1e-6 * ones(1,3);         
noiseParams.initialIMUCovar = diag([q_var_init, p_var_init]);

% POKF parameters (similar to MSCKF)
pokfParams.minTrackLength = 10;        
pokfParams.maxTrackLength = Inf;      
pokfParams.maxGNCostNorm  = 1e-2;     
pokfParams.minRCOND       = 1e-12;
pokfParams.doNullSpaceTrick = true;
pokfParams.doQRdecomp = true;

% Initialize state storage
imuStates = cell(1,numel(t));
prunedStates = {};

% Measurements
dT = [0, diff(t)];
measurements = cell(1,numel(t));

% Replace invalid measurements with NaN
y_k_j(y_k_j == -1) = NaN;

% Prepare measurements and ground truth
for state_k = kStart:kEnd 
    measurements{state_k}.dT    = dT(state_k);                      
    measurements{state_k}.y     = squeeze(y_k_j(1:2,state_k,:));    
    measurements{state_k}.omega = w_vk_vk_i(:,state_k);             
    measurements{state_k}.v     = v_vk_vk_i(:,state_k);             
    
    % Idealize measurements
    validMeas = ~isnan(measurements{state_k}.y(1,:));
    measurements{state_k}.y(1,validMeas) = (measurements{state_k}.y(1,validMeas) - camera.c_u)/camera.f_u;
    measurements{state_k}.y(2,validMeas) = (measurements{state_k}.y(2,validMeas) - camera.c_v)/camera.f_v;
    
    % Ground Truth
    q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k)));
    p_I_G = r_i_vk_i(:,state_k);
    
    groundTruthStates{state_k}.imuState.q_IG = q_IG;
    groundTruthStates{state_k}.imuState.p_I_G = p_I_G;
    
    % Compute camera pose from current IMU pose
    C_IG = quatToRotMat(q_IG);
    q_CG = quatLeftComp(camera.q_CI) * q_IG;
    p_C_G = p_I_G + C_IG' * camera.p_C_I;
    
    groundTruthStates{state_k}.camState.q_CG = q_CG;
    groundTruthStates{state_k}.camState.p_C_G = p_C_G;
end

% Feature tracking
featureTracks = {};
trackedFeatureIds = [];

%% Initial State
% Use ground truth for first state
firstImuState.q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,kStart)));
firstImuState.p_I_G = r_i_vk_i(:,kStart);

% Initialize POKF (simplified version of MSCKF initialization)
[pokfState, featureTracks, trackedFeatureIds] = initializePOKF(firstImuState, measurements{kStart}, camera, kStart, noiseParams);
imuStates = updateStateHistory(imuStates, pokfState, camera, kStart);

%% Main Loop
numFeatureTracksResidualized = 0;
map = [];

for state_k = kStart:(kEnd-1)
    fprintf('state_k = %4d\n', state_k);
    
    %% State Propagation
    pokfState = propagatePokfStateAndCovar(pokfState, measurements{state_k}, noiseParams);
    
    %% State Augmentation
    % Get ground truth quaternion for this frame
    q_IG_true = groundTruthStates{state_k+1}.imuState.q_IG;
    
    % Augment POKF state with new camera pose
    pokfState = augmentStatePOKF(pokfState, camera, state_k+1, q_IG_true);
    
    %% Feature Tracking
    featureTracksToResidualize = {};
    
    for featureId = 1:numLandmarks
        meas_k = measurements{state_k+1}.y(:, featureId);
        outOfView = isnan(meas_k(1,1));
        
        if ismember(featureId, trackedFeatureIds)
            if ~outOfView
                featureTracks{trackedFeatureIds == featureId}.observations(:, end+1) = meas_k;
                pokfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
            end
            
            track = featureTracks{trackedFeatureIds == featureId};
            
            if outOfView ...
                    || size(track.observations, 2) >= pokfParams.maxTrackLength ...
                    || state_k+1 == kEnd
                                
                [pokfState, camStates, camStateIndices] = removeTrackedFeature(pokfState, featureId);
                
                if length(camStates) >= pokfParams.minTrackLength
                    track.camStates = camStates;
                    track.camStateIndices = camStateIndices;
                    featureTracksToResidualize{end+1} = track;
                end
               
                featureTracks = featureTracks(trackedFeatureIds ~= featureId);
                trackedFeatureIds(trackedFeatureIds == featureId) = []; 
            end
        
        elseif ~outOfView && state_k+1 < kEnd
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;
            pokfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
    end
     
    %% Feature Residual Corrections
    if ~isempty(featureTracksToResidualize)
        H_o = [];
        r_o = [];
        R_o = [];
        
        for f_i = 1:length(featureTracksToResidualize)
            track = featureTracksToResidualize{f_i};     
            [p_f_G, Jcost, RCOND] = calcGNPosEst(track.camStates, track.observations, noiseParams);
            nObs = size(track.observations,2);
            JcostNorm = Jcost / nObs^2;
            fprintf('Jcost = %f | JcostNorm = %f | RCOND = %f\n', Jcost, JcostNorm, RCOND);
            
            if JcostNorm > pokfParams.maxGNCostNorm || RCOND < pokfParams.minRCOND
                break;
            else
                map(:,end+1) = p_f_G;
                numFeatureTracksResidualized = numFeatureTracksResidualized + 1;
                fprintf('Using new feature track with %d observations. Total track count = %d.\n', nObs, numFeatureTracksResidualized);
            end
            
            [r_j] = calcResidual(p_f_G, track.camStates, track.observations);
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, pokfState, track.camStateIndices);

            if pokfParams.doNullSpaceTrick
                H_o = [H_o; H_o_j];
                if ~isempty(A_j)
                    r_o_j = A_j' * r_j;
                    r_o = [r_o ; r_o_j];
                    R_o_j = A_j' * R_j * A_j;
                    R_o(end+1 : end+size(R_o_j,1), end+1 : end+size(R_o_j,2)) = R_o_j;
                end
            else
                H_o = [H_o; H_x_j];
                r_o = [r_o; r_j];
                R_o(end+1 : end+size(R_j,1), end+1 : end+size(R_j,2)) = R_j;
            end
        end
        
        if ~isempty(r_o)
            if pokfParams.doQRdecomp
                [T_H, Q_1] = calcTH(H_o);
                r_n = Q_1' * r_o;
                R_n = Q_1' * R_o * Q_1;
            else
                T_H = H_o;
                r_n = r_o;
                R_n = R_o;
            end           
            
            % Build POKF covariance matrix
            P = [pokfState.imuCovar, pokfState.imuCamCovar;
                   pokfState.imuCamCovar', pokfState.camCovar];

            % Calculate Kalman gain
            K = (P*T_H') / ( T_H*P*T_H' + R_n );

            % State correction
            deltaX = K * r_n;
            pokfState = updateState(pokfState, deltaX);

            % Covariance correction
            tempMat = (eye(6 + 6*size(pokfState.camStates,2)) - K*T_H);
            P_corrected = tempMat * P * tempMat' + K * R_n * K';

            pokfState.imuCovar = P_corrected(1:6,1:6);
            pokfState.camCovar = P_corrected(7:end,7:end);
            pokfState.imuCamCovar = P_corrected(1:6, 7:end);
        end
    end
    
    %% State History
    imuStates = updateStateHistory(imuStates, pokfState, camera, state_k+1);
    
    %% State Pruning
    [pokfState, deletedCamStates] = pruneStates(pokfState);
    if ~isempty(deletedCamStates)
        prunedStates(end+1:end+length(deletedCamStates)) = deletedCamStates;
    end    
    
    plot_traj;
end

toc

fprintf('POKF processing complete. Total features residualized: %d\n', numFeatureTracksResidualized);
