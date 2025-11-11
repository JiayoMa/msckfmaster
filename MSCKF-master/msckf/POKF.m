%% Position-Only Kalman Filter (po-KF)
% Minimal modification of MSCKF to estimate position only
% Orientation is taken from ground truth or external source

%% =============================Setup============================== %%
clear;
close all;
clc;
addpath('utils');

tic
dataDir = '../datasets';

% Good KITTI runs
fileName = '2011_09_26_drive_0001_sync_KLT'; kStart = 2; kEnd = 98;

% Load dataset
load(sprintf('%s/%s.mat',dataDir,fileName));

% Dataset window bounds
numLandmarks = size(y_k_j,3);

% Set up the camera parameters
camera.c_u      = cu;
camera.c_v      = cv;
camera.f_u      = fu;
camera.f_v      = fv;
camera.b        = b;
camera.q_CI     = rotMatToQuat(C_c_v);
camera.p_C_I    = rho_v_c_v;

% Set up the noise parameters
y_var = 11^2 * ones(1,4);
noiseParams.u_var_prime = y_var(1)/camera.f_u^2;
noiseParams.v_var_prime = y_var(2)/camera.f_v^2;

% Position-only noise parameters (reduced state)
v_var = 4e-2 * ones(1,3);              % lin vel var
dbv_var = 1e-6 * ones(1,3);            % vel bias change var
% Q matrix for position-only: [v_var, dbv_var]
noiseParams.Q_imu = diag([v_var, dbv_var]);

p_var_init = 1e-6 * ones(1,3);         % init pos var
bv_var_init = 1e-6 * ones(1,3);        % init vel bias var
noiseParams.initialIMUCovar = diag([bv_var_init, p_var_init]);

% po-KF parameters (same as MSCKF)
pokfParams.minTrackLength = 10;
pokfParams.maxTrackLength = Inf;
pokfParams.maxGNCostNorm  = 1e-2;
pokfParams.minRCOND       = 1e-12;
pokfParams.doNullSpaceTrick = true;
pokfParams.doQRdecomp = true;

% State history for plotting
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

%% ==========================Initial State======================== %%
% Initialize po-KF state (position-only, orientation from ground truth)
firstImuState.q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,kStart)));
firstImuState.p_I_G = r_i_vk_i(:,kStart);

[pokfState, featureTracks, trackedFeatureIds] = initializePOKF(firstImuState, measurements{kStart}, camera, kStart, noiseParams);
imuStates = updateStateHistoryPOKF(imuStates, pokfState, camera, kStart, groundTruthStates{kStart}.imuState.q_IG);

%% ============================MAIN LOOP========================== %%
numFeatureTracksResidualized = 0;
map = [];

for state_k = kStart:(kEnd-1)
    fprintf('state_k = %4d\n', state_k);
    
    %% ==========================STATE PROPAGATION======================== %%
    % Get orientation from ground truth for next state
    q_IG_true = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k+1)));
    
    % Propagate position-only state and covariance
    pokfState = propagatePokfStateAndCovar(pokfState, measurements{state_k}, noiseParams, q_IG_true);
    
    % Augment state with camera pose
    pokfState = augmentStatePOKF(pokfState, camera, state_k+1, q_IG_true);
    
    %% ==========================FEATURE TRACKING======================== %%
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
                
                [pokfState, camStates, camStateIndices] = removeTrackedFeaturePOKF(pokfState, featureId);
                
                if length(camStates) >= pokfParams.minTrackLength
                    track.camStates = camStates;
                    track.camStateIndices = camStateIndices;
                    featureTracksToResidualize{end+1} = track;
                end
               
                featureTracks = featureTracks(trackedFeatureIds ~= featureId);
                trackedFeatureIds(trackedFeatureIds ~= featureId) = []; 
            end
        
        elseif ~outOfView && state_k+1 < kEnd
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;
            pokfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
    end
     
    %% ==========================FEATURE RESIDUAL CORRECTIONS======================== %%
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
            
            % Calculate residual and Hoj for position-only
            [r_j] = calcResidual(p_f_G, track.camStates, track.observations);
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            [H_o_j, A_j, H_x_j] = calcHojPOKF(p_f_G, pokfState, track.camStateIndices);

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
            
            % Build po-KF covariance matrix (position-only: 3 + 3*numCamStates)
            P = [pokfState.imuCovar, pokfState.imuCamCovar;
                   pokfState.imuCamCovar', pokfState.camCovar];

            % Calculate Kalman gain
            K = (P*T_H') / ( T_H*P*T_H' + R_n );

            % State correction (position-only)
            deltaX = K * r_n;
            pokfState = updateStatePOKF(pokfState, deltaX);

            % Covariance correction
            tempMat = (eye(3 + 3*size(pokfState.camStates,2)) - K*T_H);
            P_corrected = tempMat * P * tempMat' + K * R_n * K';

            pokfState.imuCovar = P_corrected(1:3,1:3);
            pokfState.camCovar = P_corrected(4:end,4:end);
            pokfState.imuCamCovar = P_corrected(1:3, 4:end);
        end
    end
    
    %% ==========================STATE HISTORY======================== %%
    q_IG_current = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k+1)));
    imuStates = updateStateHistoryPOKF(imuStates, pokfState, camera, state_k+1, q_IG_current);
    
    %% ==========================STATE PRUNING======================== %%
    [pokfState, deletedCamStates] = pruneStatesPOKF(pokfState);
    if ~isempty(deletedCamStates)
        prunedStates(end+1:end+length(deletedCamStates)) = deletedCamStates;
    end    
end

toc

%% ==========================PLOT ERRORS======================== %%
kNum = length(prunedStates);
p_C_G_est = NaN(3, kNum);
p_C_G_GT = NaN(3, kNum);
err_sigma = NaN(3,kNum);
tPlot = NaN(1, kNum);

for k = 1:kNum
    state_k = prunedStates{k}.state_k;
    
    p_C_G_GT(:,k) = groundTruthStates{state_k}.camState.p_C_G;
    p_C_G_est(:,k) = prunedStates{k}.p_C_G;
    err_sigma(:,k) = prunedStates{k}.sigma;
    tPlot(k) = t(state_k);
end

p_I_G_GT = r_i_vk_i(:,kStart:kEnd);
p_C_G_GT = p_I_G_GT + repmat(rho_v_c_v,[1,size(p_I_G_GT,2)]);

% Save estimates
pokf_trans_err = p_C_G_est - p_C_G_GT;
save(sprintf('../KITTI Trials/pokf_%s', fileName));

armse_trans_pokf = mean(sqrt(sum(pokf_trans_err.^2, 1)/3));
final_trans_err_pokf = norm(pokf_trans_err(:,end));

fprintf('Trans ARMSE: po-KF %f\n', armse_trans_pokf);
fprintf('Final Trans Err: po-KF %f\n', final_trans_err_pokf);

% Translation Errors
figure
subplot(3,1,1)
plot(tPlot, p_C_G_est(1,:) - p_C_G_GT(1,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(1,:), '--r')
plot(tPlot, -3*err_sigma(1,:), '--r')
xlim([tPlot(1) tPlot(end)])
title('Position-Only KF: Translational Error')
ylabel('\delta r_x')

subplot(3,1,2)
plot(tPlot, p_C_G_est(2,:) - p_C_G_GT(2,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(2,:), '--r')
plot(tPlot, -3*err_sigma(2,:), '--r')
xlim([tPlot(1) tPlot(end)])
ylabel('\delta r_y')

subplot(3,1,3)
plot(tPlot, p_C_G_est(3,:) - p_C_G_GT(3,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(3,:), '--r')
plot(tPlot, -3*err_sigma(3,:), '--r')
xlim([tPlot(1) tPlot(end)])
ylabel('\delta r_z')
xlabel('t_k')
