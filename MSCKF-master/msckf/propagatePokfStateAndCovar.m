function pokfState_prop = propagatePokfStateAndCovar(pokfState, measurements_k, noiseParams, q_IG_true)
%Position-only KF state and covariance propagation
%Orientation is taken from ground truth (q_IG_true)
%
%Output:
%      pokfState_prop: propagated state
%Input:
%      pokfState: current state
%      measurements_k: IMU measurements (velocity only used)
%      noiseParams: noise parameters
%      q_IG_true: ground truth orientation for this timestep

    % Noise parameters for position-only (no rotation terms)
    Q_imu = noiseParams.Q_imu;
    
    % For position-only: state is [b_v(3), p_I_G(3)] = 6 dimensions
    % But we only estimate [p_I_G(3)] = 3 dimensions
    % Velocity bias is not estimated in minimal version
    
    % Simplified F matrix for position-only
    % dp/dt = C_IG' * (v - b_v)
    % Since we treat orientation as known, F is 3x3
    C_IG = quatToRotMat(q_IG_true);
    vHat = measurements_k.v - pokfState.imuState.b_v;
    
    % For position-only, F relates position error to velocity
    % In simplified form with fixed orientation: F = 0 (position doesn't depend on itself in error dynamics)
    F = zeros(3,3);
    
    % G matrix for position-only (maps noise to state)
    G = -C_IG';  % Only velocity noise affects position
    
    % Propagate State (position-only)
    pokfState_prop.imuState = propagateImuStatePOKF(pokfState.imuState, measurements_k, q_IG_true);
    
    % State Transition Matrix (simplified for position-only)
    Phi = eye(3) + F * measurements_k.dT;
    
    % Position-only Covariance propagation
    % Only consider velocity noise contribution
    Q_v = diag(noiseParams.Q_imu(1:3));  % velocity variance
    pokfState_prop.imuCovar = Phi * pokfState.imuCovar * Phi' ...
                                + G * Q_v * G' * measurements_k.dT;
    
    % Enforce PSD
    pokfState_prop.imuCovar = enforcePSD(pokfState_prop.imuCovar);
                                    
    % Camera-Camera Covariance (unchanged)
    pokfState_prop.camCovar = pokfState.camCovar;
    
    % IMU-Camera Covariance
    if isempty(pokfState.imuCamCovar)
        pokfState_prop.imuCamCovar = [];
    else
        pokfState_prop.imuCamCovar = Phi * pokfState.imuCamCovar;
    end
    pokfState_prop.camStates = pokfState.camStates;
end
