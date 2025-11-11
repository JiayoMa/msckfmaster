function pokfState_prop = propagatePokfStateAndCovar(pokfState, measurements_k, noiseParams)
%POKF state and covariance propagation
% Propagates POKF state and covariance using IMU measurements
%
% Outputs:
%      pokfState_prop: Propagated POKF state
% Inputs:
%      pokfState: Current POKF state
%      measurements_k: IMU measurements (angular velocity and linear velocity)
%      noiseParams: Noise parameters

    % Get noise covariance
    Q_imu = noiseParams.Q_imu;
    
    % Jacobians for POKF (simplified - no bias states)
    F = calcF_POKF(pokfState.imuState, measurements_k);
    G = calcG_POKF(pokfState.imuState);

    % Propagate IMU state
    pokfState_prop.imuState = propagateImuState(pokfState.imuState, measurements_k);

    % State Transition Matrix
    Phi = eye(size(F,1)) + F * measurements_k.dT;
    
    % IMU-IMU Covariance propagation
    pokfState_prop.imuCovar = Phi * pokfState.imuCovar * Phi' ...
                                + G * Q_imu * G' * measurements_k.dT;
    
    % Enforce PSD-ness
    pokfState_prop.imuCovar = enforcePSD(pokfState_prop.imuCovar);
                                    
    % Camera-Camera Covariance (unchanged)
    pokfState_prop.camCovar = pokfState.camCovar;
    
    % IMU-Camera Covariance propagation
    pokfState_prop.imuCamCovar = Phi * pokfState.imuCamCovar;
    pokfState_prop.camStates = pokfState.camStates;
end
