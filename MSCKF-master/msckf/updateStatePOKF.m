function pokfState_up = updateStatePOKF(pokfState, deltaX)
%Update position-only state
%Only positions are updated, orientations remain from ground truth
%
%Output:
%      pokfState_up: updated state
%Input:
%      pokfState: current state
%      deltaX: state correction from Kalman filter

    pokfState_up = pokfState;

    % Update IMU position only
    deltap_I_G = deltaX(1:3);
    pokfState_up.imuState.p_I_G = pokfState.imuState.p_I_G + deltap_I_G;
    
    % Orientation, biases remain unchanged (from ground truth)
    pokfState_up.imuState.q_IG = pokfState.imuState.q_IG;
    pokfState_up.imuState.b_g = pokfState.imuState.b_g;
    pokfState_up.imuState.b_v = pokfState.imuState.b_v;
    
    % Update camera positions only
    for i = 1:size(pokfState.camStates, 2)
        pStart = 3 + 3*(i-1) + 1;
        
        deltap_C_G = deltaX(pStart:pStart+2);
        
        % Update camera position
        pokfState_up.camStates{i}.p_C_G = pokfState.camStates{i}.p_C_G + deltap_C_G;
        
        % Camera orientation remains unchanged (derived from ground truth IMU orientation)
        pokfState_up.camStates{i}.q_CG = pokfState.camStates{i}.q_CG;
    end
    
end
