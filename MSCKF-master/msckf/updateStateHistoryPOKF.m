function imuStates = updateStateHistoryPOKF(imuStates, pokfState, camera, state_k, q_IG_current)
%Update IMU state history for position-only KF
%
%Output:
%      imuStates: updated state history
%Input:
%      imuStates: current state history
%      pokfState: current po-KF state
%      camera: camera parameters
%      state_k: current state index
%      q_IG_current: current ground truth orientation

    % Store IMU state (position from filter, orientation from ground truth)
    imuStates{state_k}.q_IG = q_IG_current;
    imuStates{state_k}.p_I_G = pokfState.imuState.p_I_G;
    imuStates{state_k}.b_g = pokfState.imuState.b_g;
    imuStates{state_k}.b_v = pokfState.imuState.b_v;
    imuStates{state_k}.covar = pokfState.imuCovar;
end
