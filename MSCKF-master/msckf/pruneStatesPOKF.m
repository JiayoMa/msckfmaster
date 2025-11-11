function [pokfState, deletedCamStates] = pruneStatesPOKF(pokfState)
%Prune camera states that are no longer tracking any features
%Position-only version
%
%Output:
%      pokfState: state with pruned camera states
%      deletedCamStates: the deleted camera states
%Input:
%      pokfState: current state

    deletedCamStates = {};
    
    % Find camera states with no tracked features
    pruneCamStateIndices = [];
    for i = 1:length(pokfState.camStates)
        if isempty(pokfState.camStates{i}.trackedFeatureIds)
            pruneCamStateIndices(end+1) = i;
        end
    end
    
    if isempty(pruneCamStateIndices)
        return;
    end
    
    % Save deleted states for plotting
    for i = 1:length(pruneCamStateIndices)
        idx = pruneCamStateIndices(i);
        deletedCamStates{end+1} = pokfState.camStates{idx};
        
        % Add covariance for this camera state
        covStartIdx = 3 + 3*(idx-1) + 1;
        covEndIdx = covStartIdx + 2;
        deletedCamStates{end}.sigma = sqrt(diag(pokfState.camCovar(covStartIdx:covEndIdx, covStartIdx:covEndIdx)));
    end
    
    % Remove from state
    keepCamStates = true(1, length(pokfState.camStates));
    keepCamStates(pruneCamStateIndices) = false;
    pokfState.camStates = pokfState.camStates(keepCamStates);
    
    % Update covariance matrices
    % Build indices to keep
    keepCovIndices = true(1, 3*length(keepCamStates));
    for i = 1:length(pruneCamStateIndices)
        idx = pruneCamStateIndices(i);
        covStartIdx = 3*(idx-1) + 1;
        keepCovIndices(covStartIdx:covStartIdx+2) = false;
    end
    
    pokfState.camCovar = pokfState.camCovar(keepCovIndices, keepCovIndices);
    pokfState.imuCamCovar = pokfState.imuCamCovar(:, keepCovIndices);
end
