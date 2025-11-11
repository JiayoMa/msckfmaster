function [pokfState, camStates, camStateIndices] = removeTrackedFeaturePOKF(pokfState, featureId)
%Remove a feature from tracking in position-only KF
%Similar to MSCKF version but for position-only state
%
%Output:
%      pokfState: updated state with feature removed
%      camStates: camera states that observed this feature
%      camStateIndices: indices of those camera states
%Input:
%      pokfState: current state
%      featureId: ID of feature to remove

    camStateIndices = [];
    camStates = {};
    
    % Find all camera states that tracked this feature
    for i = 1:length(pokfState.camStates)
        if ismember(featureId, pokfState.camStates{i}.trackedFeatureIds)
            camStateIndices(end+1) = i;
            camStates{end+1} = pokfState.camStates{i};
        end
    end
    
    % Remove feature from tracking lists
    for i = 1:length(pokfState.camStates)
        pokfState.camStates{i}.trackedFeatureIds = ...
            pokfState.camStates{i}.trackedFeatureIds(...
                pokfState.camStates{i}.trackedFeatureIds ~= featureId);
    end
end
