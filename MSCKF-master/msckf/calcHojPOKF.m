function [H_o_j, A_j, H_x_j] = calcHojPOKF(p_f_G, pokfState, camStateIndices)
%Calculate observation Jacobian for position-only KF
%Only position is estimated, orientation is known
%
%Output:
%      H_o_j: observation Jacobian after null space projection
%      A_j: null space projection matrix
%      H_x_j: full observation Jacobian
%Input:
%      p_f_G: feature location in global frame
%      pokfState: current state
%      camStateIndices: indices of camera states observing this feature

    N = length(pokfState.camStates);
    M = length(camStateIndices);
    H_f_j = zeros(2*M, 3);
    H_x_j = zeros(2*M, 3 + 3*N);  % 3 for IMU position + 3 per camera position

    c_i = 1;
    for camStateIndex = camStateIndices
        camState = pokfState.camStates{camStateIndex};

        C_CG = quatToRotMat(camState.q_CG);
        % Feature position in camera frame
        p_f_C = C_CG*(p_f_G - camState.p_C_G);

        X = p_f_C(1);
        Y = p_f_C(2);
        Z = p_f_C(3);

        % Projection Jacobian
        J_i = (1/Z)*[1 0 -X/Z; 0 1 -Y/Z];

        % Jacobian w.r.t. feature position
        H_f_j((2*c_i - 1):2*c_i, :) = J_i*C_CG;

        % For position-only: only camera position affects measurement
        % No rotation error terms since rotation is known
        % H w.r.t. camera position
        H_x_j((2*c_i - 1):2*c_i, 3+3*(camStateIndex-1) + 1:3+3*(camStateIndex-1) + 3) = -J_i*C_CG;

        c_i = c_i + 1;
    end

    % Null space projection to eliminate feature dependence
    A_j = null(H_f_j');
    H_o_j = A_j'*H_x_j;

end
