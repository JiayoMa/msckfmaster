function G = calcG_POKF(imuState_k)
%POKF noise Jacobian
% Simplified version for POKF (no bias states)
% Multiplies the noise vector in the linearized continuous-time error state model

    G = zeros(6,6);
    
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    G(1:3,1:3) = -eye(3);
    G(4:6,4:6) = -C_IG';
    
end
