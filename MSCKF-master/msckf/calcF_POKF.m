function F = calcF_POKF(imuState_k, measurements_k)
%POKF state transition Jacobian
% Simplified version for POKF (no bias states)
% State order: [rotation(3), position(3)]

    F = zeros(6,6);
    
    omegaHat = measurements_k.omega;  % No bias correction in POKF
    vHat = measurements_k.v;          % No bias correction in POKF
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    F(1:3,1:3) = -crossMat(omegaHat);
    F(4:6,1:3) = -C_IG' * crossMat(vHat);
  
end
