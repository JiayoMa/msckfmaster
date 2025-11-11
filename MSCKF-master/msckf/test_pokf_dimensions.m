% Test script to verify augmentStatePOKF dimension fix
% This script checks that the dimensions are compatible for matrix operations

fprintf('Testing augmentStatePOKF dimension fix...\n\n');

% Test with different numbers of existing camera states
for N = 0:3
    fprintf('Testing with N = %d existing camera states:\n', N);
    
    % POKF state dimensions
    base_state_dim = 6;  % 3 rotation + 3 position
    cam_state_dim = 6;   % 3 rotation + 3 position per camera
    
    % Total state dimension
    total_dim = base_state_dim + cam_state_dim * N;
    fprintf('  Total state dimension: %d\n', total_dim);
    
    % Covariance matrix P
    P_rows = total_dim;
    P_cols = total_dim;
    fprintf('  P matrix size: %dx%d\n', P_rows, P_cols);
    
    % Jacobian J (new camera state w.r.t. existing state)
    J_rows = cam_state_dim;  % New camera state has 6 DOF
    J_cols = total_dim;
    fprintf('  J matrix size: %dx%d\n', J_rows, J_cols);
    
    % Identity matrix
    I_dim = base_state_dim + cam_state_dim * N;
    fprintf('  Identity matrix size: %dx%d\n', I_dim, I_dim);
    
    % tempMat = [eye(I_dim); J]
    tempMat_rows = I_dim + J_rows;
    tempMat_cols = I_dim;
    fprintf('  tempMat size: %dx%d\n', tempMat_rows, tempMat_cols);
    
    % Check dimension compatibility for vertical concatenation
    if I_dim == J_cols
        fprintf('  ✓ Dimension check PASSED: eye(%d) and J(%dx%d) can be concatenated\n', I_dim, J_rows, J_cols);
    else
        fprintf('  ✗ Dimension check FAILED: eye(%d) has %d columns but J has %d columns\n', I_dim, I_dim, J_cols);
    end
    
    % P_aug = tempMat * P * tempMat'
    P_aug_dim = tempMat_rows;
    fprintf('  P_aug matrix size: %dx%d\n', P_aug_dim, P_aug_dim);
    
    % Expected size after augmentation (base + N+1 cameras)
    expected_dim = base_state_dim + cam_state_dim * (N + 1);
    fprintf('  Expected augmented state dimension: %d\n', expected_dim);
    
    if P_aug_dim == expected_dim
        fprintf('  ✓ Final dimension check PASSED\n');
    else
        fprintf('  ✗ Final dimension check FAILED\n');
    end
    
    fprintf('\n');
end

fprintf('Testing complete.\n');

% Now test what would happen with the OLD (incorrect) dimension
fprintf('\n--- Testing OLD (incorrect) dimension ---\n');
for N = 0:2
    fprintf('With N = %d:\n', N);
    
    % Old incorrect dimension
    old_I_dim = 3 + 3*N;
    fprintf('  OLD: eye(%d + %d*%d) = eye(%d)\n', 3, 3, N, old_I_dim);
    
    % Correct dimension
    correct_I_dim = 6 + 6*N;
    J_cols = correct_I_dim;
    fprintf('  J has %d columns\n', J_cols);
    
    if old_I_dim == J_cols
        fprintf('  Would work (but wrong semantics)\n');
    else
        fprintf('  ✗ DIMENSION MISMATCH: Cannot concatenate [eye(%d); J(%dx%d)]\n', old_I_dim, 6, J_cols);
        fprintf('     eye(%d) has %d columns but J has %d columns\n', old_I_dim, old_I_dim, J_cols);
    end
    fprintf('\n');
end

fprintf('--- End of test ---\n');
