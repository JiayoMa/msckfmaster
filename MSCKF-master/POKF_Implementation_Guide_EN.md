# MSCKF to po-KF: Minimal Modification Implementation Guide

## Overview
This document explains how to convert MSCKF (Multi-State Constraint Kalman Filter) to po-KF (Position-Only Kalman Filter) with minimal modifications.

## Core Differences: po-KF vs MSCKF

### MSCKF State Vector (12-dimensional)
- Rotation error: δθ (3D)
- Gyro bias: b_g (3D)
- Velocity bias: b_v (3D)
- Position: p_I_G (3D)

### po-KF State Vector (3-dimensional)
- Position: p_I_G (3D only)
- **Orientation obtained from ground truth or external source, not estimated in Kalman filter**

## Files to Modify and Corresponding Code Changes

### 1. Main Program: POKF.m (New File)
Based on `MSCKF.m`, key modifications:

**Lines 42-46: Simplified noise parameter matrix**
```matlab
v_var = 4e-2 * ones(1,3);
dbv_var = 1e-6 * ones(1,3);
% Only velocity noise, remove rotation and gyro-related noise
noiseParams.Q_imu = diag([v_var, dbv_var]);
```

**Lines 48-49: Simplified initial covariance**
```matlab
p_var_init = 1e-6 * ones(1,3);
bv_var_init = 1e-6 * ones(1,3);
noiseParams.initialIMUCovar = diag([bv_var_init, p_var_init]);
```

**Line 117: Get orientation from ground truth**
```matlab
q_IG_true = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k+1)));
```

**Line 120: Pass true orientation to propagation**
```matlab
pokfState = propagatePokfStateAndCovar(pokfState, measurements{state_k}, noiseParams, q_IG_true);
```

**Lines 213-215: Covariance dimensions reduced to (3 + 3*numCams)**
```matlab
tempMat = (eye(3 + 3*size(pokfState.camStates,2)) - K*T_H);
pokfState.imuCovar = P_corrected(1:3,1:3);  % 3x3 instead of 12x12
pokfState.camCovar = P_corrected(4:end,4:end);
```

### 2. State Propagation: propagatePokfStateAndCovar.m (New File)
Based on `propagateMsckfStateAndCovar.m`, key modifications:

**Lines 20-21: Simplified F matrix (3x3 instead of 12x12)**
```matlab
% Position error dynamics simplified because orientation is known
F = zeros(3,3);
```

**Line 24: Simplified G matrix**
```matlab
G = -C_IG';  % Only velocity noise affects position
```

**Line 31: State transition matrix becomes 3x3**
```matlab
Phi = eye(3) + F * measurements_k.dT;
```

**Lines 34-36: Covariance propagation (3x3)**
```matlab
Q_v = diag(noiseParams.Q_imu(1:3));
pokfState_prop.imuCovar = Phi * pokfState.imuCovar * Phi' ...
                            + G * Q_v * G' * measurements_k.dT;
```

### 3. IMU State Propagation: propagateImuStatePOKF.m (New File)
Based on `propagateImuState.m`, key modifications:

**Line 14: Use ground truth orientation instead of estimation**
```matlab
imuState_prop.q_IG = q_IG_true;
```

**Remove lines 15-29: quaternion update code**
- No need for gyro measurements to update orientation

**Line 22: Set gyro bias to zero**
```matlab
imuState_prop.b_g = zeros(3,1);
```

### 4. State Augmentation: augmentStatePOKF.m (New File)
Based on `augmentState.m`, key modifications:

**Line 29: Simplified Jacobian (3x3)**
```matlab
J = eye(3);  % Camera position error = IMU position error
```

**Remove rotation-related Jacobian terms**
- Original MSCKF terms like J(1:3,1:3) = C_CI are removed

**Lines 34-36: Covariance augmentation (3D instead of 6D)**
```matlab
pokfState.camCovar = [pokfState.camCovar, tempMat(1:3, 4:end)';
                      tempMat(1:3, 4:end), tempMat(1:3, 1:3)];
```

### 5. Observation Jacobian: calcHojPOKF.m (New File)
Based on `calcHoj.m`, key modifications:

**Line 14: H_x_j dimensions changed to (2*M, 3 + 3*N)**
```matlab
H_x_j = zeros(2*M, 3 + 3*N);  % Each state only 3D position
```

**Remove line 51: rotation Jacobian**
```matlab
% Original: H_x_j(..., :) = J_i*crossMat(p_f_C);
% po-KF: Removed because orientation error not in state
```

**Line 53: Keep only position Jacobian**
```matlab
H_x_j((2*c_i - 1):2*c_i, 3+3*(camStateIndex-1) + 1:3+3*(camStateIndex-1) + 3) = -J_i*C_CG;
```

### 6. State Update: updateStatePOKF.m (New File)
Based on `updateState.m`, key modifications:

**Line 16: Only update position**
```matlab
deltap_I_G = deltaX(1:3);
pokfState_up.imuState.p_I_G = pokfState.imuState.p_I_G + deltap_I_G;
```

**Remove lines 17-24: rotation and bias updates**
- Orientation and biases remain unchanged (from ground truth)

**Lines 31-40: Camera states only update position**
```matlab
for i = 1:size(pokfState.camStates, 2)
    pStart = 3 + 3*(i-1) + 1;
    deltap_C_G = deltaX(pStart:pStart+2);
    pokfState_up.camStates{i}.p_C_G = pokfState.camStates{i}.p_C_G + deltap_C_G;
end
```

### 7. Other Helper Functions

#### initializePOKF.m (New File)
**Line 15: 3x3 covariance matrix**
```matlab
pokfState.imuCovar = noiseParams.initialIMUCovar(4:6, 4:6);
```

#### removeTrackedFeaturePOKF.m (New File)
Same as MSCKF version, no core logic modifications needed

#### pruneStatesPOKF.m (New File)
**Line 26: Adjust covariance extraction indices (3D per state)**
```matlab
covStartIdx = 3 + 3*(idx-1) + 1;
covEndIdx = covStartIdx + 2;
```

#### updateStateHistoryPOKF.m (New File)
Simple storage: orientation from ground truth, position from filter

## Summary of Core Modifications

### State Dimension Changes
- IMU state: 12D → 3D
- Camera state: 6D → 3D
- Total state: (12 + 6*N) → (3 + 3*N)

### F Matrix Changes
- MSCKF: 12x12 matrix, includes rotation-position coupling
- po-KF: 3x3 zero matrix (simplified model)

### G Matrix Changes
- MSCKF: 12x12, maps all noise sources
- po-KF: 3x3, only maps velocity noise

### H Matrix Changes
- MSCKF: Includes Jacobians for both rotation and position
- po-KF: Only includes position Jacobians

### Covariance Matrices
- IMU covariance: 12x12 → 3x3
- Camera covariance: 6*N x 6*N → 3*N x 3*N

## Usage

Run po-KF:
```matlab
cd /path/to/MSCKF-master/msckf
POKF
```

Run original MSCKF for comparison:
```matlab
MSCKF
```

## Advantages and Disadvantages

### po-KF Advantages
1. Significantly reduced computational cost (75% state dimension reduction)
2. Simple implementation, easy to understand and debug
3. High accuracy when orientation is known

### po-KF Disadvantages
1. Requires external accurate orientation information
2. Cannot estimate IMU biases
3. Sensitive to orientation errors

## Applicable Scenarios

po-KF is suitable for:
- Systems with accurate orientation sensors (e.g., high-precision IMU or compass)
- Real-time systems requiring fast position estimation
- Research and teaching to understand MSCKF core concepts

## Key Implementation Points

1. **Ground Truth Orientation**: po-KF uses ground truth orientation at each timestep
2. **Reduced Jacobians**: All Jacobians exclude rotation error terms
3. **Simplified Propagation**: No quaternion integration needed
4. **Smaller Covariance**: More efficient matrix operations

## Testing

The implementation includes the same dataset and features as MSCKF:
- Uses KITTI dataset for validation
- Tracks features across camera frames
- Applies null space projection trick
- Uses QR decomposition for numerical stability

Compare results between MSCKF and po-KF to understand the trade-offs between full state estimation and position-only estimation.
