# MSCKF到po-KF的最小改动实现指南

## 概述
本文档说明如何通过最小改动将MSCKF（Multi-State Constraint Kalman Filter）转换为po-KF（Position-Only Kalman Filter）。

## po-KF vs MSCKF 核心区别

### MSCKF状态向量 (12维)
- 旋转误差: δθ (3维)
- 陀螺仪偏置: b_g (3维)  
- 速度偏置: b_v (3维)
- 位置: p_I_G (3维)

### po-KF状态向量 (3维)
- 位置: p_I_G (3维)
- **旋转从真值或其他来源获得，不在卡尔曼滤波中估计**

## 需要修改的文件和对应代码

### 1. 主程序文件: POKF.m (新建)
基于 `MSCKF.m` 创建，主要改动：

```matlab
% 第42-46行: 简化噪声参数矩阵
v_var = 4e-2 * ones(1,3);
dbv_var = 1e-6 * ones(1,3);
% 仅包含速度噪声，移除旋转和陀螺仪相关噪声
noiseParams.Q_imu = diag([v_var, dbv_var]);

% 第48-49行: 简化初始协方差
p_var_init = 1e-6 * ones(1,3);
bv_var_init = 1e-6 * ones(1,3);
noiseParams.initialIMUCovar = diag([bv_var_init, p_var_init]);

% 第117行: 从真值获取方向
q_IG_true = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k+1)));

% 第120行: 状态传播时传入真实方向
pokfState = propagatePokfStateAndCovar(pokfState, measurements{state_k}, noiseParams, q_IG_true);

% 第213-215行: 协方差矩阵维度变为 (3 + 3*相机数)
tempMat = (eye(3 + 3*size(pokfState.camStates,2)) - K*T_H);
pokfState.imuCovar = P_corrected(1:3,1:3);  % 3x3而非12x12
pokfState.camCovar = P_corrected(4:end,4:end);
```

### 2. 状态传播: propagatePokfStateAndCovar.m (新建)
基于 `propagateMsckfStateAndCovar.m`，主要改动：

```matlab
% 第20-21行: 简化的F矩阵（3x3而非12x12）
% 位置误差动力学简化，因为方向已知
F = zeros(3,3);

% 第24行: 简化的G矩阵
G = -C_IG';  % 仅速度噪声影响位置

% 第31行: 状态转移矩阵变为3x3
Phi = eye(3) + F * measurements_k.dT;

% 第34-36行: 协方差传播（3x3）
Q_v = diag(noiseParams.Q_imu(1:3));
pokfState_prop.imuCovar = Phi * pokfState.imuCovar * Phi' ...
                            + G * Q_v * G' * measurements_k.dT;
```

### 3. IMU状态传播: propagateImuStatePOKF.m (新建)
基于 `propagateImuState.m`，主要改动：

```matlab
% 第14行: 使用真值方向而非估计
imuState_prop.q_IG = q_IG_true;

% 移除第15-29行的四元数更新代码
% 不需要陀螺仪测量来更新方向

% 第22行: 陀螺仪偏置设为零
imuState_prop.b_g = zeros(3,1);
```

### 4. 状态增广: augmentStatePOKF.m (新建)
基于 `augmentState.m`，主要改动：

```matlab
% 第29行: 简化的雅可比矩阵（3x3）
J = eye(3);  % 相机位置误差 = IMU位置误差

% 移除旋转相关的雅可比项
% 原MSCKF中的 J(1:3,1:3) = C_CI 等项被移除

% 第34-36行: 协方差增广（3维而非6维）
pokfState.camCovar = [pokfState.camCovar, tempMat(1:3, 4:end)';
                      tempMat(1:3, 4:end), tempMat(1:3, 1:3)];
```

### 5. 观测雅可比: calcHojPOKF.m (新建)
基于 `calcHoj.m`，主要改动：

```matlab
% 第14行: H_x_j维度改为 (2*M, 3 + 3*N)
H_x_j = zeros(2*M, 3 + 3*N);  % 每个状态仅3维位置

% 移除第51行的旋转雅可比
% 原代码: H_x_j(..., :) = J_i*crossMat(p_f_C);
% po-KF: 移除此项，因为方向误差不在状态中

% 第53行: 仅保留位置雅可比
H_x_j((2*c_i - 1):2*c_i, 3+3*(camStateIndex-1) + 1:3+3*(camStateIndex-1) + 3) = -J_i*C_CG;
```

### 6. 状态更新: updateStatePOKF.m (新建)
基于 `updateState.m`，主要改动：

```matlab
% 第16行: 仅更新位置
deltap_I_G = deltaX(1:3);
pokfState_up.imuState.p_I_G = pokfState.imuState.p_I_G + deltap_I_G;

% 移除第17-24行的旋转和偏置更新
% 方向和偏置保持不变（来自真值）

% 第31-40行: 相机状态仅更新位置
for i = 1:size(pokfState.camStates, 2)
    pStart = 3 + 3*(i-1) + 1;
    deltap_C_G = deltaX(pStart:pStart+2);
    pokfState_up.camStates{i}.p_C_G = pokfState.camStates{i}.p_C_G + deltap_C_G;
end
```

### 7. 其他辅助函数

#### initializePOKF.m (新建)
```matlab
% 第15行: 3x3协方差矩阵
pokfState.imuCovar = noiseParams.initialIMUCovar(4:6, 4:6);
```

#### removeTrackedFeaturePOKF.m (新建)
与MSCKF版本相同，无需修改核心逻辑

#### pruneStatesPOKF.m (新建)
```matlab
% 第26行: 调整协方差提取索引（每个状态3维）
covStartIdx = 3 + 3*(idx-1) + 1;
covEndIdx = covStartIdx + 2;
```

#### updateStateHistoryPOKF.m (新建)
简单存储，方向来自真值，位置来自滤波器

## 核心修改总结

### 状态维度变化
- IMU状态: 12维 → 3维
- 相机状态: 6维 → 3维
- 总状态: (12 + 6*N) → (3 + 3*N)

### F矩阵变化
- MSCKF: 12x12矩阵，包含旋转-位置耦合
- po-KF: 3x3零矩阵（简化模型）

### G矩阵变化
- MSCKF: 12x12，映射所有噪声源
- po-KF: 3x3，仅映射速度噪声

### H矩阵变化
- MSCKF: 包含旋转和位置的雅可比
- po-KF: 仅包含位置的雅可比

### 协方差矩阵
- IMU协方差: 12x12 → 3x3
- 相机协方差: 6*N x 6*N → 3*N x 3*N

## 使用方法

运行po-KF：
```matlab
cd /path/to/MSCKF-master/msckf
POKF
```

运行原MSCKF进行对比：
```matlab
MSCKF
```

## 优缺点分析

### po-KF优点
1. 计算量大幅降低（状态维度减少75%）
2. 实现简单，易于理解和调试
3. 在方向已知的场景下准确性高

### po-KF缺点
1. 需要外部提供准确的方向信息
2. 无法估计IMU偏置
3. 对方向误差敏感

## 适用场景

po-KF适用于：
- 有准确方向传感器（如高精度IMU或罗盘）的系统
- 需要快速位置估计的实时系统
- 研究和教学，理解MSCKF核心概念
