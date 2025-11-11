# MSCKF vs po-KF: 详细对比和修改说明

## 一、概述

本文档详细说明如何从MSCKF（Multi-State Constraint Kalman Filter）修改得到po-KF（Position-Only Kalman Filter）。

## 二、两种滤波器的数学对比

### 2.1 状态向量

**MSCKF状态向量 (12维):**
```
x_IMU = [δθ_IG(3), δb_g(3), δb_v(3), δp_I_G(3)]ᵀ
```
其中：
- δθ_IG: 旋转误差（相对于Global坐标系的IMU姿态误差）
- δb_g: 陀螺仪偏置误差
- δb_v: 加速度计/速度偏置误差
- δp_I_G: 位置误差

**po-KF状态向量 (3维):**
```
x_IMU = [δp_I_G(3)]ᵀ
```
仅包含位置误差，旋转从外部获得

### 2.2 状态传播方程

**MSCKF:**
```
ẋ = F·x + G·n

F = [  -[ω̂]×     -I₃      0      0   ]
    [    0         0       0      0   ]
    [    0         0       0      0   ]
    [ -C_IG'[v̂]×   0    -C_IG'   0   ]

G = [ -I₃    0     0     0  ]
    [  0    I₃     0     0  ]
    [  0     0     0    I₃  ]
    [  0     0  -C_IG'  0  ]

其中 ω̂ = ω - b_g, v̂ = v - b_v
```

**po-KF:**
```
ẋ = F·x + G·n

F = [0₃ₓ₃]  （零矩阵，因为位置误差不自相关）

G = [-C_IG']  （仅速度噪声影响位置）

其中 C_IG 来自真值或外部传感器
```

### 2.3 协方差传播

**MSCKF:**
```
P_k+1 = Φ_k·P_k·Φ_k' + G·Q·G'·Δt
Φ_k = I₁₂ + F·Δt  (12×12矩阵)
Q = diag([σ_ω², σ_bg², σ_bv², σ_v²])  (12×12)
```

**po-KF:**
```
P_k+1 = Φ_k·P_k·Φ_k' + G·Q_v·G'·Δt
Φ_k = I₃ + F·Δt = I₃  (3×3单位矩阵)
Q_v = diag([σ_v²])  (3×3)
```

### 2.4 状态增广（Adding Camera States）

**MSCKF:**
```
相机状态: [δθ_CG(3), δp_C_G(3)]ᵀ  (6维)

J = [ C_CI    0  ]  (6×12)
    [  0   I₃-[p_C_I]×C_CI ]

新协方差维度: (12 + 6N) × (12 + 6N)
```

**po-KF:**
```
相机状态: [δp_C_G(3)]ᵀ  (3维)

J = [I₃]  (3×3)

新协方差维度: (3 + 3N) × (3 + 3N)
```

### 2.5 观测方程

**MSCKF观测雅可比 H (对第i个相机):**
```
H_x = [0_{2×3}, 0_{2×3}, 0_{2×3}, 0_{2×3}, ..., H_θ_i, H_p_i, ..., 0_{2×3}, 0_{2×3}]

H_θ_i = J_π · [p_f_C]×  (对旋转的雅可比)
H_p_i = -J_π · C_CG     (对位置的雅可比)

其中 J_π = [1/Z  0  -X/Z²]
           [0  1/Z  -Y/Z²]
```

**po-KF观测雅可比 H (对第i个相机):**
```
H_x = [0_{2×3}, ..., H_p_i, ..., 0_{2×3}]

H_p_i = -J_π · C_CG  (仅对位置的雅可比)

移除了旋转项 H_θ_i，因为旋转不在状态中
```

## 三、代码逐行对比

### 3.1 主循环中的状态传播

**MSCKF.m (第227行):**
```matlab
msckfState = propagateMsckfStateAndCovar(msckfState, measurements{state_k}, noiseParams);
```

**POKF.m (第117-120行):**
```matlab
% 新增：获取真值旋转
q_IG_true = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k+1)));

% 传播时传入真值旋转
pokfState = propagatePokfStateAndCovar(pokfState, measurements{state_k}, noiseParams, q_IG_true);
```

### 3.2 propagateMsckfStateAndCovar.m vs propagatePokfStateAndCovar.m

**MSCKF版本 (第13-15行):**
```matlab
F = calcF(msckfState.imuState, measurements_k);
G = calcG(msckfState.imuState);
```

**po-KF版本 (第20-24行):**
```matlab
C_IG = quatToRotMat(q_IG_true);
vHat = measurements_k.v - pokfState.imuState.b_v;

F = zeros(3,3);  % 简化
G = -C_IG';      % 仅速度噪声
```

**MSCKF版本 (第23-24行):**
```matlab
Phi = eye(size(F,1)) + F * measurements_k.dT;  % 12×12
```

**po-KF版本 (第31行):**
```matlab
Phi = eye(3) + F * measurements_k.dT;  % 3×3
```

**MSCKF版本 (第35-36行):**
```matlab
msckfState_prop.imuCovar = Phi * msckfState.imuCovar * Phi' ...
                            + G * Q_imu * G' * measurements_k.dT;
```

**po-KF版本 (第34-36行):**
```matlab
Q_v = diag(noiseParams.Q_imu(1:3));  % 仅速度方差
pokfState_prop.imuCovar = Phi * pokfState.imuCovar * Phi' ...
                            + G * Q_v * G' * measurements_k.dT;
```

### 3.3 augmentState.m vs augmentStatePOKF.m

**MSCKF版本 (第26-39行) - 完整雅可比:**
```matlab
J = zeros(6,12);
J(1:3,1:3) = C_CI;
J(4:6,1:3) = -C_CI * crossMat(camera.p_C_I);
J(4:6,10:12) = eye(3);

% 增广协方差
msckfState.imuCamCovar = [msckfState.imuCamCovar, msckfState.imuCovar * J'];
tempMat = [J * msckfState.imuCovar, J * msckfState.imuCamCovar];
msckfState.camCovar = [msckfState.camCovar, tempMat(1:6, 7:end)';
                       tempMat(1:6, 7:end), tempMat(1:6, 1:6)];
```

**po-KF版本 (第29-36行) - 简化雅可比:**
```matlab
J = eye(3);  % 相机位置误差 = IMU位置误差

% 增广协方差
pokfState.imuCamCovar = [pokfState.imuCamCovar, pokfState.imuCovar * J'];
tempMat = [J * pokfState.imuCovar, J * pokfState.imuCamCovar];
pokfState.camCovar = [pokfState.camCovar, tempMat(1:3, 4:end)';
                      tempMat(1:3, 4:end), tempMat(1:3, 1:3)];
```

### 3.4 calcHoj.m vs calcHojPOKF.m

**MSCKF版本 (第24-25行):**
```matlab
H_f_j = zeros(2*M, 3);
H_x_j = zeros(2*M, 12 + 6*N);  % 包含旋转和位置
```

**po-KF版本 (第14-15行):**
```matlab
H_f_j = zeros(2*M, 3);
H_x_j = zeros(2*M, 3 + 3*N);   % 仅位置
```

**MSCKF版本 (第50-53行) - 包含旋转和位置雅可比:**
```matlab
% 旋转雅可比
H_x_j((2*c_i - 1):2*c_i, 12+6*(camStateIndex-1) + 1:12+6*(camStateIndex-1) + 3) = J_i*crossMat(p_f_C);
% 位置雅可比
H_x_j((2*c_i - 1):2*c_i, (12+6*(camStateIndex-1) + 4):(12+6*(camStateIndex-1) + 6)) = -J_i*C_CG;
```

**po-KF版本 (第35-37行) - 仅位置雅可比:**
```matlab
% 仅位置雅可比
H_x_j((2*c_i - 1):2*c_i, 3+3*(camStateIndex-1) + 1:3+3*(camStateIndex-1) + 3) = -J_i*C_CG;
% 无旋转项
```

### 3.5 updateState.m vs updateStatePOKF.m

**MSCKF版本 (第17-27行) - 更新所有状态:**
```matlab
deltatheta_IG = deltaX(1:3);
deltab_g = deltaX(4:6);
deltab_v = deltaX(7:9);
deltap_I_G = deltaX(10:12);

deltaq_IG = buildUpdateQuat(deltatheta_IG);
msckfState_up.imuState.q_IG = quatLeftComp(deltaq_IG) * msckfState.imuState.q_IG;
msckfState_up.imuState.b_g = msckfState.imuState.b_g + deltab_g;
msckfState_up.imuState.b_v = msckfState.imuState.b_v + deltab_v;
msckfState_up.imuState.p_I_G = msckfState.imuState.p_I_G + deltap_I_G;
```

**po-KF版本 (第16-23行) - 仅更新位置:**
```matlab
deltap_I_G = deltaX(1:3);
pokfState_up.imuState.p_I_G = pokfState.imuState.p_I_G + deltap_I_G;

% 旋转和偏置保持不变（来自真值）
pokfState_up.imuState.q_IG = pokfState.imuState.q_IG;
pokfState_up.imuState.b_g = pokfState.imuState.b_g;
pokfState_up.imuState.b_v = pokfState.imuState.b_v;
```

**MSCKF版本 (第32-40行) - 相机状态包含旋转和位置:**
```matlab
for i = 1:size(msckfState.camStates, 2)
    qStart = 12 + 6*(i-1) + 1;
    pStart = qStart+3;
    
    deltatheta_CG = deltaX(qStart:qStart+2);
    deltap_C_G = deltaX(pStart:pStart+2);
    
    deltaq_CG = buildUpdateQuat(deltatheta_CG);
    msckfState_up.camStates{i}.q_CG = quatLeftComp(deltaq_CG) * msckfState.camStates{i}.q_CG;
    msckfState_up.camStates{i}.p_C_G = msckfState.camStates{i}.p_C_G + deltap_C_G;
end
```

**po-KF版本 (第26-35行) - 相机状态仅位置:**
```matlab
for i = 1:size(pokfState.camStates, 2)
    pStart = 3 + 3*(i-1) + 1;
    
    deltap_C_G = deltaX(pStart:pStart+2);
    
    pokfState_up.camStates{i}.p_C_G = pokfState.camStates{i}.p_C_G + deltap_C_G;
    
    % 相机旋转保持不变（从真值IMU旋转派生）
    pokfState_up.camStates{i}.q_CG = pokfState.camStates{i}.q_CG;
end
```

## 四、计算复杂度分析

### 4.1 矩阵维度对比

| 操作 | MSCKF | po-KF | 降低比例 |
|-----|-------|-------|---------|
| 状态向量 | 12+6N | 3+3N | 75% |
| 协方差矩阵 | (12+6N)² | (3+3N)² | ~93% (当N=10) |
| F矩阵乘法 | 12×12 | 3×3 | 93% |
| Φ计算 | O(144) | O(9) | 93% |

### 4.2 每次迭代的主要运算

**MSCKF:**
- 四元数积分: ~40 FLOPs
- F矩阵构建: ~100 FLOPs
- G矩阵构建: ~50 FLOPs
- 协方差传播 (12×12): ~1728 FLOPs
- 总计: ~1918 FLOPs (不含观测更新)

**po-KF:**
- 无四元数积分: 0 FLOPs
- F矩阵构建: ~0 FLOPs (零矩阵)
- G矩阵构建: ~27 FLOPs
- 协方差传播 (3×3): ~54 FLOPs
- 总计: ~81 FLOPs (不含观测更新)

**加速比: ~23倍** (仅传播部分)

## 五、优缺点对比

### 5.1 po-KF优点

1. **计算效率高**
   - 状态维度减少75%
   - 协方差更新快23倍
   - 适合实时系统

2. **实现简单**
   - 无需四元数运算
   - 矩阵维度小
   - 易于调试

3. **数值稳定性好**
   - 小矩阵求逆更稳定
   - 减少累积误差

### 5.2 po-KF缺点

1. **需要外部旋转**
   - 依赖准确的旋转传感器
   - 无法自主估计姿态

2. **无法估计偏置**
   - 陀螺仪偏置不估计
   - 速度偏置不估计

3. **误差传播**
   - 旋转误差直接影响位置
   - 无法校正旋转误差

### 5.3 MSCKF优点

1. **完全自主**
   - 不依赖外部传感器
   - 同时估计位置和姿态

2. **偏置估计**
   - 在线校准IMU
   - 长时间精度高

3. **鲁棒性强**
   - 对初始条件要求低
   - 可处理大姿态变化

### 5.4 MSCKF缺点

1. **计算复杂**
   - 状态维度大
   - 需要四元数运算

2. **实现复杂**
   - 多种雅可比矩阵
   - 调试困难

## 六、使用建议

### 6.1 选择po-KF的场景

✅ **推荐使用:**
- 有可靠的外部姿态参考（如高精度INS、星敏感器）
- 计算资源受限（嵌入式系统）
- 仅需要位置估计（如某些导航任务）
- 教学和研究（理解MSCKF核心思想）

### 6.2 选择MSCKF的场景

✅ **推荐使用:**
- 无外部姿态参考
- 需要完整的6-DOF估计
- 需要在线IMU校准
- 对精度要求高的应用

## 七、总结

po-KF是MSCKF的简化版本，通过以下最小改动实现：

1. **状态简化**: 仅保留位置 (12维 → 3维)
2. **旋转外部化**: 从真值或传感器获取
3. **矩阵简化**: F、G、H矩阵维度大幅降低
4. **计算优化**: 速度提升约23倍（传播部分）

所有修改都保持了MSCKF的核心算法框架，代码重用率高，易于理解和维护。
