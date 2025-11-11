# 如何修改MSCKF代码实现po-KF（位置唯一卡尔曼滤波器）

## 快速概览

**问题**: 以最小的改动，通过MSCKF的代码，实现po-KF

**解决方案**: 已创建完整的po-KF实现，包含11个新文件和详细文档

## 核心思想

po-KF（Position-Only Kalman Filter）是MSCKF的简化版本：
- **MSCKF估计**: 旋转 + 陀螺仪偏置 + 速度偏置 + 位置 (12维)
- **po-KF估计**: 仅位置 (3维)
- **关键区别**: po-KF从真值或外部传感器获取旋转，不在滤波器中估计

## 修改对照表

| 原MSCKF文件 | po-KF对应文件 | 主要修改 |
|------------|--------------|---------|
| MSCKF.m | POKF.m | 状态维度12→3，协方差维度缩减 |
| propagateMsckfStateAndCovar.m | propagatePokfStateAndCovar.m | F矩阵12x12→3x3，G矩阵简化 |
| propagateImuState.m | propagateImuStatePOKF.m | 移除四元数积分，使用真值旋转 |
| augmentState.m | augmentStatePOKF.m | 雅可比矩阵6x12→3x3 |
| calcHoj.m | calcHojPOKF.m | 移除旋转雅可比，仅保留位置 |
| updateState.m | updateStatePOKF.m | 仅更新位置，不更新旋转和偏置 |
| initializeMSCKF.m | initializePOKF.m | 初始协方差12x12→3x3 |
| pruneStates.m | pruneStatesPOKF.m | 协方差索引调整（6→3维） |
| removeTrackedFeature.m | removeTrackedFeaturePOKF.m | 基本不变 |
| updateStateHistory.m | updateStateHistoryPOKF.m | 旋转来自真值 |

## 关键代码修改点

### 1. 状态向量维度

**MSCKF状态 (12维)**
```matlab
% [旋转误差(3), 陀螺仪偏置(3), 速度偏置(3), 位置(3)]
imuState.q_IG    % 4x1 四元数
imuState.b_g     % 3x1 陀螺仪偏置
imuState.b_v     % 3x1 速度偏置  
imuState.p_I_G   % 3x1 位置
```

**po-KF状态 (3维)**
```matlab
% [位置(3)]
imuState.q_IG    % 4x1 从真值获取（不估计）
imuState.b_g     % 3x1 设为零（不估计）
imuState.b_v     % 3x1 设为零（不估计）
imuState.p_I_G   % 3x1 位置（唯一估计的状态）
```

### 2. F矩阵（状态转移雅可比）

**MSCKF的F矩阵 (12x12)**
```matlab
F = zeros(12,12);
omegaHat = measurements_k.omega - imuState_k.b_g;
vHat = measurements_k.v - imuState_k.b_v;
C_IG = quatToRotMat(imuState_k.q_IG);

F(1:3,1:3) = -crossMat(omegaHat);     % 旋转动力学
F(1:3,4:6) = -eye(3);                 % 陀螺仪偏置影响
F(10:12,1:3) = -C_IG' * crossMat(vHat); % 旋转-位置耦合
F(10:12,7:9) = -C_IG';                % 速度偏置影响
```

**po-KF的F矩阵 (3x3)**
```matlab
F = zeros(3,3);  % 简化为零矩阵
% 因为旋转已知，位置误差动力学大幅简化
```

### 3. G矩阵（噪声映射）

**MSCKF的G矩阵 (12x12)**
```matlab
G = zeros(12,12);
C_IG = quatToRotMat(imuState_k.q_IG);

G(1:3,1:3) = -eye(3);      % 旋转噪声
G(4:6,4:6) = eye(3);       % 陀螺仪偏置噪声
G(7:9,10:12) = eye(3);     % 速度偏置噪声
G(10:12,7:9) = -C_IG';     % 速度噪声→位置
```

**po-KF的G矩阵 (3x3)**
```matlab
G = -C_IG';  % 仅速度噪声影响位置
```

### 4. H矩阵（观测雅可比）

**MSCKF的H矩阵**
```matlab
% 对每个相机状态 (6维: 3旋转 + 3位置)
H_x_j(..., 12+6*(i-1) + 1:12+6*(i-1) + 3) = J_i*crossMat(p_f_C);  % 旋转
H_x_j(..., 12+6*(i-1) + 4:12+6*(i-1) + 6) = -J_i*C_CG;           % 位置
```

**po-KF的H矩阵**
```matlab
% 对每个相机状态 (3维: 仅位置)
H_x_j(..., 3+3*(i-1) + 1:3+3*(i-1) + 3) = -J_i*C_CG;  % 仅位置
% 移除旋转项
```

### 5. 协方差矩阵维度

**MSCKF协方差**
```matlab
% IMU: 12x12
% 相机: 6*N x 6*N (N为相机数量)
% 总维度: (12 + 6*N) x (12 + 6*N)
P = [msckfState.imuCovar, msckfState.imuCamCovar;
     msckfState.imuCamCovar', msckfState.camCovar];
```

**po-KF协方差**
```matlab
% IMU: 3x3
% 相机: 3*N x 3*N
% 总维度: (3 + 3*N) x (3 + 3*N)
P = [pokfState.imuCovar, pokfState.imuCamCovar;
     pokfState.imuCamCovar', pokfState.camCovar];
```

### 6. 旋转处理

**MSCKF: 估计旋转**
```matlab
% 四元数积分
psi = (measurements_k.omega - imuState_k.b_g) * measurements_k.dT;
imuState_prop.q_IG = imuState_k.q_IG + 0.5 * omegaMat(psi) * imuState_k.q_IG;
imuState_prop.q_IG = imuState_prop.q_IG/norm(imuState_prop.q_IG);
```

**po-KF: 使用真值旋转**
```matlab
% 直接使用真值
imuState_prop.q_IG = q_IG_true;
```

## 使用方法

### 运行po-KF
```matlab
cd MSCKF-master/msckf
POKF
```

### 运行原MSCKF对比
```matlab
cd MSCKF-master/msckf
MSCKF
```

## 性能对比

| 指标 | MSCKF | po-KF | 变化 |
|-----|-------|-------|-----|
| 状态维度 | 12 + 6*N | 3 + 3*N | -75% |
| 协方差矩阵 | (12+6N)² | (3+3N)² | -75% |
| F矩阵运算 | 12x12 | 3x3 | -93% |
| 计算复杂度 | O(N³) | O(n³), n<<N | 显著降低 |
| 需要陀螺仪 | 是 | 否 | - |
| 需要外部旋转 | 否 | 是 | - |

## 适用场景

### po-KF适合：
✅ 有准确的外部旋转传感器（如高精度IMU、罗盘）
✅ 需要快速实时位置估计
✅ 计算资源受限的嵌入式系统
✅ 研究和教学目的

### po-KF不适合：
❌ 没有可靠的旋转信息源
❌ 需要估计IMU偏置
❌ 旋转精度要求高的应用

## 文件清单

实现包含以下文件（位于`MSCKF-master/msckf/`）：

1. **POKF.m** - 主程序
2. **propagatePokfStateAndCovar.m** - 状态和协方差传播
3. **propagateImuStatePOKF.m** - IMU状态传播
4. **augmentStatePOKF.m** - 状态增广
5. **calcHojPOKF.m** - 观测雅可比计算
6. **updateStatePOKF.m** - 状态更新
7. **initializePOKF.m** - 初始化
8. **pruneStatesPOKF.m** - 状态修剪
9. **removeTrackedFeaturePOKF.m** - 特征移除
10. **updateStateHistoryPOKF.m** - 历史状态更新

文档：
- **POKF_Implementation_Guide.md** - 中文详细指南
- **POKF_Implementation_Guide_EN.md** - 英文详细指南

## 总结

通过这些最小改动，我们成功实现了po-KF：

1. **状态简化**: 12维 → 3维（仅位置）
2. **旋转处理**: 从估计 → 使用真值
3. **矩阵维度**: 所有相关矩阵维度相应缩减
4. **计算效率**: 大幅提升（约75%状态维度减少）
5. **代码重用**: 最大化复用MSCKF现有函数（如calcResidual, calcGNPosEst等）

所有修改都遵循"最小改动"原则，保持MSCKF的核心算法结构不变。
