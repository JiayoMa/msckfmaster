# MSCKF to po-KF: Architecture Diagram

## System Architecture Comparison

```
┌─────────────────────────────────────────────────────────────────┐
│                          MSCKF System                            │
└─────────────────────────────────────────────────────────────────┘

Input: IMU (ω, v) + Camera Features
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  State Vector (12-dimensional)                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ δθ_IG (3)  │  b_g (3)  │  b_v (3)  │  p_I_G (3)          │  │
│  │  Rotation  │ Gyro Bias │ Vel Bias  │  Position           │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Propagation (F: 12×12, G: 12×12)                               │
│  • Quaternion integration                                        │
│  • Bias evolution                                                │
│  • Position propagation with rotation-position coupling          │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Augmentation (Add Camera States: 6D each)                      │
│  • Camera rotation (3D)                                          │
│  • Camera position (3D)                                          │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Update (H includes rotation + position Jacobians)              │
│  • Rotation Jacobian: J_i × [p_f_C]×                            │
│  • Position Jacobian: -J_i × C_CG                               │
└─────────────────────────────────────────────────────────────────┘
              ↓
Output: Position + Orientation + Biases


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


┌─────────────────────────────────────────────────────────────────┐
│                        po-KF System                              │
└─────────────────────────────────────────────────────────────────┘

Input: IMU (v only) + Camera Features + Ground Truth Orientation
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  State Vector (3-dimensional)                                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    p_I_G (3)                              │  │
│  │                    Position ONLY                          │  │
│  └──────────────────────────────────────────────────────────┘  │
│  Note: q_IG from ground truth, not estimated                    │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Propagation (F: 3×3 = 0, G: 3×3)                               │
│  • No quaternion integration (use ground truth)                  │
│  • No bias evolution                                             │
│  • Position propagation only: p = p + C_IG' × v × dt            │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Augmentation (Add Camera States: 3D each)                      │
│  • Camera position (3D) - derived from IMU position              │
│  • Camera rotation from ground truth IMU rotation               │
└─────────────────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Update (H includes position Jacobians only)                    │
│  • Position Jacobian: -J_i × C_CG                               │
│  • No rotation Jacobian (rotation not in state)                 │
└─────────────────────────────────────────────────────────────────┘
              ↓
Output: Position (Orientation from Ground Truth)
```

## Data Flow Comparison

### MSCKF Data Flow
```
┌──────────┐     ┌─────────────┐     ┌──────────────┐
│   IMU    │────▶│ Quaternion  │────▶│   Rotation   │
│ (ω, v)   │     │ Integration │     │  Estimation  │
└──────────┘     └─────────────┘     └──────────────┘
                                             │
                                             ▼
┌──────────┐     ┌─────────────┐     ┌──────────────┐
│ Camera   │────▶│   Feature   │────▶│   Position   │
│ Features │     │  Tracking   │     │  Estimation  │
└──────────┘     └─────────────┘     └──────────────┘
```

### po-KF Data Flow
```
┌──────────┐                         ┌──────────────┐
│ Ground   │────────────────────────▶│   Rotation   │
│  Truth   │                         │  (Known)     │
└──────────┘                         └──────────────┘
                                             │
                                             ▼
┌──────────┐     ┌─────────────┐     ┌──────────────┐
│   IMU    │────▶│  Velocity   │────▶│   Position   │
│   (v)    │     │ Integration │     │  Estimation  │
└──────────┘     └─────────────┘     └──────────────┘
                                             ▲
                                             │
┌──────────┐     ┌─────────────┐            │
│ Camera   │────▶│   Feature   │────────────┘
│ Features │     │  Tracking   │
└──────────┘     └─────────────┘
```

## Matrix Dimension Evolution

### MSCKF
```
Time Step k:
┌────────────────────────────────────────┐
│ State: x_k ∈ ℝ^(12+6N)                 │
│ ┌─────┬─────┬─────┬─────┬──────────┐  │
│ │ IMU │Cam₁ │Cam₂ │ ... │  CamN    │  │
│ │ 12D │ 6D  │ 6D  │     │   6D     │  │
│ └─────┴─────┴─────┴─────┴──────────┘  │
└────────────────────────────────────────┘

Covariance: P_k ∈ ℝ^((12+6N)×(12+6N))
┌────────────────────────────────────────┐
│         (12+6N) × (12+6N)              │
│  ┌──────────────────────────────────┐ │
│  │ P_II │ P_IC₁│ P_IC₂│ ... │ P_ICN│ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_C₁I│ P_C₁C₁│     │     │      │ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_C₂I│      │P_C₂C₂│     │      │ │
│  │  ⋮   │      │      │ ⋱   │      │ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_CNI│      │      │     │P_CNCN│ │
│  └──────────────────────────────────┘ │
└────────────────────────────────────────┘
```

### po-KF
```
Time Step k:
┌────────────────────────────────────────┐
│ State: x_k ∈ ℝ^(3+3N)                  │
│ ┌─────┬─────┬─────┬─────┬──────────┐  │
│ │ IMU │Cam₁ │Cam₂ │ ... │  CamN    │  │
│ │  3D │ 3D  │ 3D  │     │   3D     │  │
│ └─────┴─────┴─────┴─────┴──────────┘  │
└────────────────────────────────────────┘

Covariance: P_k ∈ ℝ^((3+3N)×(3+3N))
┌────────────────────────────────────────┐
│          (3+3N) × (3+3N)               │
│  ┌──────────────────────────────────┐ │
│  │ P_II │ P_IC₁│ P_IC₂│ ... │ P_ICN│ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_C₁I│ P_C₁C₁│     │     │      │ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_C₂I│      │P_C₂C₂│     │      │ │
│  │  ⋮   │      │      │ ⋱   │      │ │
│  │──────┼──────┼──────┼─────┼──────│ │
│  │ P_CNI│      │      │     │P_CNCN│ │
│  └──────────────────────────────────┘ │
└────────────────────────────────────────┘

75% smaller than MSCKF!
```

## Computational Complexity

### Per Iteration Operations

```
MSCKF:
┌────────────────────────────────────────┐
│ 1. Quaternion Integration    ~40 ops  │
│ 2. F Matrix (12×12)          ~100 ops │
│ 3. G Matrix (12×12)           ~50 ops │
│ 4. Covariance (12×12)       ~1728 ops │
│ ─────────────────────────────────────│
│ Total (propagation)         ~1918 ops │
└────────────────────────────────────────┘

po-KF:
┌────────────────────────────────────────┐
│ 1. Quaternion Integration      0 ops  │
│ 2. F Matrix (3×3)              0 ops  │
│ 3. G Matrix (3×3)            ~27 ops  │
│ 4. Covariance (3×3)          ~54 ops  │
│ ─────────────────────────────────────│
│ Total (propagation)          ~81 ops  │
└────────────────────────────────────────┘

Speedup: 1918 / 81 ≈ 23.7×
```

## Memory Footprint

```
MSCKF (N=10 cameras):
┌────────────────────────────────────────┐
│ State vector:     12+6×10 = 72 floats │
│ Covariance:      72×72 = 5184 floats  │
│ Total:                    5256 floats │
│                          ≈ 21 KB      │
└────────────────────────────────────────┘

po-KF (N=10 cameras):
┌────────────────────────────────────────┐
│ State vector:      3+3×10 = 33 floats │
│ Covariance:      33×33 = 1089 floats  │
│ Total:                    1122 floats │
│                          ≈ 4.5 KB     │
└────────────────────────────────────────┘

Memory Reduction: 21 KB → 4.5 KB (78% reduction)
```

## Trade-offs Summary

```
┌─────────────────────────────────────────────────────────┐
│                    MSCKF Advantages                      │
├─────────────────────────────────────────────────────────┤
│ ✓ Fully autonomous (no external sensors needed)         │
│ ✓ Estimates orientation + position + biases             │
│ ✓ Online IMU calibration                                │
│ ✓ Robust to initial conditions                          │
└─────────────────────────────────────────────────────────┘
                           vs
┌─────────────────────────────────────────────────────────┐
│                     po-KF Advantages                     │
├─────────────────────────────────────────────────────────┤
│ ✓ 23× faster computation                                │
│ ✓ 75% smaller state dimension                           │
│ ✓ 78% less memory usage                                 │
│ ✓ Simpler implementation                                │
│ ✓ Better numerical stability (smaller matrices)         │
└─────────────────────────────────────────────────────────┘
```

## Use Case Decision Tree

```
                    Need full 6-DOF estimation?
                            │
                ┌───────────┴───────────┐
               YES                      NO
                │                        │
        ┌───────┴────────┐              │
        │                │              │
  Have external    No external         │
   orientation?     orientation        │
        │                │              │
        │                │              │
    Use MSCKF       Use MSCKF      Use po-KF
   (redundant)     (necessary)    (optimal)
        │                │              │
        └────────────────┴──────────────┘
                         │
              Choose based on requirements:
              • Computation budget
              • Memory constraints
              • Accuracy needs
              • Implementation complexity
```
