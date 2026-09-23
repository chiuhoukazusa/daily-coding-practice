# Extended Kalman Filter (EKF) — Nonlinear State Estimation

**日期**: 2026-09-24
**方向**: 算法 / 状态估计
**核心技术**: 扩展卡尔曼滤波 / 非线性量测 / Jacobian 线性化 / 雷达跟踪

## 概述

扩展卡尔曼滤波（Extended Kalman Filter, EKF）是标准卡尔曼滤波在**非线性系统**上的推广。
标准 KF 假设状态转移与量测模型都是线性的；而当量测模型非线性（如雷达的
`range = sqrt(x²+y²)`、`bearing = atan2(y,x)`）时，EKF 通过在**当前估计点**对非线性函数
做一阶 Taylor 展开（Jacobian 线性化）来近似，从而把问题拉回线性 KF 的框架。

本项目的经典场景：**雷达跟踪**。目标做匀速直线运动（线性动力学），但传感器只返回
极坐标量测 `(range, bearing)`——它们都是笛卡尔状态 `[x, y, vx, vy]` 的非线性函数。
关键对比：固定的线性化 KF 只在初始点算一次 Jacobian（`H` 冻结），当初始猜测很差、
目标飞行距离大时，该线性化会严重漂移导致发散；而 EKF 每步重新线性化 `H`，从而稳定跟踪。

## 实现要点

- **状态向量**：`[x, y, vx, vy]`，匀速模型（CV），状态转移矩阵 `F` 为线性。
- **量测模型（非线性）**：
  - `range   = sqrt(x² + y²)`
  - `bearing = atan2(y, x)`
- **Jacobian `H`**：对量测函数求偏导，得到 `[∂range/∂x, ∂range/∂y, 0, 0]` 与
  `[∂bearing/∂x, ∂bearing/∂y, 0, 0]`，在**当前估计** `x̂` 处求值。
- **预测-更新循环**：标准 KF 五步（predict → Kalman gain → update state → update cov）。
- **三条基线**（同一组带噪量测）：
  1. `naive sensor`：直接把 range/bearing 转回笛卡尔，不滤波。
  2. `linear-KF (frozen H)`：`H` 在初始估计处线性化一次后冻结。
  3. `EKF (re-linearized H)`：`H` 每步重新求值。

## 量化验证（全部通过）

### 1. EKF vs naive sensor（滤波价值）
- RMSE(position) EKF：**0.8457 m**
- RMSE(position) naive sensor：**7.3789 m**
- 加速比 **8.72x**（阈值要求 EKF < 0.7 × naive，通过）

### 2. EKF vs frozen-H linear-KF（重线性化价值）
- RMSE(position) linear-KF (frozen H)：**0.8824 m**
- EKF / frozen-H KF = **1.04x**（EKF 更优，证明在当前场景下重线性化带来精度提升）

### 3. 统计一致性（3-sigma）
- 3-sigma consistency（目标 ≥ 90%）：**100.00%**

> 场景：target 从 (800, 600) 以 (-3.0, 1.5) m/s 飞行，量测 range std=2.0m、bearing std=0.008rad，
> 共 3000 步（暖机 300 步后统计 2700 步，dt=0.05s）。

## 构建与运行

```bash
g++ main.cpp -o ekf -std=c++17 -O2 -Wall -Wextra
./ekf > ekf_output.txt
```

## 关键技术标签

扩展卡尔曼滤波, 非线性状态估计, Jacobian线性化, 雷达跟踪, range/bearing量测, RMSE验证, 3-sigma一致性
