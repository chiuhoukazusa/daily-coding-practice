# RK4 Runge-Kutta ODE Solver

## 项目概述
对比 Euler 法和 4 阶 Runge-Kutta (RK4) 法求解弹簧-质量-阻尼系统的常微分方程。通过量化分析展示 RK4 相对于 Euler 法的精度优势。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
- `trajectory.ppm` - 轨迹对比图（红色=Euler, 蓝色=RK4, 黑色=解析解）
- `energy.ppm` - 能量守恒对比图（红色=Euler, 蓝色=RK4）

## 量化验证结果
- **最终位置误差**: Euler 0.908 m vs RK4 0.000004 m（RK4 精度高 37,778 倍）
- **能量守恒**: Euler 能量误差 5.15 J（50%能量误差增长）vs RK4 误差 0.000002 J
- **收敛阶**: Euler ~2.2, RK4 ~3.7（接近理论值 O(dt) 和 O(dt⁴)）
- **能量单调性**: Euler 100/200步能量异常增加, RK4 0/200步

## 技术要点
- 4 阶 Runge-Kutta 法 (RK4)
- Euler 法作为基准对比
- 解析解（欠阻尼谐振动）作为真值
- 收敛性研究（5 个不同时间步长）
- 能量守恒分析
- PPM 格式可视化
