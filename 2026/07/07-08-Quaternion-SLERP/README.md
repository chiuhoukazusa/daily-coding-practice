# Quaternion Rotation & SLERP Interpolation

## 编译运行
```bash
g++ main.cpp -o quaternion -std=c++17 -O2
./quaternion
```

## 输出结果
![结果](quaternion_output.ppm)

## 技术要点
- Quaternion 四元数类：构造、归一化、共轭、乘法
- SLERP 球面线性插值，含最短路径处理
- 欧拉角 ↔ 四元数转换
- 四元数 → 旋转矩阵转换
- 万向节死锁演示（欧拉角 vs 四元数对比）
- 3D 点旋转可视化 + 定量验证
