# PBD Soft Body Deformation

Position-Based Dynamics (PBD) soft body simulation with SDF-based ground collision,
rendered via software rasterizer. Shows elastic deformation across multiple time steps.

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果

![结果](pbd_softbody_output.png)

## 技术要点

- PBD 位置约束迭代求解（结构/剪切/弯曲约束）
- SDF 地面碰撞检测与穿透修正
- 粒子速度热图染色（蓝→青→绿→黄→红）
- 软光栅化线框渲染，约束拉伸状态可视化
- 多帧合成（8×10 帧网格布局）
