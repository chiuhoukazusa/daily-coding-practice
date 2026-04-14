# Cloth Simulation - Mass-Spring System

> **每日编程实践** | 2026-04-15

## 项目简介

实现了一个基于**质点弹簧（Mass-Spring）**系统的布料物理模拟，包含：
- **Verlet 积分**——位置 Verlet，数值稳定，无需显式速度
- **三类弹簧**：结构弹簧（Structural）/ 剪切弹簧（Shear）/ 弯曲弹簧（Bending）
- **重力 + 时变风力**（正弦风场）
- **球体碰撞检测与响应**
- **约束迭代求解**（类 XPBD）
- **软光栅化三面板渲染**（初始 / 中途 / 末尾三帧合成）

## 编译运行

```bash
g++ main.cpp -o cloth -std=c++17 -O2 -Wall -Wextra
./cloth
# 输出: cloth_output.ppm → 用 Pillow 转 PNG
python3 -c "from PIL import Image; Image.open('cloth_output.ppm').save('cloth_output.png')"
```

## 输出结果

![三帧布料模拟](cloth_output.png)

*左到右：初始帧 → 模拟中途 → 末尾帧。可看到布料在重力和风力下逐渐下垂变形，与球体发生碰撞响应。*

## 技术要点

- **Verlet 积分**：`next = pos + (pos - prevPos)*damping + acc*dt²`，无速度变量，自带隐式阻尼
- **约束迭代**：每帧 15 次投影校正，防止弹簧过拉伸
- **三类弹簧**：结构弹簧保持网格形状，剪切弹簧防止菱形变形，弯曲弹簧抵抗褶皱
- **顶点固定**：顶行 4 个点 pin 住，形成悬挂效果
- **球体碰撞**：简单距离检测 + 法向推出，`pos = sphereCenter + d.normalized() * (R + epsilon)`
- **软光栅化**：纯 CPU 投影 + Bresenham 线段绘制 + 面法向 Phong 着色
