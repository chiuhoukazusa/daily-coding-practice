# B-spline Curve Renderer

## 项目描述

基于 **Cox-de Boor 递归算法**实现 B-spline 样条曲线渲染器，展示 B-spline 曲线的核心特性。

## 技术要点

- **Cox-de Boor 递归算法** - B-spline 基函数的递归定义
- **均匀节点向量 (Uniform Knot Vector)** - 等间距节点分布
- **端点插值节点向量 (Clamped/Open Knot Vector)** - 首尾各重复 degree+1 次
- **局部控制性** - 移动一个控制点只影响局部曲线形状
- **与 Bezier 曲线对比** - 展示两者的差异（全局 vs 局部控制）

## 输出结果

| 文件 | 内容 |
|------|------|
| `bspline_output.png` | 800×1200 总览图（4个面板） |
| `bspline_quadratic.png` | 二次 B-spline (Degree 2, Uniform) |
| `bspline_cubic.png` | 三次 B-spline (Degree 3, Clamped) |
| `bspline_vs_bezier.png` | B-spline vs Bezier 对比 |

## 编译运行

```bash
g++ -std=c++17 -O2 main.cpp -o bspline
./bspline
```

## 验证结果

- ✅ 基函数单位分割性 (Partition of Unity)：∑ N_{i,p}(t) = 1
- ✅ Clamped B-spline 经过端点：起点距离 = 0，终点距离 = 2.7e-9
- ✅ 生成 4 个 PNG 图像，总像素数正常
- ✅ 量化验证：非背景像素 29438，绿/蓝曲线像素 9339

## 面板说明

1. **Quadratic B-spline (Degree 2, Uniform)** - 二次样条，均匀节点
2. **Cubic B-spline (Degree 3, Clamped)** - 三次样条，端点插值（曲线经过首尾控制点）
3. **Cubic B-spline (Degree 3, Uniform) - Local Control** - 展示局部控制性，多波浪控制点
4. **B-spline vs Bezier** - 相同控制点下两种曲线的对比

## B-spline vs Bezier 关键区别

| 特性 | B-spline | Bezier |
|------|---------|--------|
| 控制点数 | 任意多个 | n+1 个定义 n 次曲线 |
| 曲线次数 | 独立于控制点数 | 等于控制点数-1 |
| 局部控制 | ✅ 只影响局部 | ❌ 全局影响 |
| 端点插值 | Clamped 模式下✅ | ✅ 总是通过 |

## 技术总结

B-spline 的核心优势是**局部控制性**：n 个控制点只影响 (degree+1) 个相邻的曲线段，这使得大规模曲线编辑变得可预测和高效。这一特性是现代 CAD 系统和 NURBS 曲线的基础。

---
**完成时间**: 2026-03-02 05:32  
**迭代次数**: 2 次（1次编译修复）  
**编译器**: g++ -std=c++17 -O2
