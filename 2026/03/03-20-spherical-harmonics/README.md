# Spherical Harmonics Environment Lighting

每日编程实践 2026-03-20

## 项目简介

实现了 **L0-L2 球谐函数（Spherical Harmonics）环境光照**，用于将低频环境光照投影到 9 个 SH 系数，再通过 SH 重建物体表面的辐照度（Irradiance），是实时渲染中的经典技术。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

运行时间约 12 秒，输出 5 张 PNG 图像。

## 技术要点

- **L2 球谐基函数**（9个系数）：Y00, Y1-1, Y10, Y11, Y2-2, Y2-1, Y20, Y21, Y22
- **Monte Carlo SH 投影**：65536 个均匀球面采样，将程序化天空光投影到 SH 系数
- **Lambertian 辐照度重建**：Ramamoorthi & Hanrahan 2001 方法，A0=π, A1=2π/3, A2=π/4
- **程序化天空环境**：蓝天渐变 + 太阳光晕 + 地面棕色，模拟户外光照
- **PBR 材质着色**：Fresnel-Schlick, metallic/roughness 工作流，ACES 色调映射
- **MC 参考对比**：余弦加权半球采样直接积分 vs SH 近似，验证精度

## 输出文件

| 文件 | 说明 |
|------|------|
| `sh_output.png` | SH 环境光照渲染（3 个球：粗糙红、金属金、光滑蓝） |
| `sh_mc_reference.png` | Monte Carlo 参考渲染（直接积分） |
| `sh_comparison.png` | SH vs MC 对比图 |
| `sh_basis.png` | L0-L2 球谐基函数可视化（红=正值，蓝=负值） |
| `sh_probe_comparison.png` | 环境探针：原始天空（上）vs SH 重建（下） |

## 参考

- Ramamoorthi & Hanrahan 2001, "An Efficient Representation for Irradiance Environment Maps"
- Real-Time Rendering, 4th edition, Chapter 10
