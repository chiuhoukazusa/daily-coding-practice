# Bicubic Interpolation Image Upscaling

日期：2026-08-30
方向：图像处理

## 技术点
1. **双三次插值（Bicubic, Keys Catmull-Rom 核）** —— 基于 4×4 邻域像素，用三次多项式核 `a=-0.5` 加权重建，得到平滑且保留边缘的上采样结果。
2. **双线性插值（Bilinear）** —— 2×2 邻域线性加权，作为对比基准。
3. **最近邻插值（Nearest-Neighbor）** —— 直接取最近像素，产生锯齿。
4. **量化验证** —— 与高分辨率 ground truth 对比，计算 MSE / PSNR / 边缘锐度（Edge Sharpness）。
5. **双模式测试** —— Mode 0（平滑解析场）+ Mode 1（高频压力场，放大插值误差差异）。

## 双三次插值原理

双三次插值在两个方向各做一次三次卷积。每个输出像素对输入 4×4 邻域加权求和：

```
f(x,y) = Σᵢ Σⱼ pᵢⱼ · W(dx - i) · W(dy - j)
```

其中 `W` 是 Keys 三次核：

```
        (a+2)|t|³ - (a+3)|t|² + 1        |t| ≤ 1
W(t) =  a|t|³ - 5a|t|² + 8a|t| - 4a     1 < |t| < 2
        0                               otherwise
```

取 `a = -0.5`（Catmull-Rom 样条）时，核在采样点处精确通过数据点，且 C¹ 连续，避免了双线性在放大时的高频混叠和最近邻的块状锯齿。

## 量化验证（4/4 通过）

| 验证项 | 结果 |
|--------|------|
| 平滑场 bilinear & bicubic PSNR >> nearest（差距 > 15 dB） | PASS |
| 平滑场 bicubic 边缘锐度 > bilinear（更少过平滑） | PASS |
| 压力场 bicubic MSE < bilinear（更少过模糊） | PASS |
| 压力场 bicubic 边缘锐度 > bilinear | PASS |

## 输出
- `bicubic_comparison.png` —— Mode 0（平滑场）五帧对比：LowRes / Nearest / Bilinear / Bicubic / GroundTruth
- `bicubic_comparison_m1.png` —— Mode 1（高频压力场）五帧对比
- `up_bicubic_m0.png` / `up_bicubic_m1.png` —— 双三次上采样结果（2× 放大）

## 编译
```
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```
