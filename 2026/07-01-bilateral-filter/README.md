# Bilateral Filter — Edge-Preserving Smoothing

## 原理
双边滤波是一种非线性图像平滑算法，每个像素的权重由两部分组成：
- **空间高斯核** G_spatial(dist)：距离近的像素权重高
- **值域高斯核** G_range(|I_p - I_q|)：颜色相似的像素权重高

因此：在平坦区域（颜色相近）等价于高斯模糊，在边缘处（颜色差异大）则保留边缘不模糊。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出
- `clean.ppm` — 干净的对角线边缘测试图像
- `noisy.ppm` — 添加高斯噪声的图像
- `gaussian_blur.ppm` — 标准高斯模糊结果（边缘被模糊）
- `bilateral_filter.ppm` — 双边滤波结果（保持边缘）

## 量化验证
| 指标 | Noisy | Gaussian | Bilateral |
|------|-------|----------|-----------|
| PSNR | 20.04 dB | 32.69 dB | 30.82 dB |
| Edge Gradient | 0.905 | 0.469 | 0.712 |
| Edge vs Clean | 138.2% | 71.6% | 108.8% |

- ✅ Bilateral 较 Noisy 改善 10.8 dB
- ✅ Bilateral 边缘保持远优于 Gaussian（距离 0.057 vs 0.186）
- ✅ Gaussian 模糊了边缘（仅保留 71.6%），Bilateral 保持了边缘清晰度

## 技术要点
- 空间-值域双高斯核权重
- 边缘保持平滑（Edge-Preserving Smoothing）
- 量化验证：PSNR + Sobel边缘梯度 + 噪声抑制率
