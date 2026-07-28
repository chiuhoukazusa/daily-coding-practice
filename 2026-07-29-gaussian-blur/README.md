# Gaussian Blur - Separable Kernel Optimization

## 编译运行
```bash
g++ gaussian_blur.cpp -o gaussian_blur -std=c++17 -O2
./gaussian_blur
```

## 输出结果
- `blur_sigma10_r3_2d.ppm` - σ=1.0 2D 卷积结果
- `blur_sigma10_r3_sep.ppm` - σ=1.0 分离卷积结果
- `blur_sigma20_r5_2d.ppm` - σ=2.0 2D 卷积结果
- `blur_sigma20_r5_sep.ppm` - σ=2.0 分离卷积结果
- `blur_sigma30_r8_2d.ppm` - σ=3.0 2D 卷积结果
- `blur_sigma30_r8_sep.ppm` - σ=3.0 分离卷积结果
- `blur_sigma50_r12_2d.ppm` - σ=5.0 2D 卷积结果
- `blur_sigma50_r12_sep.ppm` - σ=5.0 分离卷积结果

## 技术要点
- 高斯核定义：G(x,y) = (1/(2πσ²)) * exp(-(x²+y²)/(2σ²))
- 2D 高斯核可分离为两个 1D 核的外积：G_2D = G_x ⊗ G_y
- 2D 卷积 O(W*H*k²) → 两次 1D 卷积 O(2*W*H*k) = O(W*H*k)
- 4 种 σ 配置测试 (1.0/2.0/3.0/5.0)，加速比最大 14.48x
- 核可分离性数值验证：|G_2D - G_x ⊗ G_y| < 1e-12
- PSNR 输出等价性：所有配置 PSNR > 52 dB（视觉无损）
- 方差递减验证：σ 增大，图像方差单调递减
