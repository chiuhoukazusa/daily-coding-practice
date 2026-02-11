# Perlin Noise 程序化纹理生成器

**日期**: 2026-02-10  
**类型**: 图形学 / 程序化生成

## 项目描述

实现经典的 Perlin Noise 算法，用于生成自然的程序化纹理。支持多层噪声叠加（Octave Noise），可生成云层、大理石、木纹等多种效果。

## 技术要点

- **Perlin Noise 算法**：基于梯度和平滑插值的噪声生成
- **Octave Noise**：多层噪声叠加，创造自然细节
- **程序化纹理**：
  - 云层纹理：8 层噪声，频率倍增
  - 大理石纹理：噪声 + 正弦波扰动
  - 木纹纹理：径向距离 + 环状图案

## 编译运行

```bash
# 编译
g++ -std=c++17 -Wall -Wextra -O2 perlin_noise.cpp -o perlin_noise -lm

# 运行
./perlin_noise

# 输出
# output_clouds.ppm - 云层纹理
# output_marble.ppm - 大理石纹理
# output_wood.ppm - 木纹纹理
```

## 输出结果

生成 3 个 512x512 的 PPM 图片文件：

1. **云层纹理** (`output_clouds.ppm`)：模拟自然云层的柔和渐变
2. **大理石纹理** (`output_marble.ppm`)：白色基底配合深色纹理
3. **木纹纹理** (`output_wood.ppm`)：棕色调的同心环状图案

## 算法实现

### 核心函数

```cpp
// Fade 函数（6t^5 - 15t^4 + 10t^3）
double fade(double t) {
    return t * t * t * (t * (t * 6 - 15) + 10);
}

// 梯度函数
double grad(int hash, double x, double y) {
    int h = hash & 15;
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : (h == 12 || h == 14 ? x : 0);
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

// Octave Noise
double octaveNoise(double x, double y, int octaves, double persistence) {
    double total = 0, frequency = 1, amplitude = 1, maxValue = 0;
    for (int i = 0; i < octaves; i++) {
        total += noise(x * frequency, y * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= 2;
    }
    return total / maxValue;
}
```

## 迭代历史

- **迭代 1**: ✅ 初始版本编译运行成功
- 编译时间: < 1 秒
- 运行时间: < 1 秒
- 状态: 一次性成功，无需修复

## 技术参数

| 参数 | 云层 | 大理石 | 木纹 |
|------|------|--------|------|
| Octaves | 6 | 4 | 3 |
| Persistence | 0.5 | 0.6 | 0.5 |
| Frequency | 8x | 10x | 5x |
| 特殊处理 | - | sin() 扰动 | 径向距离 |

## 扩展方向

- [ ] 添加 3D Perlin Noise
- [ ] 实现 Simplex Noise（更快的替代算法）
- [ ] 支持 PNG/JPEG 输出（集成 stb_image_write）
- [ ] 实现更多纹理类型（岩石、地形、水波）
- [ ] 添加交互式参数调整

## 参考资料

- Ken Perlin 的原始论文 (1985)
- [Understanding Perlin Noise](http://adrianb.io/2014/08/09/perlinnoise.html)

---

**完成时间**: 2026-02-10 10:10  
**代码行数**: 230 行 C++  
**编译器**: g++ (GCC) 12.3.1  
**状态**: ✅ 成功
