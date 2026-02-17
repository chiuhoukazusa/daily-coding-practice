# Perlin Noise 程序化纹理生成器

## 项目描述

基于 Perlin Noise 算法实现的程序化纹理生成器，可以生成自然的大理石、云朵和木纹纹理。使用分形布朗运动（FBM）技术叠加多层噪声，产生逼真的自然纹理效果。

## 编译运行

```bash
g++ -std=c++11 main.cpp -o perlin_noise -lm
./perlin_noise
```

## 输出结果

程序会生成三种纹理，每种尺寸为 512x512：

### 1. 大理石纹理 (marble.png)
![大理石纹理](marble.png)

使用 `sin(x + noise * 5)` 创建类似大理石的流动纹理。

### 2. 云朵纹理 (clouds.png)
![云朵纹理](clouds.png)

使用 8 层 FBM 叠加，persistence=0.6，产生蓬松的云朵效果。

### 3. 木纹纹理 (wood.png)
![木纹纹理](wood.png)

基于径向距离的环形噪声，模拟树木的年轮结构。

## 技术要点

### Perlin Noise 核心算法
- **排列表 (Permutation Table)**: 256 个随机排列的索引
- **淡化函数 (Fade Function)**: `6t⁵ - 15t⁴ + 10t³` 平滑插值
- **梯度插值**: 在单位立方体的 8 个角进行三线性插值
- **哈希函数**: 使用排列表将空间坐标映射到梯度向量

### 分形布朗运动 (FBM)
```cpp
for (int i = 0; i < octaves; i++) {
    total += noise(x * frequency, y * frequency, 0) * amplitude;
    amplitude *= persistence;  // 每层衰减
    frequency *= 2.0;         // 频率加倍
}
```

- **Octaves**: 叠加层数（6-8层）
- **Persistence**: 振幅衰减系数（通常 0.5）
- **Lacunarity**: 频率增长率（固定为 2.0）

### 纹理映射技巧

1. **大理石**: `sin((x + noise * scale) * π)` - 正弦波扰动产生流动感
2. **云朵**: 多层高频噪声叠加 - 模拟云层的复杂细节
3. **木纹**: `sin((√(x² + y²) + noise) * π)` - 径向波纹

## 依赖库

- **stb_image_write.h**: 轻量级图像写入库（单头文件）
- **标准 C++ 库**: cmath, vector, iostream

## 性能数据

- **生成时间**: ~0.5 秒 (512x512 × 3)
- **内存占用**: ~2.4 MB (3 张图片的像素数据)
- **文件大小**:
  - marble.png: 353 KB
  - clouds.png: 168 KB
  - wood.png: 317 KB

## 迭代历史

- **迭代 1**: 初始版本 - 一次编译运行成功 ✅
  - 代码结构清晰
  - Perlin Noise 实现正确
  - FBM 叠加正确
  - 三种纹理生成成功

## 扩展方向

1. **更多纹理类型**: 火焰、水波、地形高度图
2. **3D 噪声**: 生成体积纹理
3. **GPU 加速**: 使用 CUDA/OpenGL 计算着色器
4. **交互式编辑**: 实时调整参数预览效果
5. **Simplex Noise**: 性能更优的噪声算法

## 参考资料

- Ken Perlin's Original Paper (1985)
- [Understanding Perlin Noise](http://adrianb.io/2014/08/09/perlinnoise.html)
- [The Book of Shaders - Noise](https://thebookofshaders.com/11/)
