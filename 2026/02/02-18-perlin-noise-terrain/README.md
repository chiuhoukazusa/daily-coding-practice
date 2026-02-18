# Perlin Noise 地形生成器

## 项目描述

使用 Perlin Noise 算法生成程序化地形高度图。实现了经典的梯度噪声算法，通过分形布朗运动（FBM）叠加多个八度（octaves）产生自然的地形起伏效果。

## 技术要点

- **Perlin Noise 算法**: 使用梯度向量和插值生成平滑的噪声
- **Fade 函数**: 使用 Ken Perlin 的改进平滑函数 6t⁵ - 15t⁴ + 10t³
- **分形布朗运动 (FBM)**: 叠加 6 个八度，每层频率加倍、幅度减半
- **颜色映射**: 低海拔蓝色（水）→ 中海拔绿色（陆地）→ 高海拔棕色（山脉）
- **PPM 图片输出**: 无需外部库，直接生成可读的图像格式

## 编译运行

```bash
g++ -std=c++17 -O2 perlin_noise.cpp -o perlin_noise
./perlin_noise
```

编译选项：
- `-std=c++17`: 使用 C++17 标准
- `-O2`: 优化级别 2（生成速度更快）
- 无需链接额外的库

## 输出结果

![地形高度图](terrain.png)

生成 512×512 像素的地形图，展示了：
- 平滑的高度过渡
- 自然的山脉和谷地
- 多尺度的地形特征（通过多个八度实现）

## 算法细节

### Perlin Noise 核心步骤

1. **格子化**: 将输入坐标映射到整数格子
2. **梯度选择**: 为每个格子顶点选择随机梯度向量
3. **点积计算**: 计算输入点到各顶点的向量与梯度的点积
4. **插值**: 使用 fade 函数进行双线性插值
5. **归一化**: 将结果映射到 [0, 1] 范围

### FBM 参数

```cpp
octaves = 6;           // 叠加层数
persistence = 0.5;     // 振幅衰减因子
frequency = 1.0;       // 初始频率
scale = 0.02;          // 全局缩放
```

## 性能

在现代 CPU 上生成 512×512 地形图约需 0.5 秒。

## 可能的扩展

- 添加深度测试实现 3D 地形
- 实现 Simplex Noise（更高维度下性能更好）
- 添加侵蚀模拟（hydraulic erosion）
- 生成法线贴图用于实时渲染
- 实现 Worley Noise 用于其他效果

## 参考资料

- [Perlin Noise: A Procedural Generation Algorithm](https://adrianb.io/2014/08/09/perlinnoise.html)
- Ken Perlin's original paper (SIGGRAPH 1985)
- [Understanding Perlin Noise](http://flafla2.github.io/2014/08/09/perlinnoise.html)

---

**完成时间**: 2026-02-18 10:09  
**代码行数**: 170 行 C++  
**编译器**: g++ (GCC) 12.3.1  
**迭代次数**: 1 次（一次通过）
