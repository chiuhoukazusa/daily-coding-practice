# Temporal Anti-Aliasing (TAA)

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
![对比图](taa_output.png)

左：无抗锯齿 | 中：TAA (16帧累积) | 右：4x SSAA参考

## 技术要点
- Halton序列生成子像素抖动（Halton(2,3)）
- 运动向量计算与重投影（软光栅实现）
- 邻域颜色AABB钳制防止鬼影
- 方差裁剪（Variance Clipping）进一步收紧约束
- 双线性采样历史缓冲区
- 混合因子 α=0.1（10%新帧，90%历史）
- 三面板对比：NoAA vs TAA vs 4×SSAA
