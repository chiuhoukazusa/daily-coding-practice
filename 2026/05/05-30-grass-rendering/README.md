# Procedural Grass Rendering with Wind Simulation

程序化草地渲染 + 风场模拟，使用软光栅化从零实现。

## 技术要点

- **草叶几何**：二次贝塞尔曲线控制草叶形状，顶端细、底部宽
- **风场模拟**：多频率正弦叠加的风场，草叶顶部位移大于底部
- **LOD 距离分级**：近景 5 段细分，远景 3 段降精度
- **Phong 着色**：漫反射 + 环境光 + 半透射（translucency）模拟
- **深度排序**：从远到近渲染（back-to-front）确保正确叠加
- **天空渐变**：顶部深蓝 → 地平线浅蓝，加太阳光晕

## 编译运行

```bash
g++ main.cpp -o grass_renderer -std=c++17 -O2 -Wall -Wextra
./grass_renderer
# 输出: grass_output.png (800x600, ~283KB)
# 耗时: ~0.08s
```

## 输出结果

![草地渲染](grass_output.png)
