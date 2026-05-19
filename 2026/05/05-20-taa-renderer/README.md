# Temporal Anti-Aliasing (TAA) Renderer

实现现代游戏引擎中的TAA（时域抗锯齿）技术。

## 编译运行
```bash
g++ main.cpp -o taa_renderer -std=c++17 -O2
./taa_renderer
```

## 输出结果
![对比结果](taa_output.png)

三列对比：无AA（锯齿明显）| SSAA 2x（参考）| TAA 20帧积累（平滑）

## 技术要点
- Halton(2,3)序列亚像素抖动：每帧偏移采样位置，积累子像素信息
- 运动矢量计算：记录每个像素在两帧间的位移
- 历史帧重投影：通过运动矢量找到上一帧对应位置，双线性插值
- Variance Clipping（方差裁剪）：防止幽灵效果，将历史颜色裁剪到当前邻域范围
- 自适应混合权重：运动大时降低历史帧权重（默认90%历史+10%当前）
- 纯C++软光栅化，无外部图形库依赖
