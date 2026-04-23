# Bent Normal Ambient Occlusion

实现了基于弯曲法线（Bent Normal）的环境遮蔽（AO）渲染器。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
# 生成 bent_normal_ao_output.ppm，用 convert 转 PNG
```

## 输出结果

![结果](bent_normal_ao_output.png)

## 技术要点

- **Bent Normal（弯曲法线）**：在法线半球上采样 N 条射线，将未遮蔽方向的加权均值作为弯曲法线
- **AO 计算**：未遮蔽射线数 / 总射线数，用 ray marching 检测遮蔽
- **SH 环境光**：用 L2 球谐函数存储环境辐照度，用 Bent Normal 采样得到更准确的间接光照
- **Hammersley 低差异序列**：van der Corput 序列保证采样均匀分布
- **软阴影**：SDF ray marching 计算软阴影
- **场景**：带十字凹槽地面 + 橙色球体 + 蓝色圆柱 + 绿色方块
- **三分可视化**：底部区域分为 Bent Normal 可视化 / AO 灰度图 / 完整渲染对比
