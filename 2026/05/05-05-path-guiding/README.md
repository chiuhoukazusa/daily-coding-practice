# Path Guiding Renderer with SD-Tree Spatial Caching

基于 SD-Tree（空间-方向树）的路径引导渲染器，通过多轮采样学习场景中的光照分布，从而引导蒙特卡洛路径追踪的采样方向，显著减少渲染噪声。

## 技术要点

- **SD-Tree (Spatial-Directional Tree)**：将场景空间划分为 8×8×8 体素网格，每个体素维护一个方向分布缓存（16×32 方向格子），记录从该位置出发各方向的光照贡献量
- **自适应重要性采样**：初始 Pass 使用余弦加权半球采样学习分布，后续 Pass 按学习到的分布进行引导采样，减少高方差方向上的浪费
- **MIS（多重重要性采样）**：将引导 PDF 与余弦采样 PDF 混合，避免引导方向偏差造成的偏差
- **Russian Roulette**：深度 ≥3 后启用路径终止，防止无限递归
- **Fresnel 折射/反射**：玻璃材质使用精确 Fresnel 公式，支持全内反射（TIR）
- **ACES 色调映射**：HDR 输出经 ACES Filmic 映射为 LDR
- **对比渲染**：左半图为引导渲染，右半图为纯余弦采样参考，直观展示引导效果

## 参考文献

- Müller T. et al. "Practical Path Guiding for Efficient Light-Transport Simulation" (EGSR 2017)
- Rath A. et al. "Variance-Aware Path Guiding" (SIGGRAPH 2020)
- NVIDIA ReSTIR / Path Guiding research series

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
# 输出: path_guiding_output.png (1204×660, 左:引导 右:参考)
```

## 输出结果

![Path Guiding 对比](path_guiding_output.png)

左侧为 Path Guiding（128 spp，2轮学习+2轮渲染），右侧为纯余弦半球采样参考（64 spp）。  
SD-Tree 学习了 512 个空间体素中 260 个体素的方向分布，共记录 3300 万次方向更新。
