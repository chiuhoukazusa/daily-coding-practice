# Marschner Hair Rendering

实现 Marschner et al. 2003 论文中的头发光照模型，将头发散射分解为三个物理上独立的路径：

- **R**（Reflection）：光在头发表面直接反射
- **TT**（Transmission-Transmission）：光穿过头发两次，产生透射效果
- **TRT**（Transmission-Reflection-Transmission）：光透射进入、在内部反射、再透射出来，产生焦散般的高光

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

渲染时间约 50 秒（120 根 Bezier 发丝，每根 20 段，3 盏灯）。

## 输出结果

![结果](marschner_hair_output.png)

## 技术要点

- **Marschner BSDF 分解**：S(θᵢ, φᵢ, θᵣ, φᵣ) = M(θₕ) × N(φ) / cos²(θd)
- **纵向散射 M**：Gaussian 分布，带 α 偏移（R: -α, TT: -α/2, TRT: -3α/2）
- **方位散射 N**：通过对 h∈[-1,1] 数值求根，计算每条路径对观测方向的贡献
- **折射率 1.55**：人类头发的典型 IOR
- **Fresnel 衰减**：每个界面计算菲涅尔反射率
- **吸收系数**：σₐ 控制头发颜色（深棕/金色/红棕）
- **Cubic Bezier 发丝**：120 根随机卷曲发丝，dome 形排布
- **ACES Filmic 色调映射**：HDR → SDR
