# Precomputed Radiance Transfer (PRT) Renderer

每日编程实践 · 2026-05-03

## 编译运行
```bash
g++ main.cpp -o prt_output -std=c++17 -O2
./prt_output
```

## 输出结果
![PRT渲染结果](prt_output.png)

## 技术要点
- 球谐函数 L0~L2 基函数（9个系数）
- 环境光照 Monte Carlo 投影到 SH 系数
- 顶点传输向量预计算（含/不含遮挡）
- SH 系数点积实时求渲染颜色
- SH 旋转：Wigner D-矩阵（绕Y轴旋转环境光）
- 三面板对比：原始/旋转60°/旋转120°环境光
- 软光栅化渲染 + Reinhard Tone Mapping
