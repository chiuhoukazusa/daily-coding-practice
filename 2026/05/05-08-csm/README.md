# Cascaded Shadow Maps (CSM) Renderer

软光栅化实现的级联阴影贴图渲染器，支持三级 CSM 与 PCF 软阴影。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
# 生成 csm_output.ppm，用 PIL 等工具转 PNG
```

## 输出结果
![结果](csm_output.png)

## 技术要点
- **3 层级联阴影贴图**（Near / Mid / Far）分别对应不同视距范围
- **Practical Split Scheme**：对数分割与均匀分割的加权混合
- **正交光源投影**：对每个级联视锥计算紧凑的光源正交投影矩阵
- **PCF 软阴影**：3×3 卷积核的 Percentage Closer Filtering
- **级联颜色调试**：Near=红调，Mid=绿调，Far=蓝调方便可视化级联边界
- **右侧阴影图可视化**：三个级联的深度图并排显示
- 软光栅化：无任何 OpenGL/GPU 依赖，纯 CPU C++17
