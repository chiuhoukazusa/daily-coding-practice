# Radiance Cascade Global Illumination

A Lumen-style Radiance Cascade GI renderer implemented in C++ with software rasterization.

## 编译运行

```bash
# 需要 stb_image_write.h 在同目录
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果

![结果](radiance_cascade_gi_output.png)

## 技术要点

- **辐射级联 (Radiance Cascades)**：三级探针网格（16×16, 8×8, 4×4），覆盖不同尺度的间接光照
- **软光栅化 G-Buffer**：延迟渲染管线，存储世界坐标/法线/漫反射率
- **多光源直接照明**：Lambertian 漫反射 + Blinn-Phong 高光
- **级联合并**：自顶向下将粗粒度级联的远距离辐射合并至细粒度级联
- **间接光照查询**：通过最近探针 cosine 加权采样半球辐射
- **ACES Filmic 色调映射** + gamma 校正
- **Fibonacci 球面采样**：均匀分布的方向向量集

## 渲染效果

Cornell Box 场景，包含：
- 红色左墙、绿色右墙的色溢效果（GI 间接光照）
- 蓝色高盒、黄色矮盒
- 中央反射球体
- 3 个光源：顶部主光、蓝色补光、橙色侧光
