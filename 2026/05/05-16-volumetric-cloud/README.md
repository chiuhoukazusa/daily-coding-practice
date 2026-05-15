# Volumetric Cloud Ray Marching Renderer

体积云光线步进渲染器：使用 Ray Marching + FBM 噪声生成程序化云体，结合大气散射和 Beer-Lambert 光学衰减，实现真实感云彩渲染。

## 编译运行
```bash
# 需要 stb_image_write.h（单头文件）
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![体积云渲染结果](volumetric_cloud_output.png)

## 技术要点
- **Ray Marching**：沿视线方向步进采样云体密度场（64步）
- **FBM 噪声**：6阶分形布朗运动 + Perlin 噪声生成云体形状
- **Beer-Lambert 定律**：光学深度积分实现真实感透射衰减
- **Henyey-Greenstein 相函数**：前向散射 + 各向同性混合（g=0.7/-0.2）
- **内散射（In-scattering）**：每步向太阳方向采样6步光学深度
- **高度渐变**：云底/云顶插值产生蓬松感（云层 1200m~3500m）
- **大气背景**：Rayleigh 近似天空渐变 + 太阳光晕
- **ACES Filmic 色调映射**：HDR → LDR 转换
