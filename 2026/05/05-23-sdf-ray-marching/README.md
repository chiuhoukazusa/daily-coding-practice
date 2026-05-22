# SDF Ray Marching Scene Renderer

SDF（Signed Distance Functions）Ray Marching 场景渲染器，使用程序化几何体构建完整3D场景。

## 特性

- **多种 SDF 基元**：球体、盒子、胶囊体、圆环面（Torus）、圆柱体
- **平滑 SDF 融合**（Smooth Union）：前景小球群自然融合
- **软阴影**（Soft Shadows）：基于 Ray March 的半影效果
- **环境光遮蔽（AO）**：5层采样计算局部遮蔽
- **Blinn-Phong 着色**：漫反射 + 高光，支持粗糙度/金属度
- **ACES Filmic 色调映射**：HDR → SDR 过渡自然
- **程序化天空**：渐变背景 + 太阳光晕
- **地面棋盘格**：增强空间感
- **大气雾效**：随深度衰减

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -isystem..
./output
```

需要 `stb_image_write.h`（放在上级目录）。

## 输出结果

![SDF场景渲染结果](sdf_scene_output.png)

## 场景构成

| 物体 | SDF | 材质 |
|------|-----|------|
| 中心大球 | sdSphere | 金属（高光泽） |
| 左侧蓝盒 | sdBox | 漫反射蓝色 |
| 右侧金环 | sdTorus | 半金属金黄 |
| 后方红胶囊 | sdCapsule | 漫反射红色 |
| 左后紫柱 | sdCylinder | 漫反射紫色 |
| 前景绿球群 | sdSphere×4 + SmoothUnion | 漫反射绿色 |
| 地面 | sdPlane | 棋盘格 |
