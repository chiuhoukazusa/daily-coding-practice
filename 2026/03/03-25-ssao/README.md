# Screen Space Ambient Occlusion (SSAO)

## 项目简介

软光栅化实现的 SSAO（屏幕空间环境光遮蔽）渲染器。对比展示无 SSAO 与有 SSAO 的视觉差异：左半无遮蔽、右半开启遮蔽，角落与缝隙明显变暗。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

输出：`ssao_output.png`（800×600，左无SSAO，右有SSAO）

## 技术要点

- **G-Buffer 构建**：软光栅化 3D 场景（Cornell Box + 球体），存储 view-space 深度、法线、位置、漫反射颜色
- **SSAO 核心算法**：
  - 64 个半球采样核（cos-weighted，靠近原点密集）
  - 4×4 随机旋转噪声纹理（平铺采样，破坏规律性）
  - TBN 矩阵将 tangent-space 样本转换到 view space
  - 深度比较 + range check（防止远处几何体产生错误遮蔽）
- **Box Blur 模糊 Pass**：5×5 均值模糊平滑 SSAO 结果
- **最终合成**：`ambient × (1-occlusion×0.85) + diffuse + specular`（Blinn-Phong）

## 输出结果

![SSAO对比结果](ssao_output.png)

- 左半：普通光照（无 SSAO）
- 右半：SSAO 开启（角落/球体底部可见变暗）
- 黄线：左右分界线
