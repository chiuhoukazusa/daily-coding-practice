# Cel Shading & Outline Renderer

NPR 卡通渲染器：色阶量化 + 轮廓描边 + 法线膨胀外描边

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![结果](cel_shading_output.png)

## 技术要点
- Phong 光照分级量化（3-step cel diffuse）
- 硬边 Specular 高光（NdotH 阈值截断）
- 屏幕空间轮廓描边（对象 ID 边界 + 法线不连续检测）
- Icosphere 网格（4 级细分）
- Torus 网格（40×20 分段）
- 透视正确重心坐标插值
- 纯 C++17，无外部依赖
