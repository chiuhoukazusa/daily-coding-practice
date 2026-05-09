# Toon Water Shader

NPR风格卡通水面渲染器：深度泡沫、Voronoi噪声、Fresnel反射、折射扭曲。

## 编译运行
```bash
cp /path/to/stb_image_write.h .
g++ main.cpp -o toon_water -std=c++17 -O2
./toon_water
```

## 输出结果
![结果](toon_water_output.png)

## 技术要点
- 深度泡沫效果（Edge Foam based on depth difference）
- Voronoi 噪声驱动的水面波纹
- Fresnel 反射（卡通色阶化）
- 折射扭曲（FBM Normal map distortion）
- Gerstner 波水面几何变形
- 程序化海底+岛屿地形
- NPR 色阶量化 + Sobel 描边
