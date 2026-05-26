# Procedural Terrain with Hydraulic Erosion

程序化地形生成 + 水力侵蚀模拟 + 软光栅化渲染

## 编译运行
```bash
g++ main.cpp -o terrain_erosion -std=c++17 -O2
./terrain_erosion
```

## 输出结果
![侵蚀前地形](terrain_before_erosion.png)
![侵蚀后俯视](terrain_top_view.png)
![3D等角视角](terrain_3d_view.png)

## 技术要点
- Perlin 噪声 fBm（6倍频）生成基础地形
- 粒子化水力侵蚀（80,000水滴，梯度驱动 + 惯性）
- 高度分层着色（水/沙/草/岩/雪）
- Lambert漫射光照 + 近似AO
- 俯视图 + 等角3D投影软光栅化渲染
