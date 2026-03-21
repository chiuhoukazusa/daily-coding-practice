# Bidirectional Path Tracing (BDPT)

## 编译运行
```bash
g++ main.cpp -o bdpt_output -std=c++17 -O2
./bdpt_output
convert bdpt_output.ppm bdpt_output.png
```

## 输出结果
![Cornell Box BDPT](bdpt_output.png)

## 技术要点
- **双向路径追踪 (BDPT)**：同时从相机和光源发射路径，连接两端顶点
- **多重重要性采样 (MIS)**：结合 s=0（纯相机路径）、s=1（直接光照）、s≥2（双向连接）
- **Cornell Box 场景**：红绿墙、面光源、漫反射球 + 镜面球
- **Reinhard Tone Mapping**：逐通道色调映射 + 2.2 Gamma 校正
- **俄罗斯轮盘赌**：动态终止低贡献路径，提高效率
- **焦散效果**：镜面球产生的反射焦散通过双向路径自然捕获
