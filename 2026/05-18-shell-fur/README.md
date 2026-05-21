# Shell Texturing Fur Renderer

实现基于 Shell Texturing 的实时毛发渲染技术。通过将球体网格向法线方向逐层膨胀，
结合程序化噪声密度场剔除非毛发像素，模拟出真实的毛发外观。

## 编译运行

```bash
g++ main.cpp -o shell_fur -std=c++17 -O2
./shell_fur
```

## 输出结果

![Shell Fur Rendering](shell_fur_output.png)

## 技术要点

- **Shell Texturing**：32 层同心壳，逐层向法线膨胀
- **程序化噪声密度场**：Hash 噪声定义每个 UV 格子中的毛发股位置
- **重力弯曲**：外层毛发向重力方向偏移，形成自然下垂
- **Kajiya-Kay 光照**：基于切线向量的 Diffuse + Specular 光照模型
- **Tiger 纹理**：基于球面坐标的程序化虎纹配色
- **Alpha 合成**：多层 Shell 的半透明叠加合成
