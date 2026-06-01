# Screen-Space Reflections (SSR) Renderer

软光栅化 SSR 渲染器：不依赖 OpenGL，纯 CPU 实现屏幕空间反射。

## 编译运行
```bash
g++ main.cpp -o ssr_renderer -std=c++17 -O2 -Wno-missing-field-initializers
./ssr_renderer
```

## 输出结果
![结果](ssr_output.png)

## 技术要点
- G-Buffer：颜色 / 法线（视空间）/ 深度 / 粗糙度 / 金属度
- 屏幕空间光线步进（Ray Marching in Screen Space）
- Fresnel 菲涅尔混合（Schlick 近似）
- 粗糙度遮罩（高粗糙度物体不计算反射）
- 边缘淡出与距离衰减
- Reinhard 色调映射 + Gamma 2.2
