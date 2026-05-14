# FFT Ocean Surface Renderer

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![结果](fft_ocean_output.png)

## 技术要点
- Phillips 频谱：物理正确的海洋波高频率分布
- Cooley-Tukey 2D FFT：O(N²logN) 海洋面高度场逆变换
- 梯度法线：由高度场一阶偏导计算波面法线
- Blinn-Phong + Fresnel：海洋着色与镜面反射
- 深度/浪尖颜色分层：深蓝→中蓝→白色浪尖
