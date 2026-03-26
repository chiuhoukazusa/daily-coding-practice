# Motion Blur Renderer

软光栅化运动模糊渲染器，实现速度缓冲（Velocity Buffer）技术。

## 编译运行
```bash
cp ../../../stb_image_write.h .
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![对比图](motion_blur_compare.png)
![运动模糊](motion_blur_output.png)
![速度缓冲可视化](motion_blur_velocity.png)

## 技术要点
- 软光栅化渲染管线（顶点变换 → 光栅化 → 着色）
- Velocity Buffer：每像素存储当前帧与上一帧的屏幕空间位移向量
- 多物体运动：旋转球体、旋转立方体、轨道绕行小球
- 相机轨道运动产生全局运动模糊
- 后处理运动模糊：沿运动矢量方向多次采样累积（24样本）
- Phong 光照模型，双光源，透视校正插值
