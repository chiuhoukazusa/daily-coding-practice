# GPU Rendering Pipeline: Homogeneous Clipping

## 编译运行
```bash
g++ main.cpp -o pipeline -std=c++17 -O2
./pipeline
```

## 输出结果

GPU 渲染管线的完整实现，包含齐次裁剪（Homogeneous Clipping）和透视校正插值：

- **透视校正插值 vs 线性插值对比**: `pipeline_comparison.png` - 左右对比展示两种插值模式的差异
- **差异热力图**: `pipeline_diff_map.png` - 可视化两种插值之间的差异分布
- **线性插值渲染**: `pipeline_linear.png` - 仅使用线性插值的渲染结果
- **透视校正插值渲染**: `pipeline_persp_correct.png` - 使用透视校正插值的渲染结果
- **分界线对比**: `pipeline_comparison_divider.png` - 带分界线的 A/B 对比

## 技术要点
- 齐次裁剪空间 (Homogeneous Clipping Space)：在裁剪空间中对三角形进行齐次坐标裁剪
- 透视校正插值 (Perspective-Correct Interpolation)：使用 1/w 进行透视校正的属性插值
- 视锥体裁剪 (Frustum Clipping)：Sutherland-Hodgman 算法在齐次空间中裁剪三角形
- 软光栅化 (Soft Rasterization)：使用 Barycentric 坐标在 CPU 上实现 GPU 管线功能
- 深度测试 (Z-Buffer)：正确实现深度缓冲和深度比较
