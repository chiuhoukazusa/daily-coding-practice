# Shadow Ray Tracing - 带阴影的光线追踪器

**日期**: 2026-02-17  
**类型**: 图形学 / 光线追踪 / 阴影算法  
**语言**: C++11

## 项目描述

实现带阴影效果的光线追踪渲染器，支持 **Shadow Ray** 技术和 **Phong光照模型**。通过向光源发射阴影光线（Shadow Ray）判断物体是否被遮挡，从而实现真实的阴影效果。

## 核心技术

### 1. Shadow Ray（阴影光线）
当光线击中物体表面后，从交点向每个光源发射一条"阴影光线"：
- 如果阴影光线在到达光源前击中其他物体 → **在阴影中**
- 如果阴影光线没有击中任何物体 → **被光源照亮**

```cpp
bool isInShadow(const Vec3& point, const Vec3& lightPos) const {
    Vec3 toLight = lightPos - point;
    double distanceToLight = toLight.length();
    Ray shadowRay(point, toLight);
    
    for (const auto& sphere : spheres) {
        double t;
        if (sphere.intersect(shadowRay, t)) {
            if (t > EPSILON && t < distanceToLight) {
                return true;  // 被遮挡
            }
        }
    }
    return false;  // 无遮挡
}
```

### 2. Phong 光照模型
实现经典的 Phong 光照模型，包含三个分量：

**环境光 (Ambient)**
```cpp
Vec3 ambient = material.color * material.ambient;
```
- 不受光源方向影响
- 不受阴影影响（即使在阴影中也有基础亮度）

**漫反射 (Diffuse)**
```cpp
double diffuseIntensity = max(0, normal.dot(lightDir));
Vec3 diffuse = material.color * (material.diffuse * diffuseIntensity);
```
- 基于表面法线和光源方向
- **受阴影影响**（在阴影中此项为0）

**镜面反射 (Specular)**
```cpp
Vec3 reflectDir = normal * (2 * normal.dot(lightDir)) - lightDir;
double specularIntensity = pow(max(0, viewDir.dot(reflectDir)), shininess);
Vec3 specular = light.color * (material.specular * specularIntensity);
```
- 产生高光效果
- `shininess` 控制高光大小（值越大，高光越小越亮）
- **受阴影影响**（在阴影中此项为0）

### 3. 多光源支持
支持场景中多个光源同时照明：
```cpp
// 主光源（右上方，白色强光）
lights.push_back(Light(Vec3(5, 5, -2), Vec3(1, 1, 1), 1.5));

// 辅助光源（左侧，橙色弱光）
lights.push_back(Light(Vec3(-3, 3, 0), Vec3(1.0, 0.7, 0.3), 0.5));
```

每个光源独立计算漫反射和镜面反射，最终累加得到总光照。

## 场景设置

### 球体对象
1. **中心大球**（红色，高光泽 shininess=64）
2. **左侧小球**（绿色，低光泽 shininess=16）
3. **右侧小球**（蓝色，中等光泽 shininess=32）
4. **地面球**（灰白色，半径100模拟平面）

### 材质参数
```cpp
Material(Vec3 color, double ambient, double diffuse, double specular, double shininess)
```
- `ambient`: 环境光系数（0.1 = 10%基础亮度）
- `diffuse`: 漫反射系数（0.6-0.8）
- `specular`: 镜面反射系数（0.1-0.5）
- `shininess`: 光泽度（8-64，值越大越光滑）

## 编译运行

### 依赖
- C++11 编译器
- stb_image_write.h（已包含）

### 编译
```bash
g++ -std=c++11 -O2 shadow_raytracer.cpp -o shadow_raytracer -lm
```

### 运行
```bash
./shadow_raytracer
```

输出文件：`shadow_output.png`（800x600）

## 渲染效果

生成的图像包含：
- ✅ **真实阴影**：物体在地面和其他物体上投射的阴影
- ✅ **Phong高光**：不同材质的高光效果（红球最亮，绿球最暗）
- ✅ **多光源照明**：主光源（白色）+ 辅助光源（橙色）
- ✅ **环境光**：即使在阴影中也能看到物体轮廓
- ✅ **Gamma校正**：真实的颜色显示

## 技术亮点

### 阴影判断逻辑
```cpp
if (isInShadow(hitPoint, light.position)) {
    continue;  // 跳过该光源的漫反射和镜面反射计算
}
```
只有"看得见"光源的表面才会接收到该光源的光照。

### 软阴影 vs 硬阴影
当前实现是**硬阴影**（点光源）：
- 优点：计算简单，速度快
- 缺点：阴影边缘锐利，不够自然

未来可扩展为**软阴影**（面光源）：
- 对光源表面进行采样
- 计算部分遮挡度
- 阴影边缘会有渐变（半影区）

### 性能数据
- 渲染分辨率：800 × 600
- 渲染时间：~2秒
- 场景复杂度：4个球体 + 2个光源
- 每像素光线数：1（主光线）+ 2（阴影光线）

## 与之前项目的对比

| 项目 | 阴影 | 光照模型 | 多光源 | 新技术点 |
|------|------|---------|--------|---------|
| 2月12日 - Ray-Sphere | ❌ | 无 | ❌ | 基础求交 |
| 2月13日 - Simple Ray Tracer | ❌ | 简单法线着色 | ❌ | 多物体 |
| 2月15日 - Ray-Sphere V2 | ❌ | 基础漫反射 | ❌ | 优化 |
| **2月17日 - Shadow RT** | ✅ | **Phong完整模型** | ✅ | **阴影算法** |

**核心突破**：
1. 首次实现 **Shadow Ray** 技术
2. 完整的 **Phong 光照模型**（环境+漫反射+镜面）
3. 支持 **多光源** 和 **不同材质** 参数

## 扩展方向

### 短期（1-2天）
- [ ] 实现反射效果（递归光线追踪）
- [ ] 添加折射效果（透明材质）
- [ ] 实现抗锯齿（超采样）

### 中期（1周）
- [ ] 软阴影（面光源采样）
- [ ] 纹理映射
- [ ] 法线贴图

### 长期（1月）
- [ ] BVH加速结构
- [ ] 路径追踪（全局光照）
- [ ] GPU加速（CUDA/OpenCL）

## 学习收获

1. **阴影算法原理**：理解如何通过额外光线判断遮挡关系
2. **Phong光照模型**：掌握三分量光照计算
3. **多光源管理**：实现光照累加和独立阴影判断
4. **材质系统**：理解参数如何影响视觉效果
5. **性能权衡**：每个阴影光线都有成本，需要合理控制

## 参考资料

- [Ray Tracing in One Weekend](https://raytracing.github.io/)
- [Phong Reflection Model - Wikipedia](https://en.wikipedia.org/wiki/Phong_reflection_model)
- [Shadow Ray - Scratchapixel](https://www.scratchapixel.com/)

---

**开发时间**：2026-02-17 10:00-10:10  
**完成状态**：✅ 编译成功 + 运行成功 + 输出正确  
**迭代次数**：1次（一次性成功）  
**技术难度**：⭐⭐⭐（中等）  
**视觉效果**：⭐⭐⭐⭐（显著提升）
