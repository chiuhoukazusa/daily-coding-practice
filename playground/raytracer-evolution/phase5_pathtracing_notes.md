# Path Tracing vs Ray Tracing

## 核心区别

### Ray Tracing (我们当前的实现)
- 每条光线只考虑直接光照
- 反射/折射有限次数后停止
- 适合：镜面、玻璃等理想材质

### Path Tracing (全局光照)
- 光线在场景中多次弹射，累积间接光照
- 更真实的漫反射和软阴影
- 适合：真实感渲染、间接光照

## 康奈尔盒子 (Cornell Box)
经典的全局光照测试场景：
- 封闭的房间
- 左墙（红色）、右墙（绿色）、其他墙（白色）
- 顶部光源
- 两个盒子

### 预期效果
- **颜色渗透**：红墙的光反射到白盒子上，出现红色调
- **软阴影**：面光源产生渐变阴影
- **环境光遮蔽**：角落更暗

## 实现要点
```cpp
// 发光材质
struct EmissiveMaterial {
    Color emission;
};

// 路径追踪核心
Color pathTrace(const Ray& r, const Scene& scene, int depth) {
    if (depth <= 0) return Color(0, 0, 0);
    
    HitRecord rec;
    if (scene.hit(r, 0.001, INF, rec)) {
        // 发光物体
        if (rec.material->isEmissive()) {
            return rec.material->emission;
        }
        
        // 漫反射 - 随机方向采样
        Vec3 target = rec.point + rec.normal + randomUnitVector();
        Ray scattered(rec.point, target - rec.point);
        
        // 递归累积
        return rec.material->albedo * pathTrace(scattered, scene, depth - 1);
    }
    
    return Color(0, 0, 0);  // 黑色背景（封闭环境）
}
```

## 性能
- 需要更多 samples per pixel（500-1000）
- 收敛较慢，但效果惊艳
