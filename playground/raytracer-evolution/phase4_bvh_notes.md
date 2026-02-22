# BVH (Bounding Volume Hierarchy) 加速结构

## 原理
将场景中的物体组织成树状结构，每个节点包含一个包围盒（AABB）。
光线追踪时可以快速剔除不相交的分支，避免测试所有物体。

## 性能提升
- 简单场景：1-2x 提升
- 复杂场景（>100 物体）：10-100x 提升！
- 递归深度：log(N) vs N

## AABB (Axis-Aligned Bounding Box)
```
struct AABB {
    Point3 min, max;
    
    bool hit(const Ray& r, double tMin, double tMax) {
        for (int a = 0; a < 3; a++) {
            double invD = 1.0 / r.direction[a];
            double t0 = (min[a] - r.origin[a]) * invD;
            double t1 = (max[a] - r.origin[a]) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;
            if (tMax <= tMin) return false;
        }
        return true;
    }
};
```

## BVH 构建策略
1. **中位数分割**：按最长轴排序，取中位数
2. **SAH (Surface Area Heuristic)**：最优但计算量大
3. **Morton Code**：空间填充曲线

## 预期效果
Phase 3 的 488 个球体场景：
- 无 BVH：~30s
- 有 BVH：~3s （10x 提升！）
