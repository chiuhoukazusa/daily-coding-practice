# View Frustum Culling (Plane Extraction + AABB/Sphere Tests)

**日期**: 2026-11-28
**方向**: 游戏开发 / 实时渲染

## 技术点

实现**视锥体剔除（View Frustum Culling）**——实时渲染管线的核心优化：
在把场景对象送往 GPU 渲染之前，先用代价极低的几何判定，剔除掉那些
完全位于相机视锥之外、根本看不见的对象。

核心链路：

1. 由**视图矩阵 × 投影矩阵**（view-projection）经 **Gribb–Hartmann**
   算法提取出视锥的 **6 个裁剪平面**（左/右/下/上/近/远）。
2. 将每个平面归一化，保持**向内朝向**（视锥内部 = 负半空间）。
3. 对每个对象的**包围盒（AABB）**或**包围球（Sphere）**做保守判定，
   得出三种状态：`IN`（完全在内）/`INTERSECT`（部分跨越）/`OUT`（完全在外）。
4. `OUT` 的对象直接跳过渲染，从而节省大量 GPU + CPU 工作量。

关键设计：AABB 判定用"最小距离顶点（negative vertex）"做**快速拒绝**——
只要某个平面下盒子上最靠内的顶点仍在平面外侧，整个盒子就必然在外，
可立即 `OUT` 并**提前退出（early-out）**，无需再测剩余平面。

## 量化验证（非视觉检查）

```
=== Validation 1: plane extraction sanity ===
  all plane normals unit-length: PASS
  all 8 frustum corners inside (dist<=0): PASS (max dist 5.33e-15)
  point behind camera classified OUT: PASS

=== Validation 2: correctness vs brute-force sampling ===
  AABB: 2000 tests, false-negatives=0, IN/OUT disagree=0
  Sphere: 2000 tests, false-negatives=0, IN/OUT disagree=0
  AABB INTERSECT predictions verified by sampling: 218/218
  => correctness: PASS

=== Validation 3: culling cost (early-out benefit) ===
  objects tested: 200000
  culled (OUT): 157938, visible (IN+INTERSECT): 42062
  naive plane-tests (no early-out): 1200000  (== 6*N)
  analytic plane-tests (early-out):  555469
  plane-test reduction: 46.3% of naive
  => cull ratio: 79.0% of scene rejected
```

**三层验证的含义**：

- **Layer 1（几何自洽）**：6 个平面法向量均为单位长度；视锥的 8 个角点
  全部满足 6 个平面约束（最大距离 ≈ 5e-15，即严格在内部）；相机正后方
  的点被正确判定为 OUT。这证明平面提取本身没有符号/朝向错误。
- **Layer 2（正确性）**：用**稠密点采样暴力法**作为 oracle，对 2000 个
  随机 AABB 和 2000 个随机球体逐一比对。保守判定**零假阴性**（绝不把
  实际可见的对象误判为 OUT）、IN/OUT 状态完全一致、218 个 INTERSECT
  预测全部被采样验证。这证明剔除是**保守且正确**的——绝不会"误杀"。
- **Layer 3（性能收益）**：early-out 将平面测试次数从 6N 降到约 46%，
  场景 79% 的对象被剔除，验证了剔除的实际价值（每少测一个平面、少
  提交一个对象，都是实打实省下的渲染成本）。

## 文件

- `main.cpp` — 视锥平面提取 + AABB/球体保守判定 + 三层量化验证

编译：`g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra`（0 errors, 0 warnings）

## 输出展示

（本项目为纯数值/几何验证，输出为上述量化指标表）
