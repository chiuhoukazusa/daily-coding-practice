# 每日编程实践 - 项目索引

**目的**: 避免重复项目，确保每日挑战的多样性和技术进步

## 已完成项目列表

### 2026年2月

| 日期 | 项目名称 | 核心技术 | 输出 | 状态 |
|------|---------|---------|------|------|
| 02-06 | Simple Ray Tracer | 基础光线追踪, 球体渲染 | output.ppm | ✅ 完成 |
| 02-10 | Perlin Noise 纹理生成器 | Perlin Noise, FBM, 程序化纹理 | clouds/marble/wood.png | ✅ 完成 |
| 02-11 | Voronoi Diagram | 计算几何, 最近点搜索 | voronoi.png | ✅ 完成 |
| 02-12 | Ray-Sphere Intersection | 光线追踪, 球体求交 | raytracer.png | ✅ 完成 |
| 02-13 | Simple Ray Tracer | 完整光线追踪器, 多球体场景 | raytrace.png | ✅ 完成 |
| 02-14 | Bresenham Line Algorithm | 像素绘制, 光栅化 | bresenham.png | ✅ 完成 |
| 02-15 | Ray-Sphere Intersection V2 | 光线追踪优化 | render.png | ✅ 完成 |
| 02-16 | Perlin Noise 地形生成 | Perlin Noise, 高度图 | terrain_512/1024.png | ✅ 完成 |
| 02-17 | Shadow Ray Tracing | 光线追踪, Shadow Ray, Phong光照, 多光源 | shadow_output.png | ✅ 完成 |
| 02-18 | 递归光线追踪 - 镜面反射 | 递归光线追踪, 镜面反射, 材质系统 | reflection_output.png | ✅ 完成 |
| 02-19 | 递归光线追踪 - 折射效果（玻璃球） | Snell定律, Fresnel效应, 全反射, 玻璃材质 | refraction_output.png | ✅ 完成 |
| 02-20 | 纹理映射光线追踪器 | 球面UV映射, 棋盘格纹理, 纹理采样 | texture_output.png | ✅ 完成 |
| 02-22 | Bezier 曲线绘制器 | De Casteljau算法, 参数曲线, 2/3/4阶曲线 | bezier_*.png (4个) | ✅ 完成 |
| 02-23 | Normal Mapping | 法线贴图, 切线空间, TBN矩阵, 程序化纹理, Phong光照 | normal_mapping_output.png | ✅ 完成 |
| 02-24 | Parallax Mapping | 视差贴图, UV偏移, 高度图, 砖块纹理, 左右对比 | parallax_output.png | ✅ 完成 |
| 02-25 | OBJ Model Loader | OBJ格式解析, 线框渲染, 正交投影, Bresenham算法 | obj_loader_output.png | ✅ 完成 |
| 02-26 | Triangle Rasterization | Barycentric坐标, Z-Buffer深度测试, 颜色插值, 边界框优化 | rasterization_output.png | ✅ 完成 |
| 02-27 | PBR Cook-Torrance BRDF | Cook-Torrance BRDF, GGX NDF, Fresnel-Schlick, Smith几何遮蔽, ACES色调映射 | pbr_output.png | ✅ 完成 |
| 02-28 | Ambient Occlusion | 蒙特卡洛积分, 余弦加权采样, TBN矩阵, 环境光遮蔽 | ao_output.png | ✅ 完成 |
| 03-01 | BVH Accelerated Ray Tracer | BVH树, SAH构建策略, AABB求交, 光线追踪加速 | bvh_output.png, bvh_comparison.png, bvh_visualization.png | ✅ 完成 |

## 技术领域覆盖

### ✅ 已完成领域
- [x] 程序化纹理生成（Perlin Noise）
- [x] 计算几何（Voronoi）
- [x] 光线追踪（Ray Tracing）
- [x] 像素绘制算法（Bresenham）
- [x] 高度图地形生成
- [x] **阴影算法（Shadow Ray）**
- [x] **Phong光照模型**
- [x] **镜面反射（递归光线追踪）**
- [x] **折射效果（玻璃材质）**
- [x] **纹理映射（UV坐标+球面映射）**
- [x] **Bezier曲线（De Casteljau算法）**
- [x] **法线贴图（Normal Mapping）** ✅ (2026-02-23)
- [x] **视差贴图（Parallax Mapping）** ✅ (2026-02-24)
- [x] **3D模型加载（OBJ格式）** ✅ (2026-02-25)
- [x] **三角形光栅化（Rasterization）** ✅ (2026-02-26)

### 📋 待探索领域（优先级排序）

#### 🔥 高优先级（立即可做）
1. ~~**阴影算法**~~ ✅ 已完成（2026-02-17）
2. ~~**反射/折射**~~ ✅ 已完成（2026-02-18 反射，2026-02-19 折射）
3. ~~**纹理映射**~~ ✅ 已完成（2026-02-20）
4. ~~**法线贴图**~~ ✅ 已完成（2026-02-23）
5. ~~**视差贴图（Parallax Mapping）**~~ ✅ 已完成（2026-02-24）
6. ~~**3D模型加载**~~ ✅ 已完成（2026-02-25）
7. ~~**三角形光栅化**~~ ✅ 已完成（2026-02-26）
8. **B-spline曲线** - 更灵活的曲线 ← 下一步推荐

#### ⭐ 中优先级（需要一定基础）
6. ~~**Bezier曲线**~~ ✅ 已完成（2026-02-22）
7. **B-spline曲线** - 更灵活的曲线
8. **BVH加速结构** - 光线追踪加速
9. **抗锯齿技术** - MSAA / SSAA / FXAA（注：SSAA在02-21已实现但未正式发布）
10. **粒子系统** - 火焰/烟雾效果

#### 💡 低优先级（进阶主题）
11. **体积渲染** - 云雾效果
12. **PBR材质** - 基于物理的渲染
13. **SSAO** - 屏幕空间环境光遮蔽
14. **布料模拟** - 质点弹簧系统
15. **流体模拟** - SPH算法

#### 🎮 游戏相关
16. **碰撞检测** - AABB/OBB/SAT
17. **路径规划** - A* 算法
18. **地形LOD** - 多细节层次
19. **骨骼动画** - 蒙皮变形
20. **Procedural Generation** - 随机地图生成

## 项目创建前检查清单

在创建新的每日编程实践项目前，**必须**完成以下检查：

- [ ] 1. 查看 `PROJECT_INDEX.md` 确认没有重复
- [ ] 2. 确认新项目属于"待探索领域"
- [ ] 3. 评估项目难度和所需时间（目标1小时内完成）
- [ ] 4. 准备相关学习资料
- [ ] 5. 在索引中标记项目为"进行中"

## 重复检测规则

### 判断标准
两个项目视为**重复**，如果满足以下任一条件：
1. **核心算法相同**（即使输出格式不同）
2. **输出结果类型相同**（即使实现细节不同）
3. **技术栈90%重叠**

### 允许的迭代
以下情况**不视为重复**：
- 性能优化版本（如SIMD加速）
- 功能扩展版本（如从2D到3D）
- 不同应用场景（如从静态到动态）

**前提**：必须有**明显的技术提升**和**新的学习价值**

## 今日教训（2026-02-17）

### 失误
- ❌ 创建了与02-10完全重复的Perlin Noise纹理生成项目
- ❌ 没有查看历史记录
- ❌ 浪费了宝贵的学习时间

### 改进措施
- ✅ 建立此项目索引文件
- ✅ 未来每日项目前先查看索引
- ✅ 今天需要重新选择项目并完成

### 今日补救计划
**新项目选择**：从"高优先级待探索领域"中选择

**推荐方向**：
1. **阴影算法** - 在现有光线追踪基础上添加阴影
2. **反射效果** - 递归光线追踪，实现镜面反射
3. **Phong光照** - 实现完整的光照模型

**最终选择**：待确认

---

**维护规则**：
- 每完成一个项目，立即更新此文件
- 每天开始新项目前，必须先查看此文件
- 每周回顾，评估技术覆盖面和进步

**文件位置**：`/root/.openclaw/workspace/daily-coding-practice/PROJECT_INDEX.md`
**创建日期**：2026-02-17
**创建原因**：避免重复项目，确保持续进步
