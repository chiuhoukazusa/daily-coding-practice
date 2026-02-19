# Daily Coding Practice - 每日编程实践

> 每天一个可运行的小项目，持续学习和创造

[![GitHub](https://img.shields.io/badge/GitHub-daily--coding--practice-blue)](https://github.com/chiuhoukazusa/daily-coding-practice)
[![Blog](https://img.shields.io/badge/Blog-chiuhoukazusa.github.io-green)](https://chiuhoukazusa.github.io/chiuhou-tech-blog/)

## 🎯 项目理念

**核心原则**:
- ✅ 每个项目必须**能编译运行**
- ✅ 输出结果必须**符合预期**（量化验证）
- ✅ **允许并鼓励多次迭代调试**，持续修复直到完全正确
- ✅ 代码质量良好，有注释说明
- ✅ 完成后自动上传 GitHub 和发布博客

**验证体系**（避免上下颠倒等错误）:
1. ✅ **编译验证** - 代码能编译通过
2. ✅ **逻辑验证** - 程序运行成功，输出文件存在
3. ✅ **内容验证** - 像素采样、量化检查（不只是"看起来对"）
4. ✅ **发布验证** - GitHub 上传成功，博客部署正确

## 📁 仓库结构

```
daily-coding-practice/
├── 2026/
│   ├── 02/
│   │   ├── 06-simple-raytracer/         # 基础光线追踪
│   │   ├── 10-perlin-noise/             # Perlin噪声纹理
│   │   ├── 11-voronoi-generator/        # Voronoi图生成
│   │   ├── 12-raytracer-sphere/         # 球体求交
│   │   ├── 14-bresenham-line/           # Bresenham直线算法
│   │   ├── 17-shadow-ray-tracing/       # 阴影光线追踪
│   │   ├── 18-recursive-raytracing-reflection/  # 镜面反射
│   │   ├── 19-refraction-glass-ball/    # 折射效果
│   │   └── 20-纹理映射光线追踪器/       # 纹理映射
│   └── ...
├── scripts/
│   ├── check_duplicate.sh               # 重复项目检测
│   └── verify_blog.sh                   # 博客部署验证
├── PROJECT_INDEX.md                     # 项目索引（避免重复）
└── README.md
```

## 📋 命名规范

**目录命名格式**: `DD-项目名称`

- `DD`: 两位数字日期（如 `06`, `10`, `20`）
- `项目名称`: 
  - 优先使用**英文小写+连字符**（如 `simple-raytracer`, `perlin-noise`）
  - 技术性名称可用中文（如 `纹理映射光线追踪器`）
  - 避免空格，使用连字符 `-` 分隔单词

**示例**:
- ✅ `06-simple-raytracer`
- ✅ `10-perlin-noise`
- ✅ `20-纹理映射光线追踪器`
- ❌ `2026-02-06-simple-raytracer`（不要加年月）
- ❌ `simple_raytracer`（不要用下划线）
- ❌ `Simple Raytracer`（不要用空格和大写）

**文件结构**:
```
DD-项目名称/
├── README.md          # 项目说明（中文）
├── main.cpp           # 主程序（或其他语言）
├── output.png         # 输出图片
└── stb_image_write.h  # 依赖库（如果需要）
```

## 📊 项目索引

完整的项目列表和技术覆盖面请查看: [PROJECT_INDEX.md](./PROJECT_INDEX.md)

**快速统计**（截至 2026-02-20）:
- 已完成项目: **12 个**
- 技术领域: 图形学、算法、计算几何
- 核心技术: 光线追踪、程序化纹理、光栅化

## 🛠️ 技术栈

**主要语言**:
- **C++** - 图形学、性能敏感项目（光线追踪等）
- **Python** - 算法可视化、快速原型
- **Unity C#** - 游戏开发（未来）

**工具链**:
- 编译器: `g++` with C++17
- 图像库: `stb_image_write.h`
- 验证: 像素采样脚本、HTTP 状态码检查

## 🚀 使用方法

### 创建新项目前检查

```bash
# 检查是否重复（必须执行）
./scripts/check_duplicate.sh "项目名称"

# 如果提示重复，从 PROJECT_INDEX.md 的"待探索领域"选择其他项目
```

### 编译运行

```bash
cd 2026/02/DD-项目名称
g++ -std=c++17 main.cpp -o output
./output
```

### 验证输出

```bash
# 图形学项目：采样验证坐标系
bash /path/to/coordinate-system-verification/verify_coordinate_system.sh output.png

# 通用：检查文件是否生成
ls -lh output.png
```

### 发布到博客

```bash
# 自动上传图片到 blog_img 仓库
# 自动生成博客文章
# 自动部署到 Hexo 博客
# （通过 blog-code-workflow skill 完成）
```

## 🎓 学习路径

### 已完成领域 ✅
- [x] 程序化纹理（Perlin Noise, Voronoi）
- [x] 基础光线追踪（球体求交、多物体场景）
- [x] 阴影算法（Shadow Ray）
- [x] 镜面反射（递归光线追踪）
- [x] 折射效果（Snell 定律、Fresnel 方程）
- [x] 纹理映射（球面 UV 映射）
- [x] 像素绘制（Bresenham 算法）

### 进行中 🚧
- [ ] 抗锯齿（MSAA/SSAA）
- [ ] 法线贴图
- [ ] BVH 加速结构

### 计划中 📋
- [ ] Bezier 曲线
- [ ] 3D 模型加载（OBJ）
- [ ] PBR 材质
- [ ] 碰撞检测
- [ ] A* 路径规划

完整技术路线图: [PROJECT_INDEX.md](./PROJECT_INDEX.md#技术领域覆盖)

## 📝 博客集成

每个项目完成后会自动发布到博客：
- **博客地址**: https://chiuhoukazusa.github.io/chiuhou-tech-blog/
- **图床仓库**: https://github.com/chiuhoukazusa/blog_img

**博客文章包含**:
- 项目背景和目标
- 核心技术解析
- 代码实现要点
- 结果展示（图片）
- 遇到的问题和解决方案

## 🐛 常见问题

### Q: 为什么图像上下颠倒？
A: 坐标系问题！使用 `coordinate-system-verification` skill 验证：
- 采样图像顶部/底部像素
- 判断天空（浅色）/地面（深色）位置
- 如果天空在底部 → 上下颠倒

### Q: 如何避免重复项目？
A: 创建前运行 `./scripts/check_duplicate.sh "项目名称"`

### Q: 项目目录应该放在哪里？
A: `2026/02/DD-项目名称/`（两位数字日期 + 项目名）

## 📊 项目复盘

每个项目完成后记录：
- ✅ 成功的技术点
- ❌ 遇到的问题
- 🔧 解决方案
- 📚 学到的经验

**最新教训** (2026-02-20):
- 不相信"看起来对"，要量化验证
- 坐标系错误需要自动采样检测
- 发布前必须验证 GitHub + 博客部署

## 🤝 贡献

欢迎建议项目主题或技术改进！

## 📄 License

MIT License - 自由使用和学习

---

**仓库地址**: https://github.com/chiuhoukazusa/daily-coding-practice  
**维护者**: Chiuhou  
**更新时间**: 2026-02-20

*"代码是写出来的，不是想出来的" - 每日一练，持续进步*
