# Anisotropic GGX Microfacet BRDF

**日期**: 2026-11-25
**方向**: 图形学 / PBR 微表面着色

## 技术点

实现 **各向异性 GGX 法线分布**（Anisotropic GGX Normal Distribution Function），
用两个独立的粗糙度轴（切线 T 与副切线 B）替代单一粗糙度，从而在金属表面产生
**被拉伸的高光（anisotropic specular highlight）**——即拉丝金属、CD 光盘、磨砂
金属等材质特有的细长光斑。

完整 Cook-Torrance 微表面模型：
- `D` — 各向异性 GGX NDF（`1/(π·ax·ay·((h·T/ax)² + (h·B/ay)² + (h·N)²)²)`）
- `G` — Smith GGX 几何遮蔽（GGX 单边遮蔽 ×2，用等效粗糙度 √(ax·ay)）
- `F` — Schlick Fresnel（金属 F0=0.9）

同时渲染一张各向同性（iso，ax=ay）参考图作为对照。

## 量化验证结果

| 指标 | 各向异性 (ax=0.3, ay=0.05) | 各向同性 (a=0.15) |
|------|---------------------------|-------------------|
| 高光椭圆长/短轴特征值 | 179.3 / 4.44 | 30.2 / 27.7 |
| **高光椭圆度（长/短轴比）** | **6.35×** | **1.04×** |
| 高光包围盒 | 52 × 7 px | 20 × 20 px |
| 亮部像素数 | 353 | 364 |

数学验证通过：
- 各向同性参考高光椭圆度 ≈ 1.04（近圆）✅
- 各向异性高光椭圆度 ≈ 6.35（明显拉长）✅，且拉长方向沿 bitangent 轴
- 图像非全黑/全白，光照区域均值/标准差正常

## 文件

- `main.cpp` — 软光栅化金属球渲染器（各向异性 + 各向同性两种配置）
- `aniso_anisotropic.ppm` / `aniso_isotropic.ppm` — 输出图像
- `output` — 编译产物

编译：`g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra`（0 errors, 0 warnings）
