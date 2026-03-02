# Environment Mapping with Cube Map Skybox

## 项目描述

实现基于立方体贴图（Cube Map）的天空盒渲染与环境反射技术。

**核心技术**：
- **程序化 Cube Map 生成**：渐变天空 + 太阳光晕 + 程序化星空
- **天空盒渲染**：通过光线方向直接采样 Cube Map 6 个面
- **环境反射材质**：
  - 黄金金属球（反射率 0.95，微粗糙）
  - 玻璃折射球（IOR 1.52，Fresnel 混合反射/折射）
  - 铬金属球（高度镜面反射）
- **ACES 色调映射** + Gamma 校正
- **2x2 SSAA 超采样抗锯齿**

## 编译运行

```bash
g++ -O2 -std=c++17 -Wno-missing-field-initializers -o skybox main.cpp
./skybox
```

**依赖**：仅需 `stb_image_write.h`（已包含），无其他依赖。

## 输出结果

### 主渲染图（天空盒 + 反射球）

![天空盒主渲染](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/03/03-03-Environment-Mapping-Skybox/skybox_output.png)

**量化验证结果**：
- 天空区域 RGB 均值：R=180.5, G=173.8, B=215.7（蓝色天空 ✅）
- 左金属球 RGB 均值：R=220.2, G=197.5, B=161.3（暖色反射 ✅）
- 颜色方差：R=17.1, G=14.5, B=29.3（颜色变化丰富 ✅）

### Cube Map 展开图（6 个面预览）

![Cube Map 展开](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/03/03-03-Environment-Mapping-Skybox/cubemap_faces.png)

## 技术要点

### Cube Map 方向映射

将 3D 方向向量映射到 Cube Map 的核心算法：

```cpp
Vec3 CubeMap::sample(const Vec3& dir) const {
    double ax = abs(d.x), ay = abs(d.y), az = abs(d.z);
    // 找主轴，计算 uv 坐标
    if (ax >= ay && ax >= az) {
        face = d.x > 0 ? POS_X : NEG_X;
        u = (d.x > 0) ? -d.z/d.x : d.z/(-d.x);
        v = -d.y / abs(d.x);
    } // ... 其他轴类似
}
```

### 程序化天空生成

通过高度角混合不同颜色，模拟日落/黎明效果：

```cpp
Vec3 zenithColor(0.1, 0.2, 0.8);    // 天顶：深蓝
Vec3 horizonColor(0.95, 0.55, 0.2); // 地平线：橙色
Vec3 baseColor = lerp(horizonColor, zenithColor, pow(height, 0.4));
// + 太阳光晕（Mie散射近似）
// + 程序化星空（哈希函数）
```

### 玻璃球折射（Snell 定律）

```cpp
// 计算折射率比
double ratio = etaI / etaT; // n1/n2
double sinT2 = ratio*ratio * (1 - cosI*cosI);
// 全内反射检测
bool tir = (sinT2 > 1.0);
// Fresnel 混合
double fr = fresnelSchlick(cosI, etaI, etaT);
Vec3 result = lerp(refractColor, reflectColor, fr);
```

## 迭代历史

- **迭代 1**：初始版本，编译警告来自第三方 stb 头文件 → 添加 `-Wno-missing-field-initializers`，**通过**
- **最终版本**：✅ 编译 0 错误 0 警告，运行时间 0.28s，量化验证全部通过

## 代码仓库

GitHub: https://github.com/chiuhoukazusa/daily-coding-practice/tree/main/2026/03/03-03-Environment-Mapping-Skybox
