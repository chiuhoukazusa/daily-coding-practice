# Volume Rendering - Ray Marching 体积云渲染

**每日编程实践 - 2026-03-05**

## 项目描述

实现基于 Ray Marching（光线步进）算法的体积渲染器，使用 3D Perlin Noise 生成云层密度场，通过 Beer-Lambert 光吸收定律和 Henyey-Greenstein 相位函数模拟体积散射光照。

## 核心技术

### 1. Ray Marching（光线步进）
- 从相机发出光线，沿光线方向每隔固定步长采样密度
- 累积光线透射率（Beer-Lambert 衰减）
- 支持自适应步长（密度高区域步长减半）

### 2. 3D Perlin Noise + FBM
- 使用经典 Ken Perlin 置换表实现
- FBM（Fractal Brownian Motion）叠加 6 个倍频，生成细腻的云层形态
- 低频噪声控制整体形状，高频噪声增加细节

### 3. Beer-Lambert 光吸收
```
transmittance = exp(-density * stepSize * absorption)
```
光线穿过介质时按密度指数衰减，模拟云的半透明特性。

### 4. 单次散射光照
- 向光源方向进行 16 步阴影采样，计算每个体积点的光照遮蔽
- Henyey-Greenstein 相位函数（g=0.3）模拟前向散射

### 5. 大气天空背景
- 程序化天空渐变（地平线→天顶蓝色过渡）
- 太阳圆盘高光效果
- ACES 色调映射 + Gamma 校正

## 编译运行

```bash
# 编译
g++ -O2 -Wall -Wextra -std=c++17 -o volume_renderer main.cpp

# 运行（约 2 分钟）
./volume_renderer

# 输出文件
# volume_output.ppm  (1.4MB, 可用任意图像查看器打开)
# volume_output.bmp  (1.4MB)
# volume_output.png  (84KB, 用 convert 转换)
```

## 输出结果

![体积云渲染结果](volume_output.png)

渲染参数：
- 分辨率：800 × 600
- 最大步数：120 步
- 光照采样：16 步（阴影光线）
- 渲染时间：约 2 分钟（单线程）

## 量化验证结果

| 检测项 | 实际值 | 期望 |
|--------|--------|------|
| 中心区域 RGB | (225, 228, 230) | 浅色（云层） ✅ |
| 天空区域 RGB | (203, 219, 231) | 蓝色偏移 ✅ |
| 平均亮度 | 216/255 | > 30 ✅ |
| 颜色标准差 | ~15 | > 5（有变化）✅ |
| 文件大小 | 84KB (PNG) | > 1KB ✅ |

## 技术要点

- **体积渲染基础**：Ray Marching 是实时/离线体积渲染的核心算法
- **光照积分**：体积渲染方程 `L(x) = ∫ σ(t)·L_s(t)·T(t) dt`
- **Beer-Lambert 定律**：光在介质中的指数衰减
- **Perlin FBM**：多倍频叠加创造分形细节
- **ACES 色调映射**：HDR 到 LDR 的工业标准映射

## 迭代历史

1. **初始版本**：完整代码编写完成（~500行）
2. **迭代 1**：修复编译警告（未使用的变量 `t`）
3. **最终版本**：✅ 0错误 0警告，渲染成功

## 代码仓库

GitHub: https://github.com/chiuhoukazusa/daily-coding-practice/tree/main/2026/03/03-05-Volume-Rendering-Ray-Marching

---
**完成时间**: 2026-03-05 05:37  
**迭代次数**: 2 次  
**编译器**: g++ (GCC) C++17
