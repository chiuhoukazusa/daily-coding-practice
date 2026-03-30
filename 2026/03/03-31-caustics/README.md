# Caustics Renderer

玻璃球折射产生的焦散光斑渲染器，使用光子映射（Photon Mapping）算法。

## 编译运行

```bash
# 需要 stb_image_write.h（或使用 stb_wrapper.h）
g++ main.cpp -o output -std=c++17 -O3 -march=native
./output
# 输出：caustics_output.png (640×480)
```

## 效果说明

场景设置：
- 点光源从玻璃球正上方照射
- IOR=1.5 的玻璃球（半径 0.75）
- 漫反射地板 + 背景墙

渲染结果：地板上出现焦散光斑（玻璃折射聚焦的光线）

## 算法技术点

- **光子映射（Photon Mapping）**：两遍渲染，Pass1发射光子并存储到KD树，Pass2查询密度估计
- **Snell 折射定律**：n₁ sin θ₁ = n₂ sin θ₂
- **Fresnel 反射（Schlick近似）**：计算折射/反射概率
- **KD-Tree**：半径 k-近邻搜索，加速光子密度估计
- **蒙特卡洛积分**：随机采样消除噪声
- **ACES 色调映射 + Gamma 2.2 校正**

## 参数

| 参数 | 值 |
|------|-----|
| 分辨率 | 640×480 |
| SPP（每像素采样数）| 8 |
| 光子数 | 300,000 |
| 最大折射次数 | 6 |
| 密度估计半径 | 0.12 |
| 近邻光子数 k | 60 |
