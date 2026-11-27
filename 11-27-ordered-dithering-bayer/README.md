# Ordered Dithering (Bayer Matrix)

**日期**: 2026-11-27
**方向**: 图像处理 / 数字半调 / Bayer 有序抖动矩阵

## 技术点

实现**有序抖动（Ordered Dithering）**——一种与之前做过的
Floyd–Steinberg **误差扩散（error diffusion）**截然不同的半调（halftoning）方法。
二者的核心区别：

| | 有序抖动（本项目） | 误差扩散（07-02） |
|---|---|---|
| 阈值来源 | 固定 **Bayer 矩阵** | 邻域误差反馈 |
| 误差传播 | **无** | 有（向左/下扩散） |
| 确定性 | 完全确定、逐像素独立 | 顺序依赖 |
| 可并行性 | **天然可并行** | 串行 |

有序抖动用一个固定阈值矩阵（Bayer Matrix）对灰度图做阈值量化，以**空间分辨率**
换取**可感知的色调范围**：画面在局部平均后仍能还原连续灰度，从而在相同的输出
色阶数（level）下，比朴素量化（naive quantization）**隐藏量化带（banding）**。

## Bayer 矩阵构造

Bayer 矩阵通过 **2×2 递归细分**生成，大小为 2 的幂：

```
M_2  = [ 0  2 ]
       [ 3  1 ]

M_{2n} = [ 4·M_n + 0    4·M_n + 2 ]
         [ 4·M_n + 3    4·M_n + 1 ]
```

矩阵内 0..n²-1 每个值恰好出现一次，且值在空间上均匀分布，保证了抖动的
蓝噪声特性（低频能量低，视觉上最舒适）。

## 量化验证结果

```
=== Bayer Matrix (8x8) validation ===
  all levels 0..63 exactly once: PASS
  range [0, 63]

=== Quantization metrics (radius-4 local integration) ===
bits   | method | distinct-lvls | global-MSE | PSNR(dB) | local-MAE
-------+--------+---------------+------------+----------+----------
1bit   | naive  |        2      |   5326.21  |  10.87   |  55.976
1bit   | bayer  |        2      |  10107.48  |   8.08   |   4.268
2bit   | naive  |        4      |    518.97  |  20.98   |  16.843
2bit   | bayer  |        4      |   1017.81  |  18.05   |   1.414
4bit   | naive  |       16      |     20.58  |  35.00   |   2.544
4bit   | bayer  |       16      |     40.97  |  32.01   |   0.308
```

**核心结论**（关键量化指标是 **local-MAE**，即窗口平均后的局部绝对误差）：

- **distinct-levels 相同**：naive 与 bayer 均只使用 N 个输出色阶（1bit=2、2bit=4、
  4bit=16），证明二者信息量一致。
- **local-MAE 大幅降低**：bayer 的局部误差在 1/2/4bit 分别从 55.98→4.27、
  16.84→1.41、2.54→0.31，**降低一个数量级**。这才是人眼真正感知的指标——
  视觉系统会在小邻域内做积分，因此抖动虽让逐像素 MSE（global-MSE）上升，
  却让**局部色调还原**显著变好。
- **tradeoff 正确**：PSNR 略降（逐像素代价），但换来的是量化带（banding）
  的消除——这正是半调的核心取舍。

## 文件

- `main.cpp` — Bayer 矩阵生成 + 有序抖动 + 朴素量化对比 + 量化验证
- `source.png` — 512×512 垂直渐变 + 棋盘格测试图
- `{1,2,4}bit_{naive,bayer}.png` — 三种位深下朴素/有序抖动的对比输出
- `cover.png` — 封面图（source + 三组 naive/bayer 对比拼接）
- `output` — 编译产物

编译：`g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra`（0 errors, 0 warnings）

## 输出展示

![结果](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/11/11-27-ordered-dithering-bayer/cover.png)
