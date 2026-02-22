# 曼德勃罗集 (Mandelbrot Set)

## 数学定义

对于复平面上的点 c = a + bi，迭代序列：

```
z₀ = 0
zₙ₊₁ = zₙ² + c
```

如果序列不发散（|zₙ| ≤ 2），则 c 属于曼德勃罗集。

## 渲染算法

1. 遍历图像每个像素 (x, y)
2. 映射到复平面坐标 c = (a, b)
3. 迭代计算 z = z² + c，最多 max_iter 次
4. 根据迭代次数着色：
   - 不发散（max_iter次内） → 黑色（属于集合）
   - 发散 → 根据发散速度用彩色渐变

## 参数
- 中心区域：(-0.5, 0)
- 缩放：1.5（显示范围 [-2, 1] × [-1.5, 1.5]）
- 最大迭代：256-1000
- 颜色映射：HSV → RGB

## 输出
- `mandelbrot_basic.png` - 基础渲染
- `mandelbrot_zoom1.png` - 放大某处细节
- `mandelbrot_zoom2.png` - 更深层次放大
- `mandelbrot_rainbow.png` - 彩虹配色
