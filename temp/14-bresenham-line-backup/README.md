# Bresenham直线绘制算法

## 项目信息
- **开发时间**：约 15 分钟
- **代码量**：2650 行 C++ (含PPM数据)
- **迭代次数**：2 次（一次修复编译警告）
- **编译状态**：✅ 编译成功（0 错误 0 警告）
- **运行结果**：✅ 成功生成 800x600 像素图像，包含 10 条不同方向的直线

## 迭代历史
1. **初始版本** - 编译成功但有类型转换警告：比较有符号整数int和无符号size_t类型
2. **第一次修复** - 将pixels[0].size()和pixels.size()转换为int：使用static_cast<int>()进行显式类型转换
3. **最终版本** - ✅ 所有测试通过：编译无警告，运行成功，正确生成图像

## 技术要点

### 核心算法：Bresenham直线算法
Bresenham算法是计算机图形学中用于绘制直线的经典算法，它仅使用整数运算，避免了浮点数计算的性能开销。

### 算法步骤
1. 计算dx和dy（x轴和y轴的差异绝对值）
2. 判断步进方向（sx, sy）
3. 初始化误差项err = dx - dy
4. 循环绘制点，每次根据误差项决定x或y方向的步进

### 关键代码
```cpp
// Bresenham line algorithm
void draw_line(std::vector<std::vector<int>>& pixels, int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // Draw current point
        if (x1 >= 0 && y1 >= 0 && 
            x1 < static_cast<int>(pixels[0].size()) && 
            y1 < static_cast<int>(pixels.size())) {
            pixels[y1][x1] = 1;
        }
        
        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}
```

## 效果展示

绘制了以下10条直线：
1. 顶部水平线 (50,50 到 750,50)
2. 底部水平线 (50,550 到 750,550)
3. 左侧垂直线 (50,50 到 50,550)
4. 右侧垂直线 (750,50 到 750,550)
5. 主对角线 (50,50 到 750,550)
6. 反对角线 (750,50 到 50,550)
7. 斜线1 (200,150 到 600,250)
8. 斜线2 (300,300 到 500,400)
9. 垂直线3 (400,200 到 400,500)
10. 斜线4 (150,450 到 650,150)

![Bresenham直线效果](bresenham_output.png)

## 编译运行

```bash
# 编译
g++ -std=c++17 -Wall -Wextra -O2 bresenham.cpp -o bresenham_output

# 运行
./bresenham_output

# 可选：将PPM转换为PNG（需要ImageMagick）
convert bresenham_output.ppm bresenham_output.png
```

## 学习收获
1. **加深对Bresenham算法的理解**：通过实现算法，更清楚地理解了整数误差项在直线绘制中的作用
2. **处理类型转换警告**：学习了如何正确处理C++中的有符号/无符号整数比较
3. **PPM图像格式**：理解了简单的PPM(P3)格式用于存储图像数据
4. **像素坐标边界检查**：在绘制前需要确保坐标在画布范围内，避免越界访问

## 应用场景
- 计算机图形学基础
- 游戏开发中的2D图形渲染
- 嵌入式系统显示驱动
- 图形库和GUI框架实现