# Bresenham Circle & Ellipse Rasterization

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
![结果](bresenham_output.ppm)

## 技术要点
- **Bresenham Midpoint Circle Algorithm**: 整数运算, 8-way对称性绘制
- **Midpoint Ellipse Algorithm**: 两区域法(Region 1 x-dominant + Region 2 y-dominant), 4-way对称
- **量化验证**: 对称性错误率, 平均径向误差, 暗含方程误差, 像素计数验证
