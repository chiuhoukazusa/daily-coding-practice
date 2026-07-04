# Sutherland-Hodgman Polygon Clipping

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![结果](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/07/07-05-sutherland-hodgman/sutherland_hodgman_output.png)

## 技术要点
- Sutherland-Hodgman算法：逐边裁剪多边形
- 四种顶点情况处理：内外→内、内→外、外→内、外→外
- Bresenham直线绘制 + Scanline扫描线填充
- 量化验证：裁剪面积 ≤ 原始面积 ≤ 裁剪窗口面积
- 4个测试用例：星形、六边形、三角形、凹多边形
