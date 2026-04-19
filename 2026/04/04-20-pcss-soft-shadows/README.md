# PCSS - Percentage Closer Soft Shadows

## 编译运行
```bash
g++ main.cpp -o pcss_renderer -std=c++17 -O2 -Wall -Wextra -lm
./pcss_renderer
# 生成 pcss_output.ppm，然后转换为 PNG:
python3 -c "from PIL import Image; Image.open('pcss_output.ppm').save('pcss_output.png')"
```

## 输出结果
![结果](pcss_output.png)

左半：硬阴影（PCF size=1）
右半：PCSS软阴影（动态PCF，半影宽度随遮挡距离自适应）

## 技术要点
- Shadow Map 生成（正交投影，从光源视角渲染深度图）
- Blocker Search（在接收点附近搜索平均遮挡深度）
- Penumbra Estimation（根据遮挡深度差估计半影宽度）
- PCF（Percentage Closer Filtering）动态核大小滤波
- 软光栅化场景：地面棋盘格 + 3个球体 + 天空渐变
