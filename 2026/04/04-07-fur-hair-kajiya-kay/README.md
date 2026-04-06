# Fur Hair Rendering - Kajiya-Kay Model

## 项目简介

使用 Kajiya-Kay (1989) 毛发着色模型，通过软光栅化渲染一个毛发覆盖的头部。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
# 生成 fur_hair_output.png
```

## 技术要点

- **Kajiya-Kay 着色模型**：Diffuse = sin(∠Light,Tangent)，Specular = cos^p(∠Eye,Tangent)，模拟各向异性毛发光照
- **程序化毛发几何**：Fibonacci 球面分布 + 随机抖动生成 6000 根毛发，每根 8 段折线模拟重力垂落和卷曲
- **软光栅化 + Z-Buffer**：自实现渲染管线，毛发线段投影到屏幕空间后按深度绘制
- **头部球体光线追踪**：背景球体使用逐像素光线-球相交，Phong 着色
- **伽马校正**：输出前应用 γ=2.2 伽马矫正

## 输出结果

![结果](fur_hair_output.png)
