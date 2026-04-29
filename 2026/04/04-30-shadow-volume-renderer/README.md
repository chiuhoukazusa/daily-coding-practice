# Shadow Volume Renderer

每日编程实践 2026-04-30

## 技术要点

- **Shadow Volume（阴影体积）**：将光源投影通过物体形成封闭几何体
- **Z-Fail（Carmack's Reverse）**：深度测试失败时更新模板缓冲，解决摄像机在阴影体内的错误问题
- **Stencil Buffer（模板缓冲）**：软件模拟模板计数，stencil != 0 的像素在阴影中
- **Silhouette Edge Detection（轮廓边检测）**：检测一个面朝光、一个面背光的共享边
- **封闭阴影体**：侧面四边形 + 前盖（朝光面）+ 后盖（背光面投影到远处）
- **软光栅化渲染管线**：无第三方依赖，纯 C++17

## 场景描述

Cornell Box 变体：
- 地面（白）、后墙（浅蓝）、左墙（红）、右墙（绿）
- 中心球体（金色）、两个随机摆放的立方体（蓝/紫）
- 点光源产生清晰硬阴影

## 输出结果

![Shadow Volume 渲染结果](shadow_volume_output.png)

- 渲染分辨率：800×600
- 阴影体三角形数：2310
- 阴影像素占比：9.1%
- 图像均值：134.5，标准差：65.1

## 编译运行

```bash
g++ main.cpp -o shadow_volume -std=c++17 -O2 -Wall -Wextra
./shadow_volume
# 输出: shadow_volume_output.png
```

## Shadow Volume 原理

```
1. 检测轮廓边：对每条边，若相邻两个三角面一面朝光一面背光 → 轮廓边
2. 从轮廓边挤出侧面四边形（延伸至光源反方向很远处）
3. 添加前盖（朝光面原始三角形）
4. 添加后盖（背光面投影到无穷远处的三角形，绕序取反）
5. Z-Fail规则：
   - 渲染正面（front face）时，若深度测试失败 → stencil--
   - 渲染背面（back face）时，若深度测试失败 → stencil++
6. 最终 stencil[px][py] != 0 的像素在阴影中
```

## 关键代码

```cpp
// Z-Fail 模板更新
if(depth >= fb.depth[py][px]) {
    if(frontFace) fb.stencil[py][px]--;
    else          fb.stencil[py][px]++;
}

// 着色时查询阴影
bool shadow = savedStencil[py][px] != 0;
Vec3 finalColor = shade(wPos, wNorm, matColor, lightPos, eyePos, shadow);
```
