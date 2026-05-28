# Stochastic Progressive Photon Mapping (SPPM) Renderer

Cornell Box 场景中实现随机渐进光子映射（SPPM），展示玻璃球产生的焦散效果和全局光照。

## 核心特性

- **SPPM 算法**：渐进半径收缩，逐步收敛到精确解
- **直接光 NEE**：Next Event Estimation 直接采样光源，高效无噪
- **玻璃球焦散**：IOR=1.5 玻璃球将平行光聚焦到地面，产生明亮焦散斑
- **镜面球反射**：完美镜面反射
- **空间哈希加速**：O(1) 光子查询，取代暴力 O(N×M) 搜索
- **Reinhard Tone Mapping + Gamma 校正**

## 渲染参数

- 分辨率：400×400
- SPPM 迭代轮数：20 passes
- 每轮光子数：60,000
- 渐进因子 α：0.7
- 初始搜索半径：20（Cornell Box 0~555 坐标系）

## 编译运行

```bash
g++ main.cpp -o sppm_renderer -std=c++17 -O2 -Wall -Wextra
./sppm_renderer
```

运行时间约 4 秒，输出：
- `sppm_final.png`：直接光 + 光子映射合成图
- `sppm_caustic.png`：光子映射通道（间接光/焦散）
- `sppm_direct.png`：直接光通道（NEE）

## 输出结果

![最终渲染](sppm_final.png)
![焦散通道](sppm_caustic.png)
![直接光](sppm_direct.png)

## 技术要点

- SPPM 辐射率公式：`L = flux / (PI * r² * passes)`
- 玻璃球参数推导：`f = r/(2*(n-1)) = 80`，`do=457, di=97` → 焦散点精确落在地面
- `cosineHemisphere` 对 `(0,-1,0)` 法线的正交基构建修复
- 空间哈希 grid cell size = 2×半径，3D 邻域查询
