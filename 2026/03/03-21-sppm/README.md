# SPPM - Stochastic Progressive Photon Mapping

随机渐进光子映射 - 全局光照渲染器

## 编译运行

```bash
g++ main.cpp -o sppm -std=c++17 -O2 -Wall -Wextra
./sppm
```

## 输出结果

![结果](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/03/03-21-sppm/sppm_output.png)

## 技术要点

- **SPPM 算法**：Hachisuka 2009 随机渐进光子映射，多迭代自适应半径收缩
- **Cornell Box 场景**：地板/天花板/背墙/左红右绿墙 + 天花板光源
- **材质系统**：漫反射（Lambertian）、镜面反射（Mirror）、玻璃折射（Glass + Fresnel）
- **kd-tree 加速**：光子近邻搜索，O(log N) 查询
- **正确单位体系**：相机 pass 记录权重（不含 albedo），光子 pass 存储路径吞吐量，密度估计时合并 BSDF/π
- **色调映射**：ACES Filmic tone mapping + gamma 2.2 校正

## 算法参数

| 参数 | 值 |
|------|-----|
| 分辨率 | 512×512 |
| 迭代次数 | 40 |
| 每次迭代光子数 | 200,000 |
| 总光子数 | 8,000,000 |
| 初始搜索半径 | 0.08 |
| 收缩因子 α | 0.7 |

## 文件说明

- `main.cpp` - 完整 SPPM 实现（单文件，约 650 行）
- `stb_image_write.h` - PNG 写入库（header-only）
