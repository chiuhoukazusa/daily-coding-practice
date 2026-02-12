# Simple Ray Tracer - Sphere Rendering

## 项目信息
- **开发时间**：约 6 分钟
- **代码量**：139 行 C++
- **迭代次数**：1 次（首次即成功）
- **编译状态**：✅ 编译成功（0 错误 0 警告）
- **运行结果**：✅ 成功生成 640×480 光线追踪渲染图像

## 迭代历史
1. **初始版本** - 一次性成功，无需迭代
   - 代码编写完成
   - 首次编译即成功
   - 首次运行即生成正确图像

## 技术要点

### 核心算法

本项目实现了一个简单的光线追踪渲染器，包含以下关键组件：

1. **Vector3 类**：用于三维向量运算（加法、减法、点积、归一化）
2. **Sphere 类**：表示三维空间中的球体，包含球心、半径和颜色
3. **光线-球体相交检测**：使用二次方程求解判断光线是否与球体相交
4. **简单的着色模型**：基于表面法向量的点积进行明暗计算

### 关键代码

```cpp
// 球体-光线相交测试
bool Sphere::intersect(const Vector3& origin, const Vector3& direction, double& t) const {
    Vector3 oc = origin - center;
    double a = direction.dot(direction);
    double b = 2.0 * oc.dot(direction);
    double c = oc.dot(oc) - radius * radius;
    double discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return false;  // 无实数解，不相交
    }
    
    discriminant = std::sqrt(discriminant);
    double t0 = (-b - discriminant) / (2.0 * a);
    double t1 = (-b + discriminant) / (2.0 * a);
    
    if (t0 > 0.1) {  // 小正数避免自相交
        t = t0;
        return true;
    } else if (t1 > 0.1) {
        t = t1;
        return true;
    }
    
    return false;
}
```

## 效果展示

![Simple Ray Tracer 输出](output.png)

场景包含：
- 红色大球 (半径 0.5，中心 (0,0,-1))
- 绿色小球 (半径 0.3，中心 (1,0,-1)) 
- 蓝色中球 (半径 0.4，中心 (-1,0,-1))
- 天蓝色渐变背景
- 简单的明暗处理

## 编译运行

```bash
# 编译
g++ -std=c++17 -Wall -Wextra -O2 raytracer.cpp -o raytracer -lm

# 运行
./raytracer

# 生成的图像
# - output.ppm: 原始 PPM 格式图像
# - output.png: 转换后的 PNG 格式图像
```

## 学习收获

1. **光线追踪基础**：实现了基本的光线追踪渲染管线
2. **数学与几何**：实践了向量运算和二次方程求解
3. **计算机图形学**：理解了图形渲染的基本概念和流程
4. **C++ 11/17 特性**：使用了现代 C++ 的 auto 和 constexpr
5. **文件 I/O**：实现了 PPM 图像格式的写入

## 项目结构

```
12-raytracer-sphere/
├── raytracer.cpp     # 光线追踪主程序
├── raytracer         # 编译后的可执行文件
├── output.ppm        # 原始 PPM 格式输出
├── output.png        # 转换后的 PNG 格式
├── ppm_to_png.py     # 图像转换工具
└── README.md         # 项目说明
```

## 时间线
- 10:00: 开始项目规划
- 10:01: 创建项目目录
- 10:03: 代码编写完成
- 10:04: 编译成功
- 10:05: 运行完成，生成了 640x480 渲染图像
- 10:06: 项目文档整理