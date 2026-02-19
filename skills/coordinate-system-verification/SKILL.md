# Coordinate System Verification Skill

## 适用场景
任何涉及坐标转换的图形学项目（光线追踪、光栅化、相机变换等），在生成图片后立刻验证坐标系正确性。

## 核心原则

**坐标系错误是隐蔽的**：
- 代码编译通过 ✅
- 程序运行成功 ✅
- 图片看起来"还行" ✅
- 但实际上下颠倒或左右翻转 ❌

**必须自动化验证**，不能靠肉眼！

---

## 验证清单

### ✅ 第一步：采样关键位置

```bash
# 检查图片的上下左右四个边缘和中心
convert output.png -crop 1x1+<x>+<y> txt:- | tail -1

# 位置：
# - 顶部中心：(width/2, 10)
# - 底部中心：(width/2, height-10)
# - 左侧中心：(10, height/2)
# - 右侧中心：(width-10, height/2)
# - 中心：(width/2, height/2)
```

### ✅ 第二步：判断坐标系

#### 对于光线追踪场景（天空+地面）

**规则**：
- **天空**（背景）：通常在**顶部**（图像 y=0 附近）
- **地面**（或球体阴影）：通常在**底部**（图像 y=height 附近）

**验证脚本**：
```bash
#!/bin/bash
# verify_coordinate_system.sh

IMAGE="$1"
WIDTH=$(identify -format '%w' "$IMAGE")
HEIGHT=$(identify -format '%h' "$IMAGE")

# 采样顶部中心
TOP_COLOR=$(convert "$IMAGE" -crop 1x1+$((WIDTH/2))+10 txt:- | tail -1 | grep -oP '\(\K[^)]+')
TOP_R=$(echo $TOP_COLOR | cut -d, -f1)
TOP_G=$(echo $TOP_COLOR | cut -d, -f2)
TOP_B=$(echo $TOP_COLOR | cut -d, -f3)

# 采样底部中心
BOTTOM_COLOR=$(convert "$IMAGE" -crop 1x1+$((WIDTH/2))+$((HEIGHT-10)) txt:- | tail -1 | grep -oP '\(\K[^)]+')
BOTTOM_R=$(echo $BOTTOM_COLOR | cut -d, -f1)
BOTTOM_G=$(echo $BOTTOM_COLOR | cut -d, -f2)
BOTTOM_B=$(echo $BOTTOM_COLOR | cut -d, -f3)

echo "顶部 RGB: ($TOP_R, $TOP_G, $TOP_B)"
echo "底部 RGB: ($BOTTOM_R, $BOTTOM_G, $BOTTOM_B)"

# 判断：天空通常是蓝色（B > R 且 B > G）或浅色（RGB 都较大）
# 地面/球体通常是深色或有颜色的
TOP_BRIGHTNESS=$((TOP_R + TOP_G + TOP_B))
BOTTOM_BRIGHTNESS=$((BOTTOM_R + BOTTOM_G + BOTTOM_B))

if [ $TOP_B -gt $TOP_R ] && [ $TOP_B -gt $((TOP_G + 50)) ]; then
    echo "✅ 顶部是蓝色（可能是天空）"
    TOP_IS_SKY=1
elif [ $TOP_BRIGHTNESS -gt 400 ]; then
    echo "✅ 顶部是浅色（可能是天空/背景）"
    TOP_IS_SKY=1
else
    echo "⚠️  顶部不像天空（深色或有颜色）"
    TOP_IS_SKY=0
fi

if [ $BOTTOM_BRIGHTNESS -lt 300 ]; then
    echo "✅ 底部是深色（可能是地面/阴影）"
    BOTTOM_IS_GROUND=1
elif [ $BOTTOM_B -gt $BOTTOM_R ] && [ $BOTTOM_B -gt $((BOTTOM_G + 50)) ]; then
    echo "⚠️  底部是蓝色（可能是天空）- 疑似上下颠倒！"
    BOTTOM_IS_GROUND=0
else
    echo "✅ 底部有颜色（可能是地面/球体）"
    BOTTOM_IS_GROUND=1
fi

# 综合判断
if [ $TOP_IS_SKY -eq 1 ] && [ $BOTTOM_IS_GROUND -eq 1 ]; then
    echo ""
    echo "✅ 坐标系正确：天空在上，地面在下"
    exit 0
elif [ $TOP_IS_SKY -eq 0 ] && [ $BOTTOM_IS_GROUND -eq 0 ]; then
    echo ""
    echo "❌ 坐标系翻转：天空在下，地面在上！"
    exit 1
else
    echo ""
    echo "⚠️  无法确定，建议人工检查"
    exit 2
fi
```

#### 对于其他场景（无明显天空/地面）

**替代方法**：渲染一个已知朝向的参考物体

```cpp
// 在场景中添加一个朝上的箭头或文字 "TOP"
// 渲染后检查该标记是否在图像顶部
```

---

## 常见坐标系错误

### 错误1：相机 up 向量指向错误

**症状**：图像上下颠倒

**原因**：
```cpp
Vec3 up = cross(right, forward);  // 可能向下！
```

**检测**：
```cpp
// 在渲染前检查
if (up.y < 0) {
    std::cerr << "❌ Warning: up vector points downward!" << std::endl;
    up = up * -1.0;  // 自动修复
}
```

**标准做法**：
```cpp
Vec3 up = cameraUp;  // 显式指定，而不是计算
```

### 错误2：NDC 到屏幕空间的映射错误

**症状**：图像上下颠倒或左右翻转

**常见错误**：
```cpp
// ❌ 错误：y 方向反了
double py = (2.0 * y / height - 1.0) * tan(fov/2);

// ✅ 正确：
double py = (1.0 - 2.0 * y / height) * tan(fov/2);
```

**验证**：
```cpp
// y=0 → py 应该是正数（向上）
// y=height → py 应该是负数（向下）
std::cout << "y=0: py=" << py_at_0 << " (should be > 0)" << std::endl;
std::cout << "y=height: py=" << py_at_height << " (should be < 0)" << std::endl;
```

### 错误3：图像保存时的行顺序

**症状**：图像上下颠倒

**原因**：
- BMP/PNG 格式：行从下到上
- 代码写入：行从上到下
- 不匹配 → 翻转

**检测**：
```cpp
// 保存前验证：检查第一行数据是否对应 y=0
unsigned char* first_row = &image[0];
unsigned char* last_row = &image[(height-1) * width * 3];
// 渲染天空场景，first_row 应该是蓝色，last_row 应该是深色
```

---

## 自动化集成

### 在每日编程实践中集成

```bash
#!/bin/bash
# daily-coding-practice 流程

# 1. 编译
g++ *.cpp -o output -std=c++17 -O2

# 2. 运行
./output

# 3. 🔴 立刻验证坐标系（新增）
bash verify_coordinate_system.sh output.png || {
    echo "❌ 坐标系验证失败，停止发布！"
    exit 1
}

# 4. 上传 GitHub
# 5. 发布博客
```

### 在 CI/CD 中集成

```yaml
# .github/workflows/render-test.yml
- name: Verify coordinate system
  run: |
    bash scripts/verify_coordinate_system.sh output.png
    if [ $? -eq 1 ]; then
      echo "::error::Coordinate system is flipped!"
      exit 1
    fi
```

---

## 使用方式

### 方式1：手动验证（开发阶段）

```bash
# 渲染完成后立刻运行
bash verify_coordinate_system.sh texture_output.png
```

### 方式2：自动集成（生产环境）

在 `daily-coding-practice` 流程中添加验证步骤，任何坐标系错误立刻阻止发布。

### 方式3：视觉参考对比（最可靠）

```bash
# 渲染一个已知正确的参考图
./render_reference.sh  # 生成 reference.png

# 对比新图和参考图
compare output.png reference.png diff.png
```

---

## 教训总结

### 为什么肉眼检查不可靠？

1. **没有明显的上下参考**（昨天的折射球，今天的纹理映射）
2. **对称性**（球体上下翻转后看起来差不多）
3. **习惯性忽略**（生成图片后就觉得"应该是对的"）

### 为什么需要自动化？

1. **人会犯错**（我连续两天都没发现坐标系问题）
2. **验证必须在发布前**（不能依赖用户反馈）
3. **量化比定性可靠**（像素采样 > 肉眼观察）

### 核心原则

**不要相信"看起来对"，要验证"确实对"！**

```
渲染完成 → 自动采样关键位置 → 量化判断 → 通过才发布
            ↓
          失败 → 立刻停止 → 修复 → 重新验证
```

---

## 检查清单（快速版）

每次渲染图形学项目后：

- [ ] 采样顶部中心（应该是天空/背景）
- [ ] 采样底部中心（应该是地面/物体）
- [ ] 检查 up 向量方向（up.y > 0）
- [ ] 检查 py 映射公式（y=0 → py>0, y=height → py<0）
- [ ] 对比参考图（如果有）

**任何一项失败 → 停止发布 → 修复 → 重新验证**

---

## 参考资料

- [OpenGL Coordinate Systems](https://learnopengl.com/Getting-started/Coordinate-Systems)
- [Ray Tracing: Camera Coordinate Systems](https://raytracing.github.io/books/RayTracingInOneWeekend.html#rays,asimplecamera,andbackground)
- [Right-Handed vs Left-Handed Coordinate Systems](https://www.evl.uic.edu/ralph/508S98/coordinates.html)
