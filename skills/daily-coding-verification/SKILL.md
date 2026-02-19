# Daily Coding Practice - 验证体系 Skill

## 适用场景
每次完成每日编程实践后，发布前的系统性验证。

## 验证清单

### ✅ 第一层：编译验证（自动化）
```bash
# 编译检查
g++ *.cpp -o output -std=c++17 -O2 -Wall -Wextra
# 期望：0 错误，0 警告

# 运行检查
./output
# 期望：正常退出，code 0

# 输出检查
ls -lh *.png *.jpg 2>/dev/null
# 期望：文件存在，大小合理（>10KB）
```

**验证项**：
- [ ] 编译通过（0错误 0警告）
- [ ] 运行成功（无崩溃，exit code 0）
- [ ] 输出文件生成（图片/数据）
- [ ] 文件大小合理（不是空文件或异常大）

### ✅ 第二层：逻辑验证（自动化 + 半自动）

#### 2.1 算法正确性
**对于图形学项目**：
```bash
# 像素采样检查
python3 << 'EOF'
from PIL import Image
import sys

img = Image.open('output.png')
pixels = img.load()
width, height = img.size

# 检查：中心点颜色
center_color = pixels[width//2, height//2]
print(f"Center pixel: RGB{center_color}")

# 检查：颜色统计
colors = set()
for y in range(0, height, 50):
    for x in range(0, width, 50):
        colors.add(pixels[x, y])

print(f"Unique colors (sampled): {len(colors)}")
print(f"Min/Max RGB: {min(colors)}, {max(colors)}")

# 基本验证
if len(colors) < 10:
    print("⚠️  Warning: Too few colors, possible rendering issue")
    sys.exit(1)

if max(colors) == (0, 0, 0) or min(colors) == (255, 255, 255):
    print("⚠️  Warning: Suspicious color range")
    sys.exit(1)

print("✅ Color statistics look reasonable")
EOF
```

**对于算法项目**：
```bash
# 边界条件测试
./output test_boundary_case_1
./output test_boundary_case_2

# 性能测试
time ./output large_input.dat
```

**验证项**：
- [ ] 输出符合预期（颜色/数值/格式）
- [ ] 边界条件处理正确
- [ ] 性能可接受

#### 2.2 物理合理性

**图形学项目**：
- [ ] 阴影方向与光源一致
- [ ] 反射角度符合物理规律
- [ ] 颜色没有溢出（0-255范围）
- [ ] 透视/坐标系正确（地板在下方，天空在上方）

**算法项目**：
- [ ] 输出在合理范围内
- [ ] 边界情况不会崩溃
- [ ] 符合算法的时间/空间复杂度

#### 2.3 与之前版本对比

```bash
# 如果是迭代/优化之前的项目
diff <(./old_version --dump) <(./new_version --dump)

# 或图形学：对比渲染结果
compare old_output.png new_output.png diff.png  # ImageMagick
```

**验证项**：
- [ ] 新功能按预期工作
- [ ] 旧功能没有退化
- [ ] 性能提升符合预期

### ✅ 第三层：内容一致性（自动化）

```bash
# 检查图片是否上传
IMAGE_URL="https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/02/$(basename $(pwd))/$(ls *.png | head -1)"
curl -s -o /dev/null -w "%{http_code}" "$IMAGE_URL"
# 期望：200

# 检查 MD5 一致性
LOCAL_MD5=$(md5sum *.png | awk '{print $1}')
REMOTE_MD5=$(curl -s "$IMAGE_URL" | md5sum | awk '{print $1}')
if [ "$LOCAL_MD5" = "$REMOTE_MD5" ]; then
    echo "✅ Image MD5 matches"
else
    echo "❌ Image MD5 mismatch!"
    exit 1
fi
```

**验证项**：
- [ ] 代码已提交到 GitHub
- [ ] 图片已上传到 blog_img
- [ ] URL 可访问（HTTP 200）
- [ ] 本地与远程文件一致（MD5）

### ✅ 第四层：博客发布（自动化）

```bash
# 检查博客文章生成
cd chiuhou-blog-source
hexo clean && hexo g

# 检查文章存在
ARTICLE_PATH="public/2026/02/$(date +%d)/$(basename $(pwd))/index.html"
if [ -f "$ARTICLE_PATH" ]; then
    echo "✅ Article generated"
else
    echo "❌ Article not found!"
    exit 1
fi

# 部署
hexo deploy

# 验证线上可访问
BLOG_URL="https://chiuhoukazusa.github.io/chiuhou-tech-blog/2026/02/$(date +%d)/$(basename $(pwd))/"
sleep 10  # 等待 GitHub Pages 部署
curl -s -o /dev/null -w "%{http_code}" "$BLOG_URL"
# 期望：200
```

**验证项**：
- [ ] Hexo 生成成功
- [ ] 部署到 GitHub Pages
- [ ] 线上可访问（HTTP 200）
- [ ] 封面图显示正常

---

## 🔴 人工验证（用户最后检查）

**由 @jiyunzhou 在博客上最终验证**：
- [ ] 视觉效果符合预期
- [ ] 技术描述准确
- [ ] 没有明显 bug（上下颠倒、颜色错误等）

**这一步不阻碍发布流程**，可以在发布后进行。如果发现问题，再迭代修复。

---

## 使用方式

### 方式 1：逐项检查（手动）
按照清单逐项执行，遇到失败立刻停止并修复。

### 方式 2：自动化脚本（推荐）
```bash
#!/bin/bash
# verify.sh - 自动验证脚本

set -e  # 遇到错误立即退出

echo "=== 第一层：编译验证 ==="
g++ *.cpp -o output -std=c++17 -O2 -Wall -Wextra || exit 1
./output || exit 1
ls -lh *.png *.jpg || exit 1

echo "=== 第二层：逻辑验证 ==="
python3 verify_pixels.py || exit 1

echo "=== 第三层：内容一致性 ==="
bash verify_upload.sh || exit 1

echo "=== 第四层：博客发布 ==="
cd ../../../chiuhou-blog-source
hexo clean && hexo g && hexo deploy || exit 1

echo "✅ 所有自动化验证通过！"
echo "👤 等待 @jiyunzhou 最终人工验证"
```

### 方式 3：集成到 daily-coding-practice 流程
在每日编程实践结束后，自动调用验证脚本。

---

## 关键原则

### 1. 分层验证，逐层推进
```
编译层 → 逻辑层 → 内容层 → 发布层 → 人工验证
  ↓        ↓        ↓        ↓        ↓
 失败     失败     失败     失败     反馈修复
  ↓        ↓        ↓        ↓
 修复     修复     修复     修复
  ↓        ↓        ↓        ↓
重新验证 重新验证 重新验证 重新验证
```

**任何一层失败 → 立刻停止 → 修复 → 重新从该层开始验证**

### 2. 自动化优先，人工兜底

**能自动化的都自动化**：
- 编译检查
- 像素采样
- HTTP 状态码
- MD5 校验

**人工只做最后的视觉/语义验证**：
- 整体视觉效果
- 技术描述的准确性
- 细微的物理不合理

### 3. 验证结果可追溯

每次验证生成日志：
```bash
# 验证日志示例
=== 2026-02-20 05:50:00 ===
Project: 02-20-texture-mapping
✅ 编译通过 (0 warnings)
✅ 运行成功 (exit 0, 6 seconds)
✅ 输出生成 (texture_output.png, 70KB)
✅ 像素检查通过 (253 colors, min=10, max=254)
✅ 图片已上传 (HTTP 200, MD5 match)
✅ 博客已部署 (commit 8907a4f)
🔄 等待人工验证 (@jiyunzhou)
```

### 4. 失败后的处理流程

```
发现问题
  ↓
确认根本原因（不要瞎改！）
  ↓
修复代码
  ↓
重新走完整验证流程（不要跳过！）
  ↓
提交修复 commit（说明根本原因）
  ↓
更新博客（添加修复说明）
```

**关键**：
- 不要急于修复，先理解根本原因
- 不要只改一处，确保相关代码一致性
- 不要跳过验证，重新走完整流程
- 修复后必须提交说明，方便后续回溯

---

## 常见问题

### Q1: 验证太慢，能跳过吗？
**A**: 不能。验证清单是从六次错误中总结出来的，每一项都对应着真实发生过的 bug。跳过任何一项都可能导致新的问题。

### Q2: 人工验证什么时候做？
**A**: 发布后再做，不阻碍流程。如果发现问题，走迭代修复流程。

### Q3: 自动化验证失败了怎么办？
**A**: 立刻停止，不要继续发布。修复问题后，重新从失败的那一层开始验证。

### Q4: 验证通过但用户还是发现问题怎么办？
**A**: 说明验证清单不完善，添加新的检查项。验证清单是动态演进的，不是一成不变的。

---

## 演进记录

**v1.0 (2026-02-20)**：
- 初始版本
- 基于六次错误总结的完整验证流程
- 分离自动化验证与人工验证
- 建立分层验证体系

**未来改进方向**：
- [ ] 集成到 GitHub Actions（CI/CD）
- [ ] 添加视觉回归测试（Visual Regression Testing）
- [ ] 自动生成验证报告（Markdown 格式）
- [ ] 集成到每日编程实践的自动化流程
