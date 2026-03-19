---
name: daily-coding-practice
description: 每日编程实践的【开发阶段】Skill。负责选题、编码、编译、调试，直到代码跑通输出正确为止。完成后交棒给 daily-coding-verification 做发布前验证。
---

# Daily Coding Practice — 开发阶段

## 定位

**这个 Skill 只负责一件事：让代码跑通，输出正确。**

流程编排关系：

```
[daily-coding-practice]  ← 你在这里
         ↓ 代码跑通、输出验证通过
[daily-coding-verification]
         ↓ 四层验证全部通过
[blog-code-workflow]
         ↓ 博客发布上线
         ✅ 完成
```

**不要在这个 Skill 里做任何发布操作。**

---

## Step 0：防重复检查（必须先做）

```bash
cd /root/.openclaw/workspace/daily-coding-practice

# 检查今日是否已有项目
TODAY=$(date +%m-%d)
if grep -q "| $TODAY |" PROJECT_INDEX.md 2>/dev/null; then
    echo "❌ 今天已有项目，禁止重复！"
    cat PROJECT_INDEX.md | grep "$TODAY"
    exit 1
fi

# 检查近 30 天是否有相似主题
echo "近期项目列表（检查主题是否重复）："
tail -35 PROJECT_INDEX.md
```

**如果发现重复 → 停止，从 PROJECT_INDEX.md 的"待探索领域"中重新选题。**

---

## Step 1：选题

从以下方向选择（优先未做过的技术点）：

**图形学**：光线追踪、软光栅、着色模型（PBR/NPR）、后处理效果、体积渲染、粒子系统  
**游戏开发**：物理模拟、AI 寻路、程序化生成、碰撞检测  
**算法可视化**：排序/图算法动画、数据结构演示、数值方法  

选好后在 PROJECT_INDEX.md 中**立即预占今日条目**（防止后续流程中忘记）：

```bash
echo "| $TODAY | <项目名> | <技术标签> | in-progress |" >> PROJECT_INDEX.md
git add PROJECT_INDEX.md && git commit -m "Reserve: $TODAY - <项目名>"
```

---

## Step 2：开发迭代

```
循环（最多 30 分钟）：
  编写代码
    ↓
  编译 (g++ -std=c++17 -O2 -Wall -Wextra)
    ↓ 失败 → 读编译日志 → 定位错误行 → 修复 → 回到编译
  运行
    ↓ 崩溃 → 分析日志/gdb → 找根因 → 修复 → 回到编译
  输出验证（见 Step 3）
    ↓ 不符合预期 → 调整逻辑 → 回到编译
  全部通过 → 进入 Step 4
```

**编译命令**：
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
# 目标：0 errors, 0 warnings
```

---

## Step 3：输出验证（必须量化，不能靠眼睛）

### 图形学项目

```bash
# 检查文件存在且非空
ls -lh *.png && [ $(stat -c%s *.png | head -1) -gt 10240 ] || { echo "❌ 图片太小或不存在"; exit 1; }

# 像素采样检查（需要 ImageMagick）
python3 << 'EOF'
from PIL import Image
import numpy as np, sys

img = Image.open(sorted(__import__('glob').glob('*.png'))[0])
pixels = np.array(img).astype(float)

mean = pixels.mean()
std  = pixels.std()
print(f"像素均值: {mean:.1f}  标准差: {std:.1f}")

if mean < 5:
    print("❌ 图像过暗，可能全黑"); sys.exit(1)
if mean > 250:
    print("❌ 图像过亮，可能全白"); sys.exit(1)
if std < 5:
    print("❌ 图像几乎无变化，可能渲染错误"); sys.exit(1)

print("✅ 像素统计正常")
EOF
```

**必须验证的项目（逐条检查）**：
- [ ] 文件大小 > 10KB
- [ ] 像素均值在 10~240 之间（非全黑/全白）
- [ ] 像素标准差 > 5（图像有内容变化）
- [ ] 坐标系正确：天空在上、地面在下（图形学项目）

> **历史教训**：2026-02-18 反射 Bug——代码跑通、文件存在，但中心球纯黑。  
> 2026-02-20 坐标系 Bug——上下颠倒，只靠眼看没发现。  
> **量化检查不可省略。**

### 数值/算法项目

```bash
# 检查输出范围
./output | python3 -c "
import sys, statistics
vals = [float(x) for x in sys.stdin.read().split() if x.strip()]
print(f'均值={statistics.mean(vals):.3f}  范围=[{min(vals):.3f}, {max(vals):.3f}]')
assert min(vals) >= EXPECTED_MIN and max(vals) <= EXPECTED_MAX, '❌ 值超出预期范围'
print('✅ 数值范围正常')
"
```

---

## Step 4：更新 PROJECT_INDEX.md

```bash
cd /root/.openclaw/workspace/daily-coding-practice
# 将 in-progress 改为 dev-done
sed -i "s/| $TODAY | \(.*\) | in-progress |/| $TODAY | \1 | dev-done |/" PROJECT_INDEX.md
git add PROJECT_INDEX.md && git commit -m "Dev done: $TODAY - <项目名>"
git push origin main
```

---

## Step 5：交棒

开发阶段完成。**现在读取并执行 `daily-coding-verification` Skill。**

---

## 失败处理

超时（30分钟）仍未跑通：

```bash
# 记录状态，下次继续
cat > /root/.openclaw/workspace/daily-coding-practice/status.json << EOF
{
  "date": "$(date +%Y-%m-%d)",
  "project": "<项目名>",
  "stage": "dev",
  "lastError": "<错误描述>",
  "time": "$(date -Iseconds)"
}
EOF
```

通知用户后停止，不要强行进入验证/发布阶段。
