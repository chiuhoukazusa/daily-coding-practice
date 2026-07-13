# Hexo 博客部署完整指南

最后更新：2026-02-10

## 问题诊断

### 当前状态
- **部署仓库**: `chiuhoukazusa.github.io` (只有生成后的 HTML)
- **源码仓库**: 不存在或未找到
- **问题**: 直接添加 HTML 文件不会更新首页和归档

### 发现的事实
从 Hexo 官方文档（https://hexo.io/docs/github-pages）了解到：
- Hexo 需要**源码仓库**（包含 `source/_posts/*.md`）
- 通过 `hexo generate` 生成完整网站（包括首页、归档）
- 直接修改部署仓库的 HTML 不会更新导航

## 解决方案

### 方案 A: 创建 Hexo 源码仓库 + GitHub Actions 自动部署（推荐）

#### 第 1 步：创建源码仓库

```bash
# 1. 初始化 Hexo 项目
cd /root/.openclaw/workspace
mkdir hexo-blog-source
cd hexo-blog-source
hexo init .
npm install

# 2. 配置 _config.yml
# 设置网站 URL 等基本信息
```

#### 第 2 步：配置 GitHub Actions 自动部署

创建 `.github/workflows/pages.yml`：

```yaml
name: Deploy Hexo to GitHub Pages

on:
  push:
    branches:
      - main

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    
    steps:
      - name: Checkout
        uses: actions/checkout@v4
        with:
          submodules: recursive
      
      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'
      
      - name: Cache NPM dependencies
        uses: actions/cache@v4
        with:
          path: node_modules
          key: ${{ runner.OS }}-npm-cache
          restore-keys: |
            ${{ runner.OS }}-npm-cache
      
      - name: Install Dependencies
        run: npm install
      
      - name: Build Hexo
        run: |
          npx hexo clean
          npx hexo generate
      
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./public
          publish_branch: main
          destination_dir: .
          cname: chiuhoukazusa.github.io  # 如果有自定义域名
```

#### 第 3 步：GitHub 仓库设置

1. 在 GitHub 创建新仓库：`hexo-blog-source` (或直接用 chiuhoukazusa.github.io 存源码)
2. Settings → Pages → Source → 选择 "GitHub Actions"
3. Push 源码后，GitHub Actions 会自动构建和部署

### 方案 B: 本地生成 + 手动部署（简单但不自动）

#### 工作流程

```bash
# 1. 创建文章（Markdown）
cd hexo-blog-source
hexo new post "文章标题"
# 编辑 source/_posts/文章标题.md

# 2. 本地生成
hexo clean
hexo generate

# 3. 部署到 GitHub Pages
hexo deploy
# 或者手动复制 public/ 内容到部署仓库
```

#### 配置 _config.yml

```yaml
# Deployment
deploy:
  type: git
  repo: https://github.com/chiuhoukazusa/chiuhoukazusa.github.io
  branch: main
```

## Daily Coding Practice 集成

### 更新后的博客发布流程

#### 如果使用方案 A（GitHub Actions）

```bash
# 1. Clone 源码仓库
cd /root/.openclaw/workspace
git clone https://github.com/chiuhoukazusa/hexo-blog-source.git

# 2. 创建新文章（Markdown）
cd hexo-blog-source
cat > source/_posts/2026-02-10-perlin-noise.md <<'EOF'
---
title: Perlin Noise 程序化纹理生成器
date: 2026-02-10 10:00:00
tags: [图形学, 程序化生成, C++]
categories: [每日编程实践]
---

## 项目描述
[内容...]

## 代码
[代码片段...]

## 效果图
![云层纹理](clouds.png)
EOF

# 3. 添加图片到 source/images/
cp output_clouds.png source/images/2026-02-10-clouds.png

# 4. Git 提交（GitHub Actions 会自动构建和部署）
git add .
git commit -m "Daily Practice: 2026-02-10 Perlin Noise"
git push origin main

# 5. 等待 GitHub Actions 完成（约 2-3 分钟）
# 首页、归档都会自动更新 ✅
```

#### 如果使用方案 B（本地生成）

```bash
# 1. Clone 源码仓库（如果存在）或初始化
cd /root/.openclaw/workspace
hexo init hexo-blog-source  # 如果没有源码
cd hexo-blog-source

# 2. 创建新文章
hexo new post "Perlin Noise 程序化纹理生成器"
# 编辑 source/_posts/Perlin-Noise-程序化纹理生成器.md

# 3. 安装依赖（如果需要）
npm install
npm install hexo-deployer-git --save

# 4. 配置 _config.yml
deploy:
  type: git
  repo: https://github.com/chiuhoukazusa/chiuhoukazusa.github.io
  branch: main

# 5. 生成并部署
hexo clean
hexo generate
hexo deploy

# 这会自动更新 chiuhoukazusa.github.io 仓库
# 首页、归档都会自动更新 ✅
```

## Skill 更新建议

需要在 `daily-coding-practice/SKILL.md` 中添加 Hexo 支持：

```markdown
### 4. 博客发布阶段（Hexo）

**检测博客类型**：
- 检查是否存在 hexo-blog-source 仓库
- 或者检查 _config.yml 判断是 Hexo/Jekyll/Hugo

**Hexo 发布流程**：
1. Clone 源码仓库（如果不存在）
2. 创建 Markdown 文章到 source/_posts/
3. 添加图片到 source/images/
4. 配置 Front Matter（标题、日期、标签）
5. Git commit & push
6. 如果使用 GitHub Actions，等待构建完成（2-3分钟）
7. 验证文章在首页可见

**关键文件**：
- `source/_posts/YYYY-MM-DD-title.md` - 文章源文件
- `.github/workflows/pages.yml` - 自动构建配置
- `_config.yml` - Hexo 配置文件
```

## 下一步行动

请告诉我：
1. **你是否有 Hexo 源码仓库？**（另一个 GitHub 仓库或本地）
2. **你之前是怎么发布博客的？**（手动还是自动）
3. **想用哪种方案？**
   - 方案 A: GitHub Actions 自动构建（推荐，完全自动化）
   - 方案 B: 本地生成手动部署（简单，但 OpenClaw 需要本地 Hexo 环境）

我可以帮你：
- 创建 Hexo 源码仓库
- 配置 GitHub Actions
- 更新 daily-coding-practice skill 的博客发布逻辑
- 迁移今天的文章到正确的流程

---

**临时解决方案**：今天的文章虽然首页看不到，但可以直接访问：
https://chiuhoukazusa.github.io/2026/02/10/perlin-noise-texture-generator/
