# Blog Deployment Checklist

## Purpose

Ensure blog posts are **actually deployed** and **publicly accessible**, not just written to disk.

## The Problem

Common failure mode:
1. Create Markdown file in `source/_posts/`
2. Commit to git
3. **Forget to run `hexo deploy`**
4. Report "blog published" when it's not live
5. Blog URL returns 404

## The Solution

**Never report blog as published until HTTP 200 verified.**

## Full Deployment Workflow

### Phase 1: Create Blog Post

```bash
cd /root/.openclaw/workspace/chiuhou-blog-source

# Create new post file
cat > source/_posts/YYYY-MM-DD-article-slug.md << 'EOF'
---
title: Article Title
date: YYYY-MM-DD HH:MM:SS
categories: [Category]
tags: [tag1, tag2]
cover: https://raw.githubusercontent.com/user/repo/main/image.png
---

# Content here
EOF
```

**Status**: Article EXISTS but NOT PUBLISHED

### Phase 2: Upload Cover Image

```bash
cd /root/.openclaw/workspace/blog_img

# Copy image
cp /path/to/image.png category/image.png

# Commit and push
git add .
git commit -m "Add cover image"
git push origin main
```

**Status**: Image HOSTED but article NOT PUBLISHED

### Phase 3: Generate Static Files

```bash
cd /root/.openclaw/workspace/chiuhou-blog-source

# Clean old files
hexo clean

# Generate new files
hexo generate
# or: hexo g
```

**Status**: Files GENERATED but NOT DEPLOYED

Check:
```bash
ls -la public/YYYY/MM/DD/article-slug/
# Should see index.html
```

### Phase 4: Deploy to GitHub Pages

```bash
cd /root/.openclaw/workspace/chiuhou-blog-source

# Deploy
hexo deploy
# or: hexo d
```

This command:
1. Pushes `public/` to the GitHub Pages branch
2. GitHub Actions builds the site
3. Takes 2-3 minutes to propagate

**Status**: DEPLOYING (not live yet)

### Phase 5: Wait for Propagation

```bash
# Wait 2-3 minutes for GitHub Pages cache
sleep 180
```

**Do NOT skip this step.** GitHub Pages needs time to update.

### Phase 6: Verify Deployment ⚠️ CRITICAL

```bash
BLOG_URL="https://chiuhoukazusa.github.io/chiuhou-tech-blog/YYYY/MM/DD/article-slug/"

# Run verification script
./scripts/verify_blog.sh "$BLOG_URL"
```

Expected output:
```
🔍 Verifying blog deployment...
URL: https://...
HTTP Status: 200
✅ Blog post is LIVE and accessible!
```

**ONLY NOW** can you report "blog published."

## URL Structure

### Hexo Permalink Format

Default: `:year/:month/:day/:title/`

Examples:
- Post: `2026-02-17-shadow-ray-tracing.md`
- URL: `https://chiuhoukazusa.github.io/chiuhou-tech-blog/2026/02/17/shadow-ray-tracing/`

**Note**: The URL uses the `title` field from frontmatter (slugified), NOT the filename.

### Common URL Mistakes

❌ Wrong: Using filename with date prefix
```
/2026/02/17/2026-02-17-shadow-ray-tracing/
```

✅ Correct: Using slugified title only
```
/2026/02/17/shadow-ray-tracing/
```

### Slugification Rules

Hexo converts titles to URLs by:
1. Converting to lowercase
2. Replacing spaces with hyphens
3. Removing special characters
4. Removing leading/trailing hyphens

Example:
```yaml
title: "Shadow Ray Tracing - 光线追踪中的阴影算法实现"
```
Becomes:
```
shadow-ray-tracing
```
(Chinese characters removed, special chars removed)

## Troubleshooting

### Issue 1: 404 Not Found

**Symptoms**:
```bash
$ curl -I https://blog.com/article/
HTTP/2 404
```

**Causes**:
1. Forgot to run `hexo deploy`
2. GitHub Pages cache not updated (wait longer)
3. Wrong URL structure (check slug)

**Fix**:
```bash
cd chiuhou-blog-source
hexo clean && hexo g && hexo deploy
sleep 180
./verify_blog.sh "https://..."
```

### Issue 2: Old Content Showing

**Symptoms**: Blog shows outdated version

**Cause**: Hexo cache not cleared

**Fix**:
```bash
hexo clean  # This is critical!
hexo g
hexo deploy
```

### Issue 3: Images Not Loading

**Symptoms**: Article loads but cover image 404

**Cause**: Image not pushed to `blog_img` repo

**Fix**:
```bash
cd /root/.openclaw/workspace/blog_img
git add .
git commit -m "Add missing image"
git push origin main

# Wait for GitHub CDN
sleep 60
```

### Issue 4: Deployment Fails

**Symptoms**:
```
fatal: Could not read from remote repository.
```

**Cause**: Git credentials or remote URL issue

**Fix**:
```bash
cd chiuhou-blog-source
git remote -v  # Check remote URL
git config --list | grep user  # Check git config

# Re-configure if needed
git config user.name "..."
git config user.email "..."
```

## Automation Script

For convenience, create `deploy_and_verify.sh`:

```bash
#!/bin/bash
set -e

ARTICLE_SLUG="$1"
ARTICLE_DATE="$2"

if [ -z "$ARTICLE_SLUG" ] || [ -z "$ARTICLE_DATE" ]; then
    echo "Usage: ./deploy_and_verify.sh <slug> <YYYY-MM-DD>"
    exit 1
fi

echo "📝 Deploying blog post: $ARTICLE_SLUG"
echo "📅 Date: $ARTICLE_DATE"
echo ""

# Extract date components
YEAR=$(echo $ARTICLE_DATE | cut -d'-' -f1)
MONTH=$(echo $ARTICLE_DATE | cut -d'-' -f2)
DAY=$(echo $ARTICLE_DATE | cut -d'-' -f3)

# Build URL
BLOG_URL="https://chiuhoukazusa.github.io/chiuhou-tech-blog/$YEAR/$MONTH/$DAY/$ARTICLE_SLUG/"

echo "🏗️  Building site..."
cd /root/.openclaw/workspace/chiuhou-blog-source
hexo clean
hexo generate

echo "🚀 Deploying..."
hexo deploy

echo "⏳ Waiting for GitHub Pages (3 minutes)..."
sleep 180

echo "🔍 Verifying deployment..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$BLOG_URL")

if [ "$HTTP_CODE" = "200" ]; then
    echo "✅ SUCCESS! Blog post is live:"
    echo "   $BLOG_URL"
    exit 0
else
    echo "❌ FAILED! HTTP $HTTP_CODE"
    echo "   URL: $BLOG_URL"
    exit 1
fi
```

Usage:
```bash
./deploy_and_verify.sh "shadow-ray-tracing" "2026-02-17"
```

## Checklist Summary

Before reporting "blog published":

- [ ] Markdown file created in `source/_posts/`
- [ ] Cover image uploaded to `blog_img` repo
- [ ] `hexo clean` executed
- [ ] `hexo generate` executed successfully
- [ ] `hexo deploy` executed successfully
- [ ] Waited 2-3 minutes for propagation
- [ ] **HTTP 200 verified** with curl or verify_blog.sh
- [ ] Blog URL provided to user
- [ ] GitHub repo URL provided to user

**All must be ✅ before claiming completion.**

## Emergency Rollback

If a broken post goes live:

```bash
cd chiuhou-blog-source

# Remove the post
rm source/_posts/broken-post.md

# Rebuild without it
hexo clean && hexo g && hexo deploy
```

GitHub Pages will update in 2-3 minutes.

## Prevention

To avoid false "published" reports:

1. **Use this checklist** every time
2. **Run verify_blog.sh** before claiming done
3. **Never skip the wait** (GitHub Pages delay)
4. **Test the URL** in a browser or with curl

**Zero tolerance for false positives.**
