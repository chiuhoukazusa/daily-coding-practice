#!/bin/bash
# verify_blog.sh - Verify that blog post is actually deployed and accessible

set -e

BLOG_URL="$1"

if [ -z "$BLOG_URL" ]; then
    echo "❌ Error: Blog URL required"
    echo "Usage: ./verify_blog.sh https://example.com/path/to/post/"
    exit 1
fi

echo "🔍 Verifying blog deployment..."
echo "URL: $BLOG_URL"
echo ""

# Check HTTP status code
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$BLOG_URL")

echo "HTTP Status: $HTTP_CODE"

if [ "$HTTP_CODE" = "200" ]; then
    echo "✅ Blog post is LIVE and accessible!"
    exit 0
elif [ "$HTTP_CODE" = "404" ]; then
    echo "❌ Blog post NOT FOUND (404)"
    echo ""
    echo "Common causes:"
    echo "1. Forgot to run: hexo deploy"
    echo "2. GitHub Pages cache (wait 2-3 minutes)"
    echo "3. Incorrect URL structure"
    echo ""
    echo "Fix steps:"
    echo "  cd /root/.openclaw/workspace/chiuhou-blog-source"
    echo "  hexo clean && hexo g && hexo deploy"
    echo ""
    echo "Then wait 2-3 minutes and run this script again."
    exit 1
else
    echo "⚠️  Unexpected HTTP status: $HTTP_CODE"
    echo "Expected: 200 (OK)"
    exit 1
fi
