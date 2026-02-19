#!/bin/bash
# verify_coordinate_system.sh - 自动验证图形学渲染的坐标系正确性

set -e

IMAGE="$1"

if [ -z "$IMAGE" ]; then
    echo "Usage: $0 <image_path>"
    exit 1
fi

if [ ! -f "$IMAGE" ]; then
    echo "❌ Error: Image file not found: $IMAGE"
    exit 1
fi

WIDTH=$(identify -format '%w' "$IMAGE")
HEIGHT=$(identify -format '%h' "$IMAGE")

echo "图像尺寸: ${WIDTH}x${HEIGHT}"

# 采样顶部中心（使用txt格式，解析十进制RGB）
TOP_LINE=$(convert "$IMAGE" -crop 1x1+$((WIDTH/2))+10 txt:- | tail -1)
TOP_R=$(echo "$TOP_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\1/')
TOP_G=$(echo "$TOP_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\2/')
TOP_B=$(echo "$TOP_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\3/')

# 采样底部中心
BOTTOM_LINE=$(convert "$IMAGE" -crop 1x1+$((WIDTH/2))+$((HEIGHT-10)) txt:- | tail -1)
BOTTOM_R=$(echo "$BOTTOM_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\1/')
BOTTOM_G=$(echo "$BOTTOM_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\2/')
BOTTOM_B=$(echo "$BOTTOM_LINE" | sed 's/.*(\([0-9]*\),\([0-9]*\),\([0-9]*\)).*/\3/')

echo "顶部中心 RGB: ($TOP_R, $TOP_G, $TOP_B)"
echo "底部中心 RGB: ($BOTTOM_R, $BOTTOM_G, $BOTTOM_B)"

# 计算亮度
TOP_BRIGHTNESS=$((TOP_R + TOP_G + TOP_B))
BOTTOM_BRIGHTNESS=$((BOTTOM_R + BOTTOM_G + BOTTOM_B))

# 判断顶部是否像天空
TOP_IS_SKY=0
if [ $TOP_B -gt $TOP_R ] && [ $TOP_B -gt $((TOP_G + 50)) ]; then
    echo "✅ 顶部是蓝色（可能是天空）"
    TOP_IS_SKY=1
elif [ $TOP_BRIGHTNESS -gt 400 ]; then
    echo "✅ 顶部是浅色（可能是天空/背景）"
    TOP_IS_SKY=1
else
    echo "⚠️  顶部不像天空（深色或有颜色）"
fi

# 判断底部是否像地面
BOTTOM_IS_GROUND=0
if [ $BOTTOM_BRIGHTNESS -lt 300 ]; then
    echo "✅ 底部是深色（可能是地面/阴影）"
    BOTTOM_IS_GROUND=1
elif [ $BOTTOM_B -gt $BOTTOM_R ] && [ $BOTTOM_B -gt $((BOTTOM_G + 50)) ]; then
    echo "⚠️  底部是蓝色（可能是天空）- 疑似上下颠倒！"
else
    echo "✅ 底部有颜色（可能是地面/球体）"
    BOTTOM_IS_GROUND=1
fi

# 综合判断
echo ""
if [ $TOP_IS_SKY -eq 1 ] && [ $BOTTOM_IS_GROUND -eq 1 ]; then
    echo "✅ 坐标系正确：天空在上，地面在下"
    exit 0
elif [ $TOP_IS_SKY -eq 0 ] && [ $BOTTOM_IS_GROUND -eq 0 ]; then
    echo "❌ 坐标系翻转：天空在下，地面在上！"
    exit 1
else
    echo "⚠️  无法确定，建议人工检查"
    echo "   提示：如果场景没有明显的天空/地面，此脚本可能误判"
    exit 2
fi
