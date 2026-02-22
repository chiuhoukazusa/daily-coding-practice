#!/bin/bash
# 重复项目检测脚本
# 使用: ./scripts/check_duplicate.sh "项目主题"
# 例如: ./scripts/check_duplicate.sh "抗锯齿"

set -e

if [ -z "$1" ]; then
    echo "❌ 错误: 请提供项目主题作为参数"
    echo "使用方法: $0 '项目主题'"
    echo "示例: $0 '抗锯齿'"
    exit 1
fi

THEME="$1"
PROJECT_INDEX="PROJECT_INDEX.md"

if [ ! -f "$PROJECT_INDEX" ]; then
    echo "❌ 错误: 找不到 PROJECT_INDEX.md"
    exit 1
fi

echo "🔍 检查项目主题: '$THEME'"
echo ""

# 搜索相似项目（不区分大小写）
echo "📋 相似项目列表:"
echo "========================================"

FOUND=0
while IFS='|' read -r line; do
    # 跳过表头和分隔线
    if [[ "$line" =~ ^[[:space:]]*\|[[:space:]]*日期 ]] || [[ "$line" =~ ^[[:space:]]*\|[-:]+ ]]; then
        continue
    fi
    
    # 提取日期、项目名称、核心技术
    if [[ "$line" =~ \|[[:space:]]*([0-9-]+)[[:space:]]*\|[[:space:]]*([^\|]+)[[:space:]]*\|[[:space:]]*([^\|]+)[[:space:]]*\| ]]; then
        DATE="${BASH_REMATCH[1]}"
        NAME="${BASH_REMATCH[2]}"
        TECH="${BASH_REMATCH[3]}"
        
        # 搜索匹配（项目名或核心技术）
        if echo "$NAME $TECH" | grep -qi "$THEME"; then
            echo "⚠️  [$DATE] $NAME"
            echo "    核心技术: $TECH"
            echo ""
            FOUND=$((FOUND + 1))
        fi
    fi
done < "$PROJECT_INDEX"

echo "========================================"

if [ "$FOUND" -gt 0 ]; then
    echo "❌ 发现 $FOUND 个相似项目！"
    echo ""
    echo "⚠️  警告: 可能存在重复！"
    echo "建议:"
    echo "  1. 检查上述项目是否与你的计划重复"
    echo "  2. 如果重复，选择其他主题"
    echo "  3. 如果是扩展/改进，确保有明显差异"
    echo ""
    exit 1
else
    echo "✅ 未发现相似项目，可以开始！"
    echo ""
fi
