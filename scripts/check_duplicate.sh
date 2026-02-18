#!/bin/bash
# check_duplicate.sh - Check if today's project is duplicate

set -e

PROJECT_INDEX="/root/.openclaw/workspace/daily-coding-practice/PROJECT_INDEX.md"
TODAY=$(date +%m-%d)
CURRENT_YEAR=$(date +%Y)

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "🔍 Checking for duplicate projects..."
echo "Date: $TODAY"
echo ""

# Check if today already has a project
if grep -q "| $TODAY |" "$PROJECT_INDEX"; then
    echo -e "${RED}❌ DUPLICATE DETECTED: Today ($TODAY) already has a project!${NC}"
    echo ""
    echo "Existing project:"
    grep "| $TODAY |" "$PROJECT_INDEX"
    echo ""
    exit 1
fi

# Extract project proposal (passed as argument)
PROPOSED_PROJECT="$1"

if [ -z "$PROPOSED_PROJECT" ]; then
    echo -e "${YELLOW}⚠️  No project name provided for duplicate check${NC}"
    exit 0
fi

echo "Proposed project: $PROPOSED_PROJECT"
echo ""

# Check for similar projects in the past 7 days
echo "Checking recent projects (past 7 days):"

# Get past 7 days' dates
for i in {0..7}; do
    CHECK_DATE=$(date -d "$i days ago" +%m-%d)
    if grep -q "| $CHECK_DATE |" "$PROJECT_INDEX"; then
        EXISTING_PROJECT=$(grep "| $CHECK_DATE |" "$PROJECT_INDEX" | awk -F'|' '{print $3}' | xargs)
        echo "  $CHECK_DATE: $EXISTING_PROJECT"
        
        # Fuzzy match: check if project names share keywords
        # Extract keywords (lowercase, remove spaces)
        PROPOSED_LOWER=$(echo "$PROPOSED_PROJECT" | tr '[:upper:]' '[:lower:]' | tr -d ' ')
        EXISTING_LOWER=$(echo "$EXISTING_PROJECT" | tr '[:upper:]' '[:lower:]' | tr -d ' ')
        
        # Check for common substrings
        if [[ "$PROPOSED_LOWER" == *"perlin"* && "$EXISTING_LOWER" == *"perlin"* ]]; then
            echo -e "${RED}❌ SIMILAR PROJECT DETECTED!${NC}"
            echo "  Proposed: $PROPOSED_PROJECT"
            echo "  Existing: $EXISTING_PROJECT (on $CHECK_DATE)"
            echo ""
            echo "⚠️  Please choose a different project from 待探索领域 list"
            exit 1
        fi
        
        if [[ "$PROPOSED_LOWER" == *"ray"* && "$EXISTING_LOWER" == *"ray"* ]]; then
            # Allow ray tracing if it's a major feature upgrade
            if [[ "$PROPOSED_LOWER" == *"reflect"* || "$PROPOSED_LOWER" == *"refract"* ]]; then
                # New features: reflection or refraction - ALLOW
                continue
            elif [[ "$EXISTING_LOWER" == *"shadow"* && "$PROPOSED_LOWER" != *"shadow"* && "$PROPOSED_LOWER" != *"reflect"* && "$PROPOSED_LOWER" != *"refract"* ]]; then
                echo -e "${RED}❌ RAY TRACING DOWNGRADE DETECTED!${NC}"
                echo "  Previous project already has shadows"
                exit 1
            fi
        fi
    fi
done

echo ""
echo -e "${GREEN}✅ No duplicates found. Safe to proceed.${NC}"
exit 0
