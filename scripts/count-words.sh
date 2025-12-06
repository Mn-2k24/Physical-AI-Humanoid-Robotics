#!/bin/bash

###############################################################################
# count-words.sh - Count total words across all documentation
#
# Usage:
#   bash scripts/count-words.sh
#   bash scripts/count-words.sh --detailed
#
# Description:
#   Counts words in all markdown files in the docs/ directory.
#   Excludes code blocks, YAML frontmatter, and special markdown syntax.
###############################################################################

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$REPO_ROOT/docs"

# Detailed flag
DETAILED=false
if [[ "$1" == "--detailed" ]] || [[ "$1" == "-d" ]]; then
    DETAILED=true
fi

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Word Count Analysis${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# Check if docs directory exists
if [ ! -d "$DOCS_DIR" ]; then
    echo -e "${RED}✗ Error: docs/ directory not found${NC}"
    exit 1
fi

# Function to count words in a markdown file
# Removes code blocks, YAML frontmatter, and counts remaining words
count_words_in_file() {
    local file="$1"

    # Use Python to properly parse markdown and count words
    python3 -c "
import re
import sys

with open('$file', 'r', encoding='utf-8') as f:
    content = f.read()

# Remove YAML frontmatter
content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

# Remove code blocks (``` ... ```)
content = re.sub(r'\`\`\`.*?\`\`\`', '', content, flags=re.DOTALL)

# Remove inline code (\`...\`)
content = re.sub(r'\`[^\`]+\`', '', content)

# Remove HTML comments
content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

# Remove URLs
content = re.sub(r'https?://[^\s\)]+', '', content)

# Remove markdown image syntax
content = re.sub(r'!\[.*?\]\(.*?\)', '', content)

# Remove markdown link syntax but keep the text
content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

# Remove markdown headers (#, ##, etc.) but keep text
content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)

# Remove markdown bold/italic syntax
content = re.sub(r'\*\*([^\*]+)\*\*', r'\1', content)
content = re.sub(r'\*([^\*]+)\*', r'\1', content)
content = re.sub(r'__([^_]+)__', r'\1', content)
content = re.sub(r'_([^_]+)_', r'\1', content)

# Count words
words = content.split()
print(len(words))
" 2>/dev/null || echo "0"
}

# Initialize counters
TOTAL_WORDS=0
declare -A MODULE_COUNTS
declare -A FILE_COUNTS

echo -e "${BLUE}Analyzing markdown files in docs/${NC}"
echo ""

# Find all markdown files and count words
while IFS= read -r file; do
    WORD_COUNT=$(count_words_in_file "$file")
    TOTAL_WORDS=$((TOTAL_WORDS + WORD_COUNT))

    # Get relative path
    REL_PATH=$(echo "$file" | sed "s|$DOCS_DIR/||")

    # Store count by file
    FILE_COUNTS["$REL_PATH"]=$WORD_COUNT

    # Categorize by module/section
    if [[ "$REL_PATH" == module-1-ros2/* ]]; then
        MODULE_COUNTS["Module 1: ROS 2"]=$((${MODULE_COUNTS["Module 1: ROS 2"]:-0} + WORD_COUNT))
    elif [[ "$REL_PATH" == module-2-simulation/* ]]; then
        MODULE_COUNTS["Module 2: Simulation"]=$((${MODULE_COUNTS["Module 2: Simulation"]:-0} + WORD_COUNT))
    elif [[ "$REL_PATH" == module-3-isaac/* ]]; then
        MODULE_COUNTS["Module 3: Isaac Sim"]=$((${MODULE_COUNTS["Module 3: Isaac Sim"]:-0} + WORD_COUNT))
    elif [[ "$REL_PATH" == module-4-vla/* ]]; then
        MODULE_COUNTS["Module 4: VLA"]=$((${MODULE_COUNTS["Module 4: VLA"]:-0} + WORD_COUNT))
    elif [[ "$REL_PATH" == appendices/* ]]; then
        MODULE_COUNTS["Appendices"]=$((${MODULE_COUNTS["Appendices"]:-0} + WORD_COUNT))
    else
        MODULE_COUNTS["Other"]=$((${MODULE_COUNTS["Other"]:-0} + WORD_COUNT))
    fi

    if $DETAILED; then
        echo -e "  ${REL_PATH}: ${WORD_COUNT} words"
    fi
done < <(find "$DOCS_DIR" -name "*.md" -type f | sort)

# Print summary by module
echo ""
echo -e "${BLUE}Word Count by Section:${NC}"
echo ""

for module in "Module 1: ROS 2" "Module 2: Simulation" "Module 3: Isaac Sim" "Module 4: VLA" "Appendices" "Other"; do
    count=${MODULE_COUNTS["$module"]:-0}
    if [ $count -gt 0 ]; then
        printf "  %-25s %6d words\n" "$module:" "$count"
    fi
done

echo ""
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Total Word Count: ${GREEN}${TOTAL_WORDS}${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# Check if within target range (25,000-40,000)
TARGET_MIN=25000
TARGET_MAX=40000

if [ $TOTAL_WORDS -lt $TARGET_MIN ]; then
    SHORTFALL=$((TARGET_MIN - TOTAL_WORDS))
    echo -e "${YELLOW}⚠ Below target range${NC}"
    echo -e "  Target: ${TARGET_MIN}-${TARGET_MAX} words"
    echo -e "  Current: ${TOTAL_WORDS} words"
    echo -e "  Need: ${SHORTFALL} more words"
    exit 1
elif [ $TOTAL_WORDS -gt $TARGET_MAX ]; then
    OVERAGE=$((TOTAL_WORDS - TARGET_MAX))
    echo -e "${YELLOW}⚠ Above target range${NC}"
    echo -e "  Target: ${TARGET_MIN}-${TARGET_MAX} words"
    echo -e "  Current: ${TOTAL_WORDS} words"
    echo -e "  Exceeds by: ${OVERAGE} words"
    exit 1
else
    PERCENT=$(( (TOTAL_WORDS - TARGET_MIN) * 100 / (TARGET_MAX - TARGET_MIN) ))
    echo -e "${GREEN}✓ Within target range${NC}"
    echo -e "  Target: ${TARGET_MIN}-${TARGET_MAX} words"
    echo -e "  Current: ${TOTAL_WORDS} words (${PERCENT}% through range)"
    exit 0
fi
