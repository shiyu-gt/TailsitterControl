#!/bin/bash
# 快捷保存脚本 - 一键提交修改
cd "$(dirname "$0")"

# 检查是否有修改
if git diff --quiet && git diff --cached --quiet; then
    echo "没有修改需要保存"
    exit 0
fi

# 显示修改摘要
echo "=== 修改文件 ==="
git diff --name-only
echo ""

# 自动生成提交消息
TIMESTAMP=$(date "+%Y-%m-%d %H:%M")
MSG="auto-save: $TIMESTAMP"

# 提交
git add -A
git commit -m "$MSG"
echo "✓ 已保存: $MSG"
