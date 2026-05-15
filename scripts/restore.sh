#!/bin/bash
# 回退到指定版本
cd "$(dirname "$0")"

if [ -z "$1" ]; then
    echo "用法: ./restore.sh <commit-hash>"
    echo ""
    git log --oneline -10
    exit 1
fi

git checkout "$1" -- .
echo "✓ 已回退到版本 $1"
