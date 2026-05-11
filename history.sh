#!/bin/bash
# 查看版本历史
cd "$(dirname "$0")"
git log --oneline --graph -20
