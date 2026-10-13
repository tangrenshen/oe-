#!/bin/sh
# 用法（工程根目录）: sh tools/mmg_validation/run.sh <结果标签> [xP 覆盖值]
# MMG 模块不依赖 Qt，直接用 g++ 编译
set -e
TAG=${1:-run}
if [ $# -gt 0 ]; then shift; fi
OUT=tools/mmg_validation/build
mkdir -p "$OUT" tools/mmg_validation/results
g++ -O2 -std=gnu++11 -Wall -Wextra -I. \
    tools/mmg_validation/main.cpp mmg_model.cpp mmg_kvlcc2.cpp \
    -o "$OUT/mmg_validation.exe"
"$OUT/mmg_validation.exe" "$@" | tee "tools/mmg_validation/results/$TAG.txt"
