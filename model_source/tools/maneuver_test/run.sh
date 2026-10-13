#!/bin/sh
# 用法（工程根目录）: sh tools/maneuver_test/run.sh <结果标签>
# 可用 SHIP_SRC 指定要测试的船舶模型源文件，例如测试已提交版本：
#   git show HEAD:ship.cpp > tools/maneuver_test/build/ship_head.cpp
#   SHIP_SRC=tools/maneuver_test/build/ship_head.cpp sh tools/maneuver_test/run.sh head
set -e
QT=D:/QT5/5.14.2/mingw73_32
OUT=tools/maneuver_test/build
SHIP_SRC=${SHIP_SRC:-ship.cpp}
mkdir -p "$OUT" tools/maneuver_test/results
g++ -O2 -std=gnu++11 -DQT_NO_DEBUG -Wno-unused-variable -Wno-unused-parameter \
    -I. -I"$QT/include" -I"$QT/include/QtCore" -I"$QT/mkspecs/win32-g++" \
    tools/maneuver_test/main.cpp "$SHIP_SRC" baseline_controller.cpp mmg_model.cpp mmg_kvlcc2.cpp \
    -L"$QT/lib" -lQt5Core -o "$OUT/maneuver_test.exe"
"$OUT/maneuver_test.exe" | tee "tools/maneuver_test/results/${1:-run}.txt"
