#!/bin/bash

echo "🔧 编译简单光线追踪器..."

# 检查编译器
g++ --version

# 编译
echo "编译 main.cpp..."
g++ -std=c++17 -O2 -o raytracer main.cpp

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo "运行: ./raytracer"
else
    echo "❌ 编译失败"
    exit 1
fi