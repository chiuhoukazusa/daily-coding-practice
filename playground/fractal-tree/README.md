# 分形树生成器 (Fractal Tree Generator)

## 项目描述
使用递归算法生成分形树，支持多种风格：
- 对称二叉树
- 不对称随机树
- 多分支树
- 樱花树（带粉色叶子）

## 数学原理
递归定义：
```
Tree(x, y, length, angle, depth):
    if depth == 0: return
    
    end_x = x + length * cos(angle)
    end_y = y + length * sin(angle)
    draw_line(x, y, end_x, end_y)
    
    Tree(end_x, end_y, length * scale, angle + branch_angle, depth - 1)
    Tree(end_x, end_y, length * scale, angle - branch_angle, depth - 1)
```

## 参数说明
- `length`: 树枝长度（逐级缩短）
- `angle`: 当前生长方向
- `branch_angle`: 分叉角度（30-45度效果较好）
- `scale`: 长度衰减系数（0.7-0.8）
- `depth`: 递归深度（8-12层）

## 预期效果
- 自然的分形结构
- 从粗到细的树枝
- 可调参数生成不同风格

## 编译运行
```bash
g++ -std=c++17 -O2 fractal_tree.cpp -o fractal_tree
./fractal_tree
```

输出：
- `tree_symmetric.png` - 对称树
- `tree_random.png` - 随机树
- `tree_cherry.png` - 樱花树
- `tree_autumn.png` - 秋天树
