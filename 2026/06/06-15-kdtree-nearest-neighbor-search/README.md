# KD-Tree Nearest Neighbor Search

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
![结果](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/06/06-15-kdtree-nearest-neighbor-search/kdtree_output.png)

## 技术要点
- **KD-Tree**: 多维空间二叉树索引，每次递归按不同轴（x/y）交替进行中位数分割
- **KNN搜索**: 利用KD-Tree剪枝（若查询点到分割平面距离 < 当前最远距离才搜索远侧），大幅减少比较次数
- **暴力搜索基准对比**: 对50000个随机2D点做20次K=7查询，KD-Tree达到~100x加速
- **正确性验证**: KD-Tree结果与暴力搜索完全一致（0/140错误）
- **量化验证**: 像素均值/标准差检查、文件大小检查、数值范围检查
