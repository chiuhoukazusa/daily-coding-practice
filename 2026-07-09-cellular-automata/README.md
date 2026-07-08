# Cellular Automata - Conway's Game of Life

## 项目简介
实现 Conway 生命游戏元胞自动机，包含 7 种经典模式，支持周期/振荡器检测、网格哈希优化、密度扫描统计和双枪碰撞实验。

## 编译运行
```bash
g++ game_of_life.cpp -o game_of_life -std=c++17 -O2
./game_of_life
```

## 输出结果
- Glider (滑翔机): glider_final.ppm + glider_frames.ppm
- Blinker (闪光灯): blinker_final.ppm + blinker_frames.ppm
- Pulsar (脉冲星): pulsar_final.ppm + pulsar_frames.ppm
- Gosper Glider Gun (滑翔机枪): gosper_glider_gun_final.ppm + gosper_glider_gun_frames.ppm
- R-pentomino: r-pentomino_final.ppm + r-pentomino_frames.ppm
- Diehard: diehard_final.ppm + diehard_frames.ppm
- Acorn: acorn_final.ppm + acorn_frames.ppm
- Collision (双枪碰撞): collision_frames.ppm
- 定量报告: quantitative_report.txt

## 技术要点
- 元胞自动机：Conway 生命游戏规则（B3/S23）
- 7 种经典模式：Glider, Blinker, Pulsar, Gosper Glider Gun, R-pentomino, Diehard, Acorn
- 周期/振荡器检测算法
- 网格哈希加速
- 双枪碰撞实验
