# Poisson Disk Sampling

Blue noise sampling using Bridson's algorithm.

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
![Poisson Disk Sampling](poisson_disk_output.ppm)

## 技术要点
- Bridson's O(N) Poisson Disk Sampling algorithm
- Blue noise property: no low-frequency clustering
- Exclusion radius enforcement via spatial grid
- Fourier-based blue noise spectrum verification
- Comparison with random sampling
