# Simulated Annealing TSP Solver

## 编译运行
```bash
g++ main.cpp -o tsp_solver -std=c++17 -O2 -Wall -Wextra
./tsp_solver
```

## 输出结果

### 50-city Best Tour (SA)
![Best Tour](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/06/06-24-simulated-annealing-tsp/tsp_best_tour.png)

### 50-city Greedy Initial Tour
![Greedy Tour](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/06/06-24-simulated-annealing-tsp/tsp_greedy_tour.png)

### 8-city Optimality Verification
![Small Tour](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/06/06-24-simulated-annealing-tsp/tsp_small_tour.png)

## 技术要点
- **Simulated Annealing**: Metropolis acceptance criterion for heuristic optimization
- **2-opt neighborhood**: Reversal-based local search move for TSP
- **Cooling schedules**: Compared 4 different cooling rates (α = 0.90, 0.95, 0.98, 0.99)
- **Greedy initialization**: Nearest-neighbor heuristic for starting solution
- **Optimality verification**: Brute-force on 8-city instance confirms SA finds global optimum
- **Quantitative validation**: Monotonic convergence, improvement metrics, schedule comparison
