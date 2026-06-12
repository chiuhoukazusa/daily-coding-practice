# A* Pathfinding Visualizer (2026-06-12)

4-connected grid pathfinding with **A\*** search, cross-validated against
**Dijkstra** and **BFS** to *prove* correctness quantitatively (not by eye).

## Technique
- A\* with admissible **Manhattan heuristic** on a uniform-cost grid
- Lazy-deletion binary heap (stale entries skipped via closed set)
- Parent-pointer path reconstruction
- Obstacle field: deterministic random scatter (seed 20260612) + 3 vertical
  maze walls with single gaps, forcing non-trivial routing

## Quantitative verification (all PASS)
| Check | Result |
|-------|--------|
| All 3 algorithms found a path | PASS |
| A\* cost == Dijkstra cost (optimality) | 234 == 234 |
| A\* cost == BFS cost (unit-cost shortest) | 234 == 234 |
| A\* expands ≤ Dijkstra (heuristic helps) | 3419 ≤ 3680 |
| path length == cost+1 (no gaps) | 235 == 235 |
| endpoints correct (start→goal) | PASS |
| every step 4-adjacent (connected) | PASS |
| no path cell on an obstacle | PASS |

Image: `astar_output.png` 1120×840, 15KB, mean 99.1, std 57.5.
Colors: green=start, yellow=goal, red=optimal path, blue=A\* explored set,
dark=obstacles.

## Build & run
```bash
g++ main.cpp -o astar -std=c++17 -O2 -Wall -Wextra
./astar
```
