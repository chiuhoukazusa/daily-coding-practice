# Convex Hull — Three Algorithms Compared

## Overview
Implementation and comparison of three convex hull algorithms:
1. **Graham Scan** — polar angle sort + stack
2. **Monotone Chain (Andrew's)** — split lower/upper hull
3. **QuickHull** — recursive divide-and-conquer

## Build & Run
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## Output
All three algorithms produce identical results across all test cases.
Output includes PPM visualizations showing data points + hull polygons.

## Test Cases
| Test | Points | Result |
|------|--------|--------|
| Small Random | 50 | ✅ |
| Circle | 100 | ✅ |
| Large Random | 500 | ✅ |
| Grid 15x15 | 225 | ✅ |
| Noisy Circle | 200 | ✅ |
| Collinear | 20 | ✅ |
| Degenerate (2 pts) | 2 | ✅ |

## Technical Highlights
- Cross product for orientation (CCW/CW)
- Quantitative validation: all hulls verified convex, cover all points, identical area
- Degenerate case handling (collinear, <3 points)
- PPM image output with color-coded hull visualization
