# Strassen Matrix Multiplication

Divide-and-conquer matrix multiplication using Strassen's algorithm.

## Algorithm

Strassen's algorithm reduces the asymptotic complexity from O(n³) to O(n^log₂7) ≈ O(n^2.81) by recursively splitting matrices into quadrants and using 7 multiplications instead of 8.

## Compile & Run

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## Output

The program tests with matrix sizes 128, 256, and 512, producing:
- Standard multiplication result (grayscale PPM)
- Strassen multiplication result (grayscale PPM)
- Side-by-side comparison (PPM)
- Error heatmap (PPM)

## Verification Results (n=512)

| Metric | Value |
|--------|-------|
| Max absolute error | 1.50e-11 |
| MSE | 3.39e-24 |
| Exact match % | 100% |
| Speedup vs standard | 1.46x |

## Technical Points

- Strassen's divide-and-conquer algorithm with base case n ≤ 64
- Matrix padding to next power of 2
- Quantitative verification against standard O(n³) multiplication
- Maximum absolute error < 2e-11 across all tested sizes
- PPM image output for visual comparison
