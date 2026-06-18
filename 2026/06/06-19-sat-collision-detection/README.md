# SAT Collision Detection Visualizer

## Algorithm
Separating Axis Theorem (SAT) for 2D convex polygon collision detection.

## Compile & Run
```bash
g++ main.cpp -o sat_prog -std=c++17 -O2 -Wall -Wextra
./sat_prog
```

## Output
- `output/sat_composite.png` — 6 edge case visualizations (3x2 grid)
- `output/sat_monte_carlo.png` — 20 random polygon pair samples

## Technical Highlights
- **SAT Algorithm**: Projects polygons onto edge normals to find separating axes
- **MTV (Minimum Translation Vector)**: Computes the minimal push vector for collision resolution
- **Edge Case Coverage**: Separated, overlapping, edge-touching, containment, vertex-touch
- **Monte Carlo Verification**: 10,000 random convex polygon pairs vs brute-force ground truth
  - **100.00% accuracy** (0 false negatives, 0 false positives)
- **PPM Image Format**: Custom software rendering with polygon fill (even-odd rule) and wireframe
