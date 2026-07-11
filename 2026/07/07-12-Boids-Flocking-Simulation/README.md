# Boids Flocking Simulation

## Overview
A classic Craig Reynolds boids flocking simulation implementing three steering behaviors:
- **Separation**: Steer to avoid crowding local flockmates
- **Alignment**: Steer towards the average heading of local flockmates  
- **Cohesion**: Steer to move toward the average position of local flockmates

## Key Features
- 300 boids simulated over 300 timesteps
- Spatial hashing for O(n) neighbor lookup (instead of O(n²))
- Triangular boid rendering with velocity-based color
- Quantitative validation metrics (polarization, separation, speed)

## Compilation & Run
```bash
g++ main.cpp -o boids -std=c++17 -O2 -Wall -Wextra
./boids
```

## Output
Generates PPM frames showing boid positions and orientations at different time steps:
- `frame_0000.ppm` - Initial random state
- `frame_0150.ppm` - Mid-simulation (flocking emerging)
- `frame_0299.ppm` - Final state (aligned flock)

## Validation Metrics
| Metric | Initial | Final |
|--------|---------|-------|
| Polarization | ~0.04 | >0.30 |
| Avg Speed | ~2.7 | ~3.9 |
| Avg Separation | ~18px | ~16px |

## Technical Details
- **Boundary**: Soft margin force prevents boids from leaving the 800×600 canvas
- **Spatial Hash**: Grid-based cell lookup with 50px cell size
- **Rendering**: Barycentric triangle rasterization with directional coloring
- **Steering**: Reynolds' three rules with weighted force accumulation
