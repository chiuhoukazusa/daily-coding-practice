# Ray-Sphere Intersection Visualization

## Project Overview
This project implements a simple ray tracer that visualizes ray-sphere intersections. The program calculates ray intersections with spheres using basic geometric formulas and generates a PPM image as output.

## Key Features
- Ray-sphere intersection calculation using geometric approach
- Simple 3D scene with a sphere at the origin
- Multiple ray directions for visualization
- PPM image format output

## Implementation Details
The program uses the following main components:
1. **Ray definition**: origin and direction vectors
2. **Sphere definition**: center and radius
3. **Intersection algorithm**: Uses discriminant to determine hit/miss
4. **Color calculation**: Based on intersection results

## File Structure
- `ray_sphere_intersection.cpp`: Main C++ source code
- `ray_sphere_intersection`: Compiled executable
- `ray_sphere_intersection.ppm`: Output image file (~1.1MB)

## Build & Run
```bash
# Compile
g++ -o ray_sphere_intersection ray_sphere_intersection.cpp

# Run
./ray_sphere_intersection
```

## Technical Concepts
- Ray-sphere intersection formula: (ray.origin - sphere.center)·(ray.origin - sphere.center) - R²
- Discriminant calculation for intersection detection
- Simple shading based on surface normal and viewing direction