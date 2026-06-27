# Skeletal Animation — Linear Blend Skinning (LBS)

A 3-bone arm hierarchy (shoulder → elbow → wrist → fingertip) with Linear Blend Skinning for smooth joint deformation. Includes soft rasterization, Phong shading, and quantitative verification.

## Compile & Run
```bash
g++ main.cpp -o skeletal_animation -std=c++17 -O2
./skeletal_animation
```

## Output
- `skeletal_animation_rest.png` — Rest pose (straight arm)
- `skeletal_animation_anim.png` — Animated pose (bent arm)
- `skeletal_animation_comparison.png` — Side-by-side comparison

## Technical Highlights
- **Linear Blend Skinning (LBS)**: Vertex deformation via weighted bone transforms
- **Forward Kinematics**: Recursive bone chain with local rotations
- **Blend Zones**: Smooth transitions at joints with adaptive weight distribution
- **Quantitative Verification**: 
  - Weight range check [0.5, 1.0]
  - Rest-pose identity check (LBS error ≤ 0.01)
  - Animation displacement verification (98.5% vertices moved)
  - Forward kinematics fingertip displacement measurement
  - Pixel statistics (mean/std/range) on rendered images
- **Soft Rasterization**: Triangle rasterization with barycentric coordinates, Z-buffer depth testing, and Phong illumination
