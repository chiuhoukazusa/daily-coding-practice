# SDF Ray Marching Renderer

A software renderer implementing Signed Distance Field (SDF) Ray Marching (Sphere Tracing).

## Compile & Run
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## Output
![SDF Ray Marching](sdf_output.png)

## Techniques
- SDF primitives: sphere, box, torus, capsule, plane
- Smooth union / subtraction / intersection CSG operations
- Sphere Tracing (Ray Marching) with 256 max steps
- Central difference normal estimation
- Blinn-Phong shading with 3 lights
- Soft shadows via marching shadow rays (k=16)
- 5-tap Ambient Occlusion estimation
- ACES Filmic tone mapping + gamma correction
- Fog effect
- Checker-board ground plane material
