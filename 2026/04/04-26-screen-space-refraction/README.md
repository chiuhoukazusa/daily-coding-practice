# Screen Space Refraction

Real-time screen space refraction renderer using G-Buffer data.

## Compile & Run
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## Output
![Result](ssr_refraction_output.png)

## Key Techniques
- **G-Buffer construction**: depth, normal, albedo, glass mask
- **Snell's Law refraction**: screen-space direction offset for glass objects
- **Schlick Fresnel**: physically-based blend between reflection and refraction
- **Chromatic aberration**: per-channel UV offset simulates glass dispersion
- **Multi-pass rendering**: background render → G-buffer → SSR refraction composite
