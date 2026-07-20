# Median Filter Denoising

Daily Coding Practice 2026-07-21 — Image Processing: Median Filter for Salt & Pepper Noise Removal.

## Concept

The **median filter** is a non-linear digital filtering technique used to remove impulse noise (salt & pepper) while preserving edges better than linear filters like Gaussian blur. It replaces each pixel with the median value of its neighboring pixels within a sliding window.

Key property: for impulse noise where corrupted pixels take extreme values (0 or 255), the median naturally rejects these outliers, while the mean (used by Gaussian blur) gets distorted by them.

## Compilation & Execution

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## Results

### Quantitative Verification (8/8 checks passed)

| Metric | Noisy | Median 3×3 | Median 5×5 | Gaussian σ=1.5 |
|--------|-------|------------|------------|-----------------|
| PSNR (dB) | 13.07 | 31.51 | 32.48 | 22.74 |
| Noise Removal | — | 99.7% | 99.9% | 99.7% |
| Edge Preservation | — | 0.952 | 0.956 | 0.702 |
| Uniform Region Variance | 4110.9 | 40.7 | — | 257.3 |

### Key Findings

1. Median filter achieves **+19.4 dB PSNR improvement** over noisy image (vs +9.7 dB for Gaussian)
2. **Edge preservation 0.956** for median vs 0.702 for Gaussian — median preserves structural edges
3. Uniform region variance is **6× lower** with median (40.7 vs 257.3)
4. **Noise removal rate 99.9%** with 5×5 median kernel on 15% salt & pepper noise

## Output Images

- `original.ppm` — Clean test image (4-quadrant + shapes)
- `noisy.ppm` — With 15% salt & pepper noise
- `median_3x3.ppm` — Median filter 3×3 result
- `median_5x5.ppm` — Median filter 5×5 result
- `gaussian_s1.5.ppm` — Gaussian blur comparison

## Technical Highlights

- Non-linear median filter with configurable window size
- Salt & pepper noise generator with reproducible seed
- Gaussian blur (separable 2-pass) for comparison
- Quantitative metrics: PSNR, edge preservation index, uniform region variance
- Statistical verification: pixel mean/std, noise removal rate, residual noise count
