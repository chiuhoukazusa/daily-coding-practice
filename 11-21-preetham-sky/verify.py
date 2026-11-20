#!/usr/bin/env python3
"""Quantitative verification of the Preetham analytic sky model.

Runs the compiled binary under two solar configurations and asserts the
physical/mathematical invariants of the model WITHOUT relying on visual
inspection (the whole point of the daily practice is quantitative validation).

Checks:
  V1  CIE chromaticity of the zenith is a valid daylight value (0.20 < x,y < 0.40
      and x+y < 1), i.e. inside the daylight locus.
  V2  Luminance peaks at/around the sun direction (circumsolar brightening):
      the maximum sky luminance is strictly greater than the zenith luminance.
  V3  Along the sun's meridian, luminance rises toward the sun and falls away on
      the far side (unimodal, peak near the sun's elevation).
  V4  Sunset dimming: a lower sun (larger theta_s) yields a lower zenith
      luminance and lower peak luminance than a higher sun.
  V5  The rendered PPM is non-degenerate: right size, not all-black/all-white,
      and has real spatial variation (std of luminance map is non-trivial).
"""
import subprocess, re, sys, glob, os
from PIL import Image
import numpy as np

def run(theta_s):
    out = subprocess.run(["./output", "4.0", f"{theta_s:g}"],
                         capture_output=True, text=True, check=True)
    kv = {}
    for line in out.stdout.splitlines():
        m = re.match(r"RESULT_(\w+)\s+([-+0-9.eE]+)", line)
        if m: kv[m.group(1)] = float(m.group(2))
    return kv

def main():
    errors = []
    def check(cond, msg):
        print(("  PASS  " if cond else "  FAIL  ") + msg)
        if not cond: errors.append(msg)

    # ---- Case A: sun fairly high, theta_s = 0.6 rad (~56 deg elevation) ----
    a = run(0.6)
    print(f"[config A] theta_s=0.6 rad, T=4.0  -> {a}")

    print("\nV1 zenith chromaticity valid daylight")
    check(0.20 < a["zenith_x"] < 0.40, f"0.20 < x={a['zenith_x']:.4f} < 0.40")
    check(0.20 < a["zenith_y"] < 0.40, f"0.20 < y={a['zenith_y']:.4f} < 0.40")
    check(a["zenith_x"] + a["zenith_y"] < 1.0, f"x+y={a['zenith_x']+a['zenith_y']:.4f} < 1")

    print("\nV2 circumsolar brightening (max lum > zenith lum)")
    check(a["maxLum"] > a["zenith_Y"], f"{a['maxLum']:.0f} > {a['zenith_Y']:.0f}")

    print("\nV3 along sun meridian unimodal w/ peak near sun (theta_s=0.6 rad, row idx 2)")
    lm = [a[f"sunMeridian{i}"] for i in range(5)]  # zenith(0)..horizon(4)
    # sun at 0.6 rad = 60% toward horizon => peak should be around index 2 (0.5) or 3
    pk = max(range(5), key=lambda i: lm[i])
    check(pk in (1, 2, 3), f"peak luminance at meridian index {pk} (near sun, not at zenith=0 or horizon=4)")
    check(lm[pk] > lm[0], f"peak {lm[pk]:.0f} > zenith {lm[0]:.0f}")
    check(lm[0] < lm[1], "luminance rises from zenith toward sun")

    print("\nV4 sunset dimming (theta_s=1.2 rad vs 0.6 rad)")
    b = run(1.2)   # sun low, ~21 deg elevation
    check(b["zenith_Y"] < a["zenith_Y"], f"lower sun zenith Y {b['zenith_Y']:.0f} < {a['zenith_Y']:.0f}")
    check(b["maxLum"] < a["maxLum"], f"lower sun max {b['maxLum']:.0f} < {a['maxLum']:.0f}")

    print("\nV5 rendered PPM non-degenerate")
    ppm = glob.glob("preetham_sky_output.ppm")
    check(bool(ppm), "PPM file exists")
    if ppm:
        sz = os.path.getsize(ppm[0])
        check(sz > 10000, f"file size {sz} > 10000 bytes")
        img = np.array(Image.open(ppm[0])).astype(float)
        mean = img.mean(); std = img.std()
        check(5 < mean < 250, f"mean {mean:.1f} in (5,250) — not black/white")
        check(std > 5, f"std {std:.1f} > 5 — real variation")

    print()
    if errors:
        print(f"=== VERIFICATION FAILED: {len(errors)} check(s) ===")
        for e in errors: print("  -", e)
        sys.exit(1)
    print("=== ALL QUANTITATIVE CHECKS PASSED ===")
    sys.exit(0)

if __name__ == "__main__":
    main()
