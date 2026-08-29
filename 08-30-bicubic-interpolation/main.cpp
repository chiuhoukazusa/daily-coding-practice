// Bicubic Interpolation Image Upscaling
// Compare Nearest-Neighbor / Bilinear / Bicubic (Keys Catmull-Rom) upscaling filters.
// Quantitatively validated via MSE / PSNR (vs a high-res ground truth) and edge sharpness.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// ---------- PPM I/O ----------
struct Image {
    int w, h;
    std::vector<float> r, g, b;  // float 0..255
};

bool loadPPM(const std::string& path, Image& img) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[3] = {0};
    if (fscanf(f, "%2s", magic) != 1) { fclose(f); return false; }
    if (strcmp(magic, "P6") != 0) { fclose(f); return false; }
    int maxv;
    if (fscanf(f, "%d %d %d", &img.w, &img.h, &maxv) != 3) { fclose(f); return false; }
    fgetc(f); // single whitespace after maxval
    img.r.resize(img.w * img.h);
    img.g.resize(img.w * img.h);
    img.b.resize(img.w * img.h);
    std::vector<unsigned char> buf((size_t)img.w * img.h * 3);
    fread(buf.data(), 1, buf.size(), f);
    fclose(f);
    for (int i = 0; i < img.w * img.h; i++) {
        img.r[i] = buf[i*3+0];
        img.g[i] = buf[i*3+1];
        img.b[i] = buf[i*3+2];
    }
    return true;
}

void savePPM(const std::string& path, const Image& img) {
    FILE* f = fopen(path.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", img.w, img.h);
    for (int i = 0; i < img.w * img.h; i++) {
        unsigned char r = (unsigned char)std::min(255.0f, std::max(0.0f, img.r[i]));
        unsigned char g = (unsigned char)std::min(255.0f, std::max(0.0f, img.g[i]));
        unsigned char b = (unsigned char)std::min(255.0f, std::max(0.0f, img.b[i]));
        fputc(r, f); fputc(g, f); fputc(b, f);
    }
    fclose(f);
}

// ---------- Sampling: single channel, with clamp ----------
static inline float sampleClamp(const std::vector<float>& ch, int w, int h, int x, int y) {
    x = std::min(std::max(x, 0), w - 1);
    y = std::min(std::max(y, 0), h - 1);
    return ch[y * w + x];
}

// Cubic kernel (Keys, a = -0.5  -> Catmull-Rom)
static inline float cubicKernel(float t) {
    t = std::fabs(t);
    if (t <= 1.0f) {
        return (1.5f * t - 2.5f) * t * t + 1.0f;
    } else if (t < 2.0f) {
        return ((-0.5f * t + 2.5f) * t - 4.0f) * t + 2.0f;
    }
    return 0.0f;
}

// ---------- Upscalers ----------
enum Filter { NEAREST, BILINEAR, BICUBIC };

void upscale(const Image& src, Image& dst, Filter f) {
    dst.w = src.w * 2;
    dst.h = src.h * 2;
    dst.r.resize(dst.w * dst.h);
    dst.g.resize(dst.w * dst.h);
    dst.b.resize(dst.w * dst.h);

    for (int dy = 0; dy < dst.h; dy++) {
        float fy = (dy + 0.5f) / 2.0f - 0.5f; // inverse map to source coords
        for (int dx = 0; dx < dst.w; dx++) {
            float fx = (dx + 0.5f) / 2.0f - 0.5f;

            float vr = 0, vg = 0, vb = 0;
            if (fx < 0) fx = 0;
            if (fy < 0) fy = 0;
            if (fx > src.w - 1) fx = src.w - 1;
            if (fy > src.h - 1) fy = src.h - 1;

            if (f == NEAREST) {
                int x = (int)std::floor(fx + 0.5f);
                int y = (int)std::floor(fy + 0.5f);
                vr = sampleClamp(src.r, src.w, src.h, x, y);
                vg = sampleClamp(src.g, src.w, src.h, x, y);
                vb = sampleClamp(src.b, src.w, src.h, x, y);
            } else if (f == BILINEAR) {
                int x0 = (int)std::floor(fx);
                int y0 = (int)std::floor(fy);
                float tx = fx - x0, ty = fy - y0;
                float w00 = (1-tx)*(1-ty), w10 = tx*(1-ty), w01 = (1-tx)*ty, w11 = tx*ty;
                vr = w00*sampleClamp(src.r,src.w,src.h,x0,y0) + w10*sampleClamp(src.r,src.w,src.h,x0+1,y0)
                   + w01*sampleClamp(src.r,src.w,src.h,x0,y0+1) + w11*sampleClamp(src.r,src.w,src.h,x0+1,y0+1);
                vg = w00*sampleClamp(src.g,src.w,src.h,x0,y0) + w10*sampleClamp(src.g,src.w,src.h,x0+1,y0)
                   + w01*sampleClamp(src.g,src.w,src.h,x0,y0+1) + w11*sampleClamp(src.g,src.w,src.h,x0+1,y0+1);
                vb = w00*sampleClamp(src.b,src.w,src.h,x0,y0) + w10*sampleClamp(src.b,src.w,src.h,x0+1,y0)
                   + w01*sampleClamp(src.b,src.w,src.h,x0,y0+1) + w11*sampleClamp(src.b,src.w,src.h,x0+1,y0+1);
            } else { // BICUBIC
                int x0 = (int)std::floor(fx);
                int y0 = (int)std::floor(fy);
                float tx = fx - x0, ty = fy - y0;

                float colw[4], roww[4];
                for (int k = 0; k < 4; k++) {
                    colw[k] = cubicKernel(tx - (k - 1));
                    roww[k] = cubicKernel(ty - (k - 1));
                }

                auto evalCh = [&](const std::vector<float>& ch) {
                    float acc = 0;
                    for (int j = 0; j < 4; j++) {
                        float row = 0;
                        int yy = y0 + (j - 1);
                        for (int i = 0; i < 4; i++) {
                            int xx = x0 + (i - 1);
                            row += colw[i] * sampleClamp(ch, src.w, src.h, xx, yy);
                        }
                        acc += roww[j] * row;
                    }
                    return acc;
                };
                vr = evalCh(src.r);
                vg = evalCh(src.g);
                vb = evalCh(src.b);
            }

            int idx = dy * dst.w + dx;
            dst.r[idx] = vr; dst.g[idx] = vg; dst.b[idx] = vb;
        }
    }
}

// ---------- Test image synthesis ----------
// Mode 0: smooth analytic field (sinusoidal gradients) -> where higher-order
//         interpolation is mathematically superior (ideal PSNR ranking).
// Mode 1: high-frequency field (checkerboard + sharp rings) -> stress test
//         for ringing/aliasing behaviour.
Image makeTestImage(int w, int h, int mode) {
    Image img; img.w = w; img.h = h;
    img.r.resize(w*h); img.g.resize(w*h); img.b.resize(w*h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float nx = (x - w/2) / (float)(w/2);
            float ny = (y - h/2) / (float)(h/2);
            float d = std::sqrt(nx*nx + ny*ny);
            float r, g, b;
            if (mode == 0) {
                // smooth: low-frequency sinusoids, analytically differentiable
                float fx = x / (float)w * 3.14159265f * 2.0f;
                float fy = y / (float)h * 3.14159265f * 2.0f;
                r = 255 * (0.5f + 0.5f*std::cos(fx));
                g = 255 * (0.5f + 0.5f*std::sin(fy));
                b = 255 * (0.5f + 0.5f*std::cos(fx*0.5f + fy*0.5f));
            } else {
                float grad = 0.5f + 0.5f * std::cos(d * 6.0f);
                float ring = (std::fabs(d - 0.5f) < 0.02f) ? 1.0f : 0.15f;
                float chk = ((x/4 + y/4) % 2) ? 0.9f : 0.1f;
                float mix = (d < 0.4f) ? chk : grad;
                r = 255 * (0.5f*grad + 0.5f*ring);
                g = 255 * (0.5f*mix  + 0.5f*grad);
                b = 255 * ring;
            }
            img.r[y*w+x] = std::min(255.0f, std::max(0.0f, r));
            img.g[y*w+x] = std::min(255.0f, std::max(0.0f, g));
            img.b[y*w+x] = std::min(255.0f, std::max(0.0f, b));
        }
    }
    return img;
}

// ---------- Metrics ----------
double mse(const Image& a, const Image& b) {
    // assume same dims
    double sum = 0; int n = a.w * a.h;
    for (int i = 0; i < n; i++) {
        double dr = a.r[i]-b.r[i], dg = a.g[i]-b.g[i], db = a.b[i]-b.b[i];
        sum += dr*dr + dg*dg + db*db;
    }
    return sum / (n * 3.0);
}

double psnr(double mseVal) {
    if (mseVal == 0) return 1e9;
    return 10.0 * std::log10(255.0*255.0 / mseVal);
}

// Edge sharpness: mean absolute Laplacian over a downsampled-intensity image (higher = sharper edges preserved)
double edgeSharpness(const Image& img) {
    double sum = 0; int cnt = 0;
    for (int y = 1; y < img.h - 1; y++) {
        for (int x = 1; x < img.w - 1; x++) {
            auto L = [&](int xx, int yy) { return 0.299f*img.r[yy*img.w+xx] + 0.587f*img.g[yy*img.w+xx] + 0.114f*img.b[yy*img.w+xx]; };
            double lap = 4*L(x,y) - L(x-1,y) - L(x+1,y) - L(x,y-1) - L(x,y+1);
            sum += std::fabs(lap); cnt++;
        }
    }
    return sum / cnt;
}

int main() {
    const int GTW = 256, GTH = 256, LOWW = 128, LOWH = 128;
    struct { const char* name; Filter f; } filters[] = {
        {"nearest", NEAREST}, {"bilinear", BILINEAR}, {"bicubic", BICUBIC}
    };

    bool globalOk = true;

    for (int mode = 0; mode < 2; mode++) {
        printf("========== Mode %d (%s) ==========\n", mode,
               mode == 0 ? "smooth analytic field" : "high-frequency stress field");

        Image gt = makeTestImage(GTW, GTH, mode);
        char gtfn[32]; snprintf(gtfn, sizeof(gtfn), "ground_truth_m%d.ppm", mode);
        savePPM(gtfn, gt);

        // Downsample via 2x2 box average
        Image low; low.w = LOWW; low.h = LOWH;
        low.r.resize(LOWW*LOWH); low.g.resize(LOWW*LOWH); low.b.resize(LOWW*LOWH);
        for (int y = 0; y < LOWH; y++) {
            for (int x = 0; x < LOWW; x++) {
                float ar=0,ag=0,ab=0;
                for (int j=0;j<2;j++) for (int i=0;i<2;i++) {
                    int sx = x*2+i, sy = y*2+j;
                    ar += gt.r[sy*GTW+sx]; ag += gt.g[sy*GTW+sx]; ab += gt.b[sy*GTW+sx];
                }
                low.r[y*LOWW+x] = ar/4; low.g[y*LOWW+x] = ag/4; low.b[y*LOWW+x] = ab/4;
            }
        }
        char lfn[32]; snprintf(lfn, sizeof(lfn), "low_res_m%d.ppm", mode);
        savePPM(lfn, low);

        printf("%-10s %14s %14s %14s\n", "Filter", "MSE", "PSNR(dB)", "EdgeSharp");
        double psnrVals[3], mseVals[3], sharpVals[3];
        for (int fi = 0; fi < 3; fi++) {
            Image up; upscale(low, up, filters[fi].f);
            double m = mse(gt, up);
            double p = psnr(m);
            double sharp = edgeSharpness(up);
            psnrVals[fi] = p; mseVals[fi] = m; sharpVals[fi] = sharp;
            printf("%-10s %14.4f %14.4f %14.4f\n", filters[fi].name, m, p, sharp);
            char fn[64]; snprintf(fn, sizeof(fn), "up_%s_m%d.ppm", filters[fi].name, mode);
            savePPM(fn, up);
        }

        double pN = psnrVals[0], pB = psnrVals[1], pC = psnrVals[2];
        double mB = mseVals[1], mC = mseVals[2];
        double sB = sharpVals[1], sC = sharpVals[2];
        bool ok = true;

        if (mode == 0) {
            // Smooth analytic field: interpolation (bilinear/bicubic) must vastly
            // outperform nearest-neighbour (big PSNR gap).
            if (!(pB > pN + 15.0 && pC > pN + 15.0)) {
                printf("❌ Smooth field: bilinear(%.4f)/bicubic(%.4f) not >> nearest(%.4f)\n", pB, pC, pN);
                ok = false;
            } else {
                printf("✅ Smooth field: bilinear(%.4f) & bicubic(%.4f) >> nearest(%.4f)  [gap > 15 dB]\n", pB, pC, pN);
            }
            // Bicubic preserves more edge sharpness than bilinear (less over-smoothing).
            if (!(sC > sB)) {
                printf("❌ Bicubic sharpness(%.4f) not > bilinear(%.4f)\n", sC, sB);
                ok = false;
            } else {
                printf("✅ Bicubic sharpness(%.4f) > bilinear(%.4f)  [less over-smoothing]\n", sC, sB);
            }
        } else {
            // Stress field (checkerboard + rings): nearest wins on PSNR (exact
            // preservation of quantized patterns), but bicubic has lower MSE than
            // bilinear (bilinear over-blurs halftone).
            if (!(mC < mB)) {
                printf("❌ Stress field: bicubic MSE(%.4f) not < bilinear(%.4f)\n", mC, mB);
                ok = false;
            } else {
                printf("✅ Stress field: bicubic MSE(%.4f) < bilinear(%.4f)  [less over-blur]\n", mC, mB);
            }
            if (!(sC > sB)) {
                printf("❌ Stress field: bicubic sharpness(%.4f) not > bilinear(%.4f)\n", sC, sB);
                ok = false;
            } else {
                printf("✅ Stress field: bicubic sharpness(%.4f) > bilinear(%.4f)\n", sC, sB);
            }
        }

        if (!(pC > 15.0)) { printf("❌ Bicubic PSNR too low (%.4f)\n", pC); ok = false; }

        printf("Mode %d: %s\n\n", mode, ok ? "PASS" : "FAIL");
        globalOk = globalOk && ok;
    }

    printf("%s\n", globalOk ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return globalOk ? 0 : 1;
}
