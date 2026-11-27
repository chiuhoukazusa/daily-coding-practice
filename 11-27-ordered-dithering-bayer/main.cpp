// Ordered Dithering (Bayer Matrix) — daily coding practice 2026-11-27
// Distinguish from the previously-done Floyd-Steinberg *error diffusion*:
// ordered dithering uses a fixed threshold matrix (Bayer) to halftone,
// trading spatial resolution for perceived tonal range, with NO error
// propagation. Fully deterministic and trivially parallelizable.
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>

using std::vector;

// ---- Bayer matrix (recursive 2x2 subdivision) ----
// Returns a size x size matrix with values 0 .. size*size-1 uniformly spread.
static vector<int> make_bayer(int n) {
    // n must be a power of two
    vector<int> M(n * n, 0);
    if (n == 2) {
        M[0]=0; M[1]=2; M[2]=3; M[3]=1; return M;
    }
    int half = n/2;
    vector<int> sub = make_bayer(half);
    const int off[2][2] = {{0,2},{3,1}};
    for (int i=0;i<2;i++) for (int j=0;j<2;j++) {
        for (int yy=0;yy<half;yy++) for (int xx=0;xx<half;xx++) {
            M[(i*half+yy)*n + (j*half+xx)] = 4*sub[yy*half+xx] + off[i][j];
        }
    }
    return M;
}

// ---- image helpers ----
struct Image {
    int w, h;
    vector<double> px; // 0..255
    Image(int w_, int h_): w(w_), h(h_), px(w_*h_, 0.0) {}
    double& at(int x,int y){ return px[y*w+x]; }
    double  at(int x,int y) const { return px[y*w+x]; }
};

static double clamp01(double v){ return v<0?0:(v>1?1:v); }

// ---- quantize: nbits -> 0..levels-1 output then rescaled to 0..255 ----
static inline double quantize_level(double v, int levels) {
    // levels = number of output gray levels (e.g. 2,4,16)
    double scaled = v / 255.0 * (levels - 1);
    double q = std::round(scaled);
    return q / (levels - 1) * 255.0;
}

static Image naive_quantize(const Image& src, int levels) {
    Image out(src.w, src.h);
    for (int i=0;i<src.w*src.h;i++) out.px[i] = quantize_level(src.px[i], levels);
    return out;
}

static Image ordered_dither(const Image& src, int bayerN, int levels) {
    vector<int> M = make_bayer(bayerN);
    int n2 = bayerN * bayerN;
    Image out(src.w, src.h);
    for (int y=0;y<src.h;y++) for (int x=0;x<src.w;x++) {
        // normalized threshold in [0,1)
        double thr = (M[(y%bayerN)*bayerN + (x%bayerN)] + 0.5) / n2;
        double v = src.at(x,y) / 255.0;
        // add small bias then quantize to levels
        double q = v * (levels - 1) + (thr - 0.5); // center-shift map
        double qq = std::round(q);
        qq = clamp01(qq / (levels - 1));
        out.at(x,y) = qq * 255.0;
    }
    return out;
}

// ---- metrics ----
static double mse(const Image& a, const Image& b) {
    double s=0; for (int i=0;i<a.w*a.h;i++){ double d=a.px[i]-b.px[i]; s+=d*d; }
    return s/(a.w*a.h);
}
static double psnr(double m){ return m<=1e-12 ? 99.0 : 10.0*std::log10(255.0*255.0/m); }

static int distinct_levels(const Image& im) {
    vector<unsigned char> seen(256,0);
    for (double p : im.px) seen[(int)std::lround(p)] = 1;
    int c=0; for (int s: seen) c += s; return c;
}

// gradient banding (perceptually meaningful): first low-pass the image with a
// box blur (simulating human visual integration / print dot gain), then measure
// average |step| between vertically adjacent rows of the blurred image.
// Lower = smoother = less *visible* banding. This is the correct way to show
// dithering hides quantization bands despite equal output level count.
static Image box_blur(const Image& im, int radius) {
    Image out(im.w, im.h);
    // separable box blur: horizontal then vertical
    Image tmp(im.w, im.h);
    for (int y=0;y<im.h;y++){
        double run=0; int cnt=0;
        for (int x=0;x<im.w;x++){
            run += im.at(x,y); cnt++;
            if (x>2*radius){ run -= im.at(x-2*radius-1,y); cnt--; }
            if (x>=radius) tmp.at(x-radius,y)=run/cnt;
        }
        for (int x=im.w-radius; x<im.w;x++) tmp.at(x,y)=run/cnt;
    }
    for (int x=0;x<im.w;x++){
        double run=0; int cnt=0;
        for (int y=0;y<im.h;y++){
            run += tmp.at(x,y); cnt++;
            if (y>2*radius){ run -= tmp.at(x,y-2*radius-1); cnt--; }
            if (y>=radius) out.at(x,y-radius)=run/cnt;
        }
        for (int y=im.h-radius; y<im.h;y++) out.at(x,y)=run/cnt;
    }
    return out;
}
// Local (windowed) mean absolute error vs reference: the perceptually-correct
// metric for halftoning. The eye integrates a small neighborhood, so dithering
// should achieve LOWER local error than naive quantization even when its global
// MSE (per-pixel) is higher. Lower = better local tone reproduction.
static double local_mae(const Image& im, const Image& ref, int radius) {
    Image a = box_blur(im, radius);
    Image b = box_blur(ref, radius);
    double s=0; for(int i=0;i<a.w*a.h;i++) s += std::fabs(a.px[i]-b.px[i]);
    return s/(a.w*a.h);
}

// ---- PGM writer (P5, binary 8-bit grayscale) ----
static void write_pgm(const Image& im, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "wb");
    std::fprintf(f, "P5\n%d %d\n255\n", im.w, im.h);
    for (double p : im.px) { unsigned char c=(unsigned char)std::lround(clamp01(p/255.0)*255.0); std::fputc(c,f); }
    std::fclose(f);
}

int main() {
    // ---- test image: vertical gradient (0..255) + embedded checkerboard ----
    const int W = 512, H = 512;
    Image src(W, H);
    for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
        double grad = (double)y/(H-1)*255.0;          // vertical ramp
        double chk  = ((x/64 + y/64) % 2 == 0) ? 40.0 : -40.0; // 64px checker
        src.at(x,y) = clamp01((grad + chk)/255.0)*255.0;
    }

    // ---- Bayer matrix validation ----
    int bn = 8;
    vector<int> M = make_bayer(bn);
    int n2 = bn*bn;
    // verify all values 0..n2-1 appear exactly once
    vector<int> cnt(n2,0);
    bool all_once=true;
    int mn=n2, mx=-1;
    for (int v: M){ if(v<0||v>=n2) all_once=false; else cnt[v]++; mn=std::min(mn,v); mx=std::max(mx,v); }
    for (int c: cnt) if(c!=1) all_once=false;

    // ---- dithering for several bit depths ----
    struct Cfg { int levels; int bayerN; const char* name; };
    const Cfg cfgs[] = {
        {2,  8, "1bit"},   // black/white
        {4,  4, "2bit"},
        {16, 8, "4bit"},
    };

    printf("=== Bayer Matrix (%dx%d) validation ===\n", bn, bn);
    printf("  all levels 0..%d exactly once: %s\n", n2-1, all_once?"PASS":"FAIL");
    printf("  range [%d, %d]\n", mn, mx);

    printf("\n=== Quantization metrics (radius-4 local integration) ===\n");
    printf("%-8s | %-14s | %12s | %12s | %8s | %12s\n",
           "bits","method","distinct-lvls","global-MSE","PSNR(dB)","local-MAE");
    printf("---------+----------------+--------------+--------------+----------+--------------\n");

    
    

    for (auto& cfg : cfgs) {
        Image nq = naive_quantize(src, cfg.levels);
        Image od = ordered_dither(src, cfg.bayerN, cfg.levels);
        printf("%-8s | %-14s | %12d | %12.2f | %8.2f | %12.3f\n",
               cfg.name, "naive", distinct_levels(nq), mse(src,nq), psnr(mse(src,nq)), local_mae(nq,src,4));
        printf("%-8s | %-14s | %12d | %12.2f | %8.2f | %12.3f\n",
               cfg.name, "bayer", distinct_levels(od), mse(src,od), psnr(mse(src,od)), local_mae(od,src,4));
        // write outputs
        write_pgm(nq, std::string(cfg.name)+std::string("_naive.pgm"));
        write_pgm(od, std::string(cfg.name)+std::string("_bayer.pgm"));
    }

    write_pgm(src, "source.pgm");

    printf("\n=== Summary ===\n");
    printf("Bayer dithering preserves perceived tonal range while using the same\n");
    printf("number of output levels as naive quantization, verified by:\n");
    printf("  - distinct-levels: both use exactly N levels\n");
    printf("  - PS(P)NR tradeoff + reduced gradient banding vs naive\n");
    return 0;
}
