// Median Filter Denoising — Daily Coding Practice 2026-07-21
// Implements: median filter with configurable window, Gaussian blur (comparison),
//              salt & pepper noise, PSNR, noise removal rate, edge preservation
// Focus: clean demonstration of median filter superiority for impulse noise

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <vector>

constexpr int WIDTH = 512;
constexpr int HEIGHT = 512;
constexpr int MAX_COLOR = 255;

struct Image {
    int w, h;
    std::vector<uint8_t> r, g, b;
    Image(int ww, int hh) : w(ww), h(hh), r(w*h), g(w*h), b(w*h) {}

    void setPixel(int x, int y, uint8_t vr, uint8_t vg, uint8_t vb) {
        if ((unsigned)x>=(unsigned)w || (unsigned)y>=(unsigned)h) return;
        int idx = y*w + x;
        r[idx]=vr; g[idx]=vg; b[idx]=vb;
    }
    void getPixel(int x, int y, uint8_t &vr, uint8_t &vg, uint8_t &vb) const {
        int idx = y*w + x;
        vr=r[idx]; vg=g[idx]; vb=b[idx];
    }
    Image clone() const { Image out(w,h); out.r=r; out.g=g; out.b=b; return out; }

    void savePPM(const char* fn) const {
        FILE* f = fopen(fn, "wb");
        if (!f) { fprintf(stderr,"Cannot open %s\n",fn); return; }
        fprintf(f,"P6\n%d %d\n%d\n",w,h,MAX_COLOR);
        for (int y=0;y<h;y++) for (int x=0;x<w;x++) {
            uint8_t p[3]; getPixel(x,y,p[0],p[1],p[2]); fwrite(p,1,3,f);
        }
        fclose(f);
        printf("Saved: %s (%d bytes)\n", fn, w*h*3+20);
    }
};

// Generate a test image with large uniform regions and thick edges
Image generateTestImage() {
    Image img(WIDTH, HEIGHT);

    // Background: 4 quadrants of solid color (large uniform regions)
    for (int y=0; y<HEIGHT; y++) {
        for (int x=0; x<WIDTH; x++) {
            if (x < WIDTH/2 && y < HEIGHT/2)
                img.setPixel(x, y, 50, 50, 80);         // dark blue
            else if (x >= WIDTH/2 && y < HEIGHT/2)
                img.setPixel(x, y, 200, 180, 60);       // gold
            else if (x < WIDTH/2 && y >= HEIGHT/2)
                img.setPixel(x, y, 60, 160, 80);        // green
            else
                img.setPixel(x, y, 180, 70, 70);        // red
        }
    }

    // White square (top-left region), thick border
    for (int y=30; y<200; y++)
        for (int x=30; x<200; x++)
            img.setPixel(x, y, 255, 255, 255);

    // Black square (right region)
    for (int y=280; y<460; y++)
        for (int x=300; x<480; x++)
            img.setPixel(x, y, 0, 0, 0);

    // Thick horizontal and vertical bars (structural edges)
    for (int y=210; y<230; y++)
        for (int x=40; x<470; x++)
            img.setPixel(x, y, 255, 200, 50);

    for (int x=230; x<250; x++)
        for (int y=40; y<280; y++)
            img.setPixel(x, y, 50, 255, 200);

    // Small colored dots (to test detail preservation)
    for (int i=0; i<20; i++) {
        int cx = 50 + (i*23) % 400;
        int cy = 320 + (i*17) % 160;
        for (int dy=-3; dy<=3; dy++)
            for (int dx=-3; dx<=3; dx++)
                if (dx*dx+dy*dy <= 9)
                    img.setPixel(cx+dx, cy+dy, 255, 255, 0);
    }

    return img;
}

// Salt & pepper noise — replaces pixels with pure 0 or 255
Image addSaltPepper(const Image& src, double density) {
    Image out = src.clone();
    int total = out.w * out.h;
    int noisePixels = (int)(total * density);
    srand(12345);
    for (int i=0; i<noisePixels; i++) {
        int idx = rand() % total;
        out.r[idx] = out.g[idx] = out.b[idx] = (rand()&1) ? 255 : 0;
    }
    return out;
}

// Median filter — replaces each pixel with median of its window
Image medianFilter(const Image& src, int window) {
    Image out(src.w, src.h);
    int r2 = window/2;
    int area = window*window;
    std::vector<uint8_t> rv(area), gv(area), bv(area);

    for (int y=0; y<src.h; y++) {
        for (int x=0; x<src.w; x++) {
            int n = 0;
            for (int dy=-r2; dy<=r2; dy++) {
                int ny = y+dy;
                if ((unsigned)ny>=(unsigned)src.h) continue;
                for (int dx=-r2; dx<=r2; dx++) {
                    int nx = x+dx;
                    if ((unsigned)nx>=(unsigned)src.w) continue;
                    int idx = ny*src.w + nx;
                    rv[n]=src.r[idx]; gv[n]=src.g[idx]; bv[n]=src.b[idx];
                    n++;
                }
            }
            int mid=n/2;
            std::nth_element(rv.begin(), rv.begin()+mid, rv.begin()+n);
            std::nth_element(gv.begin(), gv.begin()+mid, gv.begin()+n);
            std::nth_element(bv.begin(), bv.begin()+mid, bv.begin()+n);
            out.setPixel(x, y, rv[mid], gv[mid], bv[mid]);
        }
    }
    return out;
}

// Gaussian blur (separable: horizontal then vertical)
Image gaussianBlur(const Image& src, double sigma) {
    int r = std::max(1, (int)(3.0*sigma));
    std::vector<double> kernel(2*r+1);
    double sum=0;
    for (int i=-r; i<=r; i++) { kernel[i+r]=exp(-i*i/(2*sigma*sigma)); sum+=kernel[i+r]; }
    for (auto& v:kernel) v/=sum;

    // horizontal
    Image tmp(src.w, src.h);
    for (int y=0; y<src.h; y++)
        for (int x=0; x<src.w; x++) {
            double vr=0,vg=0,vb=0;
            for (int dx=-r; dx<=r; dx++) {
                int nx=std::clamp(x+dx,0,src.w-1);
                int idx=y*src.w+nx;
                double w=kernel[dx+r];
                vr+=src.r[idx]*w; vg+=src.g[idx]*w; vb+=src.b[idx]*w;
            }
            tmp.setPixel(x,y,(uint8_t)std::clamp((int)(vr+0.5),0,255),
                              (uint8_t)std::clamp((int)(vg+0.5),0,255),
                              (uint8_t)std::clamp((int)(vb+0.5),0,255));
        }
    // vertical
    Image out(src.w, src.h);
    for (int y=0; y<src.h; y++)
        for (int x=0; x<src.w; x++) {
            double vr=0,vg=0,vb=0;
            for (int dy=-r; dy<=r; dy++) {
                int ny=std::clamp(y+dy,0,src.h-1);
                int idx=ny*src.w+x;
                double w=kernel[dy+r];
                vr+=tmp.r[idx]*w; vg+=tmp.g[idx]*w; vb+=tmp.b[idx]*w;
            }
            out.setPixel(x,y,(uint8_t)std::clamp((int)(vr+0.5),0,255),
                              (uint8_t)std::clamp((int)(vg+0.5),0,255),
                              (uint8_t)std::clamp((int)(vb+0.5),0,255));
        }
    return out;
}

// PSNR
double computePSNR(const Image& a, const Image& b) {
    double mse=0;
    int n=a.w*a.h;
    for (int i=0; i<n; i++) {
        double dr=(double)a.r[i]-b.r[i], dg=(double)a.g[i]-b.g[i], db=(double)a.b[i]-b.b[i];
        mse+=(dr*dr+dg*dg+db*db)/3.0;
    }
    mse/=n;
    if (mse<1e-10) return 100.0;
    return 10.0*log10(255.0*255.0/mse);
}

// Noise removal rate: fraction of noise pixels that are "corrected" (closer to original)
struct NRStat { double density, removed; };
NRStat noiseRemoval(const Image& clean, const Image& noisy, const Image& filt) {
    int n=clean.w*clean.h, noiseCount=0, removed=0;
    for (int i=0; i<n; i++) {
        bool isNoise = false;
        if (noisy.r[i]==0 && clean.r[i]!=0) isNoise=true;
        if (noisy.r[i]==255 && clean.r[i]!=255) isNoise=true;
        if (!isNoise) continue;
        noiseCount++;
        int dFilt=abs((int)filt.r[i]-(int)clean.r[i]);
        int dNoisy=abs((int)noisy.r[i]-(int)clean.r[i]);
        if (dFilt<dNoisy) removed++;
    }
    NRStat s;
    s.density=(double)noiseCount/n;
    s.removed=noiseCount>0?(double)removed/noiseCount:1.0;
    return s;
}

// Edge preservation: Pearson correlation of gradient magnitudes between clean and filtered
// Uses Sobel operator on luminance channel
double edgePreservation(const Image& clean, const Image& filt) {
    auto lum = [](uint8_t r,uint8_t g,uint8_t b){return 0.299*r+0.587*g+0.114*b;};
    auto sobel = [&](const Image& im, int x, int y)->double {
        if (x<=0||x>=im.w-1||y<=0||y>=im.h-1) return 0;
        int tl=(y-1)*im.w+(x-1), tc=(y-1)*im.w+x, tr=(y-1)*im.w+(x+1);
        int ml=y*im.w+(x-1),                   mr=y*im.w+(x+1);
        int bl=(y+1)*im.w+(x-1), bc=(y+1)*im.w+x, br=(y+1)*im.w+(x+1);
        double gx = -lum(im.r[tl],im.g[tl],im.b[tl]) -2*lum(im.r[tc],im.g[tc],im.b[tc]) -lum(im.r[tr],im.g[tr],im.b[tr])
                    + lum(im.r[bl],im.g[bl],im.b[bl]) +2*lum(im.r[bc],im.g[bc],im.b[bc]) +lum(im.r[br],im.g[br],im.b[br]);
        double gy = -lum(im.r[tl],im.g[tl],im.b[tl]) -2*lum(im.r[ml],im.g[ml],im.b[ml]) -lum(im.r[bl],im.g[bl],im.b[bl])
                    + lum(im.r[tr],im.g[tr],im.b[tr]) +2*lum(im.r[mr],im.g[mr],im.b[mr]) +lum(im.r[br],im.g[br],im.b[br]);
        return sqrt(gx*gx+gy*gy);
    };

    double sO=0,sF=0,sOF=0,sO2=0,sF2=0;
    int cnt=0;
    for (int y=2; y<clean.h-2; y++) {
        for (int x=2; x<clean.w-2; x++) {
            double go=sobel(clean,x,y), gf=sobel(filt,x,y);
            sO+=go; sF+=gf; sOF+=go*gf; sO2+=go*go; sF2+=gf*gf;
            cnt++;
        }
    }
    double mO=sO/cnt, mF=sF/cnt;
    double num=sOF-cnt*mO*mF, den=sqrt((sO2-cnt*mO*mO)*(sF2-cnt*mF*mF));
    return den<1e-10?0:num/den;
}

// Variance in a rectangular region (uniformity check)
double regionVariance(const Image& im, int x0, int y0, int rw, int rh) {
    double s=0,s2=0; int n=0;
    for (int y=y0; y<y0+rh && y<im.h; y++)
        for (int x=x0; x<x0+rw && x<im.w; x++) {
            int idx=y*im.w+x;
            double L=0.299*im.r[idx]+0.587*im.g[idx]+0.114*im.b[idx];
            s+=L; s2+=L*L; n++;
        }
    if (n==0) return 0;
    double m=s/n;
    return s2/n - m*m;
}

// Count remaining noise pixels in filtered image (pixels still at 0 or 255 that shouldn't be)
int countResidualNoise(const Image& clean, const Image& filt) {
    int n=clean.w*clean.h, residual=0;
    for (int i=0; i<n; i++) {
        if (clean.r[i]!=0 && filt.r[i]==0) residual++;
        if (clean.r[i]!=255 && filt.r[i]==255) residual++;
    }
    return residual;
}

int main() {
    printf("=== Median Filter Denoising — 2026-07-21 ===\n\n");

    // 1. Generate clean test image
    Image clean = generateTestImage();
    clean.savePPM("original.ppm");
    printf("Clean image: %dx%d, 4-quadrant solid colors + shapes\n", WIDTH, HEIGHT);

    // 2. Add 15% salt & pepper noise
    double noiseDensity = 0.15;
    Image noisy = addSaltPepper(clean, noiseDensity);
    noisy.savePPM("noisy.ppm");
    printf("Salt & pepper noise %d%%: PSNR=%.2f dB\n", (int)(noiseDensity*100), computePSNR(clean,noisy));

    // 3. Median filter variants
    printf("\n--- Median Filter ---\n");
    int windows[] = {3, 5};
    double bestMedianPSNR = 0;
    Image bestMedian = noisy;
    for (int ws : windows) {
        Image f = medianFilter(noisy, ws);
        char fn[64]; snprintf(fn,64,"median_%dx%d.ppm",ws,ws);
        f.savePPM(fn);
        double psnr = computePSNR(clean, f);
        double ep = edgePreservation(clean, f);
        auto ns = noiseRemoval(clean, noisy, f);
        int residual = countResidualNoise(clean, f);
        printf("  %dx%d: PSNR=%.2fdB  NoiseRemoved=%.1f%%  Residual=%d  EdgePres=%.3f\n",
               ws,ws,psnr,ns.removed*100,residual,ep);
        if (psnr > bestMedianPSNR) { bestMedianPSNR=psnr; bestMedian=f; }
    }

    // 4. Gaussian blur for comparison
    printf("\n--- Gaussian Blur (comparison) ---\n");
    double sigmas[] = {0.8, 1.5, 2.5};
    double bestGaussPSNR = 0;
    Image bestGauss = noisy;
    for (double s : sigmas) {
        Image f = gaussianBlur(noisy, s);
        char fn[64]; snprintf(fn,64,"gaussian_s%.1f.ppm",s);
        f.savePPM(fn);
        double psnr = computePSNR(clean, f);
        double ep = edgePreservation(clean, f);
        auto ns = noiseRemoval(clean, noisy, f);
        int residual = countResidualNoise(clean, f);
        printf("  σ=%.1f:   PSNR=%.2fdB  NoiseRemoved=%.1f%%  Residual=%d  EdgePres=%.3f\n",
               s,psnr,ns.removed*100,residual,ep);
        if (psnr > bestGaussPSNR) { bestGaussPSNR=psnr; bestGauss=f; }
    }

    // ========== QUANTITATIVE VERIFICATION ==========
    printf("\n============= QUANTITATIVE VERIFICATION =============\n");

    double psnrNoisy = computePSNR(clean, noisy);
    double psnrMed = computePSNR(clean, bestMedian);
    double psnrGau = computePSNR(clean, bestGauss);
    auto nsMed = noiseRemoval(clean, noisy, bestMedian);
    auto nsGau = noiseRemoval(clean, noisy, bestGauss);
    double epMed = edgePreservation(clean, bestMedian);
    double epGau = edgePreservation(clean, bestGauss);
    double rvClean = regionVariance(clean, 30, 30, 170, 170);
    double rvNoisy = regionVariance(noisy, 30, 30, 170, 170);
    double rvMed   = regionVariance(bestMedian, 30, 30, 170, 170);
    double rvGau   = regionVariance(bestGauss, 30, 30, 170, 170);

    int checks=0, passed=0;
    auto check = [&](bool ok, const char* desc) {
        checks++; if (ok) passed++;
        printf("    %s %s\n", ok?"✅":"❌", desc);
    };

    printf("\n[1] PSNR Improvement over noisy:\n");
    printf("    Noisy:      %.2f dB\n", psnrNoisy);
    printf("    Median 3x3: %.2f dB  (+%.1f dB)\n", psnrMed, psnrMed-psnrNoisy);
    printf("    Gaussian:   %.2f dB  (+%.1f dB)\n", psnrGau, psnrGau-psnrNoisy);
    check(psnrMed > psnrNoisy + 2.0, "Median PSNR > noisy + 2 dB");

    printf("\n[2] Noise Removal Rate:\n");
    printf("    Median:  %.1f%%\n", nsMed.removed*100);
    printf("    Gaussian: %.1f%%\n", nsGau.removed*100);
    check(nsMed.removed > 0.90, "Median noise removal > 90%");

    printf("\n[3] Edge Preservation (correlation with clean edges):\n");
    printf("    Median:  %.4f\n", epMed);
    printf("    Gaussian: %.4f\n", epGau);
    check(epMed > epGau, "Median edge preservation > Gaussian");

    printf("\n[4] Uniform Region Restoration (white square, target: ~0):\n");
    printf("    Clean:    %.1f\n", rvClean);
    printf("    Noisy:    %.1f\n", rvNoisy);
    printf("    Median:   %.1f  (%.1f%% of noisy)\n", rvMed, rvMed/rvNoisy*100);
    printf("    Gaussian: %.1f  (%.1f%% of noisy)\n", rvGau, rvGau/rvNoisy*100);
    check(rvMed < rvGau * 0.5, "Median uniform region variance < 50% of Gaussian");

    printf("\n[5] Median advantages over Gaussian for impulse noise:\n");
    printf("    ± Noise Removal:  Median+(%.1f%%)  Gaussian+(%.1f%%)\n",
           nsMed.removed*100, nsGau.removed*100);
    printf("    ± Edges:          Median+(%.4f)   Gaussian+(%.4f)\n", epMed, epGau);
    printf("    ± Uniform Areas:  Median+(%.1f)   Gaussian+(%.1f) (variance, lower=better)\n", rvMed, rvGau);
    check(nsMed.removed > nsGau.removed, "Median removes more noise than Gaussian");
    check(rvMed < rvGau, "Median preserves uniform regions better than Gaussian");

    printf("\n[6] Output Files:\n");
    const char* files[] = {
        "original.ppm","noisy.ppm",
        "median_3x3.ppm","median_5x5.ppm",
        "gaussian_s0.8.ppm","gaussian_s1.5.ppm","gaussian_s2.5.ppm"
    };
    int nf = sizeof(files)/sizeof(files[0]);
    for (int i=0; i<nf; i++) {
        FILE* fp=fopen(files[i],"rb");
        if (fp) {
            fseek(fp,0,SEEK_END); long sz=ftell(fp); fclose(fp);
            bool ok=sz>10000;
            if (!ok) check(false, "File too small");
            printf("    %s %s (%ld bytes)\n", ok?"✅":"❌", files[i], sz);
        } else {
            printf("    ❌ %s MISSING\n", files[i]);
            check(false, "Missing file");
        }
    }

    // Also quantitatively check pixel statistics
    printf("\n[7] Pixel Statistics (original.ppm):\n");
    {
        // Compute mean/std directly from clean image
        double rMean=0,gMean=0,bMean=0,rM2=0,gM2=0,bM2=0;
        int n=clean.w*clean.h;
        for (int i=0;i<n;i++) {
            rMean+=clean.r[i]; gMean+=clean.g[i]; bMean+=clean.b[i];
        }
        rMean/=n; gMean/=n; bMean/=n;
        for (int i=0;i<n;i++) {
            double dr=clean.r[i]-rMean,dg=clean.g[i]-gMean,db=clean.b[i]-bMean;
            rM2+=dr*dr; gM2+=dg*dg; bM2+=db*db;
        }
        double rStd=sqrt(rM2/n), gStd=sqrt(gM2/n), bStd=sqrt(bM2/n);
        printf("    Mean RGB: (%.1f, %.1f, %.1f)  Std RGB: (%.1f, %.1f, %.1f)\n", rMean,gMean,bMean,rStd,gStd,bStd);
        bool brightOK = rMean>5 && rMean<250;
        bool varOK   = rStd>20;
        check(brightOK, "Mean not all-black/all-white");
        check(varOK,   "Std deviation > 20 (diverse content)");
    }

    printf("\n==================================================\n");
    printf("  VERDICT: %s  (%d/%d checks passed)\n",
           passed==checks?"✅ ALL PASSED":"❌ FAILURES",
           passed, checks);
    printf("==================================================\n");

    return passed==checks ? 0 : 1;
}
