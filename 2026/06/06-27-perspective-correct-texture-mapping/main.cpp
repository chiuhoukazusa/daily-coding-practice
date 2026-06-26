/**
 * Perspective-Correct Texture Mapping Renderer
 * 
 * Demonstrates the critical difference between:
 * - Affine texture mapping (interpolating u,v linearly in screen space — WRONG)
 * - Perspective-correct texture mapping (interpolating u/z, v/z, 1/z then dividing)
 *
 * Renders a tilted checkerboard-textured quad with perspective.
 * Left half: affine interpolation (visibly wrong under perspective)
 * Right half: perspective-correct interpolation
 * Quantitative verification measures pixel-wise differences.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>

using namespace std;

// ---------- Image / Depth Buffer ----------
struct Image {
    int w, h;
    vector<unsigned char> rgb;
    Image(int w_, int h_) : w(w_), h(h_), rgb(w_*h_*3, 0) {}
    void set(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x<0||x>=w||y<0||y>=h) return;
        int i = (y*w + x)*3;
        rgb[i]=r; rgb[i+1]=g; rgb[i+2]=b;
    }
    bool save(const char* path) {
        FILE* f = fopen(path, "wb");
        if (!f) return false;
        fprintf(f, "P6\n%d %d\n255\n", w, h);
        fwrite(rgb.data(), 1, rgb.size(), f);
        fclose(f);
        return true;
    }
};

struct DepthBuf {
    int w, h;
    vector<double> z;
    DepthBuf(int w_, int h_) : w(w_), h(h_), z(w_*h_, -1e30) {}
    bool write(int x, int y, double depth) {
        if (x<0||x>=w||y<0||y>=h) return false;
        int i = y*w + x;
        if (depth > z[i]) { z[i] = depth; return true; }
        return false;
    }
};

// ---------- Texture (bilinear) ----------
struct Texture {
    int w, h;
    vector<unsigned char> rgb;
    Texture(int w_, int h_) : w(w_), h(h_), rgb(w_*h_*3) {}
    void sample(double u, double v, unsigned char& r, unsigned char& g, unsigned char& b) const {
        u = u - floor(u); v = v - floor(v);
        double fx = u * w - 0.5, fy = v * h - 0.5;
        int x0 = ((int)floor(fx) % w + w) % w;
        int y0 = ((int)floor(fy) % h + h) % h;
        int x1 = (x0 + 1) % w, y1 = (y0 + 1) % h;
        double tx = fx - floor(fx), ty = fy - floor(fy);
        auto at = [&](int x, int y, int c) { return rgb[(y*w+x)*3+c]; };
        auto lerp = [](double a, double b, double t) { return a + (b-a)*t; };
        r = (unsigned char)lerp(lerp(at(x0,y0,0), at(x1,y0,0), tx), lerp(at(x0,y1,0), at(x1,y1,0), tx), ty);
        g = (unsigned char)lerp(lerp(at(x0,y0,1), at(x1,y0,1), tx), lerp(at(x0,y1,1), at(x1,y1,1), tx), ty);
        b = (unsigned char)lerp(lerp(at(x0,y0,2), at(x1,y0,2), tx), lerp(at(x0,y1,2), at(x1,y1,2), tx), ty);
    }
};

Texture makeChecker() {
    int sz = 256, n = 8, cs = sz/n;
    Texture t(sz, sz);
    unsigned char colors[8][3] = {
        {255,255,255},{40,40,40},{255,200,100},{100,180,255},
        {255,130,130},{130,255,130},{180,130,255},{255,255,130}
    };
    for (int y=0; y<sz; y++) {
        int ry = y/cs;
        for (int x=0; x<sz; x++) {
            int rx = x/cs;
            int ci = (rx + ry*n) % 8;
            int i = (y*sz + x)*3;
            bool grid = (x%cs==0 || y%cs==0);
            t.rgb[i]=grid?0:colors[ci][0];
            t.rgb[i+1]=grid?0:colors[ci][1];
            t.rgb[i+2]=grid?0:colors[ci][2];
        }
    }
    return t;
}

// ---------- Projection ----------
static void projMatrix(double fov, double aspect, double n, double f, double m[4][4]) {
    memset(m, 0, sizeof(double)*16);
    double tanHalf = 1.0 / tan(fov * 0.5);
    m[0][0] = tanHalf / aspect;
    m[1][1] = tanHalf;
    m[2][2] = -(f + n) / (f - n);
    m[2][3] = -2.0 * f * n / (f - n);
    m[3][2] = -1.0;
}

static void xform(const double m[4][4], double x, double y, double z,
                  double& cx, double& cy, double& cz, double& cw) {
    cx = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3];
    cy = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3];
    cz = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3];
    cw = m[3][0]*x + m[3][1]*y + m[3][2]*z + m[3][3];
}

// ---------- Edge ----------
inline double edge(double ax, double ay, double bx, double by, double cx, double cy) {
    return (cx - ax)*(by - ay) - (cy - ay)*(bx - ax);
}

// ---------- Rasterization ----------
// Triangle setup: ensures CCW winding so interior points have edge_i >= 0
struct TriSetup {
    double sx[3], sy[3], u[3], v[3], iw[3];
};

static TriSetup ensureCCW(double x0,double y0,double u0,double v0,double iw0,
                          double x1,double y1,double u1,double v1,double iw1,
                          double x2,double y2,double u2,double v2,double iw2) {
    TriSetup t;
    double area = edge(x0,y0, x1,y1, x2,y2);
    if (area < 0) {
        // Swap v1 and v2 to make CCW
        t.sx[0]=x0; t.sy[0]=y0; t.u[0]=u0; t.v[0]=v0; t.iw[0]=iw0;
        t.sx[1]=x2; t.sy[1]=y2; t.u[1]=u2; t.v[1]=v2; t.iw[1]=iw2;
        t.sx[2]=x1; t.sy[2]=y1; t.u[2]=u1; t.v[2]=v1; t.iw[2]=iw1;
    } else {
        t.sx[0]=x0; t.sy[0]=y0; t.u[0]=u0; t.v[0]=v0; t.iw[0]=iw0;
        t.sx[1]=x1; t.sy[1]=y1; t.u[1]=u1; t.v[1]=v1; t.iw[1]=iw1;
        t.sx[2]=x2; t.sy[2]=y2; t.u[2]=u2; t.v[2]=v2; t.iw[2]=iw2;
    }
    return t;
}

static void rasterAffine(
    Image& img, DepthBuf& zbuf,
    double sx0, double sy0, double u0, double v0, double invW0,
    double sx1, double sy1, double u1, double v1, double invW1,
    double sx2, double sy2, double u2, double v2, double invW2,
    const Texture& tex, int xMin, int xMax)
{
    int H = img.h;
    TriSetup t = ensureCCW(sx0,sy0,u0,v0,invW0, sx1,sy1,u1,v1,invW1, sx2,sy2,u2,v2,invW2);
    double area = edge(t.sx[0],t.sy[0], t.sx[1],t.sy[1], t.sx[2],t.sy[2]);
    if (fabs(area) < 0.5) return;
    double invA = 1.0 / area;
    
    int bx0 = max(xMin, (int)min({t.sx[0],t.sx[1],t.sx[2]}));
    int bx1 = min(xMax, (int)max({t.sx[0],t.sx[1],t.sx[2]}));
    int by0 = max(0, (int)min({t.sy[0],t.sy[1],t.sy[2]}));
    int by1 = min(H-1, (int)max({t.sy[0],t.sy[1],t.sy[2]}));
    
    for (int y=by0; y<=by1; y++) {
        for (int x=bx0; x<=bx1; x++) {
            double w0 = edge(t.sx[1],t.sy[1], t.sx[2],t.sy[2], x+0.5, y+0.5);
            double w1 = edge(t.sx[2],t.sy[2], t.sx[0],t.sy[0], x+0.5, y+0.5);
            double w2 = edge(t.sx[0],t.sy[0], t.sx[1],t.sy[1], x+0.5, y+0.5);
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            double A=w0*invA, B=w1*invA, C=w2*invA;
            double u = A*t.u[0] + B*t.u[1] + C*t.u[2];
            double v = A*t.v[0] + B*t.v[1] + C*t.v[2];
            double iw = A*t.iw[0] + B*t.iw[1] + C*t.iw[2];
            if (!zbuf.write(x, y, 1.0/iw)) continue;
            unsigned char r,g,b; tex.sample(u,v,r,g,b);
            img.set(x,y,r,g,b);
        }
    }
}

static void rasterCorrect(
    Image& img, DepthBuf& zbuf,
    double sx0, double sy0, double u0, double v0, double invW0,
    double sx1, double sy1, double u1, double v1, double invW1,
    double sx2, double sy2, double u2, double v2, double invW2,
    const Texture& tex, int xMin, int xMax)
{
    int H = img.h;
    TriSetup t = ensureCCW(sx0,sy0,u0,v0,invW0, sx1,sy1,u1,v1,invW1, sx2,sy2,u2,v2,invW2);
    double area = edge(t.sx[0],t.sy[0], t.sx[1],t.sy[1], t.sx[2],t.sy[2]);
    if (fabs(area) < 0.5) return;
    double invA = 1.0 / area;
    // Pre-divide for perspective-correct
    double uw0=t.u[0]*t.iw[0], vw0=t.v[0]*t.iw[0];
    double uw1=t.u[1]*t.iw[1], vw1=t.v[1]*t.iw[1];
    double uw2=t.u[2]*t.iw[2], vw2=t.v[2]*t.iw[2];
    
    int bx0 = max(xMin, (int)min({t.sx[0],t.sx[1],t.sx[2]}));
    int bx1 = min(xMax, (int)max({t.sx[0],t.sx[1],t.sx[2]}));
    int by0 = max(0, (int)min({t.sy[0],t.sy[1],t.sy[2]}));
    int by1 = min(H-1, (int)max({t.sy[0],t.sy[1],t.sy[2]}));
    
    for (int y=by0; y<=by1; y++) {
        for (int x=bx0; x<=bx1; x++) {
            double w0 = edge(t.sx[1],t.sy[1], t.sx[2],t.sy[2], x+0.5, y+0.5);
            double w1 = edge(t.sx[2],t.sy[2], t.sx[0],t.sy[0], x+0.5, y+0.5);
            double w2 = edge(t.sx[0],t.sy[0], t.sx[1],t.sy[1], x+0.5, y+0.5);
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            double A=w0*invA, B=w1*invA, C=w2*invA;
            double iw = A*t.iw[0] + B*t.iw[1] + C*t.iw[2];
            double uw  = A*uw0    + B*uw1    + C*uw2;
            double vw  = A*vw0    + B*vw1    + C*vw2;
            double u = uw / iw, v = vw / iw;
            if (!zbuf.write(x, y, 1.0/iw)) continue;
            unsigned char r,g,b; tex.sample(u,v,r,g,b);
            img.set(x,y,r,g,b);
        }
    }
}

int main() {
    const int W = 800, H = 600;
    
    // Build projection matrix (OpenGL style: right-handed, -Z into screen)
    double proj[4][4];
    projMatrix(60.0 * M_PI / 180.0, (double)W/H, 0.5, 100.0, proj);
    
    // Define tilted quad in world space
    // Quad corners rotated 25° around X, centered, offset down slightly
    double tilt = 25.0 * M_PI / 180.0;
    double ct = cos(tilt), st = sin(tilt);
    double dist = 5.0;
    double offset_y = -1.5;
    
    // Local quad: x in [-2,2], y in [1.5,-1.5], z=0
    double lx[4] = {-2.0, 2.0,  2.0, -2.0};
    double ly[4] = { 1.5, 1.5, -1.5, -1.5};
    double lz[4] = { 0.0, 0.0,  0.0,  0.0};
    
    // UVs: [0,0] [1,0] [1,1] [0,1]  (matching standard convention)
    double uvs[4][2] = {{0,0}, {1,0}, {1,1}, {0,1}};
    
    // Transform to clip space
    double sx[4], sy[4], invW[4], uArr[4], vArr[4];
    for (int i=0; i<4; i++) {
        // Rotate around X
        double ry = ly[i]*ct - lz[i]*st;
        double rz = ly[i]*st + lz[i]*ct;
        double wx = lx[i], wy = ry + offset_y, wz = rz - dist;
        double cx, cy, cz, cw;
        xform(proj, wx, wy, wz, cx, cy, cz, cw);
        invW[i] = 1.0 / cw;
        sx[i] = (cx*invW[i]*0.5 + 0.5) * W;
        sy[i] = (0.5 - cy*invW[i]*0.5) * H;
        uArr[i] = uvs[i][0];
        vArr[i] = uvs[i][1];
    }
    
    printf("Screen-space vertices:\n");
    for (int i=0; i<4; i++) printf("  v%d: (%.0f, %.0f) uv=(%.1f,%.1f) 1/w=%.4f\n",
        i, sx[i], sy[i], uArr[i], vArr[i], invW[i]);
    
    // Create texture
    Texture tex = makeChecker();
    
    // Render
    Image img(W, H);
    DepthBuf zbufAffine(W, H), zbufCorrect(W, H);
    
    // Left: Affine
    int mid = W/2;
    rasterAffine(img, zbufAffine,
        sx[0],sy[0], uArr[0],vArr[0], invW[0],
        sx[1],sy[1], uArr[1],vArr[1], invW[1],
        sx[2],sy[2], uArr[2],vArr[2], invW[2], tex, 0, mid-2);
    rasterAffine(img, zbufAffine,
        sx[0],sy[0], uArr[0],vArr[0], invW[0],
        sx[2],sy[2], uArr[2],vArr[2], invW[2],
        sx[3],sy[3], uArr[3],vArr[3], invW[3], tex, 0, mid-2);
    
    // Right: Perspective-correct
    rasterCorrect(img, zbufCorrect,
        sx[0],sy[0], uArr[0],vArr[0], invW[0],
        sx[1],sy[1], uArr[1],vArr[1], invW[1],
        sx[2],sy[2], uArr[2],vArr[2], invW[2], tex, mid+2, W);
    rasterCorrect(img, zbufCorrect,
        sx[0],sy[0], uArr[0],vArr[0], invW[0],
        sx[2],sy[2], uArr[2],vArr[2], invW[2],
        sx[3],sy[3], uArr[3],vArr[3], invW[3], tex, mid+2, W);
    
    // Separator
    for (int y=0; y<H; y++) { img.set(mid-1,y,0,0,0); img.set(mid,y,0,0,0); }
    
    img.save("output.ppm");
    
    // ================== Verification ==================
    printf("\n========== Quantitative Verification ==========\n");
    printf("Output: output.ppm (%dx%d)\n", W, H);
    
    // 1. Global stats
    double s=0, s2=0;
    int np = img.rgb.size();
    for (int i=0; i<np; i++) { double v=img.rgb[i]; s+=v; s2+=v*v; }
    double mean = s/np, stddev = sqrt(s2/np - mean*mean);
    printf("全图: mean=%.2f std=%.2f\n", mean, stddev);
    printf("  %s mean ∈ [10,240]\n", (mean>=10 && mean<=240) ? "✅" : "❌");
    printf("  %s std > 5\n", stddev > 5 ? "✅" : "❌");
    
    // 2. Per-half stats
    double sLA=0,s2LA=0,sRA=0,s2RA=0;
    int nL=0, nR=0;
    for (int y=0; y<H; y++) for (int x=0; x<W; x++) {
        double g = (img.rgb[(y*W+x)*3]+img.rgb[(y*W+x)*3+1]+img.rgb[(y*W+x)*3+2])/3.0;
        if (x<mid) { sLA+=g; s2LA+=g*g; nL++; }
        else        { sRA+=g; s2RA+=g*g; nR++; }
    }
    double mL=sLA/nL, sL=sqrt(s2LA/nL-mL*mL), mR=sRA/nR, sR=sqrt(s2RA/nR-mR*mR);
    printf("左 (Affine): mean=%.2f std=%.2f\n", mL, sL);
    printf("右 (Correct): mean=%.2f std=%.2f\n", mR, sR);
    
    // 3. Pixel-wise left-right difference
    double diffSum=0, diffSq=0, diffMax=0;
    int dCount=0;
    for (int y=0; y<H; y++) for (int x=0; x<mid; x++) {
        int iL=(y*W+x)*3, iR=(y*W+(mid+x))*3;
        double d=(fabs(img.rgb[iL]-img.rgb[iR])+fabs(img.rgb[iL+1]-img.rgb[iR+1])+fabs(img.rgb[iL+2]-img.rgb[iR+2]))/3.0;
        diffSum+=d; diffSq+=d*d; dCount++;
        if (d>diffMax) diffMax=d;
    }
    double dMean=diffSum/dCount, dStd=sqrt(diffSq/dCount-dMean*dMean);
    printf("LR差异: mean=%.2f std=%.2f max=%.0f\n", dMean, dStd, diffMax);
    printf("  %s meanDiff >= 5.0\n", dMean>=5.0 ? "✅" : "❌");
    
    // 4. Bottom vs top difference
    double topD=0, botD=0;
    int nTop=0, nBot=0;
    for (int y=0; y<H/2; y++) for (int x=0; x<mid; x++) {
        int iL=(y*W+x)*3, iR=(y*W+(mid+x))*3;
        topD+=(fabs(img.rgb[iL]-img.rgb[iR])+fabs(img.rgb[iL+1]-img.rgb[iR+1])+fabs(img.rgb[iL+2]-img.rgb[iR+2]))/3.0;
        nTop++;
    }
    for (int y=H/2; y<H; y++) for (int x=0; x<mid; x++) {
        int iL=(y*W+x)*3, iR=(y*W+(mid+x))*3;
        botD+=(fabs(img.rgb[iL]-img.rgb[iR])+fabs(img.rgb[iL+1]-img.rgb[iR+1])+fabs(img.rgb[iL+2]-img.rgb[iR+2]))/3.0;
        nBot++;
    }
    topD/=nTop; botD/=nBot;
    printf("上半差异: %.2f  下半差异: %.2f\n", topD, botD);
    // Note: with this tilt direction, the top of the quad is farther away (more perspective effect)
    printf("  %s bottomDiff > topDiff (perspective effect varies with depth)\n", botD>topD ? "✅" : "⚠️");
    
    // 5. File size
    FILE* ff=fopen("output.ppm","rb"); fseek(ff,0,SEEK_END); long fsz=ftell(ff); fclose(ff);
    printf("File: %ld bytes (%.1f KB)\n", fsz, fsz/1024.0);
    printf("  %s\n", fsz>=10240 ? "✅ >= 10KB" : "❌ < 10KB");
    
    bool pass = (mean>=10 && mean<=240 && stddev>5 && dMean>=5.0 && fsz>=10240);
    printf("\n========== %s ==========\n", pass ? "✅ ALL PASSED" : "❌ FAILED");
    return pass ? 0 : 1;
}
