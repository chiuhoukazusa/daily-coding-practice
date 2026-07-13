#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
using namespace std;

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

inline double edge(double ax, double ay, double bx, double by, double cx, double cy) {
    return (cx - ax)*(by - ay) - (cy - ay)*(bx - ax);
}

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

int main() {
    Image img(800, 600);
    DepthBuf zbuf(800, 600);
    
    // CCW triangle: v0(100,100) v1(200,300) v2(300,100) — this should be CCW
    double sx0=100,sy0=100, sx1=200,sy1=300, sx2=300,sy2=100;
    
    double area = edge(sx0,sy0, sx1,sy1, sx2,sy2);
    printf("Area=%.1f\n", area);
    double invA = 1.0/area;
    
    int bx0=max(0,(int)min({sx0,sx1,sx2}));
    int bx1=min(799,(int)max({sx0,sx1,sx2}));
    int by0=max(0,(int)min({sy0,sy1,sy2}));
    int by1=min(599,(int)max({sy0,sy1,sy2}));
    
    int written=0, tested=0;
    for (int y=by0; y<=by1; y++) {
        for (int x=bx0; x<=bx1; x++) {
            tested++;
            double w0=edge(sx1,sy1, sx2,sy2, x+0.5, y+0.5);
            double w1=edge(sx2,sy2, sx0,sy0, x+0.5, y+0.5);
            double w2=edge(sx0,sy0, sx1,sy1, x+0.5, y+0.5);
            // Accept if all >= 0 (CCW) OR all < 0 (CW) — top-left fill rule
            if ((w0 >= 0 && w1 >= 0 && w2 >= 0)) {
                if (zbuf.write(x, y, 1.0)) {
                    img.set(x, y, 255, 0, 0);
                    written++;
                }
            }
        }
    }
    
    printf("Tested: %d  Written: %d\n", tested, written);
    img.save("test_output2.ppm");
    
    // Check the result
    FILE* ff=fopen("test_output2.ppm","rb"); fseek(ff,0,SEEK_END);
    printf("File size: %ld\n", ftell(ff)); fclose(ff);
    
    // Also check: our real quad vertices
    printf("\n=== Check real quad winding ===\n");
    double qx[4]={162,638,584,216};
    double qy[4]={317,317,564,564};
    for (int t=0; t<2; t++) {
        int i0=t?0:0, i1=t?2:1, i2=t?3:2;
        double a = edge(qx[i0],qy[i0], qx[i1],qy[i1], qx[i2],qy[i2]);
        printf("Tri %d (v%d-v%d-v%d): area=%.0f\n", t, i0,i1,i2, a);
    }
    return 0;
}
