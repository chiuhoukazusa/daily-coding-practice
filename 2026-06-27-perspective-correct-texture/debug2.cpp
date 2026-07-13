#include <cmath>
#include <cstdio>
#include <cstring>

void perspective(double fov, double aspect, double n, double f, double m[4][4]) {
    memset(m, 0, sizeof(double)*16);
    double t = 1.0 / tan(fov * 0.5);
    m[0][0] = t / aspect;
    m[1][1] = t;
    m[2][2] = (f + n) / (n - f);
    m[2][3] = 2.0 * f * n / (n - f);
    m[3][2] = -1.0;
}

void transform(const double m[4][4], double x, double y, double z,
               double& cx, double& cy, double& cz, double& cw) {
    cx = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3];
    cy = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3];
    cz = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3];
    cw = m[3][0]*x + m[3][1]*y + m[3][2]*z + m[3][3];
}

int main() {
    int W=800, H=600;
    double proj[4][4];
    perspective(60.0 * M_PI / 180.0, (double)W/H, 0.5, 100.0, proj);
    
    double tilt_degs[] = {30.0, 40.0, 50.0, 60.0};
    for (int ti=0; ti<4; ti++) {
        double tilt_deg = tilt_degs[ti];
        printf("\n=== Tilt = %.0f deg ===\n", tilt_deg);
        double size=1.8, dist=5.0;
        double tilt=tilt_deg * M_PI / 180.0;
        double ct=cos(tilt), st=sin(tilt);
        
        double wx[4]={-size,size,size,-size};
        double wy[4]={size,size,-size,-size};
        double wz[4]={-dist,-dist,-dist,-dist};
        
        bool all_ok = true;
        for (int i=0; i<4; i++) {
            double wxt = wx[i];
            double wyt = wy[i]*ct - wz[i]*st;
            double wzt = wy[i]*st + wz[i]*ct;
            
            double cx, cy, cz, cw;
            transform(proj, wxt, wyt, wzt, cx, cy, cz, cw);
            double ndc_z = cz/cw;
            double sx = (cx/cw*0.5+0.5)*W;
            double sy = (0.5 - cy/cw*0.5)*H;
            
            printf("  v%d: world_z=%.2f ndc_z=%.3f screen=(%.0f,%.0f) cw=%.3f\n", i, wzt, ndc_z, sx, sy, cw);
            if (cw <= 0) { printf("    ❌ cw <= 0\n"); all_ok = false; }
            if (ndc_z < -1 || ndc_z > 1) { printf("    ⚠️ ndc_z out of [-1,1]\n"); }
        }
        if (all_ok) printf("  ✅ All in front\n");
    }
    return 0;
}
