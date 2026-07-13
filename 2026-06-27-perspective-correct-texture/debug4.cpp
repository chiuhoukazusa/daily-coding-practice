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
    
    // Approach: define quad in world space explicitly
    // We want a quad that maps to a good screen region.
    // Let's place the quad in world space with known depths:
    // Bottom edge at world z = -3 (close to camera), top edge at z = -7 (far)
    // The quad is tilted around X, lying roughly in the XZ plane with Y variation.
    
    double wz_near = 3.5;
    double wz_far  = 8.0;
    double quad_width = 3.0;  // X extent at center depth
    
    // Since it's tilted, compute actual world positions
    // We want: bottom of quad (v2,v3) = closest to camera
    //          top of quad (v0,v1) = farthest from camera
    // Tilt around X: quad in XZ plane rotated around X by tilt angle
    
    double tilt_deg = 35.0;
    double tilt = tilt_deg * M_PI / 180.0;
    double ct = cos(tilt), st = sin(tilt);
    
    // Quad corners in "local" coordinates (before tilt):
    // In the quad's local frame: x = ±width/2, y = 0, z = -dist (negative = into screen)
    // After tilt around X (y rotates toward z):
    double ly[4] = { 2.0,  2.0, -2.0, -2.0}; // local y (vertical in quad)
    double lz[4] = {-5.0, -5.0, -5.0, -5.0}; // local z (depth)
    double lx[4] = {-1.8, 1.8,  1.8, -1.8};
    
    printf("Quad in world space (after %.0f deg tilt):\n", tilt_deg);
    for (int i=0; i<4; i++) {
        double wx = lx[i];
        double wy = ly[i]*ct - lz[i]*st;
        double wz = ly[i]*st + lz[i]*ct;
        
        double cx, cy, cz, cw;
        transform(proj, wx, wy, wz, cx, cy, cz, cw);
        double sx = (cx/cw*0.5+0.5)*W;
        double sy = (0.5 - cy/cw*0.5)*H;
        double dz = cz/cw;
        
        printf("  v%d: world=(%.2f,%.2f,%.2f) screen=(%.0f,%.0f) ndc_z=%.3f cw=%.3f\n",
            i, wx, wy, wz, sx, sy, dz, cw);
    }
    return 0;
}
