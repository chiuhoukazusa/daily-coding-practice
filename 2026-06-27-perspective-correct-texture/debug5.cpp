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
    
    // Use a 20° tilt, smaller quad, offset downward
    // Quad local: x in [-1.5, 1.5], y in [-1.0, 1.0], z=0
    // Then rotate around X by tilt, then translate backward by -dist
    
    double tilt_deg = 25.0;
    double tilt = tilt_deg * M_PI / 180.0;
    double ct = cos(tilt), st = sin(tilt);
    double dist = 5.0;
    double offset_y = -1.5; // shift quad down
    
    double ly[4] = { 1.5,  1.5, -1.5, -1.5}; // local y (vertical)
    double lz[4] = { 0.0,  0.0,  0.0,  0.0}; // flat initially
    double lx[4] = {-2.0, 2.0,  2.0, -2.0};
    
    printf("Quad (%.0f deg tilt):\n", tilt_deg);
    for (int i=0; i<4; i++) {
        // Rotate around X
        double ry = ly[i]*ct - lz[i]*st;
        double rz = ly[i]*st + lz[i]*ct;
        // Translate backward and down
        double wx = lx[i];
        double wy = ry + offset_y;
        double wz = rz - dist;
        
        double cx, cy, cz, cw;
        transform(proj, wx, wy, wz, cx, cy, cz, cw);
        double sx = (cx/cw*0.5+0.5)*W;
        double sy = (0.5 - cy/cw*0.5)*H;
        double dz = cz/cw;
        
        printf("  v%d: world=(%.2f,%.2f,%.2f) screen=(%.0f,%.0f) cw=%.3f ndc_z=%.3f\n",
            i, wx, wy, wz, sx, sy, cw, dz);
    }
    
    // Also try approach: just make a quad where each corner has explicit world position
    // This is simpler and more controllable
    printf("\n--- Manual corner placement ---\n");
    // Define a quad in world space directly:
    // Bottom edge at z=-3 (close), top edge at z=-8 (far)
    // Width = 4, spread across X
    // Y adjusted so all corners are visible on screen
    double wc[4][3] = {
        {-2.8, -0.5, -7.5},  // top-left (far)
        { 2.8, -0.5, -7.5},  // top-right (far)
        { 2.8,  1.5, -3.5},  // bottom-right (near)
        {-2.8,  1.5, -3.5},  // bottom-left (near)
    };
    for (int i=0; i<4; i++) {
        double cx, cy, cz, cw;
        transform(proj, wc[i][0], wc[i][1], wc[i][2], cx, cy, cz, cw);
        double sx = (cx/cw*0.5+0.5)*W;
        double sy = (0.5 - cy/cw*0.5)*H;
        double dz = cz/cw;
        printf("  v%d: screen=(%.0f,%.0f) ndc_z=%.3f cw=%.3f\n", i, sx, sy, dz, cw);
    }
    return 0;
}
