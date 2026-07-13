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
    
    printf("Projection matrix:\n");
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) printf("%10.4f ", proj[i][j]);
        printf("\n");
    }
    
    double size=1.8, dist=5.0;
    double tilt=65.0 * M_PI / 180.0;
    double ct=cos(tilt), st=sin(tilt);
    
    double wx[4]={-size,size,size,-size};
    double wy[4]={size,size,-size,-size};
    double wz[4]={-dist,-dist,-dist,-dist};
    
    for (int i=0; i<4; i++) {
        double wxt = wx[i];
        double wyt = wy[i]*ct - wz[i]*st;
        double wzt = wy[i]*st + wz[i]*ct;
        
        double cx, cy, cz, cw;
        transform(proj, wxt, wyt, wzt, cx, cy, cz, cw);
        double invW = 1.0/cw;
        double ndc_x = cx*invW, ndc_y = cy*invW, ndc_z = cz*invW;
        double sx = (ndc_x*0.5+0.5)*W;
        double sy = (0.5 - ndc_y*0.5)*H;
        
        printf("Corner %d: world=(%.2f,%.2f,%.2f) clip=(%.2f,%.2f,%.2f,%.2f) ndc=(%.3f,%.3f,%.3f) screen=(%.1f,%.1f)\n",
            i, wxt, wyt, wzt, cx, cy, cz, cw, ndc_x, ndc_y, ndc_z, sx, sy);
    }
    return 0;
}
