#include <cmath>
#include <cstdio>

struct Vec3 { float x,y,z; };
struct Vec4 { float x,y,z,w; Vec3 xyz(){return {x,y,z};} };
struct Mat4 { float m[4][4]={}; };

Vec4 mat4mul(const Mat4& M, Vec4 v) {
    return {
        M.m[0][0]*v.x+M.m[0][1]*v.y+M.m[0][2]*v.z+M.m[0][3]*v.w,
        M.m[1][0]*v.x+M.m[1][1]*v.y+M.m[1][2]*v.z+M.m[1][3]*v.w,
        M.m[2][0]*v.x+M.m[2][1]*v.y+M.m[2][2]*v.z+M.m[2][3]*v.w,
        M.m[3][0]*v.x+M.m[3][1]*v.y+M.m[3][2]*v.z+M.m[3][3]*v.w
    };
}

// Build view matrix
Mat4 buildView(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 fwd = {center.x-eye.x, center.y-eye.y, center.z-eye.z};
    float fl = sqrt(fwd.x*fwd.x+fwd.y*fwd.y+fwd.z*fwd.z);
    fwd = {fwd.x/fl, fwd.y/fl, fwd.z/fl};
    Vec3 rgt = {fwd.y*up.z-fwd.z*up.y, fwd.z*up.x-fwd.x*up.z, fwd.x*up.y-fwd.y*up.x};
    float rl = sqrt(rgt.x*rgt.x+rgt.y*rgt.y+rgt.z*rgt.z);
    rgt = {rgt.x/rl, rgt.y/rl, rgt.z/rl};
    Vec3 upv = {rgt.y*fwd.z-rgt.z*fwd.y, rgt.z*fwd.x-rgt.x*fwd.z, rgt.x*fwd.y-rgt.y*fwd.x};
    Mat4 v;
    v.m[0][0]=rgt.x; v.m[0][1]=rgt.y; v.m[0][2]=rgt.z; v.m[0][3]=-(rgt.x*eye.x+rgt.y*eye.y+rgt.z*eye.z);
    v.m[1][0]=upv.x; v.m[1][1]=upv.y; v.m[1][2]=upv.z; v.m[1][3]=-(upv.x*eye.x+upv.y*eye.y+upv.z*eye.z);
    v.m[2][0]=-fwd.x;v.m[2][1]=-fwd.y;v.m[2][2]=-fwd.z;v.m[2][3]=(fwd.x*eye.x+fwd.y*eye.y+fwd.z*eye.z);
    v.m[3][3]=1;
    return v;
}

Mat4 buildProj(float fovy, float aspect, float near, float far) {
    Mat4 p;
    float f = 1.0f/tan(fovy*0.5f);
    p.m[0][0]=f/aspect; p.m[1][1]=f;
    p.m[2][2]=(far+near)/(near-far); p.m[2][3]=2*far*near/(near-far);
    p.m[3][2]=-1;
    return p;
}

Mat4 matmul(const Mat4& A, const Mat4& B) {
    Mat4 r;
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) r.m[i][j]+=A.m[i][k]*B.m[k][j];
    return r;
}

int main() {
    Vec3 eye={0,2.5f,8.0f}, center={0,0,0}, up={0,1,0};
    Mat4 view = buildView(eye,center,up);
    Mat4 proj = buildProj(M_PI/3.0f, 800.0f/600, 0.1f, 100.0f);
    Mat4 VP = matmul(proj, view);

    // Test: transparent panel at (−1.2, 0.5, 1.5), world pos
    // The panel vertices after buildMesh transform would be in world space
    // Let's test the raw panel center: translate(-1.2, 0.5, 1.5)
    // A corner of a 1.2x1.8 quad: world pos around (-2.4, 0.5-1.8, 1.5) to (0, 0.5+1.8, 1.5)
    // Test: center point (-1.2, 0.5, 1.5)
    Vec3 testPts[] = {
        {-1.2f, 0.5f, 1.5f},
        { 0.0f, 0.0f, 0.0f},
        { 0.9f, 0.2f, 0.8f},
        { 1.3f,-0.5f, 1.9f},  // orange box
        {-0.5f, 0.0f, 1.5f},  // white box
    };
    printf("Camera at (0, 2.5, 8), center (0,0,0)\n");
    printf("View fwd: 0,0,0 - 0,2.5,8 = (0,-2.5,-8) normalized\n\n");
    for (auto& p : testPts) {
        Vec4 clip = mat4mul(VP, {p.x, p.y, p.z, 1.0f});
        printf("World(%.1f,%.1f,%.1f) -> clip(%.2f,%.2f,%.2f,%.2f)", p.x,p.y,p.z,clip.x,clip.y,clip.z,clip.w);
        if (abs(clip.w) > 1e-6f) {
            float nx=clip.x/clip.w, ny=clip.y/clip.w, nz=clip.z/clip.w;
            printf(" -> NDC(%.2f,%.2f,%.2f)", nx, ny, nz);
            int sx = (int)((nx*0.5f+0.5f)*799);
            int sy = (int)((1-(ny*0.5f+0.5f))*599);
            printf(" -> screen(%d,%d)", sx, sy);
            printf(" depth=%.3f", nz);
        }
        printf("\n");
    }
    return 0;
}
