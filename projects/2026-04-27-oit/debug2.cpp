#include <cmath>
#include <cstdio>
#include <cfloat>

struct Vec3 { float x,y,z; };
struct Vec4 { float x,y,z,w; };
struct Mat4 { float m[4][4]={}; };

Vec4 mv(const Mat4& M, Vec4 v) {
    return { M.m[0][0]*v.x+M.m[0][1]*v.y+M.m[0][2]*v.z+M.m[0][3]*v.w,
             M.m[1][0]*v.x+M.m[1][1]*v.y+M.m[1][2]*v.z+M.m[1][3]*v.w,
             M.m[2][0]*v.x+M.m[2][1]*v.y+M.m[2][2]*v.z+M.m[2][3]*v.w,
             M.m[3][0]*v.x+M.m[3][1]*v.y+M.m[3][2]*v.z+M.m[3][3]*v.w };
}
Mat4 mm(const Mat4& A, const Mat4& B) {
    Mat4 r;
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) r.m[i][j]+=A.m[i][k]*B.m[k][j];
    return r;
}

Mat4 buildView() {
    Vec3 eye={0,2.5f,8.0f};
    // fwd direction toward (0,0,0)
    float fx=0-0, fy=0-2.5f, fz=0-8.0f;
    float fl=sqrt(fx*fx+fy*fy+fz*fz);
    fx/=fl; fy/=fl; fz/=fl;
    // right = fwd x up
    float rx=fy*1-fz*0, ry=fz*0-fx*1, rz=fx*0-fy*0;
    // Actually up=(0,1,0)
    rx = fy*1-fz*0; ry = fz*0-fx*1; rz = fx*0-fy*0;
    float rl=sqrt(rx*rx+ry*ry+rz*rz);
    rx/=rl; ry/=rl; rz/=rl;
    float ux=ry*fz-rz*fy, uy=rz*fx-rx*fz, uz=rx*fy-ry*fx;
    Mat4 v;
    v.m[0][0]=rx; v.m[0][1]=ry; v.m[0][2]=rz; v.m[0][3]=-(rx*eye.x+ry*eye.y+rz*eye.z);
    v.m[1][0]=ux; v.m[1][1]=uy; v.m[1][2]=uz; v.m[1][3]=-(ux*eye.x+uy*eye.y+uz*eye.z);
    v.m[2][0]=-fx;v.m[2][1]=-fy;v.m[2][2]=-fz;v.m[2][3]=(fx*eye.x+fy*eye.y+fz*eye.z);
    v.m[3][3]=1;
    return v;
}
Mat4 buildProj() {
    Mat4 p;
    float f=1.0f/tan(M_PI/6.0f);
    p.m[0][0]=f/(800.0f/600); p.m[1][1]=f;
    p.m[2][2]=(100.0f+0.1f)/(0.1f-100.0f); p.m[2][3]=2*100.0f*0.1f/(0.1f-100.0f);
    p.m[3][2]=-1;
    return p;
}

int main() {
    Mat4 view=buildView(), proj=buildProj(), VP=mm(proj,view);
    // Test key points at different depths
    struct P { float x,y,z; const char* name; };
    P pts[] = {
        {0,0,-3,"back_wall_center"},
        {-3.2f,0,-1,"left_pillar"},
        {0,-1.3f,0,"pedestal"},
        {-1.2f,0.5f,1.5f,"red_panel_front"},
        {0.9f,0.2f,0.8f,"green_panel"},
        {0,0,3.5f,"near_transparent"},
    };
    printf("%-25s  NDC_z    depth_cmp\n", "point");
    for (auto& p : pts) {
        Vec4 c = mv(VP, {p.x,p.y,p.z,1});
        if (fabs(c.w)<1e-6f) { printf("%-25s  w≈0\n",p.name); continue; }
        float nz = c.z/c.w;
        printf("%-25s  %.4f   (depth buffer stores NDC_z)\n", p.name, nz);
    }
    return 0;
}
