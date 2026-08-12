#include <iostream>
#include <cmath>

struct Vec4 { float x,y,z,w; };
struct Vec3 { float x,y,z; };

struct Mat4 { float m[16]; 
    Mat4() { for(int i=0;i<16;i++) m[i]=(i%5==0)?1:0; }
    static Mat4 translate(float x,float y,float z){ Mat4 r; r.m[3]=x;r.m[7]=y;r.m[11]=z; return r; }
    static Mat4 rotateX(float a){ Mat4 r; float c=cos(a),s=sin(a); r.m[5]=c;r.m[6]=-s;r.m[9]=s;r.m[10]=c; return r; }
    static Mat4 perspective(float fovY,float aspect,float n,float f){
        Mat4 r; r.m[0]=1/(aspect*tan(fovY*0.5f)); r.m[5]=1/tan(fovY*0.5f);
        r.m[10]=-(f+n)/(f-n); r.m[11]=-2*f*n/(f-n); r.m[14]=-1; r.m[15]=0; return r;
    }
    Mat4 operator*(const Mat4& b) const {
        Mat4 r; for(int i=0;i<4;i++) for(int j=0;j<4;j++) { r.m[i*4+j]=0;
            for(int k=0;k<4;k++) r.m[i*4+j]+=m[i*4+k]*b.m[k*4+j]; } return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {m[0]*v.x+m[1]*v.y+m[2]*v.z+m[3]*v.w, m[4]*v.x+m[5]*v.y+m[6]*v.z+m[7]*v.w,
                m[8]*v.x+m[9]*v.y+m[10]*v.z+m[11]*v.w, m[12]*v.x+m[13]*v.y+m[14]*v.z+m[15]*v.w};
    }
    void print(){ printf("[%7.3f %7.3f %7.3f %7.3f]\n[%7.3f %7.3f %7.3f %7.3f]\n[%7.3f %7.3f %7.3f %7.3f]\n[%7.3f %7.3f %7.3f %7.3f]\n",
        m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8],m[9],m[10],m[11],m[12],m[13],m[14],m[15]); }
};

Vec4 clipPlanes[] = {
    {-1,0,0,-1}, {1,0,0,-1}, {0,-1,0,-1}, {0,1,0,-1}, {0,0,-1,-1}, {0,0,1,-1},
};
float dotPlane(Vec4 p, Vec4 v) { return p.x*v.x+p.y*v.y+p.z*v.z+p.w*v.w; }

int main(){
    Mat4 view=Mat4::translate(0,-0.3f,-4)*Mat4::rotateX(-0.15f);
    Mat4 proj=Mat4::perspective(55*M_PI/180, 800.0f/600, 0.5f, 20);
    Mat4 mvp=proj*view;
    
    printf("View matrix:\n"); view.print();
    printf("\nProj matrix:\n"); proj.print();
    printf("\nMVP matrix:\n"); mvp.print();
    
    // Test a simple point
    Vec4 p{0,0,0,1}; // cube center
    Vec4 cp=mvp*p;
    printf("\nPoint (0,0,0,1) -> clip: (%.3f, %.3f, %.3f, %.3f)\n", cp.x, cp.y, cp.z, cp.w);
    printf("Check against clip planes:\n");
    for(int i=0;i<6;i++){
        float d=dotPlane(clipPlanes[i],cp);
        printf("  plane %d: dot=%.3f %s\n", i, d, d>=0?"PASS":"FAIL");
    }
    
    // Test a cube corner
    Vec4 c1{0.6f,0.6f,0.6f,1};
    Vec4 cc1=mvp*c1;
    printf("\nPoint (0.6,0.6,0.6,1) -> clip: (%.3f, %.3f, %.3f, %.3f)\n", cc1.x, cc1.y, cc1.z, cc1.w);
    printf("Check against clip planes:\n");
    for(int i=0;i<6;i++){
        float d=dotPlane(clipPlanes[i],cc1);
        printf("  plane %d: dot=%.3f %s\n", i, d, d>=0?"PASS":"FAIL");
    }
    
    // What OpenGL convention do we actually want?
    // GL convention: -w <= x <= w, meaning inside if -w <= x <= w
    // Our plane for x: plane = (-1,0,0,-1) => dot = -x - w >= 0 => -x >= w => x <= -w
    // That's WRONG!!! We want x >= -w => -x - w <= 0 => positive: x + w >= 0 => x >= -w
    // Wait, let me reconsider.
    // We define "inside" as dot(plane, point) >= 0
    // For left clip: we want x >= -w, i.e., x + w >= 0. plane = (1,0,0,1), dot = x + w >= 0 => inside
    // For right clip: x <= w, i.e., -x + w >= 0. plane = (-1,0,0,1), dot = -x + w >= 0 => inside
    // Current plane LEFT = (-1,0,0,-1): dot = -x-w >= 0 => -x >= w => x <= -w. WRONG!
    
    printf("\nLEFT plane fix needed? Current: (-1,0,0,-1) => -x-w>=0 => x<=-w\n");
    Vec4 correctLeft{1,0,0,1}; // x+w>=0 => x>=-w
    printf("Correct LEFT: (1,0,0,1) => x+w>=0 => x>=-w\n");
    printf("Test with (0.6,0.6,0.6,1): dot(1,0,0,1)=%.3f\n", correctLeft.x*cc1.x+correctLeft.y*cc1.y+correctLeft.z*cc1.z+correctLeft.w*cc1.w);
}
