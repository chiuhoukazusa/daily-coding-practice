// Minimal test: project one panel and check rasterization
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>

struct Vec3 { float x,y,z;
    Vec3(){}; Vec3(float a,float b,float c):x(a),y(b),z(c){}
    Vec3 operator-(Vec3 o){return {x-o.x,y-o.y,z-o.z};}
    Vec3 operator/(float t){return {x/t,y/t,z/t};}
    Vec3 cross(Vec3 o){return {y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float dot(Vec3 o){return x*o.x+y*o.y+z*o.z;}
    float len(){return sqrt(x*x+y*y+z*z);}
    Vec3 norm(){float l=len();return l>0?*this/l:*this;}
};
struct Vec4 { float x,y,z,w; Vec3 xyz(){return {x,y,z};} };
struct Mat4 { float m[4][4]={};
    Vec4 operator*(Vec4 v){
        return {m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
                m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
                m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
                m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w};
    }
    Mat4 operator*(Mat4 o){
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;
    }
    static Mat4 id(){Mat4 m; m.m[0][0]=m.m[1][1]=m.m[2][2]=m.m[3][3]=1; return m;}
};

Mat4 translate(Vec3 t){
    Mat4 m=Mat4::id(); m.m[0][3]=t.x; m.m[1][3]=t.y; m.m[2][3]=t.z; return m;
}
Mat4 rotateY(float a){
    Mat4 m=Mat4::id(); float c=cos(a),s=sin(a);
    m.m[0][0]=c; m.m[0][2]=s; m.m[2][0]=-s; m.m[2][2]=c; return m;
}
Mat4 scaleM(Vec3 s){
    Mat4 m=Mat4::id(); m.m[0][0]=s.x; m.m[1][1]=s.y; m.m[2][2]=s.z; return m;
}

int main(){
    const int W=800,H=600;
    Vec3 eye={0,2.5f,8.0f}, center={0,0,0};
    Vec3 fwd=(center-eye).norm();
    Vec3 up={0,1,0};
    Vec3 rgt=fwd.cross(up).norm();
    Vec3 upv=rgt.cross(fwd);
    Mat4 view=Mat4::id();
    view.m[0][0]=rgt.x; view.m[0][1]=rgt.y; view.m[0][2]=rgt.z; view.m[0][3]=-rgt.dot(eye);
    view.m[1][0]=upv.x; view.m[1][1]=upv.y; view.m[1][2]=upv.z; view.m[1][3]=-upv.dot(eye);
    view.m[2][0]=-fwd.x;view.m[2][1]=-fwd.y;view.m[2][2]=-fwd.z;view.m[2][3]=fwd.dot(eye);

    Mat4 proj=Mat4::id();
    float f=1.0f/tan(M_PI/6.0f);
    proj.m[0][0]=f/(800.0f/600); proj.m[1][1]=f;
    proj.m[2][2]=(100+0.1f)/(0.1f-100); proj.m[2][3]=2*100*0.1f/(0.1f-100);
    proj.m[3][2]=-1;
    Mat4 VP=proj*view;

    // Build the red panel model matrix
    Mat4 model = translate({-1.2f,0.5f,1.5f}) * rotateY(-(float)(M_PI/6)) * scaleM({1.2f,1.8f,1.0f});

    // quad vertices in model space
    Vec3 verts[4]={{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}};
    printf("Red panel vertices (world space and screen space):\n");
    for(int i=0;i<4;i++){
        Vec4 p4 = model*Vec4{verts[i].x,verts[i].y,verts[i].z,1};
        printf("  v%d world=(%.2f,%.2f,%.2f)", i, p4.x,p4.y,p4.z);
        Vec4 clip = VP*p4;
        printf(" clip=(%.2f,%.2f,%.2f,%.2f)", clip.x,clip.y,clip.z,clip.w);
        if(fabs(clip.w)>1e-6f){
            float nx=clip.x/clip.w, ny=clip.y/clip.w, nz=clip.z/clip.w;
            float sx=(nx*0.5f+0.5f)*(W-1);
            float sy=(1-(ny*0.5f+0.5f))*(H-1);
            printf(" NDC=(%.2f,%.2f,%.2f) screen=(%.0f,%.0f)", nx,ny,nz,sx,sy);
            bool inview = (nz>=-1&&nz<=1);
            printf(inview?" IN_FRUSTUM":" OUT_FRUSTUM");
        }
        printf("\n");
    }
    // Triangle 1: v0,v1,v2
    // Check edge function for a center pixel
    // project v0,v1,v2
    struct SV { float sx,sy,nz; };
    SV sv[4];
    for(int i=0;i<4;i++){
        Vec4 p4=model*Vec4{verts[i].x,verts[i].y,verts[i].z,1};
        Vec4 clip=VP*p4;
        float nx=clip.x/clip.w,ny=clip.y/clip.w,nz=clip.z/clip.w;
        sv[i]={  (nx*0.5f+0.5f)*(W-1), (1-(ny*0.5f+0.5f))*(H-1), nz };
    }
    // tri1: 0,1,2
    printf("\nTriangle 0,1,2 screen coords: (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)\n",
           sv[0].sx,sv[0].sy, sv[1].sx,sv[1].sy, sv[2].sx,sv[2].sy);
    float e1x=sv[1].sx-sv[0].sx, e1y=sv[1].sy-sv[0].sy;
    float e2x=sv[2].sx-sv[0].sx, e2y=sv[2].sy-sv[0].sy;
    float area=e1x*e2y-e1y*e2x;
    printf("area=%.2f (>0 CCW, <0 CW)\n", area);
    return 0;
}
