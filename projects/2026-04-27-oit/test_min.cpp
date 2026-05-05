// Minimal OIT test - just one triangle
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>

struct Vec3 { float x,y,z;
    Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){}
    Vec3 operator-(Vec3 o){return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator/(float t){return{x/t,y/t,z/t};}
    Vec3 cross(Vec3 o){return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float dot(Vec3 o){return x*o.x+y*o.y+z*o.z;}
    float len(){return sqrt(x*x+y*y+z*z);}
    Vec3 norm(){float l=len();return l>0?*this/l:*this;}
};
struct Vec4{float x,y,z,w; Vec3 xyz(){return{x,y,z};}};
struct Mat4{float m[4][4]={};
    Vec4 operator*(Vec4 v){
        return{m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
               m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
               m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
               m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w};}
    Mat4 operator*(Mat4 o){Mat4 r;
        for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;}
    static Mat4 id(){Mat4 m;m.m[0][0]=m.m[1][1]=m.m[2][2]=m.m[3][3]=1;return m;}
};
Mat4 translate(Vec3 t){Mat4 m=Mat4::id();m.m[0][3]=t.x;m.m[1][3]=t.y;m.m[2][3]=t.z;return m;}
Mat4 rotateY(float a){Mat4 m=Mat4::id();float c=cos(a),s=sin(a);m.m[0][0]=c;m.m[0][2]=s;m.m[2][0]=-s;m.m[2][2]=c;return m;}
Mat4 scaleM(Vec3 s){Mat4 m=Mat4::id();m.m[0][0]=s.x;m.m[1][1]=s.y;m.m[2][2]=s.z;return m;}

inline float ef(float ax,float ay,float bx,float by,float px,float py){
    return (px-ax)*(by-ay)-(py-ay)*(bx-ax);
}

int main(){
    const int W=800,H=600;
    Vec3 eye={0,2.5f,8.0f},center={0,0,0};
    Vec3 fwd=(center-eye).norm();
    Vec3 up={0,1,0};
    Vec3 rgt=fwd.cross(up).norm();
    Vec3 upv=rgt.cross(fwd);
    Mat4 view=Mat4::id();
    view.m[0][0]=rgt.x;view.m[0][1]=rgt.y;view.m[0][2]=rgt.z;view.m[0][3]=-rgt.dot(eye);
    view.m[1][0]=upv.x;view.m[1][1]=upv.y;view.m[1][2]=upv.z;view.m[1][3]=-upv.dot(eye);
    view.m[2][0]=-fwd.x;view.m[2][1]=-fwd.y;view.m[2][2]=-fwd.z;view.m[2][3]=fwd.dot(eye);
    Mat4 proj=Mat4::id();
    float f=1.0f/tan(M_PI/6.0f);
    proj.m[0][0]=f/(800.0f/600);proj.m[1][1]=f;
    proj.m[2][2]=(100+0.1f)/(0.1f-100);proj.m[2][3]=2*100*0.1f/(0.1f-100);
    proj.m[3][2]=-1;
    Mat4 VP=proj*view;

    // Build red panel model
    Mat4 model=translate({-1.2f,0.5f,1.5f})*rotateY(-(float)(M_PI/6))*scaleM({1.2f,1.8f,1.0f});
    // Quad verts (local): (-1,-1,0), (1,-1,0), (1,1,0), (-1,1,0)
    // Triangle 0: (0,1,2) and Triangle 1: (0,2,3)
    Vec3 qv[4]={{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}};
    // World verts
    Vec3 wv[4];
    float sx[4],sy[4],nz[4];
    for(int i=0;i<4;i++){
        Vec4 p=model*Vec4{qv[i].x,qv[i].y,qv[i].z,1};
        wv[i]={p.x,p.y,p.z};
        Vec4 c=VP*p;
        float nx=c.x/c.w,ny=c.y/c.w;
        nz[i]=c.z/c.w;
        sx[i]=(nx*0.5f+0.5f)*(W-1);
        sy[i]=(1-(ny*0.5f+0.5f))*(H-1);
    }
    // Tri0: verts 0,1,2
    printf("Tri screen: (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)\n",
           sx[0],sy[0],sx[1],sy[1],sx[2],sy[2]);
    float area=(sx[1]-sx[0])*(sy[2]-sy[0])-(sy[1]-sy[0])*(sx[2]-sx[0]);
    bool ccw=(area>0);
    printf("area=%.1f ccw=%s\n",area,ccw?"true":"false");

    // Try rasterize
    int minX=(int)std::max(0.0f,std::min({sx[0],sx[1],sx[2]}));
    int maxX=(int)std::min((float)(W-1),std::max({sx[0],sx[1],sx[2]}));
    int minY=(int)std::max(0.0f,std::min({sy[0],sy[1],sy[2]}));
    int maxY=(int)std::min((float)(H-1),std::max({sy[0],sy[1],sy[2]}));
    printf("BBox: x[%d,%d] y[%d,%d]\n",minX,maxX,minY,maxY);

    int hit=0;
    // Opaque depth = 1e9 (background)
    for(int py=minY;py<=maxY;py++) for(int px=minX;px<=maxX;px++){
        float w0=ef(sx[1],sy[1],sx[2],sy[2],(float)px,(float)py);
        float w1=ef(sx[2],sy[2],sx[0],sy[0],(float)px,(float)py);
        float w2=ef(sx[0],sy[0],sx[1],sy[1],(float)px,(float)py);
        bool inside;
        if(!ccw){inside=(w0<=0&&w1<=0&&w2<=0);w0=-w0;w1=-w1;w2=-w2;}
        else{inside=(w0>=0&&w1>=0&&w2>=0);}
        if(!inside) continue;
        float denom=w0+w1+w2;
        if(denom<1e-8f) continue;
        float z=w0/denom*nz[0]+w1/denom*nz[1]+w2/denom*nz[2];
        // depth test: opaque depth = 1e9 (background). z should pass.
        if(z>=1e9f) continue;  // This would fail
        hit++;
    }
    printf("Hits: %d\n",hit);
    return 0;
}
