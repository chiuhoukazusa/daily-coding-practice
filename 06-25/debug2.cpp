#include <iostream>
#include <cmath>
#include <cstring>

struct Vec3 { float x,y,z; Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){} };
struct Vec4 { float x,y,z,w; };

struct Mat4 {
    float m[4][4];
    Mat4(){memset(m,0,sizeof(m));}
    static Mat4 perspective(float fovY,float aspect,float n,float f){
        Mat4 r;float t=std::tan(fovY*0.5f*3.14159f/180);
        r.m[0][0]=1/(aspect*t);r.m[1][1]=1/t;
        r.m[2][2]=-(f+n)/(f-n);r.m[2][3]=-1;
        r.m[3][2]=-(2*f*n)/(f-n);return r;
    }
    static Mat4 lookAt(const Vec3&eye,const Vec3&ctr,const Vec3&up){
        Mat4 r;Vec3 f=(ctr-eye);
        float flen=std::sqrt(f.x*f.x+f.y*f.y+f.z*f.z);
        f={f.x/flen,f.y/flen,f.z/flen};
        Vec3 s={f.y*up.z-f.z*up.y, f.z*up.x-f.x*up.z, f.x*up.y-f.y*up.x};
        float slen=std::sqrt(s.x*s.x+s.y*s.y+s.z*s.z);
        s={s.x/slen,s.y/slen,s.z/slen};
        Vec3 u={s.y*f.z-s.z*f.y, s.z*f.x-s.x*f.z, s.x*f.y-s.y*f.x};
        r.m[0][0]=s.x; r.m[0][1]=u.x; r.m[0][2]=-f.x; r.m[0][3]=0;
        r.m[1][0]=s.y; r.m[1][1]=u.y; r.m[1][2]=-f.y; r.m[1][3]=0;
        r.m[2][0]=s.z; r.m[2][1]=u.z; r.m[2][2]=-f.z; r.m[2][3]=0;
        r.m[3][0]=-(s.x*eye.x+s.y*eye.y+s.z*eye.z);
        r.m[3][1]=-(u.x*eye.x+u.y*eye.y+u.z*eye.z);
        r.m[3][2]= (f.x*eye.x+f.y*eye.y+f.z*eye.z);
        r.m[3][3]=1;
        return r;
    }
};

Vec4 operator*(const Mat4&m,const Vec4&v){
    return{
        m.m[0][0]*v.x+m.m[1][0]*v.y+m.m[2][0]*v.z+m.m[3][0]*v.w,
        m.m[0][1]*v.x+m.m[1][1]*v.y+m.m[2][1]*v.z+m.m[3][1]*v.w,
        m.m[0][2]*v.x+m.m[1][2]*v.y+m.m[2][2]*v.z+m.m[3][2]*v.w,
        m.m[0][3]*v.x+m.m[1][3]*v.y+m.m[2][3]*v.z+m.m[3][3]*v.w
    };
}

Mat4 operator*(const Mat4&a,const Mat4&b){
    Mat4 r;
    for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r.m[i][j]+=a.m[i][k]*b.m[k][j];
    return r;
}

int main(){
    Mat4 proj=Mat4::perspective(60,800.0f/600,0.1f,50);
    Mat4 view=Mat4::lookAt({0,5,-12},{0,0,0},{0,1,0});
    Mat4 vp=proj*view;
    
    std::cout<<"=== View ==="<<std::endl;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) std::cout<<view.m[i][j]<<" ";
        std::cout<<std::endl;
    }
    
    std::cout<<"=== Proj ==="<<std::endl;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) std::cout<<proj.m[i][j]<<" ";
        std::cout<<std::endl;
    }
    
    std::cout<<"=== VP ==="<<std::endl;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) std::cout<<vp.m[i][j]<<" ";
        std::cout<<std::endl;
    }
    
    // 测试投影
    Vec4 p(0,0,0,1);
    Vec4 clip=vp*p;
    std::cout<<"Origin clip: ("<<clip.x<<","<<clip.y<<","<<clip.z<<","<<clip.w<<")"<<std::endl;
    
    Vec4 p2(0,0,5,1);
    clip=vp*p2;
    std::cout<<"(0,0,5) clip: ("<<clip.x<<","<<clip.y<<","<<clip.z<<","<<clip.w<<")"<<std::endl;
    
    // 提取视锥体平面
    struct {
        Vec3 n; float d;
    } planes[6];
    planes[0]={{vp.m[0][3]+vp.m[0][0],vp.m[1][3]+vp.m[1][0],vp.m[2][3]+vp.m[2][0]},vp.m[3][3]+vp.m[3][0]};
    planes[1]={{vp.m[0][3]-vp.m[0][0],vp.m[1][3]-vp.m[1][0],vp.m[2][3]-vp.m[2][0]},vp.m[3][3]-vp.m[3][0]};
    planes[2]={{vp.m[0][3]+vp.m[0][1],vp.m[1][3]+vp.m[1][1],vp.m[2][3]+vp.m[2][1]},vp.m[3][3]+vp.m[3][1]};
    planes[3]={{vp.m[0][3]-vp.m[0][1],vp.m[1][3]-vp.m[1][1],vp.m[2][3]-vp.m[2][1]},vp.m[3][3]-vp.m[3][1]};
    planes[4]={{vp.m[0][3]+vp.m[0][2],vp.m[1][3]+vp.m[1][2],vp.m[2][3]+vp.m[2][2]},vp.m[3][3]+vp.m[3][2]};
    planes[5]={{vp.m[0][3]-vp.m[0][2],vp.m[1][3]-vp.m[1][2],vp.m[2][3]-vp.m[2][2]},vp.m[3][3]-vp.m[3][2]};
    
    for(int i=0;i<6;i++){
        float len=std::sqrt(planes[i].n.x*planes[i].n.x+planes[i].n.y*planes[i].n.y+planes[i].n.z*planes[i].n.z);
        planes[i].n={planes[i].n.x/len,planes[i].n.y/len,planes[i].n.z/len};
        planes[i].d/=len;
        std::cout<<"NormPlane "<<i<<": n=("<<planes[i].n.x<<","<<planes[i].n.y<<","<<planes[i].n.z<<") d="<<planes[i].d<<std::endl;
    }
    
    // 测试原点
    for(int i=0;i<6;i++){
        float sd=planes[i].n.x*0+planes[i].n.y*0+planes[i].n.z*0+planes[i].d;
        std::cout<<"Plane "<<i<<" origin dist="<<sd<<std::endl;
    }
    
    // 测试 (-5,-5,-5)
    Vec3 vmin(-5,-5,-5), vmax(5,5,5);
    for(int i=0;i<6;i++){
        Vec3 pv(planes[i].n.x>0?vmax.x:vmin.x,
                planes[i].n.y>0?vmax.y:vmin.y,
                planes[i].n.z>0?vmax.z:vmin.z);
        float sd=planes[i].n.x*pv.x+planes[i].n.y*pv.y+planes[i].n.z*pv.z+planes[i].d;
        std::cout<<"Plane "<<i<<" AABB(-5,-5,-5)/5 pv dist="<<sd<<std::endl;
    }
}
