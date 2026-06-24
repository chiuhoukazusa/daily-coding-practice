#include <iostream>
#include <cmath>
#include <cstring>

struct Vec3{float x,y,z;Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l<1e-10f?Vec3{0,0,0}:Vec3{x/l,y/l,z/l};}
};

struct Mat4{
    float m[16];
    Mat4(){memset(m,0,sizeof(m));}
    float& operator()(int c,int r){return m[c*4+r];}
    float operator()(int c,int r)const{return m[c*4+r];}
    static Mat4 perspective(float fovY,float a,float n,float f){
        Mat4 r;float t=std::tan(fovY*0.5f*3.14159f/180);
        r(0,0)=1/(a*t);r(1,1)=1/t;r(2,2)=-(f+n)/(f-n);r(2,3)=-1;r(3,2)=-2*f*n/(f-n);return r;
    }
    static Mat4 lookAt(const Vec3&e,const Vec3&c,const Vec3&up){
        Mat4 r;Vec3 f=(c-e).norm(),s=f.cross(up).norm(),u=s.cross(f);
        r(0,0)=s.x;r(1,0)=s.y;r(2,0)=s.z;
        r(0,1)=u.x;r(1,1)=u.y;r(2,1)=u.z;
        r(0,2)=-f.x;r(1,2)=-f.y;r(2,2)=-f.z;
        r(3,0)=-s.dot(e);r(3,1)=-u.dot(e);r(3,2)=f.dot(e);r(3,3)=1;
        return r;
    }
};

Mat4 mulMat(const Mat4&a,const Mat4&b){
    Mat4 r;for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r(i,j)+=a(i,k)*b(k,j);return r;
}

Vec3 mulVec(const Mat4&m,const Vec3&v,float w,float&ow){
    float x=m(0,0)*v.x+m(1,0)*v.y+m(2,0)*v.z+m(3,0)*w;
    float y=m(0,1)*v.x+m(1,1)*v.y+m(2,1)*v.z+m(3,1)*w;
    float z=m(0,2)*v.x+m(1,2)*v.y+m(2,2)*v.z+m(3,2)*w;
    ow=m(0,3)*v.x+m(1,3)*v.y+m(2,3)*v.z+m(3,3)*w;return{x,y,z};
}

int main(){
    Mat4 proj=Mat4::perspective(60,800.0f/600,0.1f,50);
    Mat4 view=Mat4::lookAt({0,5,-12},{0,0,0},{0,1,0});
    Mat4 vp=mulMat(proj,view);

    // 手动投影一些点
    for(float zz=0; zz<=10; zz+=1){
        float w; Vec3 v=mulVec(vp,{0,1,zz},1,w);
        printf("(0,1,%.0f) -> clip: (%.3f,%.3f,%.3f,%.3f)  NDC z=%.3f\n", zz,v.x,v.y,v.z,w,v.z/w);
    }

    // 现在看看: 在clip空间中 (x/w, y/w, z/w) 是否在 [-1,1] 内
    float w; Vec3 v=mulVec(vp,{0,0,10},1,w);
    printf("(0,0,10) clip=(%.3f,%.3f,%.3f,%.3f) ndc=(%.3f,%.3f,%.3f)\n",v.x,v.y,v.z,w,v.x/w,v.y/w,v.z/w);
    
    v=mulVec(vp,{0,0,0.5},1,w);
    printf("(0,0,0.5) clip=(%.3f,%.3f,%.3f,%.3f) ndc=(%.3f,%.3f,%.3f)\n",v.x,v.y,v.z,w,v.x/w,v.y/w,v.z/w);
}
