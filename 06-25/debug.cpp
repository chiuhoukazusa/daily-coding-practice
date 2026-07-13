#include <iostream>
#include <cmath>
#include <cstring>

struct Vec3 {
    float x,y,z;
    Vec3():x(0),y(0),z(0){}
    Vec3(float a,float b,float c):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{
        return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float length()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 normalized()const{float l=length();return l<1e-10f?Vec3():Vec3(x/l,y/l,z/l);}
};

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
        Mat4 r;Vec3 f=(ctr-eye).normalized();
        Vec3 s=f.cross(up).normalized();Vec3 u=s.cross(f);
        r.m[0][0]=s.x;r.m[0][1]=s.y;r.m[0][2]=s.z;
        r.m[1][0]=u.x;r.m[1][1]=u.y;r.m[1][2]=u.z;
        r.m[2][0]=-f.x;r.m[2][1]=-f.y;r.m[2][2]=-f.z;
        r.m[3][0]=-s.dot(eye);r.m[3][1]=-u.dot(eye);r.m[3][2]=f.dot(eye);r.m[3][3]=1;
        return r;
    }
};

Mat4 operator*(const Mat4&a,const Mat4&b){
    Mat4 r;
    for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r.m[i][j]+=a.m[i][k]*b.m[k][j];
    return r;
}

struct Plane{ Vec3 n; float d; };

int main(){
    Mat4 proj=Mat4::perspective(60,800.0f/600,0.1f,50);
    Mat4 view=Mat4::lookAt({0,5,-12},{0,0,0},{0,1,0});
    Mat4 vp=proj*view;
    
    std::cout<<"=== MVP矩阵 ==="<<std::endl;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) std::cout<<vp.m[i][j]<<" ";
        std::cout<<std::endl;
    }
    
    // 提取平面
    Plane planes[6];
    planes[0]=Plane{{vp.m[0][3]+vp.m[0][0],vp.m[1][3]+vp.m[1][0],vp.m[2][3]+vp.m[2][0]},vp.m[3][3]+vp.m[3][0]};
    planes[1]=Plane{{vp.m[0][3]-vp.m[0][0],vp.m[1][3]-vp.m[1][0],vp.m[2][3]-vp.m[2][0]},vp.m[3][3]-vp.m[3][0]};
    planes[2]=Plane{{vp.m[0][3]+vp.m[0][1],vp.m[1][3]+vp.m[1][1],vp.m[2][3]+vp.m[2][1]},vp.m[3][3]+vp.m[3][1]};
    planes[3]=Plane{{vp.m[0][3]-vp.m[0][1],vp.m[1][3]-vp.m[1][1],vp.m[2][3]-vp.m[2][1]},vp.m[3][3]-vp.m[3][1]};
    planes[4]=Plane{{vp.m[0][3]+vp.m[0][2],vp.m[1][3]+vp.m[1][2],vp.m[2][3]+vp.m[2][2]},vp.m[3][3]+vp.m[3][2]};
    planes[5]=Plane{{vp.m[0][3]-vp.m[0][2],vp.m[1][3]-vp.m[1][2],vp.m[2][3]-vp.m[2][2]},vp.m[3][3]-vp.m[3][2]};
    
    // 归一化
    for(int i=0;i<6;i++){
        float len=planes[i].n.length();
        std::cout<<"Plane "<<i<<": n=("<<planes[i].n.x<<","<<planes[i].n.y<<","<<planes[i].n.z<<") len="<<len<<" d="<<planes[i].d<<"  normalized d="<<planes[i].d/len<<std::endl;
        planes[i].n=planes[i].n*(1.0f/len);
        planes[i].d/=len;
    }
    
    // 测试原点
    Vec3 origin(0,0,0);
    for(int i=0;i<6;i++){
        float sd=planes[i].n.dot(origin)+planes[i].d;
        std::cout<<"Plane "<<i<<" dist to origin(0,0,0): "<<sd<<(sd<0?" (OUTSIDE)":" (inside)")<<std::endl;
    }
    
    // 测试(0,0,0)为中心的球
    Vec3 c(0,0,0);
    for(int i=0;i<6;i++){
        float sd=planes[i].n.dot(c)+planes[i].d;
        std::cout<<"Plane "<<i<<" dist to (0,0,0): "<<sd<<std::endl;
    }
    
    // AABB测试 (0,0,0) 附近
    Vec3 vmin(-5,-5,-5), vmax(5,5,5);
    for(int i=0;i<6;i++){
        Vec3 pv(planes[i].n.x>0?vmax.x:vmin.x,
                planes[i].n.y>0?vmax.y:vmin.y,
                planes[i].n.z>0?vmax.z:vmin.z);
        float sd=planes[i].n.dot(pv)+planes[i].d;
        std::cout<<"Plane "<<i<<" AABB pv dist: "<<sd<<std::endl;
    }
}
