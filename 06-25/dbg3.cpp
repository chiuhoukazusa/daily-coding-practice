#include <iostream>
#include <cmath>
#include <cstring>

struct Vec3 {
    float x,y,z;
    Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l<1e-10f?Vec3{0,0,0}:Vec3{x/l,y/l,z/l};}
};

struct Mat4 {
    float m[16];
    Mat4(){memset(m,0,sizeof(m));}
    float& operator()(int c,int r){return m[c*4+r];}
    float operator()(int c,int r)const{return m[c*4+r];}
    static Mat4 perspective(float fovY,float aspect,float n,float f){
        Mat4 r;float t=std::tan(fovY*0.5f*3.14159f/180);
        r(0,0)=1/(aspect*t);r(1,1)=1/t;
        r(2,2)=-(f+n)/(f-n);r(2,3)=-1;
        r(3,2)=-2*f*n/(f-n);return r;
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
    Mat4 r;
    for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r(i,j)+=a(i,k)*b(k,j);
    return r;
}

int main(){
    Mat4 proj=Mat4::perspective(60,800.0f/600,0.1f,50);
    Mat4 view=Mat4::lookAt({0,5,-12},{0,0,0},{0,1,0});
    Mat4 vp=mulMat(proj,view);
    
    std::cout<<"VP Matrix:"<<std::endl;
    for(int r=0;r<4;r++){
        for(int c=0;c<4;c++) printf("%10.4f ",vp(c,r));
        printf("\n");
    }

    // 提取平面
    struct Plane{ Vec3 n; float d; }p[6];
    p[0]={{vp(0,3)+vp(0,0),vp(1,3)+vp(1,0),vp(2,3)+vp(2,0)},vp(3,3)+vp(3,0)};
    p[1]={{vp(0,3)-vp(0,0),vp(1,3)-vp(1,0),vp(2,3)-vp(2,0)},vp(3,3)-vp(3,0)};
    p[2]={{vp(0,3)+vp(0,1),vp(1,3)+vp(1,1),vp(2,3)+vp(2,1)},vp(3,3)+vp(3,1)};
    p[3]={{vp(0,3)-vp(0,1),vp(1,3)-vp(1,1),vp(2,3)-vp(2,1)},vp(3,3)-vp(3,1)};
    p[4]={{vp(0,3)+vp(0,2),vp(1,3)+vp(1,2),vp(2,3)+vp(2,2)},vp(3,3)+vp(3,2)};
    p[5]={{vp(0,3)-vp(0,2),vp(1,3)-vp(1,2),vp(2,3)-vp(2,2)},vp(3,3)-vp(3,2)};

    for(int i=0;i<6;i++){
        float l=p[i].n.len();
        p[i].n=p[i].n*(1.0f/l); p[i].d/=l;
        printf("Plane %d: n=(%.3f,%.3f,%.3f) d=%.4f\n",i,p[i].n.x,p[i].n.y,p[i].n.z,p[i].d);
    }
    
    // 测试原点附近
    Vec3 vmin(-5,-5,-5), vmax(5,5,5);
    for(int i=0;i<6;i++){
        Vec3 pv(p[i].n.x>0?vmax.x:vmin.x, p[i].n.y>0?vmax.y:vmin.y, p[i].n.z>0?vmax.z:vmin.z);
        float sd=p[i].n.dot(pv)+p[i].d;
        printf("Plane %d: pv=(%.1f,%.1f,%.1f) sd=%.4f %s\n",i,pv.x,pv.y,pv.z,sd,sd<0?"CULL":"OK");
    }
    
    // 直接测试场景中的球
    Vec3 scenesphere={0,0,0}; // 中心球
    for(int i=0;i<6;i++){
        float sd=p[i].n.dot(scenesphere)+p[i].d;
        printf("Center(0,0,0) Plane %d sd=%.4f\n",i,sd);
    }
    
    // 测试(0,5,-12) 相机位置到平面的距离应该为负（近平面在相机后方）
    Vec3 campos(0,5,-12);
    for(int i=0;i<6;i++){
        float sd=p[i].n.dot(campos)+p[i].d;
        printf("CamPos Plane %d sd=%.4f\n",i,sd);
    }
}
