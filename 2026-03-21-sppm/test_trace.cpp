// 测试：只追踪少量射线，看 hit point 是否有效
#include <cmath>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <random>
#include <cstring>

struct Vec3 {
    double x,y,z;
    Vec3(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(double t)const{return{x/t,y/t,z/t};}
    Vec3 operator-()const{return{-x,-y,-z};}
    double dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    double len2()const{return x*x+y*y+z*z;}
    double len()const{return std::sqrt(len2());}
    Vec3 norm()const{double l=len();return l>1e-12?*this/l:Vec3(0,1,0);}
    bool nz()const{return x!=0||y!=0||z!=0;}
};

struct Ray { Vec3 o,d; Ray(Vec3 o,Vec3 d):o(o),d(d.norm()){} Vec3 at(double t)const{return o+d*t;} };

struct HitRecord { double t; Vec3 p,n; bool ff; int mat;
    void setN(const Ray&r,Vec3 on){ff=r.d.dot(on)<0;n=ff?on:-on;}
};

struct Sphere { Vec3 c; double r; int mat;
    bool hit(const Ray& ray,double tMin,double tMax,HitRecord& rec)const{
        Vec3 oc=ray.o-c; double a=ray.d.dot(ray.d),hb=oc.dot(ray.d),cc=oc.dot(oc)-r*r;
        double disc=hb*hb-a*cc; if(disc<0)return false;
        double sq=std::sqrt(disc); double root=(-hb-sq)/a;
        if(root<tMin||root>tMax){root=(-hb+sq)/a;if(root<tMin||root>tMax)return false;}
        rec.t=root;rec.p=ray.at(root);rec.setN(ray,(rec.p-c)/r);rec.mat=mat;return true;
    }
};

struct Plane {
    int axis; double pos; double u0,u1,v0,v1; int mat; bool flip;
    bool hit(const Ray& ray,double tMin,double tMax,HitRecord& rec)const{
        double rayD,rayO;
        if(axis==0){rayD=ray.d.x;rayO=ray.o.x;}
        else if(axis==1){rayD=ray.d.y;rayO=ray.o.y;}
        else{rayD=ray.d.z;rayO=ray.o.z;}
        if(std::abs(rayD)<1e-10)return false;
        double t=(pos-rayO)/rayD;
        if(t<tMin||t>tMax)return false;
        Vec3 p=ray.at(t);
        double pu,pv;
        if(axis==0){pu=p.y;pv=p.z;}
        else if(axis==1){pu=p.x;pv=p.z;}
        else{pu=p.x;pv=p.y;}
        if(pu<u0||pu>u1||pv<v0||pv>v1)return false;
        rec.t=t;rec.p=p;
        Vec3 on;
        if(axis==0)on=flip?Vec3(-1,0,0):Vec3(1,0,0);
        else if(axis==1)on=flip?Vec3(0,-1,0):Vec3(0,1,0);
        else on=flip?Vec3(0,0,-1):Vec3(0,0,1);
        rec.setN(ray,on);rec.mat=mat;return true;
    }
};

bool sceneHit(const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
              const Ray& ray, HitRecord& rec) {
    bool hit=false; double tMax=1e18; HitRecord tmp;
    for(auto&s:spheres)if(s.hit(ray,1e-4,tMax,tmp)){hit=true;tMax=tmp.t;rec=tmp;}
    for(auto&p:planes)if(p.hit(ray,1e-4,tMax,tmp)){hit=true;tMax=tmp.t;rec=tmp;}
    return hit;
}

int main() {
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;
    
    // 材质: 0=white, 1=red, 2=green, 3=mirror, 4=glass, 5=light
    // 地板
    planes.push_back({1,-1.0,-1.0,1.0,-1.0,1.0,0,false});
    // 天花板
    planes.push_back({1,1.0,-1.0,1.0,-1.0,1.0,0,true});
    // 背墙
    planes.push_back({2,1.0,-1.0,1.0,-1.0,1.0,0,true});
    // 左墙 红
    planes.push_back({0,-1.0,-1.0,1.0,-1.0,1.0,1,false});
    // 右墙 绿
    planes.push_back({0,1.0,-1.0,1.0,-1.0,1.0,2,true});
    // 光源
    planes.push_back({1,0.999,-0.35,0.35,-0.35,0.35,5,true});
    
    spheres.push_back({{-0.45,-0.65,0.2},0.35,3});
    spheres.push_back({{0.45,-0.65,-0.2},0.35,4});
    
    // 相机
    Vec3 camPos(0,0,-2.8);
    Vec3 camDir(0,0,1);
    Vec3 worldUp(0,1,0);
    Vec3 camRight = camDir.cross(worldUp).norm(); // (-1,0,0)
    Vec3 camUp = camRight.cross(camDir).norm();   // (0,1,0)
    double fov = 40.0*M_PI/180.0;
    double halfH = std::tan(fov/2.0);
    
    printf("halfH=%.3f, camRight=(%.2f,%.2f,%.2f), camUp=(%.2f,%.2f,%.2f)\n",
           halfH, camRight.x,camRight.y,camRight.z, camUp.x,camUp.y,camUp.z);
    
    int W=16,H=16;
    int hitCount=0;
    std::string matNames[]={"white","red","green","mirror","glass","light"};
    
    for(int py=0;py<H;py++) for(int px=0;px<W;px++) {
        double u=((px+0.5)/W*2-1)*halfH;
        double v=(1-(py+0.5)/H*2)*halfH;
        Ray ray(camPos,(camDir+camRight*u+camUp*v).norm());
        HitRecord rec;
        if(sceneHit(spheres,planes,ray,rec)) {
            hitCount++;
            if(hitCount<=5)
                printf("Hit at (px=%d,py=%d): mat=%s p=(%.2f,%.2f,%.2f)\n",
                       px,py,matNames[rec.mat].c_str(),rec.p.x,rec.p.y,rec.p.z);
        }
    }
    printf("Total hits: %d/%d\n", hitCount, W*H);
    
    // 测试光子从光源出发能打到什么
    printf("\n--- 光子追踪测试 ---\n");
    // 光源在 y=0.999, 朝下发射
    Ray photonRay(Vec3(0,0.999,0), Vec3(0,-1,0));
    HitRecord prec;
    if(sceneHit(spheres,planes,photonRay,prec)) {
        printf("光子打到: mat=%s p=(%.2f,%.2f,%.2f)\n",
               matNames[prec.mat].c_str(),prec.p.x,prec.p.y,prec.p.z);
    } else {
        printf("光子未命中!\n");
    }
    
    return 0;
}
