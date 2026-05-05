#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

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
    bool isZero()const{return x==0&&y==0&&z==0;}
};

struct Ray {
    Vec3 o, d;
    Ray(Vec3 o, Vec3 d):o(o),d(d.norm()){}
    Vec3 at(double t)const{return o+d*t;}
};

struct HitRecord {
    double t; Vec3 p, n; bool ff; int mat;
    void setNorm(const Ray& r, Vec3 on) {
        ff = r.d.dot(on) < 0;
        n = ff ? on : -on;
    }
};

struct Sphere {
    Vec3 c; double r; int mat;
    bool hit(const Ray& ray, double tMin, double tMax, HitRecord& rec)const{
        Vec3 oc=ray.o-c;
        double a=ray.d.dot(ray.d),hb=oc.dot(ray.d),cc=oc.dot(oc)-r*r;
        double disc=hb*hb-a*cc; if(disc<0)return false;
        double sq=std::sqrt(disc);
        double root=(-hb-sq)/a;
        if(root<tMin||root>tMax){root=(-hb+sq)/a;if(root<tMin||root>tMax)return false;}
        rec.t=root; rec.p=ray.at(root); rec.setNorm(ray,(rec.p-c)/r); rec.mat=mat;
        return true;
    }
};

// 简单场景：一个球 + 背景
int main() {
    Sphere sphere{{0,0,0}, 0.5, 0};
    
    Vec3 camPos(0, 0, -2.8);
    Vec3 camDir = Vec3(0,0,0) - camPos; // 指向原点
    camDir = camDir.norm();
    Vec3 worldUp(0,1,0);
    Vec3 camRight = camDir.cross(worldUp).norm(); // 注意：Z x Y = -X
    Vec3 camUp = camRight.cross(camDir).norm();
    
    printf("camRight: %.3f %.3f %.3f\n", camRight.x, camRight.y, camRight.z);
    printf("camUp: %.3f %.3f %.3f\n", camUp.x, camUp.y, camUp.z);
    
    double fov = 60.0 * M_PI / 180.0;
    double halfH = std::tan(fov/2.0);
    
    int W=64, H=64;
    int hits = 0;
    for (int py = 0; py < H; py++) {
        for (int px = 0; px < W; px++) {
            double u = ((px+0.5)/W*2-1)*halfH;
            double v = (1-(py+0.5)/H*2)*halfH;
            Vec3 d = (camDir + camRight*u + camUp*v).norm();
            Ray ray(camPos, d);
            HitRecord rec;
            if (sphere.hit(ray, 0.001, 1e9, rec)) hits++;
        }
    }
    printf("Sphere hits: %d/%d = %.1f%%\n", hits, W*H, 100.0*hits/(W*H));
    
    return 0;
}
