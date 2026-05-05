// 快速诊断：检查 camera pass 是否命中场景

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <cstdio>
#include <algorithm>

struct Vec3 {
    double x,y,z;
    Vec3(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(double t)const{return{x/t,y/t,z/t};}
    double dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    double len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{double l=len();return l>1e-12?*this/l:Vec3(0,1,0);}
};

int main() {
    // 相机参数（同 main.cpp）
    Vec3 camPos(0, 0, -2.8);
    Vec3 camDir(0, 0, 1);
    Vec3 camUp(0, 1, 0);
    Vec3 camRight = camDir.cross(camUp).norm();
    camUp = camRight.cross(camDir).norm();
    
    printf("camRight: %.3f %.3f %.3f\n", camRight.x, camRight.y, camRight.z);
    printf("camUp: %.3f %.3f %.3f\n", camUp.x, camUp.y, camUp.z);
    
    double fov = 40.0 * M_PI / 180.0;
    double halfH = std::tan(fov / 2.0);
    double halfW = halfH; // 512x512
    
    // 测试中心射线
    Vec3 centerDir = (camDir + camRight * 0 + camUp * 0).norm();
    printf("Center ray dir: %.3f %.3f %.3f\n", centerDir.x, centerDir.y, centerDir.z);
    
    // 中心射线是否打到背墙 z=1?
    // ray: origin=(0,0,-2.8), dir=(0,0,1)
    // z=1 at t = (1 - (-2.8)) / 1 = 3.8
    double t_back = (1.0 - (-2.8)) / 1.0;
    printf("t to back wall (z=1): %.2f\n", t_back);
    Vec3 hitPt = Vec3(0,0,-2.8) + centerDir * t_back;
    printf("Hit point: %.3f %.3f %.3f\n", hitPt.x, hitPt.y, hitPt.z);
    
    // 检查边界：u0=-1,u1=1,v0=-1,v1=1
    printf("In bounds? x=%.2f [%d], y=%.2f [%d]\n", 
           hitPt.x, (hitPt.x>=-1&&hitPt.x<=1),
           hitPt.y, (hitPt.y>=-1&&hitPt.y<=1));
    
    // 测试角射线（左上角）
    double u = -1.0 * halfW, v = 1.0 * halfH;
    Vec3 cornerDir = (camDir + camRight * u + camUp * v).norm();
    printf("\nCorner ray dir: %.3f %.3f %.3f\n", cornerDir.x, cornerDir.y, cornerDir.z);
    double tz = (1.0 - (-2.8)) / cornerDir.z;
    Vec3 cornerHit = Vec3(0,0,-2.8) + cornerDir * tz;
    printf("Corner hits back wall at: %.3f %.3f %.3f\n", cornerHit.x, cornerHit.y, cornerHit.z);
    printf("halfH=%.3f halfW=%.3f\n", halfH, halfW);
    
    return 0;
}
