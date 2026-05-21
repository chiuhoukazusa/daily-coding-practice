// Debug: test ray-scene intersection
#include <cmath>
#include <vector>
#include <cstdio>

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(float s) const { return {x/s, y/s, z/s}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalized() const { float l=len(); return l>1e-6f?*this/l:Vec3(0,1,0); }
};

struct Triangle { Vec3 v[3]; };

static bool rayTri(Vec3 ro, Vec3 rd, const Triangle& tri, float& t) {
    Vec3 e1 = tri.v[1]-tri.v[0], e2=tri.v[2]-tri.v[0];
    Vec3 h=rd.cross(e2);
    float a=e1.dot(h);
    if(std::abs(a)<1e-7f) return false;
    float f=1/a;
    Vec3 s=ro-tri.v[0];
    float u=f*s.dot(h);
    if(u<0||u>1) return false;
    Vec3 q=s.cross(e1);
    float v=f*rd.dot(q);
    if(v<0||u+v>1) return false;
    t=f*e2.dot(q);
    return t>1e-4f;
}

int main() {
    float s=2.5f;
    // Floor: {-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s}
    // Quads split as: tri1 = v0,v1,v2 and tri2 = v0,v2,v3
    
    // Camera
    Vec3 camPos = {0, 0, -4.5f};
    Vec3 camFwd = {0, 0, 1};
    Vec3 camRight = {1, 0, 0};
    Vec3 camUp = {0, 1, 0};
    float halfH = std::tan(60.0f*3.14159f/360.0f);
    float halfW = halfH * 800.0f/600.0f;
    
    // Test center ray
    float u=0, v=0;
    Vec3 dir = (camFwd + camRight*(u*halfW) + camUp*(v*halfH)).normalized();
    printf("Center ray dir: (%.3f, %.3f, %.3f)\n", dir.x, dir.y, dir.z);
    
    // Back wall {-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s}
    Triangle backTri1;
    backTri1.v[0]={-s,-s,s}; backTri1.v[1]={s,-s,s}; backTri1.v[2]={s,s,s};
    float t1=-1;
    if(rayTri(camPos,dir,backTri1,t1)) printf("Hit back wall tri1 at t=%.3f\n",t1);
    Triangle backTri2;
    backTri2.v[0]={-s,-s,s}; backTri2.v[1]={s,s,s}; backTri2.v[2]={-s,s,s};
    float t2=-1;
    if(rayTri(camPos,dir,backTri2,t2)) printf("Hit back wall tri2 at t=%.3f\n",t2);

    // Test floor
    Triangle floorTri1;
    floorTri1.v[0]={-s,-s,-s}; floorTri1.v[1]={s,-s,-s}; floorTri1.v[2]={s,-s,s};
    float t3=-1;
    if(rayTri(camPos,dir,floorTri1,t3)) printf("Hit floor tri1 at t=%.3f\n",t3);
    else printf("Miss floor tri1\n");
    
    printf("Camera Z=%f, scene extends from z=%f to %f\n", camPos.z, -s, s);
    printf("Ray at t=7: (%.2f, %.2f, %.2f)\n",
        camPos.x+dir.x*7, camPos.y+dir.y*7, camPos.z+dir.z*7);
    
    return 0;
}
