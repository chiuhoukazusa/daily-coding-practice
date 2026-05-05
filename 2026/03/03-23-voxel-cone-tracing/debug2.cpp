// Quick test: does shadow ray from left wall hit ceiling?
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include <climits>

struct Vec3 {
    float x, y, z;
    Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x,y-o.y,z-o.z}; }
    Vec3 operator*(float t) const { return {x*t,y*t,z*t}; }
    Vec3 operator/(float t) const { return {x/t,y/t,z/t}; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return {x/l,y/l,z/l}; }
};

// AABB intersection
bool hitBox(Vec3 ro, Vec3 rd, Vec3 bmin, Vec3 bmax, float& t) {
    auto inv = [](float v){ return 1.0f/(std::abs(v)>1e-9f?v:1e-9f); };
    float ix=inv(rd.x), iy=inv(rd.y), iz=inv(rd.z);
    float tx0=(bmin.x-ro.x)*ix, tx1=(bmax.x-ro.x)*ix;
    float ty0=(bmin.y-ro.y)*iy, ty1=(bmax.y-ro.y)*iy;
    float tz0=(bmin.z-ro.z)*iz, tz1=(bmax.z-ro.z)*iz;
    float tmin=std::max({std::min(tx0,tx1),std::min(ty0,ty1),std::min(tz0,tz1)});
    float tmax=std::min({std::max(tx0,tx1),std::max(ty0,ty1),std::max(tz0,tz1)});
    if(tmax<tmin||tmax<1e-4f) return false;
    t=(tmin>1e-4f)?tmin:tmax;
    return true;
}

int main(){
    // Hit point on left wall (x=-0.98), y=0, z=-0.3
    Vec3 hitPt(-0.98f, 0.0f, -0.3f);
    // Normal after isFlip=true: (+1,0,0)
    Vec3 normal(1.0f, 0, 0);
    // Shadow ray origin
    Vec3 shadowOrig = hitPt + normal * 1e-4f;
    
    // Light at (0, 0.96, 0)
    Vec3 lightP(0, 0.96f, 0);
    Vec3 toLight = lightP - hitPt;
    float dist2 = toLight.len();
    Vec3 lightDir = toLight.norm();
    
    std::cout << "Shadow ray origin: " << shadowOrig.x << " " << shadowOrig.y << " " << shadowOrig.z << "\n";
    std::cout << "Shadow ray dir: " << lightDir.x << " " << lightDir.y << " " << lightDir.z << "\n";
    std::cout << "Dist to light: " << dist2 << "\n\n";
    
    // Test each wall
    struct Wall { const char* name; Vec3 mn, mx; };
    Wall walls[] = {
        {"Floor",    Vec3(-1,-1,-1), Vec3(1,-0.98f,1)},
        {"Ceiling",  Vec3(-1,0.98f,-1), Vec3(1,1,1)},
        {"BackWall", Vec3(-1,-1,-1), Vec3(1,1,-0.98f)},
        {"LeftWall", Vec3(-1,-1,-1), Vec3(-0.98f,1,1)},
        {"RightWall",Vec3(0.98f,-1,-1), Vec3(1,1,1)},
        {"LightBox", Vec3(-0.3f,0.94f,-0.3f), Vec3(0.3f,0.96f,0.3f)},
    };
    
    float bestT = 1e30f;
    const char* blocker = nullptr;
    for(auto& w : walls){
        float t;
        if(hitBox(shadowOrig, lightDir, w.mn, w.mx, t)){
            std::cout << w.name << " hit at t=" << t << (t < dist2-0.01f ? " [BLOCKS]" : " [behind light]") << "\n";
            if(t < bestT && t < dist2-0.01f){ bestT=t; blocker=w.name; }
        }
    }
    if(blocker) std::cout << "\n=> Shadow blocked by: " << blocker << "\n";
    else std::cout << "\n=> No blocker, wall should be lit!\n";
    return 0;
}
