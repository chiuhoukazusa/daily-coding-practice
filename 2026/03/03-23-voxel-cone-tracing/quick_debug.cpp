// Reproduce exactly what the renderer does for one pixel hitting the left red wall
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <climits>

struct Vec3 {
    float x, y, z;
    Vec3(float a=0,float b=0,float c=0):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3& o) const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o) const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t) const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t) const{return{x/t,y/t,z/t};}
    float dot(const Vec3& o) const{return x*o.x+y*o.y+z*o.z;}
    float len() const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm() const{float l=len();return{x/l,y/l,z/l};}
};

struct HitRecord { float t=1e30f; bool hit=false; Vec3 point, normal; bool isLight=false; Vec3 albedo; };

bool hitBox(Vec3 ro, Vec3 rd, Vec3 bmin, Vec3 bmax, bool flip, HitRecord& rec){
    auto inv=[](float v){return 1.0f/(std::abs(v)>1e-9f?v:1e-9f);};
    float ix=inv(rd.x),iy=inv(rd.y),iz=inv(rd.z);
    float tx0=(bmin.x-ro.x)*ix,tx1=(bmax.x-ro.x)*ix;
    float ty0=(bmin.y-ro.y)*iy,ty1=(bmax.y-ro.y)*iy;
    float tz0=(bmin.z-ro.z)*iz,tz1=(bmax.z-ro.z)*iz;
    float tmin=std::max({std::min(tx0,tx1),std::min(ty0,ty1),std::min(tz0,tz1)});
    float tmax=std::min({std::max(tx0,tx1),std::max(ty0,ty1),std::max(tz0,tz1)});
    if(tmax<tmin||tmax<1e-4f) return false;
    float t=(tmin>1e-4f)?tmin:tmax;
    if(t>=rec.t) return false;
    rec.t=t; rec.hit=true; rec.point=ro+rd*t;
    Vec3 c=(bmin+bmax)*0.5f;
    Vec3 d=rec.point-c, sz=(bmax-bmin)*0.5f;
    Vec3 n(0);
    if(std::abs(d.x/sz.x)>std::abs(d.y/sz.y)&&std::abs(d.x/sz.x)>std::abs(d.z/sz.z))
        n.x=(d.x>0)?1:-1;
    else if(std::abs(d.y/sz.y)>std::abs(d.z/sz.z))
        n.y=(d.y>0)?1:-1;
    else n.z=(d.z>0)?1:-1;
    rec.normal=flip?n*(-1.0f):n;
    return true;
}

// same as operator* for Vec3
Vec3 vadd(Vec3 a, Vec3 b){return a+b;}

int main(){
    struct Wall{ const char* name; Vec3 mn,mx; bool flip; Vec3 albedo; bool isLight; };
    std::vector<Wall> walls = {
        {"Floor",    {-1,-1,-1},{1,-0.98f,1}, true, {0.8f,0.8f,0.8f}, false},
        {"Ceiling",  {-1,0.98f,-1},{1,1,1},   true, {0.8f,0.8f,0.8f}, false},
        {"BackWall", {-1,-1,-1},{1,1,-0.98f},  true, {0.8f,0.8f,0.8f}, false},
        {"LeftWall", {-1,-1,-1},{-0.98f,1,1},  true, {0.85f,0.1f,0.1f}, false},
        {"RightWall",{0.98f,-1,-1},{1,1,1},    true, {0.1f,0.75f,0.1f}, false},
        {"LightBox", {-0.3f,0.94f,-0.3f},{0.3f,0.96f,0.3f}, false, {1,1,1}, true},
    };
    
    // Camera shooting toward left wall
    Vec3 ro(0,0,3.5f);
    // direction toward left wall center ~(-0.98, 0, -0.3) 
    Vec3 rd = (Vec3(-0.98f,0,-0.3f)-ro).norm();
    
    HitRecord best;
    const char* hitName = nullptr;
    for(auto& w:walls){
        HitRecord tmp = best;
        if(hitBox(ro,rd,w.mn,w.mx,w.flip,tmp)){
            best=tmp; best.isLight=w.isLight; best.albedo=w.albedo;
            hitName=w.name;
        }
    }
    
    std::cout << "Hit: " << (hitName?hitName:"none") << "\n";
    std::cout << "Normal: " << best.normal.x << " " << best.normal.y << " " << best.normal.z << "\n";
    std::cout << "Point: " << best.point.x << " " << best.point.y << " " << best.point.z << "\n";
    
    if(!best.hit || best.isLight) { std::cout << "Not shading\n"; return 0; }
    
    // Direct light
    Vec3 lightP(0, 0.96f, 0);
    Vec3 toLight = lightP - best.point;
    float distSq = toLight.dot(toLight);
    float dist2 = std::sqrt(distSq);
    Vec3 lightDir = toLight / dist2;
    
    float NdotL = std::max(0.0f, best.normal.dot(lightDir));
    std::cout << "NdotL: " << NdotL << "\n";
    if(NdotL < 1e-6f){ std::cout << "FAIL: NdotL too small\n"; return 0; }
    
    // Shadow ray
    Vec3 shadowOrig = best.point + best.normal * 1e-4f;
    HitRecord shadow;
    for(auto& w:walls){
        HitRecord tmp=shadow;
        if(hitBox(shadowOrig,lightDir,w.mn,w.mx,w.flip,tmp)){
            shadow=tmp; shadow.isLight=w.isLight;
        }
    }
    
    std::cout << "Shadow hit: " << shadow.hit << " t=" << shadow.t << " dist2=" << dist2 << "\n";
    bool inShadow = shadow.hit && shadow.t < dist2-0.01f && !shadow.isLight;
    std::cout << "In shadow: " << inShadow << "\n";
    
    if(!inShadow){
        float atten = 1.0f/(distSq+0.01f);
        Vec3 radiance = Vec3(15,14,12)*atten*20.0f;
        Vec3 diffuse = best.albedo / float(M_PI) * 1.0f;
        Vec3 r_=Vec3(radiance.x*diffuse.x,radiance.y*diffuse.y,radiance.z*diffuse.z); Vec3 result=r_*NdotL;
        std::cout << "RESULT color: " << result.x << " " << result.y << " " << result.z << "\n";
    }
    return 0;
}
