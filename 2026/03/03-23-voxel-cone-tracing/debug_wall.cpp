#include <cmath>
#include <iostream>
#include <algorithm>

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x),y(y),z(z){}
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return {x/l,y/l,z/l}; }
};

int main() {
    // Camera: position z=3.5, target (0,0,0)
    // A pixel hitting the left (red) wall
    // Left wall: x = -1, normal with isFlip=true -> points right (+x direction)
    
    Vec3 camPos(0, 0, 3.5f);
    // Ray direction toward left wall center (approx)
    Vec3 wallPoint(-0.98f, 0.0f, -0.3f);
    Vec3 rd = (wallPoint - camPos).norm();
    
    std::cout << "Ray direction: " << rd.x << " " << rd.y << " " << rd.z << "\n";
    
    // Wall normal with isFlip=true: left wall's outward normal is (-1,0,0), flipped = (+1,0,0)
    Vec3 wallNormal(1, 0, 0);  // after isFlip=true
    
    // Light position: (0, 0.96, 0), above
    Vec3 hitPoint(-0.98f, 0.0f, -0.3f);
    Vec3 lightPos(0, 0.96f, 0);
    Vec3 toLight = (lightPos - hitPoint).norm();
    
    float NdotL = wallNormal.dot(toLight);
    std::cout << "Wall normal (after flip): " << wallNormal.x << " " << wallNormal.y << " " << wallNormal.z << "\n";
    std::cout << "toLight: " << toLight.x << " " << toLight.y << " " << toLight.z << "\n";
    std::cout << "NdotL (should be > 0 for lit): " << NdotL << "\n";
    
    if (NdotL > 0) std::cout << "✓ Wall SHOULD be lit\n";
    else std::cout << "✗ Wall in shadow (NdotL <= 0)\n";
    
    // Also check: shadow ray from wall point to light - does it hit anything?
    // The ceiling is at y=1, so toLight goes from y=0 upward, should reach light
    std::cout << "\ntoLight y component: " << toLight.y << " (should be positive, going up to light)\n";
    
    return 0;
}
