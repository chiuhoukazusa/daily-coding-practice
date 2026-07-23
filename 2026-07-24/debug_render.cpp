#include <iostream>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double l=length(); return l>1e-12 ? (*this)/l : Vec3(0,0,0); }
};

int main() {
    // Camera
    Vec3 eye(3*cos(0.5)*cos(0.6), 3*sin(0.5), 3*cos(0.5)*sin(0.6));
    std::cout << "Eye: " << eye.x << " " << eye.y << " " << eye.z << std::endl;
    
    Vec3 center(0, 0, 0);
    Vec3 up(0, 1, 0);
    
    Vec3 vz = (eye - center).normalized();
    Vec3 vx = up.cross(vz).normalized();
    Vec3 vy = vz.cross(vx).normalized();
    
    std::cout << "vz: " << vz.x << " " << vz.y << " " << vz.z << std::endl;
    std::cout << "vx: " << vx.x << " " << vx.y << " " << vx.z << std::endl;  
    std::cout << "vy: " << vy.x << " " << vy.y << " " << vy.z << std::endl;
    
    // Test vertex
    Vec3 p(0, -0.6, 1.0);
    Vec3 rel = p - eye;
    double depth = rel.dot(vz);
    double sx = rel.dot(vx) / depth;
    double sy = rel.dot(vy) / depth;
    double px = 400 + sx * 800 * 0.4;
    double py = 300 - sy * 600 * 0.4;
    
    std::cout << "Test vertex (0,-0.6,1): screen (" << px << "," << py << ") depth=" << depth << std::endl;
    
    // Test face normal / back face culling
    Vec3 p0(0, -0.6, 1.0);
    Vec3 p1(0.951, -0.6, 0.309);
    Vec3 p2(0.588, -0.6, -0.809);
    Vec3 fn = (p1-p0).cross(p2-p0).normalized();
    Vec3 vd = (eye - p0).normalized();
    std::cout << "Bottom triangle face normal: " << fn.x << " " << fn.y << " " << fn.z << std::endl;
    std::cout << "View dir to p0: " << vd.x << " " << vd.y << " " << vd.z << std::endl;
    std::cout << "fn.dot(vd) = " << fn.dot(vd) << " (should be > 0 for front-facing)" << std::endl;
    
    return 0;
}
