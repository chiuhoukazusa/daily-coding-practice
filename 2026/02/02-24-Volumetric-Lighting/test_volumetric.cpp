#include <iostream>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double t) const { return Vec3(x*t, y*t, z*t); }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { return *this / length(); }
    Vec3 operator/(double t) const { return Vec3(x/t, y/t, z/t); }
};

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(double t) const { return origin + direction * t; }
};

Vec3 simple_volumetric(const Ray& ray, const Vec3& light_pos, double max_dist) {
    const int STEPS = 20;
    const double SCATTER = 0.5;
    double step_size = max_dist / STEPS;
    Vec3 accum(0, 0, 0);
    
    std::cout << "Ray marching from " << ray.origin.x << "," << ray.origin.y << "," << ray.origin.z << std::endl;
    
    for (int i = 0; i < STEPS; i++) {
        double t = (i + 0.5) * step_size;
        Vec3 pos = ray.at(t);
        Vec3 to_light = light_pos - pos;
        double dist = to_light.length();
        double atten = 1.0 / (1.0 + 0.1 * dist);
        double contrib = SCATTER * step_size * atten;
        accum = accum + Vec3(1, 1, 1) * contrib;
        
        if (i < 3) {
            std::cout << "  Step " << i << ": pos=(" << pos.x << "," << pos.y << "," << pos.z 
                      << ") dist=" << dist << " contrib=" << contrib << std::endl;
        }
    }
    
    std::cout << "Final accumulated: " << accum.x << "," << accum.y << "," << accum.z << std::endl;
    return accum;
}

int main() {
    Vec3 camera(0, 0, 0);
    Vec3 light(5, 3, -2);
    Ray ray(camera, Vec3(0, 0, -1));
    
    Vec3 result = simple_volumetric(ray, light, 10.0);
    
    std::cout << "\nResult RGB: (" << result.x << ", " << result.y << ", " << result.z << ")" << std::endl;
    std::cout << "As 0-255: (" << int(result.x * 255.99) << ", " 
              << int(result.y * 255.99) << ", " << int(result.z * 255.99) << ")" << std::endl;
    
    return 0;
}
