#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <limits>
#include <algorithm>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    double length() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalized() const { double l=length(); Vec3 r = *this; if (l>1e-12) { r.x/=l; r.y/=l; r.z/=l; } return r; }
};

int main() {
    int w = 800, h = 600;
    std::vector<double> depth(w*h, std::numeric_limits<double>::max());
    std::vector<Vec3> color(w*h, Vec3(0,0,0));
    
    // Simple triangle test
    double tx0=100, ty0=100, tx1=700, ty1=100, tx2=400, ty2=500;
    
    auto edge = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
    };
    
    double area = edge(tx0, ty0, tx1, ty1, tx2, ty2);
    std::cout << "Triangle area: " << area << std::endl;
    
    int minX = std::max(0, (int)std::min({tx0, tx1, tx2}));
    int maxX = std::min(w-1, (int)std::max({tx0, tx1, tx2}));
    int minY = std::max(0, (int)std::min({ty0, ty1, ty2}));
    int maxY = std::min(h-1, (int)std::max({ty0, ty1, ty2}));
    
    std::cout << "BBox: " << minX << "-" << maxX << " x " << minY << "-" << maxY << std::endl;
    
    int filled = 0;
    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            double cx = x + 0.5, cy = y + 0.5;
            double w0 = edge(tx1, ty1, tx2, ty2, cx, cy);
            double w1 = edge(tx2, ty2, tx0, ty0, cx, cy);
            double w2 = edge(tx0, ty0, tx1, ty1, cx, cy);
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                depth[y*w+x] = 0;
                color[y*w+x] = Vec3(1,0,0);
                filled++;
            }
        }
    }
    
    std::cout << "Filled pixels: " << filled << std::endl;
    
    std::ofstream out("test.ppm");
    out << "P3\n" << w << " " << h << "\n255\n";
    for (int i = 0; i < w*h; i++) {
        int r = std::min(255, std::max(0, (int)(color[i].x*255)));
        int g = std::min(255, std::max(0, (int)(color[i].y*255)));
        int b = std::min(255, std::max(0, (int)(color[i].z*255)));
        out << r << " " << g << " " << b << "\n";
    }
    std::cout << "Saved test.ppm" << std::endl;
    return 0;
}
