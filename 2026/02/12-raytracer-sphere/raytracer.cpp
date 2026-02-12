// Simple Ray Tracer - Sphere Rendering
// Date: 2026-02-12

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <limits>

// Vector3 class for 3D operations
class Vector3 {
public:
    double x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
    double dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3 normalize() const {
        double len = length();
        return Vector3(x / len, y / len, z / len);
    }
};

// Sphere class
class Sphere {
public:
    Vector3 center;
    double radius;
    Vector3 color;
    
    Sphere(const Vector3& c, double r, const Vector3& col) : center(c), radius(r), color(col) {}
    
    bool intersect(const Vector3& origin, const Vector3& direction, double& t) const {
        Vector3 oc = origin - center;
        double a = direction.dot(direction);
        double b = 2.0 * oc.dot(direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) {
            return false;
        }
        
        discriminant = std::sqrt(discriminant);
        double t0 = (-b - discriminant) / (2.0 * a);
        double t1 = (-b + discriminant) / (2.0 * a);
        
        if (t0 > 0.1) {
            t = t0;
            return true;
        } else if (t1 > 0.1) {
            t = t1;
            return true;
        }
        
        return false;
    }
};

// Simple PPM image writer
void writePPM(const std::string& filename, int width, int height, const std::vector<Vector3>& pixels) {
    std::ofstream file(filename);
    file << "P6\n" << width << " " << height << "\n255\n";
    
    for (const auto& pixel : pixels) {
        unsigned char r = static_cast<unsigned char>(std::min(255.0, std::max(0.0, pixel.x * 255.0)));
        unsigned char g = static_cast<unsigned char>(std::min(255.0, std::max(0.0, pixel.y * 255.0)));
        unsigned char b = static_cast<unsigned char>(std::min(255.0, std::max(0.0, pixel.z * 255.0)));
        file << r << g << b;
    }
    
    file.close();
}

int main() {
    std::cout << "Simple Ray Tracer - Sphere Rendering\n";
    std::cout << "=====================================\n";
    
    // Image dimensions
    const int width = 640;
    const int height = 480;
    
    // Camera setup
    Vector3 cameraOrigin(0, 0, 3);
    double aspectRatio = static_cast<double>(width) / height;
    double viewportHeight = 2.0;
    double viewportWidth = viewportHeight * aspectRatio;
    
    // Spheres in the scene
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vector3(0, 0, -1), 0.5, Vector3(0.8, 0.2, 0.2)));    // Red sphere
    spheres.push_back(Sphere(Vector3(1, 0, -1), 0.3, Vector3(0.2, 0.8, 0.2)));    // Green sphere
    spheres.push_back(Sphere(Vector3(-1, 0, -1), 0.4, Vector3(0.2, 0.2, 0.8)));   // Blue sphere
    
    // Background color (sky blue)
    Vector3 backgroundColor(0.5, 0.7, 1.0);
    
    // Render loop
    std::vector<Vector3> pixels(width * height);
    
    std::cout << "Rendering image (" << width << "x" << height << ")...\n";
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Convert pixel coordinates to viewport coordinates
            double u = (x + 0.5) / width;
            double v = ((height - y - 1) + 0.5) / height;  // Flip Y for correct orientation
            
            double viewportX = (u * 2.0 - 1.0) * viewportWidth / 2.0;
            double viewportY = (v * 2.0 - 1.0) * viewportHeight / 2.0;
            
            Vector3 rayDirection(viewportX, viewportY, -1.0);
            rayDirection = rayDirection.normalize();
            
            // Ray tracing
            double closestT = std::numeric_limits<double>::max();
            Vector3 pixelColor = backgroundColor;
            
            for (const auto& sphere : spheres) {
                double t;
                if (sphere.intersect(cameraOrigin, rayDirection, t)) {
                    if (t < closestT) {
                        closestT = t;
                        pixelColor = sphere.color;
                        
                        // Simple shading based on surface normal
                        Vector3 hitPoint = cameraOrigin + rayDirection * t;
                        Vector3 normal = (hitPoint - sphere.center).normalize();
                        double light = std::max(0.0, normal.dot(Vector3(0, 1, -1).normalize()));
                        pixelColor = pixelColor * (0.5 + 0.5 * light);
                    }
                }
            }
            
            pixels[y * width + x] = pixelColor;
        }
        
        // Progress indicator
        if (y % (height / 10) == 0) {
            std::cout << "Progress: " << (y * 100 / height) << "%\n";
        }
    }
    
    std::cout << "Writing output image...\n";
    writePPM("output.ppm", width, height, pixels);
    std::cout << "Rendering complete! Image saved as output.ppm\n";
    
    return 0;
}