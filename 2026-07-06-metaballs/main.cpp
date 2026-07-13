#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <cstring>

const int WIDTH = 800;
const int HEIGHT = 600;
const double THRESHOLD = 1.0;

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 normalize() const {
        double l = length();
        return (l > 1e-12) ? Vec3(x/l, y/l, z/l) : Vec3(0,0,0);
    }
};

struct Metaball {
    Vec3 center;
    double radius;
    double strength;
    
    double evaluate(const Vec3& p) const {
        Vec3 dir = p - center;
        double dist_sq = dir.dot(dir);
        double R_sq = radius * radius;
        if (dist_sq >= R_sq) return 0.0;
        double t = 1.0 - dist_sq / R_sq;
        return strength * t * t * t;
    }
};

double evaluateField(const Vec3& p, const std::vector<Metaball>& balls) {
    double sum = 0.0;
    for (const auto& b : balls) {
        sum += b.evaluate(p);
    }
    return sum;
}

Vec3 gradient(const Vec3& p, const std::vector<Metaball>& balls, double eps = 0.001) {
    double dx = evaluateField(Vec3(p.x+eps, p.y, p.z), balls) - evaluateField(Vec3(p.x-eps, p.y, p.z), balls);
    double dy = evaluateField(Vec3(p.x, p.y+eps, p.z), balls) - evaluateField(Vec3(p.x, p.y-eps, p.z), balls);
    double dz = evaluateField(Vec3(p.x, p.y, p.z+eps), balls) - evaluateField(Vec3(p.x, p.y, p.z-eps), balls);
    return Vec3(dx, dy, dz) * (1.0 / (2.0 * eps));
}

bool raymarch(const Vec3& origin, const Vec3& dir, const std::vector<Metaball>& balls,
              double& t_hit, Vec3& normal, double t_min=0.0, double t_max=20.0) {
    const double step = 0.01;
    const int max_steps = 4000;
    double t = t_min;
    double prev_val = evaluateField(origin + dir * t, balls);
    
    for (int i = 0; i < max_steps; i++) {
        t += step;
        if (t > t_max) return false;
        Vec3 p = origin + dir * t;
        double val = evaluateField(p, balls);
        
        if (prev_val < THRESHOLD && val >= THRESHOLD) {
            double t_lo = t - step;
            double t_hi = t;
            for (int r = 0; r < 8; r++) {
                double t_mid = (t_lo + t_hi) * 0.5;
                double v_mid = evaluateField(origin + dir * t_mid, balls);
                if (v_mid < THRESHOLD) t_lo = t_mid;
                else t_hi = t_mid;
            }
            t_hit = t_lo;
            normal = gradient(origin + dir * t_hit, balls).normalize();
            return true;
        }
        prev_val = val;
    }
    return false;
}

int main() {
    std::vector<Metaball> balls;
    
    // Place metaballs centered at origin, along XY plane, Z varies slightly
    // Use larger radius and strength to ensure field exceeds threshold
    // radius=3, strength=2 → at center field=2, at distance 1.5 field=2*(1-2.25/9)^3=2*(0.75)^3=2*0.422=0.844
    // Sum of multiple balls at midpoints should exceed 1.0
    
    balls.push_back({Vec3(-2.0,  1.0, 0.5), 3.5, 2.0});
    balls.push_back({Vec3( 2.0,  1.0, -0.3), 3.5, 2.0});
    balls.push_back({Vec3( 0.0, -1.5, 0.0), 3.5, 2.0});
    balls.push_back({Vec3(-1.0, -0.5, 1.5), 2.5, 1.5});
    balls.push_back({Vec3( 1.5, -0.3, -1.2), 2.5, 1.5});
    

    Vec3 light_dir = Vec3(1.0, 1.5, 2.0).normalize();
    Vec3 ambient_color(0.15, 0.15, 0.22);
    Vec3 diffuse_color(0.55, 0.65, 0.90);
    Vec3 specular_color(1.0, 1.0, 0.9);
    double specular_power = 100.0;
    
    // Camera: looking along -Z, orthographic
    // World space: camera at z=+6, metaballs near z=0
    // Ray origin on a plane at z=+3 (where field=0 for metaballs), march to z=-3
    double view_plane_z = 3.0;
    double far_plane_z = -4.0;
    double view_width = 8.0;
    double view_height = view_width * HEIGHT / WIDTH;
    
    Vec3 cam_pos(0.0, 0.0, 7.0);
    
    std::vector<unsigned char> image(WIDTH * HEIGHT * 3, 0);
    
    int hit_count = 0;
    for (int py = 0; py < HEIGHT; py++) {
        for (int px = 0; px < WIDTH; px++) {
            double u = (px + 0.5) / WIDTH;
            double v = (py + 0.5) / HEIGHT;
            double wx = (u - 0.5) * view_width;
            double wy = (0.5 - v) * view_height;
            
            Vec3 ray_origin(wx, wy, view_plane_z);
            Vec3 ray_dir(0.0, 0.0, -1.0);
            
            double t_hit;
            Vec3 normal;
            
            if (raymarch(ray_origin, ray_dir, balls, t_hit, normal, 0.0, view_plane_z - far_plane_z)) {
                hit_count++;
                Vec3 hit_point = ray_origin + ray_dir * t_hit;
                Vec3 view_dir = (cam_pos - hit_point).normalize();
                Vec3 half_vec = (light_dir + view_dir).normalize();
                
                double NdotL = std::max(0.0, normal.dot(light_dir));
                double NdotH = std::max(0.0, normal.dot(half_vec));
                double spec = std::pow(NdotH, specular_power);
                
                double r = ambient_color.x + diffuse_color.x * NdotL + specular_color.x * spec;
                double g = ambient_color.y + diffuse_color.y * NdotL + specular_color.y * spec;
                double b = ambient_color.z + diffuse_color.z * NdotL + specular_color.z * spec;
                
                int idx = (py * WIDTH + px) * 3;
                image[idx + 0] = (unsigned char)(std::min(255.0, r * 255.0));
                image[idx + 1] = (unsigned char)(std::min(255.0, g * 255.0));
                image[idx + 2] = (unsigned char)(std::min(255.0, b * 255.0));
            }
        }
    }
    
    std::ofstream ofs("metaballs_output.ppm", std::ios::binary);
    ofs << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    ofs.write(reinterpret_cast<char*>(image.data()), image.size());
    ofs.close();
    
    long long sum_r = 0, sum_g = 0, sum_b = 0;
    double min_val = 255, max_val = 0;
    int nonzero = 0;
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        int r = image[i*3], g = image[i*3+1], b = image[i*3+2];
        sum_r += r; sum_g += g; sum_b += b;
        int avg = (r + g + b) / 3;
        if (avg > 0) nonzero++;
        min_val = std::min(min_val, (double)avg);
        max_val = std::max(max_val, (double)avg);
    }
    
    int total = WIDTH * HEIGHT;
    double mean = ((double)(sum_r+sum_g+sum_b)) / (3.0 * total);
    
    double var = 0.0;
    for (int i = 0; i < total; i++) {
        int avg = (image[i*3] + image[i*3+1] + image[i*3+2]) / 3;
        double diff = avg - mean;
        var += diff * diff;
    }
    var /= total;
    double std_dev = std::sqrt(var);
    
    std::cout << "Metaballs Renderer Statistics:" << std::endl;
    std::cout << "Resolution: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "Hit count: " << hit_count << " (" << (100.0*hit_count/total) << "%)" << std::endl;
    std::cout << "Nonzero pixels: " << nonzero << " (" << (100.0*nonzero/total) << "%)" << std::endl;
    std::cout << "Pixel mean: " << mean << std::endl;
    std::cout << "Pixel std: " << std_dev << std::endl;
    std::cout << "Pixel range: [" << min_val << ", " << max_val << "]" << std::endl;
    std::cout << "Mean RGB: (" << (double)sum_r/total << ", " << (double)sum_g/total << ", " << (double)sum_b/total << ")" << std::endl;
    
    return 0;
}
