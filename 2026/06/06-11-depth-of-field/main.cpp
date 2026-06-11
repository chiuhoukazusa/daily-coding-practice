// Depth of Field Bokeh Renderer
// Thin lens model with aperture sampling for physically-based depth of field
// Compile: g++ main.cpp -o dof -std=c++17 -O2 -Wall -Wextra -Wno-missing-field-initializers

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <random>

// ============ Vector Math ============
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 mul(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }

    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    double len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 norm() const { double l = len(); return l > 1e-12 ? *this / l : Vec3(0,0,1); }
};

Vec3 operator*(double s, const Vec3& v) { return v * s; }

// ============ Random ============
struct RNG {
    std::mt19937 mt;
    RNG() : mt(42) {}
    double uniform() { return std::uniform_real_distribution<double>(0.0, 1.0)(mt); }
};

// ============ Poisson Disk Samples (for bokeh shape) ============
std::vector<Vec3> generate_poisson_disk(int n, RNG& rng) {
    std::vector<Vec3> points;
    const double min_dist = 0.85 / std::sqrt(double(n));
    
    while ((int)points.size() < n) {
        double best_dist = 0;
        Vec3 best;
        for (int attempt = 0; attempt < 100; ++attempt) {
            double angle = rng.uniform() * 2.0 * M_PI;
            double radius = std::sqrt(rng.uniform());
            Vec3 cand(radius * std::cos(angle), radius * std::sin(angle), 0);
            
            double min_d = 1e10;
            for (const auto& p : points) {
                double d = (cand - p).len();
                if (d < min_d) min_d = d;
            }
            if (min_d > best_dist) {
                best_dist = min_d;
                best = cand;
            }
        }
        if (best_dist > min_dist || points.empty()) {
            points.push_back(best);
        } else {
            break;
        }
    }
    return points;
}

// Generate hexagon aperture samples (6-blade iris)
std::vector<Vec3> generate_hexagon_aperture(int n, RNG& rng) {
    std::vector<Vec3> points;
    const double s3 = std::sqrt(3.0);
    for (int i = 0; i < n; ++i) {
        double u = rng.uniform();
        double v = rng.uniform();
        double a = u - 0.5;
        double b = v - 0.5;
        double x = a * 2.0;
        double y = b * 2.0 * s3;
        double fy = std::abs(y);
        double fx = std::abs(x);
        if (fx * s3 + fy > s3) {
            double t = (fx * s3 + fy - s3) / 4.0;
            x -= (x > 0 ? 1 : -1) * t * s3;
            y -= (y > 0 ? 1 : -1) * t;
        }
        points.push_back(Vec3(x * 0.5, y * 0.5 / s3, 0));
    }
    return points;
}

// ============ Materials & Scene ============
struct Material {
    Vec3 color;
    double roughness;
    double metallic;
};

struct Sphere {
    Vec3 center;
    double radius;
    Material mat;
};

struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;
};

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Vec3 ambient;
};

// ============ Ray-Sphere Intersection ============
struct Hit {
    double t;
    Vec3 point;
    Vec3 normal;
    Material mat;
    bool valid;
};

Hit intersect_sphere(const Vec3& ro, const Vec3& rd, const Sphere& s, double t_min) {
    Vec3 oc = ro - s.center;
    double a = rd.dot(rd);
    double b = 2.0 * oc.dot(rd);
    double c = oc.dot(oc) - s.radius * s.radius;
    double disc = b*b - 4*a*c;
    
    if (disc < 0) return {1e10, {}, {}, {}, false};
    
    double t = (-b - std::sqrt(disc)) / (2.0 * a);
    if (t < t_min) {
        t = (-b + std::sqrt(disc)) / (2.0 * a);
        if (t < t_min) return {1e10, {}, {}, {}, false};
    }
    
    Vec3 point = ro + rd * t;
    Vec3 normal = (point - s.center).norm();
    return {t, point, normal, s.mat, true};
}

Hit intersect_scene(const Vec3& ro, const Vec3& rd, const Scene& scene, double t_min = 0.001) {
    Hit best = {1e10, {}, {}, {}, false};
    for (const auto& s : scene.spheres) {
        Hit h = intersect_sphere(ro, rd, s, t_min);
        if (h.valid && h.t < best.t) best = h;
    }
    return best;
}

// ============ Shading ============
Vec3 shade(const Hit& hit, const Vec3& view_dir, const Scene& scene, RNG& /*rng*/) {
    Vec3 color = scene.ambient.mul(hit.mat.color) * 0.1;
    
    for (const auto& light : scene.lights) {
        Vec3 to_light = light.position - hit.point;
        double dist = to_light.len();
        Vec3 L = to_light / dist;
        
        // Shadow ray
        Hit shadow = intersect_scene(hit.point + L * 0.001, L, scene, 0.001);
        if (shadow.valid && shadow.t < dist) continue;
        
        double NdotL = std::max(0.0, hit.normal.dot(L));
        
        // Diffuse (Lambert)
        Vec3 diffuse = hit.mat.color * NdotL / M_PI;
        
        // Specular (Blinn-Phong)
        Vec3 H = (L + view_dir).norm();
        double NdotH = std::max(0.0, hit.normal.dot(H));
        double spec = std::pow(NdotH, 1.0 / (hit.mat.roughness * hit.mat.roughness + 0.001));
        Vec3 specular = (hit.mat.color * 0.3 + Vec3(1,1,1) * 0.7) * spec * hit.mat.metallic;
        
        double attenuation = light.intensity / (dist * dist);
        color = color + (diffuse + specular).mul(light.color) * attenuation;
    }
    
    return color;
}

// ============ Thin Lens Camera ============
struct Camera {
    Vec3 position;
    Vec3 look_at;
    Vec3 up;
    double focal_length;
    double focus_distance;
    double aperture_radius;
    double sensor_distance;
    
    Vec3 forward, right, world_up;
    
    void setup() {
        forward = (look_at - position).norm();
        world_up = up.norm();
        right = forward.cross(world_up).norm();
        world_up = right.cross(forward).norm();
        
        // Thin lens: 1/f = 1/u + 1/v
        sensor_distance = 1.0 / (1.0/focal_length - 1.0/focus_distance);
    }
    
    // Generate ray from lens sample toward focal-plane point
    // Sensor pixel (px,py) maps to focal-plane point through lens center
    void get_ray(double px, double py, double lx, double ly, Vec3& ro, Vec3& rd) const {
        // Scale factor: sensor z -> focal plane z through origin
        double scale = -focus_distance / sensor_distance;
        // Focal-plane point (through lens center, i.e., camera position)
        Vec3 focal_point = forward * focus_distance + right * (px * scale) + world_up * (py * scale);
        // Lens sample offset
        Vec3 lens_offset = right * lx + world_up * ly;
        // Ray from lens sample toward focal-plane point
        ro = position + lens_offset;
        rd = (position + focal_point - ro).norm();
    }
};

// ============ Clamp ============
inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============ Main ====
int main() {
    const int W = 800, H = 600;
    const int SAMPLES = 64;
    const int LENS_SAMPLES = 32;
    
    // Build scene
    Scene scene;
    scene.ambient = Vec3(0.15, 0.15, 0.20);
    
    // Spheres at clearly different depths, smaller, well-separated
    // Camera looks along +Z from origin
    
    // Foreground: red sphere at z=4.5 (should be noticeably blurred)
    scene.spheres.push_back({
        {-2.5, 0.3, 4.5}, 0.9, {{0.9, 0.25, 0.25}, 0.4, 0.05}
    });
    
    // Foreground-right: blue sphere at z=5.5 (slight foreground blur)
    scene.spheres.push_back({
        {2.0, -0.3, 5.5}, 0.8, {{0.25, 0.35, 0.9}, 0.35, 0.15}
    });
    
    // IN FOCUS: green sphere at z=9 (focal plane)
    scene.spheres.push_back({
        {0.0, 0.0, 9.0}, 1.3, {{0.25, 0.85, 0.35}, 0.3, 0.0}
    });
    
    // In focus region: small purple sphere at z=8.5
    scene.spheres.push_back({
        {1.5, 1.2, 8.5}, 0.5, {{0.6, 0.3, 0.8}, 0.5, 0.0}
    });
    
    // Background: gold metallic sphere at z=14
    scene.spheres.push_back({
        {-1.5, -0.8, 14.0}, 1.5, {{0.9, 0.75, 0.2}, 0.2, 0.55}
    });
    
    // Far background: white sphere at z=20
    scene.spheres.push_back({
        {2.5, 0.5, 20.0}, 2.0, {{0.85, 0.85, 0.9}, 0.25, 0.3}
    });
    
    // Ground
    scene.spheres.push_back({
        {0.0, -1003.0, 9.0}, 1000.0, {{0.55, 0.45, 0.35}, 0.95, 0.0}
    });
    
    // Ceiling
    scene.spheres.push_back({
        {0.0, 1003.0, 9.0}, 1000.0, {{0.15, 0.15, 0.25}, 0.95, 0.0}
    });
    
    // Back wall
    scene.spheres.push_back({
        {0.0, 0.0, 10020.0}, 10000.0, {{0.12, 0.12, 0.18}, 0.98, 0.0}
    });
    
    // Lights (boosted intensity for visibility)
    scene.lights.push_back({{5.0, 4.0, 2.0}, {1.0, 0.95, 0.80}, 300.0});
    scene.lights.push_back({{-4.0, 2.0, 4.0}, {0.5, 0.6, 1.0}, 200.0});
    scene.lights.push_back({{0.0, -1.0, 6.0}, {0.6, 0.7, 0.8}, 150.0});
    scene.lights.push_back({{0.0, 6.0, 8.0}, {1.0, 1.0, 1.0}, 80.0});
    
    // Camera: looking along +Z, focus at z=9 (green sphere)
    Camera cam;
    cam.position = Vec3(0, 0, 0);
    cam.look_at = Vec3(0, 0, 9);
    cam.up = Vec3(0, 1, 0);
    cam.focal_length = 5.0;
    cam.focus_distance = 9.0;
    cam.aperture_radius = 0.8; // moderate DoF
    cam.setup();
    
    printf("Camera: sensor_distance=%.2f, aperture=%.2f, focus=%.2f\n",
           cam.sensor_distance, cam.aperture_radius, cam.focus_distance);
    printf("Rendering %dx%d with %d spp, %d lens samples...\n", W, H, SAMPLES, LENS_SAMPLES);
    
    RNG rng;
    auto lens_samples = generate_poisson_disk(LENS_SAMPLES, rng);
    auto hex_samples = generate_hexagon_aperture(LENS_SAMPLES, rng);
    
    printf("Generated %zu Poisson disk + %zu hexagon lens samples\n",
           lens_samples.size(), hex_samples.size());
    
    std::vector<double> accum(W * H * 3, 0.0);
    double sensor_height = 6.0;
    double sensor_width = sensor_height * W / H;
    
    for (int y = 0; y < H; ++y) {
        if (y % 60 == 0) printf("  row %d/%d\n", y, H);
        
        for (int x = 0; x < W; ++x) {
            Vec3 pixel_color;
            
            for (int s = 0; s < SAMPLES; ++s) {
                double sx = (x + rng.uniform()) / W - 0.5;
                double sy = (y + rng.uniform()) / H - 0.5;
                double px = sx * sensor_width;
                double py = -sy * sensor_height;
                
                // Alternate between Poisson disk and hexagon aperture
                const auto& ls = (s % 2 == 0) ? lens_samples : hex_samples;
                int li = int(rng.uniform() * ls.size());
                if (li >= (int)ls.size()) li = 0;
                
                double lx = ls[li].x * cam.aperture_radius;
                double ly = ls[li].y * cam.aperture_radius;
                
                Vec3 ro, rd;
                cam.get_ray(px, py, lx, ly, ro, rd);
                
                Hit hit = intersect_scene(ro, rd, scene);
                if (hit.valid) {
                    pixel_color = pixel_color + shade(hit, -rd, scene, rng);
                } else {
                    double t = 0.5 * (rd.y + 1.0);
                    Vec3 sky = Vec3(0.05, 0.05, 0.12) * (1.0 - t) + Vec3(0.3, 0.5, 0.9) * t;
                    pixel_color = pixel_color + sky;
                }
            }
            
            pixel_color = pixel_color / double(SAMPLES);
            int idx = (y * W + x) * 3;
            accum[idx + 0] = pixel_color.x;
            accum[idx + 1] = pixel_color.y;
            accum[idx + 2] = pixel_color.z;
        }
    }
    
    // Tonemap (ACES) + exposure boost + gamma
    printf("Tonemapping...\n");
    std::vector<uint8_t> image(W * H * 3);
    const double exposure = 0.35;
    
    auto aces = [](double x) {
        double a = 2.51, b = 0.03, c_val = 2.43, d = 0.59, e = 0.14;
        return clamp((x*(a*x + b)) / (x*(c_val*x + d) + e), 0.0, 1.0);
    };
    
    for (int i = 0; i < W * H; ++i) {
        Vec3 c(accum[i*3] * exposure, accum[i*3+1] * exposure, accum[i*3+2] * exposure);
        c.x = std::pow(aces(c.x), 1.0/2.2);
        c.y = std::pow(aces(c.y), 1.0/2.2);
        c.z = std::pow(aces(c.z), 1.0/2.2);
        image[i*3 + 0] = uint8_t(clamp(c.x * 255.0, 0, 255));
        image[i*3 + 1] = uint8_t(clamp(c.y * 255.0, 0, 255));
        image[i*3 + 2] = uint8_t(clamp(c.z * 255.0, 0, 255));
    }
    
    const char* filename = "depth_of_field_output.png";
    if (stbi_write_png(filename, W, H, 3, image.data(), W * 3)) {
        printf("Saved: %s\n", filename);
    } else {
        fprintf(stderr, "Failed to save image\n");
        return 1;
    }
    
    // Statistics
    double total = 0, min_val = 1e10, max_val = -1e10;
    for (int i = 0; i < W * H * 3; ++i) {
        double v = accum[i];
        total += v;
        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;
    }
    double mean = total / (W * H * 3);
    
    double var = 0;
    for (int i = 0; i < W * H * 3; ++i) {
        double d = accum[i] - mean;
        var += d * d;
    }
    double stddev = std::sqrt(var / (W * H * 3));
    
    printf("\n=== Statistics ===\n");
    printf("Mean: %.3f  Std: %.3f  Min: %.3f  Max: %.3f\n", mean, stddev, min_val, max_val);
    
    return 0;
}
