#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>
#include <memory>

const double PI = 3.14159265358979323846;
const double INF = std::numeric_limits<double>::infinity();

// ========== 向量类 ==========
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const { double l = length(); return Vec3(x/l, y/l, z/l); }
};

using Color = Vec3;
using Point3 = Vec3;

// ========== 光线类 ==========
struct Ray {
    Point3 origin;
    Vec3 direction;
    
    Ray(const Point3& o, const Vec3& d) : origin(o), direction(d.normalized()) {}
    
    Point3 at(double t) const { return origin + direction * t; }
};

// ========== 材质类 ==========
enum MaterialType { LAMBERTIAN, METAL, DIELECTRIC };

struct Material {
    MaterialType type;
    Color albedo;
    double fuzz;
    double refIdx;
    
    Material(MaterialType t, Color a, double f = 0, double ri = 1.5) 
        : type(t), albedo(a), fuzz(f), refIdx(ri) {}
};

// ========== 碰撞记录 ==========
struct HitRecord {
    Point3 point;
    Vec3 normal;
    double t;
    bool frontFace;
    std::shared_ptr<Material> material;
    
    void setFaceNormal(const Ray& r, const Vec3& outwardNormal) {
        frontFace = r.direction.dot(outwardNormal) < 0;
        normal = frontFace ? outwardNormal : outwardNormal * -1;
    }
};

// ========== 球体类 ==========
struct Sphere {
    Point3 center;
    double radius;
    std::shared_ptr<Material> material;
    
    Sphere(Point3 c, double r, std::shared_ptr<Material> m) : center(c), radius(r), material(m) {}
    
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.direction.dot(r.direction);
        double halfB = oc.dot(r.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = halfB * halfB - a * c;
        
        if (discriminant < 0) return false;
        
        double sqrtd = sqrt(discriminant);
        double root = (-halfB - sqrtd) / a;
        if (root < tMin || root > tMax) {
            root = (-halfB + sqrtd) / a;
            if (root < tMin || root > tMax) return false;
        }
        
        rec.t = root;
        rec.point = r.at(root);
        Vec3 outwardNormal = (rec.point - center) / radius;
        rec.setFaceNormal(r, outwardNormal);
        rec.material = material;
        
        return true;
    }
};

// ========== 场景类 ==========
struct Scene {
    std::vector<Sphere> spheres;
    
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tempRec;
        bool hitAnything = false;
        double closest = tMax;
        
        for (const auto& sphere : spheres) {
            if (sphere.hit(r, tMin, closest, tempRec)) {
                hitAnything = true;
                closest = tempRec.t;
                rec = tempRec;
            }
        }
        
        return hitAnything;
    }
};

// ========== 随机数工具 ==========
std::mt19937 rng(12345);
std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

double randomDouble() { return uniformDist(rng); }
double randomDouble(double min, double max) { return min + (max - min) * randomDouble(); }

Vec3 randomInUnitSphere() {
    while (true) {
        Vec3 p(randomDouble(-1, 1), randomDouble(-1, 1), randomDouble(-1, 1));
        if (p.dot(p) < 1) return p;
    }
}

Vec3 randomUnitVector() {
    return randomInUnitSphere().normalized();
}

Vec3 randomInUnitDisk() {
    while (true) {
        Vec3 p(randomDouble(-1, 1), randomDouble(-1, 1), 0);
        if (p.dot(p) < 1) return p;
    }
}

// ========== 材质散射 ==========
bool scatter(const Ray& rIn, const HitRecord& rec, Color& attenuation, Ray& scattered) {
    if (rec.material->type == LAMBERTIAN) {
        Vec3 scatterDirection = rec.normal + randomUnitVector();
        if (fabs(scatterDirection.x) < 1e-8 && fabs(scatterDirection.y) < 1e-8 && fabs(scatterDirection.z) < 1e-8)
            scatterDirection = rec.normal;
        scattered = Ray(rec.point, scatterDirection);
        attenuation = rec.material->albedo;
        return true;
        
    } else if (rec.material->type == METAL) {
        Vec3 reflected = rIn.direction - rec.normal * (2 * rIn.direction.dot(rec.normal));
        reflected = reflected + randomInUnitSphere() * rec.material->fuzz;
        scattered = Ray(rec.point, reflected);
        attenuation = rec.material->albedo;
        return scattered.direction.dot(rec.normal) > 0;
        
    } else if (rec.material->type == DIELECTRIC) {
        attenuation = Color(1, 1, 1);
        double refractionRatio = rec.frontFace ? (1.0 / rec.material->refIdx) : rec.material->refIdx;
        
        Vec3 unitDirection = rIn.direction.normalized();
        double cosTheta = fmin(-unitDirection.dot(rec.normal), 1.0);
        double sinTheta = sqrt(1.0 - cosTheta * cosTheta);
        
        bool cannotRefract = refractionRatio * sinTheta > 1.0;
        
        auto reflectance = [](double cosine, double refIdx) {
            double r0 = (1 - refIdx) / (1 + refIdx);
            r0 = r0 * r0;
            return r0 + (1 - r0) * pow((1 - cosine), 5);
        };
        
        Vec3 direction;
        if (cannotRefract || reflectance(cosTheta, refractionRatio) > randomDouble()) {
            direction = unitDirection - rec.normal * (2 * unitDirection.dot(rec.normal));
        } else {
            Vec3 rOutPerp = (unitDirection + rec.normal * cosTheta) * refractionRatio;
            Vec3 rOutParallel = rec.normal * (-sqrt(fabs(1.0 - rOutPerp.dot(rOutPerp))));
            direction = rOutPerp + rOutParallel;
        }
        
        scattered = Ray(rec.point, direction);
        return true;
    }
    
    return false;
}

// ========== 光线追踪核心 ==========
Color rayColor(const Ray& r, const Scene& scene, int depth) {
    if (depth <= 0) return Color(0, 0, 0);
    
    HitRecord rec;
    if (scene.hit(r, 0.001, INF, rec)) {
        Ray scattered(Point3(0,0,0), Vec3(0,0,1));
        Color attenuation;
        if (scatter(r, rec, attenuation, scattered)) {
            return attenuation * rayColor(scattered, scene, depth - 1);
        }
        return Color(0, 0, 0);
    }
    
    Vec3 unitDirection = r.direction.normalized();
    double t = 0.5 * (unitDirection.y + 1.0);
    return Color(1, 1, 1) * (1.0 - t) + Color(0.5, 0.7, 1.0) * t;
}

// ========== 带景深的相机 ==========
struct Camera {
    Point3 origin;
    Point3 lowerLeftCorner;
    Vec3 horizontal;
    Vec3 vertical;
    Vec3 u, v, w;
    double lensRadius;
    
    Camera(Point3 lookFrom, Point3 lookAt, Vec3 vup, double vfov, 
           double aspectRatio, double aperture, double focusDist) {
        double theta = vfov * PI / 180.0;
        double h = tan(theta / 2.0);
        double viewportHeight = 2.0 * h;
        double viewportWidth = aspectRatio * viewportHeight;
        
        w = (lookFrom - lookAt).normalized();
        u = vup.cross(w).normalized();
        v = w.cross(u);
        
        origin = lookFrom;
        horizontal = u * (viewportWidth * focusDist);
        vertical = v * (viewportHeight * focusDist);
        lowerLeftCorner = origin - horizontal / 2 - vertical / 2 - w * focusDist;
        
        lensRadius = aperture / 2.0;
    }
    
    Ray getRay(double s, double t) const {
        Vec3 rd = randomInUnitDisk() * lensRadius;
        Vec3 offset = u * rd.x + v * rd.y;
        
        return Ray(origin + offset, 
                   lowerLeftCorner + horizontal * s + vertical * t - origin - offset);
    }
};

// ========== 渲染函数 ==========
void render(const char* filename, int width, int height, int samplesPerPixel, int maxDepth, 
            const Scene& scene, const Camera& camera) {
    
    std::vector<unsigned char> pixels(width * height * 3);
    
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            Color pixelColor(0, 0, 0);
            
            for (int s = 0; s < samplesPerPixel; s++) {
                double u = (i + randomDouble()) / (width - 1);
                double v = (j + randomDouble()) / (height - 1);
                Ray r = camera.getRay(u, v);
                pixelColor = pixelColor + rayColor(r, scene, maxDepth);
            }
            
            pixelColor = pixelColor / samplesPerPixel;
            pixelColor.x = sqrt(pixelColor.x);
            pixelColor.y = sqrt(pixelColor.y);
            pixelColor.z = sqrt(pixelColor.z);
            
            int idx = ((height - 1 - j) * width + i) * 3;
            pixels[idx] = (unsigned char)(256 * std::clamp(pixelColor.x, 0.0, 0.999));
            pixels[idx + 1] = (unsigned char)(256 * std::clamp(pixelColor.y, 0.0, 0.999));
            pixels[idx + 2] = (unsigned char)(256 * std::clamp(pixelColor.z, 0.0, 0.999));
        }
        
        if (j % 50 == 0) {
            printf("Scanline %d / %d\n", j, height);
        }
    }
    
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
    printf("Saved: %s\n", filename);
}

// ========== 随机场景生成 ==========
Scene randomScene() {
    Scene scene;
    
    auto groundMaterial = std::make_shared<Material>(LAMBERTIAN, Color(0.5, 0.5, 0.5), 0, 0);
    scene.spheres.push_back(Sphere(Point3(0, -1000, 0), 1000, groundMaterial));
    
    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            double chooseMat = randomDouble();
            Point3 center(a + 0.9 * randomDouble(), 0.2, b + 0.9 * randomDouble());
            
            if ((center - Point3(4, 0.2, 0)).length() > 0.9) {
                std::shared_ptr<Material> sphereMaterial;
                
                if (chooseMat < 0.8) {
                    Color albedo(randomDouble() * randomDouble(), 
                                randomDouble() * randomDouble(), 
                                randomDouble() * randomDouble());
                    sphereMaterial = std::make_shared<Material>(LAMBERTIAN, albedo, 0, 0);
                } else if (chooseMat < 0.95) {
                    Color albedo(randomDouble(0.5, 1), randomDouble(0.5, 1), randomDouble(0.5, 1));
                    double fuzz = randomDouble(0, 0.5);
                    sphereMaterial = std::make_shared<Material>(METAL, albedo, fuzz, 0);
                } else {
                    sphereMaterial = std::make_shared<Material>(DIELECTRIC, Color(1, 1, 1), 0, 1.5);
                }
                
                scene.spheres.push_back(Sphere(center, 0.2, sphereMaterial));
            }
        }
    }
    
    auto material1 = std::make_shared<Material>(DIELECTRIC, Color(1, 1, 1), 0, 1.5);
    scene.spheres.push_back(Sphere(Point3(0, 1, 0), 1.0, material1));
    
    auto material2 = std::make_shared<Material>(LAMBERTIAN, Color(0.4, 0.2, 0.1), 0, 0);
    scene.spheres.push_back(Sphere(Point3(-4, 1, 0), 1.0, material2));
    
    auto material3 = std::make_shared<Material>(METAL, Color(0.7, 0.6, 0.5), 0.0, 0);
    scene.spheres.push_back(Sphere(Point3(4, 1, 0), 1.0, material3));
    
    return scene;
}

int main() {
    const double aspectRatio = 3.0 / 2.0;
    const int imageWidth = 1200;
    const int imageHeight = (int)(imageWidth / aspectRatio);
    const int samplesPerPixel = 100;
    const int maxDepth = 50;
    
    Scene scene = randomScene();
    
    Point3 lookFrom(13, 2, 3);
    Point3 lookAt(0, 0, 0);
    Vec3 vup(0, 1, 0);
    double distToFocus = 10.0;
    double aperture = 0.1;
    
    Camera camera(lookFrom, lookAt, vup, 20, aspectRatio, aperture, distToFocus);
    
    render("phase3_dof_complex.png", imageWidth, imageHeight, samplesPerPixel, maxDepth, scene, camera);
    
    return 0;
}
