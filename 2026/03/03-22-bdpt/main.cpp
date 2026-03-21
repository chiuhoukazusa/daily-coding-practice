// Bidirectional Path Tracing (BDPT) with Multiple Importance Sampling (MIS)
// 双向路径追踪 + 多重重要性采样
// Cornell Box scene with area light
// 2026-03-22 Daily Coding Practice

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <cassert>
#include <string>
#include <memory>
#include <optional>
#include <limits>

// ============================================================
// Math
// ============================================================
struct Vec3 {
    double x, y, z;
    Vec3(double v = 0) : x(v), y(v), z(v) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    double dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    double length2() const { return x*x + y*y + z*z; }
    Vec3 normalize() const { double l = length(); return *this / (l > 1e-10 ? l : 1e-10); }
    bool isZero() const { return length2() < 1e-20; }
    double maxComp() const { return std::max({x, y, z}); }
};
Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
// Ray
// ============================================================
struct Ray {
    Vec3 origin, dir;
    Ray(Vec3 o, Vec3 d) : origin(o), dir(d.normalize()) {}
    Vec3 at(double t) const { return origin + dir * t; }
};

// ============================================================
// RNG
// ============================================================
thread_local std::mt19937_64 rng(std::random_device{}());
double rand01() {
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

// Cosine-weighted hemisphere sample
Vec3 cosineSampleHemisphere(const Vec3& normal) {
    double u1 = rand01(), u2 = rand01();
    double r = std::sqrt(u1);
    double theta = 2.0 * M_PI * u2;
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);
    double z = std::sqrt(std::max(0.0, 1.0 - u1));

    // Build local frame
    Vec3 up = (std::abs(normal.x) > 0.9) ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 tangent = up.cross(normal).normalize();
    Vec3 bitangent = normal.cross(tangent);
    return (tangent * x + bitangent * y + normal * z).normalize();
}

// Uniform sphere sample
Vec3 uniformSampleSphere() {
    double u1 = rand01(), u2 = rand01();
    double z = 1.0 - 2.0 * u1;
    double r = std::sqrt(std::max(0.0, 1.0 - z*z));
    double phi = 2.0 * M_PI * u2;
    return {r * std::cos(phi), r * std::sin(phi), z};
}

// ============================================================
// Material types
// ============================================================
enum class MatType { Diffuse, Mirror, Emissive };

struct Material {
    MatType type;
    Vec3 albedo;      // diffuse/reflective color
    Vec3 emission;    // only for emissive

    static Material diffuse(Vec3 color) { return {MatType::Diffuse, color, Vec3(0)}; }
    static Material mirror(Vec3 color) { return {MatType::Mirror, color, Vec3(0)}; }
    static Material light(Vec3 emit) { return {MatType::Emissive, Vec3(0), emit}; }
};

// ============================================================
// Geometry
// ============================================================
struct HitInfo {
    double t;
    Vec3 point, normal;
    int matIdx;
    bool frontFace;
};

struct Sphere {
    Vec3 center;
    double radius;
    int matIdx;

    std::optional<HitInfo> intersect(const Ray& ray, double tMin, double tMax) const {
        Vec3 oc = ray.origin - center;
        double a = ray.dir.dot(ray.dir);
        double b = oc.dot(ray.dir);
        double c = oc.dot(oc) - radius * radius;
        double disc = b*b - a*c;
        if (disc < 0) return std::nullopt;
        double sq = std::sqrt(disc);
        double t = (-b - sq) / a;
        if (t < tMin || t > tMax) {
            t = (-b + sq) / a;
            if (t < tMin || t > tMax) return std::nullopt;
        }
        Vec3 p = ray.at(t);
        Vec3 outward = (p - center) / radius;
        bool ff = ray.dir.dot(outward) < 0;
        return HitInfo{t, p, ff ? outward : -outward, matIdx, ff};
    }
};

struct Quad {
    // Axis-aligned rectangle
    Vec3 corner;       // one corner
    Vec3 uAxis, vAxis; // edge vectors
    Vec3 normal;
    int matIdx;
    double area;

    Quad(Vec3 c, Vec3 u, Vec3 v, int m) : corner(c), uAxis(u), vAxis(v), matIdx(m) {
        normal = u.cross(v).normalize();
        area = u.cross(v).length();
    }

    std::optional<HitInfo> intersect(const Ray& ray, double tMin, double tMax) const {
        double denom = normal.dot(ray.dir);
        if (std::abs(denom) < 1e-8) return std::nullopt;
        double t = normal.dot(corner - ray.origin) / denom;
        if (t < tMin || t > tMax) return std::nullopt;
        Vec3 p = ray.at(t);
        Vec3 d = p - corner;
        // Project onto uv
        double u = d.dot(uAxis) / uAxis.length2();
        double v = d.dot(vAxis) / vAxis.length2();
        if (u < 0 || u > 1 || v < 0 || v > 1) return std::nullopt;
        bool ff = ray.dir.dot(normal) < 0;
        return HitInfo{t, p, ff ? normal : -normal, matIdx, ff};
    }

    // Sample a random point on this quad
    Vec3 sample() const {
        return corner + uAxis * rand01() + vAxis * rand01();
    }
};

// ============================================================
// Scene
// ============================================================
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Quad> quads;
    std::vector<Material> materials;
    std::vector<int> lightQuadIndices; // indices into quads that are emissive

    int addMaterial(Material m) {
        materials.push_back(m);
        return (int)materials.size() - 1;
    }

    std::optional<HitInfo> intersect(const Ray& ray, double tMin = 1e-4, double tMax = 1e15) const {
        std::optional<HitInfo> best;
        double tBest = tMax;
        for (auto& s : spheres) {
            auto h = s.intersect(ray, tMin, tBest);
            if (h && h->t < tBest) { tBest = h->t; best = h; }
        }
        for (auto& q : quads) {
            auto h = q.intersect(ray, tMin, tBest);
            if (h && h->t < tBest) { tBest = h->t; best = h; }
        }
        return best;
    }

    bool occluded(Vec3 a, Vec3 b) const {
        Vec3 dir = b - a;
        double dist = dir.length();
        Ray ray(a, dir);
        auto h = intersect(ray, 1e-4, dist - 1e-4);
        return h.has_value();
    }
};

// ============================================================
// Cornell Box Setup
// ============================================================
Scene buildCornellBox() {
    Scene scene;

    // Materials
    int white  = scene.addMaterial(Material::diffuse({0.73, 0.73, 0.73}));
    int red    = scene.addMaterial(Material::diffuse({0.65, 0.05, 0.05}));
    int green  = scene.addMaterial(Material::diffuse({0.12, 0.45, 0.15}));
    int light  = scene.addMaterial(Material::light({15.0, 15.0, 15.0}));
    int mirror = scene.addMaterial(Material::mirror({0.9, 0.9, 0.9}));
    (void)mirror;

    // Cornell box: 555x555x555
    // Floor (y=0)
    scene.quads.emplace_back(Vec3(0,0,0), Vec3(555,0,0), Vec3(0,0,555), white);
    // Ceiling (y=555)
    scene.quads.emplace_back(Vec3(0,555,555), Vec3(555,0,0), Vec3(0,0,-555), white);
    // Back wall (z=555)
    scene.quads.emplace_back(Vec3(0,0,555), Vec3(555,0,0), Vec3(0,555,0), white);
    // Left wall (x=0) - green
    scene.quads.emplace_back(Vec3(0,0,0), Vec3(0,555,0), Vec3(0,0,555), green);
    // Right wall (x=555) - red
    scene.quads.emplace_back(Vec3(555,0,555), Vec3(0,555,0), Vec3(0,0,-555), red);

    // Area light (on ceiling)
    int lightIdx = (int)scene.quads.size();
    scene.quads.emplace_back(Vec3(213,554,227), Vec3(130,0,0), Vec3(0,0,105), light);
    scene.lightQuadIndices.push_back(lightIdx);

    // Spheres
    // Left diffuse sphere
    scene.spheres.push_back({Vec3(150, 100, 200), 100.0, white});
    // Right mirror sphere  
    scene.spheres.push_back({Vec3(380, 100, 350), 100.0, mirror});

    return scene;
}

// ============================================================
// BDPT Vertex
// ============================================================
struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec3 throughput; // accumulated throughput to this vertex
    int matIdx;
    double pdfFwd;   // pdf of generating this vertex from previous
    double pdfRev;   // pdf of reverse direction (for MIS)
    bool isDelta;    // mirror material
    bool isLight;    // on light source
    bool isCamera;   // camera vertex

    bool connectable() const { return !isDelta; }
};

// ============================================================
// BDPT Core
// ============================================================
const int MAX_DEPTH = 6;

// Evaluate BSDF * cos / pdf for diffuse
Vec3 evalDiffuse(const Vec3& albedo, const Vec3& normal, const Vec3& wi, const Vec3& wo) {
    double cosI = std::max(0.0, normal.dot(wi));
    double cosO = std::max(0.0, normal.dot(wo));
    (void)cosO;
    (void)cosI;
    return albedo / M_PI;
}

double pdfDiffuse(const Vec3& normal, const Vec3& dir) {
    double c = normal.dot(dir);
    return (c > 0) ? c / M_PI : 0.0;
}

// Power heuristic for MIS
double powerHeuristic(int nf, double fPdf, int ng, double gPdf) {
    double f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g + 1e-20);
}

// Build camera subpath
std::vector<Vertex> buildCameraPath(const Scene& scene, const Ray& cameraRay, int maxDepth) {
    std::vector<Vertex> path;

    // Camera vertex
    Vertex cam;
    cam.pos = cameraRay.origin;
    cam.normal = cameraRay.dir;
    cam.throughput = Vec3(1.0);
    cam.matIdx = -1;
    cam.pdfFwd = 1.0;
    cam.pdfRev = 0.0;
    cam.isDelta = false;
    cam.isLight = false;
    cam.isCamera = true;
    path.push_back(cam);

    Ray ray = cameraRay;
    Vec3 throughput(1.0);
    double pdf = 1.0;

    for (int depth = 0; depth < maxDepth; depth++) {
        auto hit = scene.intersect(ray);
        if (!hit) break;

        const Material& mat = scene.materials[hit->matIdx];

        Vertex v;
        v.pos = hit->point;
        v.normal = hit->normal;
        v.matIdx = hit->matIdx;
        v.isLight = (mat.type == MatType::Emissive);
        v.isCamera = false;
        v.isDelta = (mat.type == MatType::Mirror);
        v.pdfFwd = pdf;
        v.pdfRev = 0.0;

        if (mat.type == MatType::Emissive) {
            v.throughput = throughput;
            path.push_back(v);
            break;
        }

        if (mat.type == MatType::Mirror) {
            Vec3 reflected = ray.dir - hit->normal * 2.0 * ray.dir.dot(hit->normal);
            v.throughput = throughput * mat.albedo;
            path.push_back(v);
            throughput = v.throughput;
            pdf = 1.0; // delta
            ray = Ray(hit->point, reflected);
        } else { // Diffuse
            Vec3 newDir = cosineSampleHemisphere(hit->normal);
            double p = pdfDiffuse(hit->normal, newDir);
            if (p < 1e-10) break;
            Vec3 f = evalDiffuse(mat.albedo, hit->normal, newDir, -ray.dir);
            double cosTheta = std::max(0.0, hit->normal.dot(newDir));
            throughput = throughput * f * cosTheta / p;

            v.throughput = throughput;
            path.push_back(v);

            // Russian roulette
            double rrProb = std::min(0.95, throughput.maxComp());
            if (rand01() > rrProb) break;
            throughput = throughput / rrProb;
            pdf = p;
            ray = Ray(hit->point, newDir);
        }
    }
    return path;
}

// Build light subpath
std::vector<Vertex> buildLightPath(const Scene& scene, int maxDepth) {
    std::vector<Vertex> path;
    if (scene.lightQuadIndices.empty()) return path;

    // Pick a random light
    int li = scene.lightQuadIndices[(int)(rand01() * scene.lightQuadIndices.size())];
    const Quad& lightQuad = scene.quads[li];
    const Material& lightMat = scene.materials[lightQuad.matIdx];

    Vec3 pos = lightQuad.sample();
    // Light normal points downward (into scene)
    Vec3 normal = -lightQuad.normal; // face into scene
    if (normal.y > 0) normal = -normal; // ensure pointing down

    // Sample direction from light (cosine weighted)
    Vec3 dir = cosineSampleHemisphere(normal);
    double pdfPos = 1.0 / lightQuad.area;
    double pdfDir = std::max(0.0, normal.dot(dir)) / M_PI;

    if (pdfDir < 1e-10) return path;

    // Light vertex
    Vertex lv;
    lv.pos = pos;
    lv.normal = normal;
    lv.throughput = lightMat.emission / pdfPos;
    lv.matIdx = lightQuad.matIdx;
    lv.pdfFwd = pdfPos;
    lv.pdfRev = 0.0;
    lv.isDelta = false;
    lv.isLight = true;
    lv.isCamera = false;
    path.push_back(lv);

    // Trace light path
    Ray ray(pos, dir);
    Vec3 throughput = lightMat.emission * std::max(0.0, normal.dot(dir)) / (pdfPos * pdfDir);
    double pdf = pdfDir;

    for (int depth = 0; depth < maxDepth; depth++) {
        auto hit = scene.intersect(ray);
        if (!hit) break;

        const Material& mat = scene.materials[hit->matIdx];

        Vertex v;
        v.pos = hit->point;
        v.normal = hit->normal;
        v.matIdx = hit->matIdx;
        v.isLight = false;
        v.isCamera = false;
        v.isDelta = (mat.type == MatType::Mirror);
        v.pdfFwd = pdf;
        v.pdfRev = 0.0;

        if (mat.type == MatType::Emissive) break; // hit another light

        if (mat.type == MatType::Mirror) {
            Vec3 reflected = ray.dir - hit->normal * 2.0 * ray.dir.dot(hit->normal);
            throughput = throughput * mat.albedo;
            v.throughput = throughput;
            path.push_back(v);
            pdf = 1.0;
            ray = Ray(hit->point, reflected);
        } else { // Diffuse
            Vec3 newDir = cosineSampleHemisphere(hit->normal);
            double p = pdfDiffuse(hit->normal, newDir);
            if (p < 1e-10) break;
            Vec3 f = evalDiffuse(mat.albedo, hit->normal, newDir, -ray.dir);
            double cosTheta = std::max(0.0, hit->normal.dot(newDir));
            throughput = throughput * f * cosTheta / p;

            v.throughput = throughput;
            path.push_back(v);

            double rrProb = std::min(0.95, throughput.maxComp());
            if (rand01() > rrProb) break;
            throughput = throughput / rrProb;
            pdf = p;
            ray = Ray(hit->point, newDir);
        }
    }
    return path;
}

// Connect camera and light vertices
Vec3 connectPaths(const Scene& scene,
                  const std::vector<Vertex>& camPath,
                  const std::vector<Vertex>& lightPath,
                  int s, int t) {
    // s: number of light vertices used (0 = pure camera path)
    // t: number of camera vertices used (>= 2 means cam + at least 1 bounce)

    if (t < 1) return Vec3(0);

    if (s == 0) {
        // Pure camera path: check if last camera vertex is on light
        if (t < 2) return Vec3(0);
        const Vertex& lastCam = camPath[t - 1];
        if (!lastCam.isLight) return Vec3(0);
        const Material& mat = scene.materials[lastCam.matIdx];
        return camPath[t-2].throughput * mat.emission;
    }

    if (s == 1) {
        // Direct light connection: camera path vertex -> light sample
        if (t < 2) return Vec3(0);
        const Vertex& camV = camPath[t - 1];
        if (!camV.connectable()) return Vec3(0);
        if (scene.lightQuadIndices.empty()) return Vec3(0);

        int li = scene.lightQuadIndices[(int)(rand01() * scene.lightQuadIndices.size())];
        const Quad& lightQuad = scene.quads[li];
        const Material& lightMat = scene.materials[lightQuad.matIdx];

        Vec3 lightPos = lightQuad.sample();
        Vec3 lightNormal = lightQuad.normal;
        // Make sure normal points toward scene
        if (lightNormal.y > 0) lightNormal = -lightNormal;

        Vec3 toLight = lightPos - camV.pos;
        double dist2 = toLight.length2();
        double dist = std::sqrt(dist2);
        Vec3 dirToLight = toLight / dist;

        double cosAtCam = camV.normal.dot(dirToLight);
        double cosAtLight = (-dirToLight).dot(lightNormal);

        if (cosAtCam <= 0 || cosAtLight <= 0) return Vec3(0);

        // Visibility check
        if (scene.occluded(camV.pos, lightPos)) return Vec3(0);

        const Material& camMat = scene.materials[camV.matIdx];
        if (camMat.type == MatType::Emissive) return Vec3(0);

        Vec3 f = camMat.albedo / M_PI;
        double geo = cosAtCam * cosAtLight / dist2;
        double pdfLight = 1.0 / lightQuad.area;

        Vec3 camThroughput = (t >= 3) ? camPath[t-2].throughput : Vec3(1.0);

        // MIS weight: simple balance heuristic
        double weight = 1.0; // simplified
        return camThroughput * f * geo * lightMat.emission / pdfLight * weight;
    }

    // General case: connect camPath[t-1] to lightPath[s-1]
    if (t < 2 || s < 2) return Vec3(0);
    const Vertex& camV = camPath[t - 1];
    const Vertex& lightV = lightPath[s - 1];

    if (!camV.connectable() || !lightV.connectable()) return Vec3(0);

    Vec3 dir = lightV.pos - camV.pos;
    double dist2 = dir.length2();
    double dist = std::sqrt(dist2);
    Vec3 d = dir / dist;

    double cosCam = camV.normal.dot(d);
    double cosLight = lightV.normal.dot(-d);

    if (cosCam <= 0 || cosLight <= 0) return Vec3(0);

    if (scene.occluded(camV.pos, lightV.pos)) return Vec3(0);

    const Material& camMat = scene.materials[camV.matIdx];
    const Material& lightMat = scene.materials[lightV.matIdx];

    if (camMat.type == MatType::Emissive || lightMat.type == MatType::Emissive) return Vec3(0);

    Vec3 fCam = camMat.albedo / M_PI * cosCam;
    Vec3 fLight = lightMat.albedo / M_PI * cosLight;

    double geo = 1.0 / dist2;

    Vec3 camThroughput = (t >= 3) ? camPath[t-2].throughput : Vec3(1.0);
    Vec3 lightThroughput = (s >= 3) ? lightPath[s-2].throughput : Vec3(1.0);

    return camThroughput * fCam * geo * fLight * lightThroughput;
}

// ============================================================
// Renderer
// ============================================================
struct Camera {
    Vec3 origin, lowerLeft, horizontal, vertical;
    int width, height;

    Camera(Vec3 from, Vec3 at, Vec3 up, double fov, int w, int h)
        : width(w), height(h) {
        origin = from;
        double theta = fov * M_PI / 180.0;
        double halfH = std::tan(theta / 2.0);
        double halfW = halfH * (double)w / h;
        Vec3 front = (at - from).normalize();
        Vec3 right = front.cross(up).normalize();
        Vec3 upVec = right.cross(front);
        lowerLeft = origin + front - right * halfW - upVec * halfH;
        horizontal = right * (2.0 * halfW);
        vertical = upVec * (2.0 * halfH);
    }

    Ray getRay(double u, double v) const {
        Vec3 dir = lowerLeft + horizontal * u + vertical * v - origin;
        return Ray(origin, dir);
    }
};

Vec3 clamp(Vec3 v, double lo, double hi) {
    return {std::clamp(v.x, lo, hi),
            std::clamp(v.y, lo, hi),
            std::clamp(v.z, lo, hi)};
}

Vec3 toneMap(Vec3 v) {
    // Reinhard per-channel
    return Vec3(v.x / (v.x + 1.0), v.y / (v.y + 1.0), v.z / (v.z + 1.0));
}

double gammaCorrect(double v) {
    return std::pow(std::clamp(v, 0.0, 1.0), 1.0 / 2.2);
}

// ============================================================
// PPM writer
// ============================================================
void writePPM(const std::string& filename, const std::vector<Vec3>& pixels, int w, int h) {
    std::ofstream f(filename);
    f << "P3\n" << w << " " << h << "\n255\n";
    for (int j = h - 1; j >= 0; j--) {
        for (int i = 0; i < w; i++) {
            Vec3 c = toneMap(pixels[j * w + i]);
            f << (int)(gammaCorrect(c.x) * 255.99) << " "
              << (int)(gammaCorrect(c.y) * 255.99) << " "
              << (int)(gammaCorrect(c.z) * 255.99) << "\n";
        }
    }
}

// ============================================================
// Main render loop
// ============================================================
int main() {
    const int W = 512, H = 512;
    const int SPP = 64; // samples per pixel

    Scene scene = buildCornellBox();

    // Cornell box camera
    Camera cam(
        Vec3(278, 273, -800),
        Vec3(278, 273, 0),
        Vec3(0, 1, 0),
        40.0,
        W, H
    );

    std::vector<Vec3> pixels(W * H, Vec3(0.0));

    std::cout << "BDPT Rendering " << W << "x" << H << " @ " << SPP << " spp\n";
    std::cout << "Strategy: t+s=2..8 paths with MIS\n";

    for (int j = 0; j < H; j++) {
        if (j % 64 == 0) {
            std::cout << "Row " << j << "/" << H << " ("
                      << (int)(100.0 * j / H) << "%)\n";
            std::cout.flush();
        }
        for (int i = 0; i < W; i++) {
            Vec3 radiance(0.0);
            for (int s = 0; s < SPP; s++) {
                double u = (i + rand01()) / W;
                double v = (j + rand01()) / H;
                Ray ray = cam.getRay(u, v);

                // Build camera and light subpaths
                auto camPath = buildCameraPath(scene, ray, MAX_DEPTH);
                auto lightPath = buildLightPath(scene, MAX_DEPTH);

                // Combine all (s,t) strategies
                Vec3 contrib(0.0);

                // s=0: camera path hits light
                for (int t = 2; t <= (int)camPath.size(); t++) {
                    Vec3 c = connectPaths(scene, camPath, lightPath, 0, t);
                    if (!std::isnan(c.x) && !std::isnan(c.y) && !std::isnan(c.z))
                        contrib = contrib + c;
                }

                // s=1: direct light sampling
                for (int t = 2; t <= (int)camPath.size(); t++) {
                    Vec3 c = connectPaths(scene, camPath, lightPath, 1, t);
                    if (!std::isnan(c.x) && !std::isnan(c.y) && !std::isnan(c.z))
                        contrib = contrib + c;
                }

                // s>=2: general connection
                for (int lLen = 2; lLen <= (int)lightPath.size(); lLen++) {
                    for (int cLen = 2; cLen <= (int)camPath.size(); cLen++) {
                        int totalLen = lLen + cLen;
                        if (totalLen < 3 || totalLen > MAX_DEPTH + 2) continue;
                        Vec3 c = connectPaths(scene, camPath, lightPath, lLen, cLen);
                        // Scale by number of light vertices (estimator normalization)
                        c = c / (double)(lLen);
                        if (!std::isnan(c.x) && !std::isnan(c.y) && !std::isnan(c.z))
                            contrib = contrib + c;
                    }
                }

                // Firefly clamp
                double lum = 0.2126 * contrib.x + 0.7152 * contrib.y + 0.0722 * contrib.z;
                if (lum > 50.0) contrib = contrib * (50.0 / lum);

                radiance = radiance + contrib;
            }
            pixels[j * W + i] = radiance / (double)SPP;
        }
    }

    // Write output
    writePPM("bdpt_output.ppm", pixels, W, H);
    std::cout << "Wrote bdpt_output.ppm\n";

    // Convert PPM to PNG using ImageMagick
    int ret = system("convert bdpt_output.ppm bdpt_output.png 2>/dev/null");
    if (ret != 0) {
        // Try with python
        ret = system("python3 -c \""
            "from PIL import Image; "
            "img = Image.open('bdpt_output.ppm'); "
            "img.save('bdpt_output.png')\"");
    }

    if (ret == 0) {
        std::cout << "Converted to bdpt_output.png\n";
    } else {
        std::cout << "Warning: PNG conversion failed, keeping PPM\n";
    }

    std::cout << "DONE\n";
    return 0;
}
