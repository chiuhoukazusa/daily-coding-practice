/*
 * Deferred Shading Renderer
 * 日期: 2026-03-18
 * 
 * 实现延迟渲染管线：
 * 1. Geometry Pass: 渲染所有几何体到 G-Buffer
 *    - GBuffer[0]: Albedo (RGB) + Specular (A)
 *    - GBuffer[1]: World Normal (RGB, 编码)
 *    - GBuffer[2]: Depth (float)
 * 2. Lighting Pass: 对每个像素用 G-Buffer 计算光照
 *    - 多个点光源 (Phong + Specular)
 *    - Attenuation 衰减
 * 3. 输出: 合并场景、G-Buffer可视化、比较图
 * 
 * 场景: 多个球体 + 地面平面 + 8个彩色点光源
 */

#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstring>
#include <sstream>
#include <cassert>

// ─────────────────────────── Math ───────────────────────────

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { double l = length(); return l>1e-8 ? *this/l : Vec3(0,1,0); }
    Vec3 reflect(const Vec3& n) const { return *this - n * (2.0 * dot(n)); }
    static Vec3 clamp(Vec3 v, double lo, double hi) {
        return {std::max(lo,std::min(hi,v.x)),
                std::max(lo,std::min(hi,v.y)),
                std::max(lo,std::min(hi,v.z))};
    }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    Vec3 at(double t) const { return origin + direction * t; }
};

// ─────────────────────────── Image ───────────────────────────

struct Image {
    int width, height;
    std::vector<Vec3> pixels;
    Image(int w, int h) : width(w), height(h), pixels(w*h, Vec3(0,0,0)) {}
    Vec3& at(int x, int y) { return pixels[y*width+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*width+x]; }

    void savePNG(const std::string& filename) const {
        // Write PPM then convert using Python/PIL
        std::string ppmFile = filename.substr(0, filename.rfind('.')) + ".ppm";
        std::ofstream ofs(ppmFile, std::ios::binary);
        ofs << "P6\n" << width << " " << height << "\n255\n";
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const Vec3& c = at(x,y);
                ofs.put((unsigned char)std::max(0,std::min(255,(int)(c.x*255.0))));
                ofs.put((unsigned char)std::max(0,std::min(255,(int)(c.y*255.0))));
                ofs.put((unsigned char)std::max(0,std::min(255,(int)(c.z*255.0))));
            }
        }
        ofs.close();
        // Convert to PNG using Python3/PIL
        std::string cmd = "python3 -c \"from PIL import Image; img=Image.open('" + ppmFile + "'); img.save('" + filename + "')\" 2>/dev/null && rm " + ppmFile;
        int ret = system(cmd.c_str());
        if (ret != 0) {
            std::cerr << "Warning: PNG conversion failed for " << filename << ", PPM kept." << std::endl;
        }
    }
};

// ─────────────────────────── G-Buffer ───────────────────────────

struct GBuffer {
    int width, height;
    std::vector<Vec3> albedo;      // RGB albedo + specular in w (use struct)
    std::vector<double> specular;  // specular power
    std::vector<Vec3> normal;      // world-space normal
    std::vector<double> depth;     // linear depth (z in view space)
    std::vector<Vec3> position;    // world position (derived from depth + ray)

    GBuffer(int w, int h)
        : width(w), height(h),
          albedo(w*h, Vec3(0,0,0)),
          specular(w*h, 0.0),
          normal(w*h, Vec3(0,0,0)),
          depth(w*h, std::numeric_limits<double>::infinity()),
          position(w*h, Vec3(0,0,0))
    {}
    
    int idx(int x, int y) const { return y*width+x; }
};

// ─────────────────────────── Scene ───────────────────────────

struct Material {
    Vec3 albedo;
    double specular;    // specular coefficient
    double shininess;   // Phong shininess
    double roughness;   // for display
};

struct Sphere {
    Vec3 center;
    double radius;
    Material mat;
    
    bool hit(const Ray& r, double tMin, double tMax, double& t, Vec3& N) const {
        Vec3 oc = r.origin - center;
        double a = r.direction.dot(r.direction);
        double b = 2.0 * oc.dot(r.direction);
        double c = oc.dot(oc) - radius*radius;
        double disc = b*b - 4*a*c;
        if (disc < 0) return false;
        double sqrtDisc = std::sqrt(disc);
        double t0 = (-b - sqrtDisc) / (2*a);
        double t1 = (-b + sqrtDisc) / (2*a);
        t = t0;
        if (t < tMin || t > tMax) {
            t = t1;
            if (t < tMin || t > tMax) return false;
        }
        Vec3 P = r.at(t);
        N = (P - center).normalize();
        return true;
    }
};

struct Plane {
    Vec3 point;
    Vec3 normal;
    Material mat;
    
    bool hit(const Ray& r, double tMin, double tMax, double& t, Vec3& N) const {
        double denom = normal.dot(r.direction);
        if (std::abs(denom) < 1e-8) return false;
        t = (point - r.origin).dot(normal) / denom;
        if (t < tMin || t > tMax) return false;
        N = normal;
        return true;
    }
};

struct PointLight {
    Vec3 position;
    Vec3 color;
    double intensity;
    double radius;  // light sphere radius for visualization
};

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane>  planes;
    std::vector<PointLight> lights;
    Vec3 ambientColor;
    double ambientIntensity;
};

// ─────────────────────────── Camera ───────────────────────────

struct Camera {
    Vec3 origin;
    Vec3 lowerLeft;
    Vec3 horizontal;
    Vec3 vertical;
    
    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, double fovY, double aspect) {
        double theta = fovY * M_PI / 180.0;
        double h = std::tan(theta/2.0);
        double viewH = 2.0 * h;
        double viewW = aspect * viewH;
        
        Vec3 w = (lookFrom - lookAt).normalize();
        Vec3 u = up.cross(w).normalize();
        Vec3 v = w.cross(u);
        
        origin = lookFrom;
        horizontal = u * viewW;
        vertical = v * viewH;
        lowerLeft = origin - horizontal*0.5 - vertical*0.5 - w;
    }
    
    Ray getRay(double s, double t) const {
        Vec3 dir = lowerLeft + horizontal*s + vertical*t - origin;
        return Ray(origin, dir);
    }
};

// ─────────────────────────── Intersection ───────────────────────────

struct HitInfo {
    double t;
    Vec3 point;
    Vec3 normal;
    const Material* mat;
    bool hit;
};

HitInfo sceneIntersect(const Ray& ray, const Scene& scene, double tMin=0.001, double tMax=1e9) {
    HitInfo best;
    best.hit = false;
    best.t = tMax;
    
    for (const auto& sphere : scene.spheres) {
        double t;
        Vec3 N;
        if (sphere.hit(ray, tMin, best.t, t, N)) {
            best.hit = true;
            best.t = t;
            best.point = ray.at(t);
            best.normal = N;
            best.mat = &sphere.mat;
        }
    }
    
    for (const auto& plane : scene.planes) {
        double t;
        Vec3 N;
        if (plane.hit(ray, tMin, best.t, t, N)) {
            best.hit = true;
            best.t = t;
            best.point = ray.at(t);
            // orient normal toward ray
            if (N.dot(ray.direction) > 0) N = N * -1.0;
            best.normal = N;
            best.mat = &plane.mat;
        }
    }
    
    return best;
}

// ─────────────────────────── ACES Tone Mapping ───────────────────────────

Vec3 aces(Vec3 x) {
    (void)x;
    return Vec3(0,0,0); // unused, kept for reference
}

Vec3 acesTonemap(Vec3 x) {
    auto f = [](double v) {
        const double a=2.51, b=0.03, c=2.43, d=0.59, e=0.14;
        return std::max(0.0, std::min(1.0, (v*(v*a+b)) / (v*(v*c+d)+e)));
    };
    return {f(x.x), f(x.y), f(x.z)};
}

// ─────────────────────────── Deferred Renderer ───────────────────────────

class DeferredRenderer {
public:
    int width, height;
    Camera camera;
    Scene scene;
    
    DeferredRenderer(int w, int h, const Camera& cam, const Scene& sc)
        : width(w), height(h), camera(cam), scene(sc) {}
    
    // Pass 1: Geometry Pass - fill G-Buffer
    GBuffer geometryPass() {
        GBuffer gbuf(width, height);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                double u = (x + 0.5) / width;
                double v = (y + 0.5) / height;
                v = 1.0 - v;  // flip Y
                
                Ray ray = camera.getRay(u, v);
                HitInfo hit = sceneIntersect(ray, scene);
                
                int i = gbuf.idx(x, y);
                if (hit.hit) {
                    gbuf.albedo[i]   = hit.mat->albedo;
                    gbuf.specular[i] = hit.mat->specular;
                    gbuf.normal[i]   = hit.normal;
                    gbuf.depth[i]    = hit.t;
                    gbuf.position[i] = hit.point;
                }
            }
        }
        return gbuf;
    }
    
    // Pass 2: Lighting Pass - compute final color from G-Buffer
    Image lightingPass(const GBuffer& gbuf) {
        Image img(width, height);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int i = gbuf.idx(x, y);
                
                // Background
                if (gbuf.depth[i] == std::numeric_limits<double>::infinity()) {
                    // Sky gradient
                    double t = (double)y / height;
                    Vec3 skyTop(0.1, 0.15, 0.25);
                    Vec3 skyBot(0.02, 0.03, 0.05);
                    img.at(x,y) = skyTop * t + skyBot * (1.0-t);
                    continue;
                }
                
                Vec3 worldPos  = gbuf.position[i];
                Vec3 worldNorm = gbuf.normal[i];
                Vec3 albedo    = gbuf.albedo[i];
                double spec    = gbuf.specular[i];
                
                Vec3 viewDir = (camera.origin - worldPos).normalize();
                
                // Ambient
                Vec3 color = albedo * scene.ambientColor * scene.ambientIntensity;
                
                // Point lights
                for (const auto& light : scene.lights) {
                    Vec3 lightVec  = light.position - worldPos;
                    double dist    = lightVec.length();
                    Vec3 lightDir  = lightVec / dist;
                    
                    // Shadow test
                    Ray shadowRay(worldPos + worldNorm*0.001, lightDir);
                    HitInfo shadowHit = sceneIntersect(shadowRay, scene, 0.001, dist - 0.1);
                    if (shadowHit.hit) continue;
                    
                    // Attenuation: 1/(1 + kl*d + kq*d²)
                    double kl = 0.09, kq = 0.032;
                    double atten = 1.0 / (1.0 + kl*dist + kq*dist*dist);
                    
                    // Diffuse
                    double diff = std::max(0.0, worldNorm.dot(lightDir));
                    Vec3 diffuse = albedo * light.color * (diff * light.intensity * atten);
                    
                    // Specular (Blinn-Phong)
                    Vec3 halfVec = (lightDir + viewDir).normalize();
                    double specDot = std::max(0.0, worldNorm.dot(halfVec));
                    // get shininess from roughness (stored in specular channel)
                    double shininess = 32.0 / (spec + 0.01);
                    double specFactor = std::pow(specDot, shininess) * spec;
                    Vec3 specular = light.color * (specFactor * light.intensity * atten);
                    
                    color += diffuse + specular;
                }
                
                // Tone mapping
                color = acesTonemap(color);
                img.at(x,y) = Vec3::clamp(color, 0, 1);
            }
        }
        return img;
    }
    
    // Visualize G-Buffer channels
    Image visualizeGBuffer(const GBuffer& gbuf) {
        // 2x2 grid: albedo | normals | depth | position
        int halfW = width/2, halfH = height/2;
        Image vis(width, height);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int srcX_unused = (x % halfW) * 2;   // map 0..halfW-1 to 0..width-1
                (void)srcX_unused;
                // Actually show 4 panels
                int panelX = x / halfW;  // 0 or 1
                int panelY = y / halfH;  // 0 or 1
                int localX = x % halfW;
                int localY = y % halfH;
                // map local to full-res
                int fullX_unused = (int)(localX * 2.0 / 1.0 * (double)width / width);
                (void)fullX_unused;
                // Keep it simple: sample directly
                int sampleX = localX * 2;
                int sampleY = localY * 2;
                sampleX = std::min(sampleX, width-1);
                sampleY = std::min(sampleY, height-1);
                
                int i = gbuf.idx(sampleX, sampleY);
                Vec3 color;
                
                if (panelX == 0 && panelY == 0) {
                    // Top-left: Albedo
                    color = gbuf.albedo[i];
                } else if (panelX == 1 && panelY == 0) {
                    // Top-right: Normals (encode to 0-1)
                    if (gbuf.depth[i] < 1e8) {
                        color = (gbuf.normal[i] + Vec3(1,1,1)) * 0.5;
                    } else {
                        color = Vec3(0.1,0.1,0.15);
                    }
                } else if (panelX == 0 && panelY == 1) {
                    // Bottom-left: Depth (linearized)
                    if (gbuf.depth[i] < 1e8) {
                        double d = std::min(1.0, gbuf.depth[i] / 30.0);
                        color = Vec3(1-d, 1-d, 1-d);
                    } else {
                        color = Vec3(0,0,0);
                    }
                } else {
                    // Bottom-right: Lighting pass result
                    // We'll fill this later
                    color = Vec3(0.05, 0.05, 0.07);
                }
                
                vis.at(x,y) = color;
            }
        }
        return vis;
    }
};

// ─────────────────────────── Horizontal concatenate ───────────────────────────

Image hconcat(const Image& a, const Image& b) {
    assert(a.height == b.height);
    Image out(a.width + b.width, a.height);
    for (int y = 0; y < a.height; y++) {
        for (int x = 0; x < a.width; x++)
            out.at(x,y) = a.at(x,y);
        for (int x = 0; x < b.width; x++)
            out.at(a.width+x,y) = b.at(x,y);
    }
    return out;
}

Image vconcat(const Image& a, const Image& b) {
    assert(a.width == b.width);
    Image out(a.width, a.height + b.height);
    for (int y = 0; y < a.height; y++)
        for (int x = 0; x < a.width; x++)
            out.at(x,y) = a.at(x,y);
    for (int y = 0; y < b.height; y++)
        for (int x = 0; x < b.width; x++)
            out.at(x, a.height+y) = b.at(x,y);
    return out;
}

// Add label to image (simple, just darken top-left area with text - we'll do without)
// Instead, draw colored border
Image addBorder(Image img, Vec3 color, int thickness=3) {
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            if (x < thickness || x >= img.width-thickness ||
                y < thickness || y >= img.height-thickness) {
                img.at(x,y) = color;
            }
        }
    }
    return img;
}

// ─────────────────────────── Main ───────────────────────────

int main() {
    const int W = 800, H = 600;
    
    std::cout << "=== Deferred Shading Renderer ===" << std::endl;
    std::cout << "Resolution: " << W << "x" << H << std::endl;
    
    // ── Build Scene ──
    Scene scene;
    scene.ambientColor = Vec3(0.3, 0.4, 0.5);
    scene.ambientIntensity = 0.15;
    
    // Materials
    Material matRed    = {Vec3(0.9, 0.2, 0.2), 0.8, 64, 0.2};
    Material matGreen  = {Vec3(0.2, 0.8, 0.3), 0.4, 32, 0.5};
    Material matBlue   = {Vec3(0.2, 0.4, 0.9), 0.9, 128, 0.1};
    Material matGold   = {Vec3(1.0, 0.8, 0.2), 0.95, 256, 0.05};
    Material matWhite  = {Vec3(0.9, 0.9, 0.9), 0.3, 16, 0.7};
    Material matPurple = {Vec3(0.7, 0.2, 0.9), 0.7, 64, 0.3};
    Material matTeal   = {Vec3(0.1, 0.8, 0.8), 0.5, 48, 0.4};
    Material matGray   = {Vec3(0.6, 0.6, 0.6), 0.2, 8, 0.8};
    
    // Spheres
    scene.spheres = {
        {{0.0,  1.0,  0.0}, 1.0,  matGold},     // center - gold shiny
        {{-2.5, 0.7, -1.0}, 0.7,  matRed},       // left-back red
        {{ 2.5, 0.7, -1.0}, 0.7,  matBlue},      // right-back blue
        {{-2.0, 0.5,  1.5}, 0.5,  matGreen},     // left-front green
        {{ 2.0, 0.5,  1.5}, 0.5,  matTeal},      // right-front teal
        {{0.0,  0.4, -3.0}, 0.4,  matPurple},    // far back purple
        {{-1.2, 0.4,  0.5}, 0.4,  matWhite},     // small white
        {{ 1.2, 0.4,  0.5}, 0.4,  matGray},      // small gray
    };
    
    // Ground plane
    Material matGround = {Vec3(0.4, 0.4, 0.35), 0.1, 8, 0.9};
    scene.planes = {
        {{0,-0.001,0}, {0,1,0}, matGround}
    };
    
    // 8 colorful point lights
    scene.lights = {
        {{ 0.0, 4.0,  1.0}, {1.0, 1.0, 1.0}, 3.0, 0.1},   // main white light above
        {{ 3.0, 3.0,  2.0}, {1.0, 0.4, 0.2}, 2.5, 0.1},   // warm orange right
        {{-3.0, 3.0,  2.0}, {0.3, 0.6, 1.0}, 2.5, 0.1},   // cool blue left
        {{ 0.0, 2.0, -3.0}, {0.5, 1.0, 0.5}, 2.0, 0.1},   // green back
        {{ 2.0, 1.0,  3.0}, {1.0, 0.2, 0.8}, 1.5, 0.1},   // magenta front-right
        {{-2.0, 1.0,  3.0}, {1.0, 0.9, 0.1}, 1.5, 0.1},   // yellow front-left
        {{ 0.5, 0.5,  0.5}, {0.2, 0.8, 1.0}, 1.0, 0.1},   // cyan near-center
        {{-0.5, 0.5, -0.5}, {1.0, 0.5, 0.5}, 1.0, 0.1},   // pink near-center-back
    };
    
    // ── Camera ──
    Camera camera(
        Vec3(0, 3, 8),        // eye position
        Vec3(0, 0.5, 0),      // look at
        Vec3(0, 1, 0),        // up
        45.0,                  // fovY
        (double)W/H            // aspect
    );
    
    // ── Deferred Renderer ──
    DeferredRenderer renderer(W, H, camera, scene);
    
    // ── Geometry Pass ──
    std::cout << "[1/4] Geometry Pass (filling G-Buffer)..." << std::endl;
    GBuffer gbuf = renderer.geometryPass();
    std::cout << "      G-Buffer filled: " << W*H << " pixels" << std::endl;
    
    // ── Lighting Pass ──
    std::cout << "[2/4] Lighting Pass (" << scene.lights.size() << " point lights)..." << std::endl;
    Image finalImg = renderer.lightingPass(gbuf);
    
    // ── Save Outputs ──
    std::cout << "[3/4] Saving outputs..." << std::endl;
    
    // Main deferred output
    finalImg.savePNG("deferred_output.png");
    std::cout << "      deferred_output.png saved" << std::endl;
    
    // G-Buffer albedo
    {
        Image albedoImg(W, H);
        for (int y=0; y<H; y++)
            for (int x=0; x<W; x++)
                albedoImg.at(x,y) = gbuf.albedo[gbuf.idx(x,y)];
        albedoImg.savePNG("gbuffer_albedo.png");
        std::cout << "      gbuffer_albedo.png saved" << std::endl;
    }
    
    // G-Buffer normals (encoded to 0-1)
    {
        Image normImg(W, H);
        for (int y=0; y<H; y++) {
            for (int x=0; x<W; x++) {
                int i = gbuf.idx(x,y);
                if (gbuf.depth[i] < 1e8) {
                    normImg.at(x,y) = (gbuf.normal[i] + Vec3(1,1,1)) * 0.5;
                } else {
                    normImg.at(x,y) = Vec3(0.1,0.1,0.15);
                }
            }
        }
        normImg.savePNG("gbuffer_normals.png");
        std::cout << "      gbuffer_normals.png saved" << std::endl;
    }
    
    // G-Buffer depth
    {
        Image depthImg(W, H);
        for (int y=0; y<H; y++) {
            for (int x=0; x<W; x++) {
                int i = gbuf.idx(x,y);
                double d = gbuf.depth[i];
                double nd = (d < 1e8) ? std::min(1.0, d / 20.0) : 1.0;
                // Invert: near=white, far=dark
                depthImg.at(x,y) = Vec3(1-nd, 1-nd, 1-nd);
            }
        }
        depthImg.savePNG("gbuffer_depth.png");
        std::cout << "      gbuffer_depth.png saved" << std::endl;
    }
    
    // ── Comparison: 2x2 Grid ──
    std::cout << "[4/4] Creating comparison grid..." << std::endl;
    {
        // Scale down to half for each panel
        int hW = W/2, hH = H/2;
        
        auto scaleDown = [&](const Image& src) {
            Image dst(hW, hH);
            for (int y=0; y<hH; y++) {
                for (int x=0; x<hW; x++) {
                    // 2x2 box filter
                    Vec3 c(0,0,0);
                    for (int dy=0; dy<2; dy++) for (int dx=0; dx<2; dx++) {
                        c += src.at(std::min(src.width-1, x*2+dx),
                                    std::min(src.height-1, y*2+dy));
                    }
                    dst.at(x,y) = c * 0.25;
                }
            }
            return dst;
        };
        
        // Build panel images
        Image albedoImg(W, H);
        for (int y=0; y<H; y++)
            for (int x=0; x<W; x++)
                albedoImg.at(x,y) = gbuf.albedo[gbuf.idx(x,y)];
        
        Image normImg(W, H);
        for (int y=0; y<H; y++) {
            for (int x=0; x<W; x++) {
                int i = gbuf.idx(x,y);
                if (gbuf.depth[i] < 1e8)
                    normImg.at(x,y) = (gbuf.normal[i]+Vec3(1,1,1))*0.5;
                else
                    normImg.at(x,y) = Vec3(0.08,0.08,0.12);
            }
        }
        
        Image depthImg(W, H);
        for (int y=0; y<H; y++) {
            for (int x=0; x<W; x++) {
                int i = gbuf.idx(x,y);
                double d = gbuf.depth[i];
                double nd = (d < 1e8) ? std::min(1.0, d/20.0) : 1.0;
                depthImg.at(x,y) = Vec3(1-nd, 1-nd, 1-nd);
            }
        }
        
        Image p00 = addBorder(scaleDown(albedoImg), Vec3(0.8,0.2,0.2));  // albedo - red border
        Image p01 = addBorder(scaleDown(normImg),   Vec3(0.2,0.8,0.2));  // normals - green border
        Image p10 = addBorder(scaleDown(depthImg),  Vec3(0.2,0.2,0.8));  // depth - blue border
        Image p11 = addBorder(scaleDown(finalImg),  Vec3(0.8,0.8,0.2));  // final - yellow border
        
        Image top = hconcat(p00, p01);
        Image bot = hconcat(p10, p11);
        Image grid = vconcat(top, bot);
        grid.savePNG("deferred_comparison.png");
        std::cout << "      deferred_comparison.png saved" << std::endl;
    }
    
    // ── Validation Stats ──
    std::cout << std::endl << "=== Validation ===" << std::endl;
    
    // Check center pixel (should be the gold sphere)
    int cx = W/2, cy = H/2;
    Vec3 centerColor = finalImg.at(cx, cy);
    std::cout << "Center pixel RGB: (" 
              << (int)(centerColor.x*255) << ","
              << (int)(centerColor.y*255) << ","
              << (int)(centerColor.z*255) << ")" << std::endl;
    
    // Count lit pixels
    int litPixels = 0;
    double totalBrightness = 0;
    for (int y=0; y<H; y++) {
        for (int x=0; x<W; x++) {
            Vec3 c = finalImg.at(x,y);
            double bright = (c.x+c.y+c.z)/3.0;
            if (bright > 0.05) litPixels++;
            totalBrightness += bright;
        }
    }
    double avgBrightness = totalBrightness / (W*H);
    std::cout << "Lit pixels: " << litPixels << " / " << W*H 
              << " (" << (int)(100.0*litPixels/(W*H)) << "%)" << std::endl;
    std::cout << "Average brightness: " << avgBrightness << std::endl;
    
    // Count pixels with non-trivial normals (geometry hits)
    int geomPixels = 0;
    for (int y=0; y<H; y++) {
        for (int x=0; x<W; x++) {
            if (gbuf.depth[gbuf.idx(x,y)] < 1e8) geomPixels++;
        }
    }
    std::cout << "Geometry pixels in G-Buffer: " << geomPixels 
              << " (" << (int)(100.0*geomPixels/(W*H)) << "%)" << std::endl;
    
    std::cout << std::endl << "=== DONE ===" << std::endl;
    return 0;
}
