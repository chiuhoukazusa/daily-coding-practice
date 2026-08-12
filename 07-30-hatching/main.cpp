// Cross-Hatching NPR Renderer
// Uses tonal hatching patterns instead of smooth shading
// Multi-angle hatch lines create 6 tonal levels + edges

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cfloat>

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;
constexpr double PI = 3.14159265358979323846;

// ============ Vector Math ============
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double l = length(); return l > 1e-10 ? *this / l : Vec3(); }
};

// ============ Scene Objects ============
struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    double specular; // 0-1 for shininess level
};

struct Plane {
    Vec3 point;
    Vec3 normal;
    Vec3 color;
    double specular;
};

// Checkerboard pattern
Vec3 checkerColor(const Vec3& p) {
    int u = (int)std::floor(p.x * 2);
    int v = (int)std::floor(p.z * 2);
    bool white = ((u + v) & 1) == 0;
    return white ? Vec3(0.9, 0.9, 0.9) : Vec3(0.2, 0.2, 0.2);
}

// ============ Lighting ============
Vec3 lightPos(-3, 5, 2);
Vec3 ambientColor(0.15, 0.15, 0.15);
Vec3 lightColor(0.85, 0.85, 0.85);
Vec3 lightPos2(4, 3, -3);
Vec3 lightColor2(0.3, 0.25, 0.35);

double computeLuminance(const Vec3& rgb) {
    return 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z;
}

Vec3 computeLighting(const Vec3& point, const Vec3& normal, const Vec3& viewDir,
                     const Vec3& objColor, double specularStrength) {
    Vec3 result = Vec3(ambientColor.x * objColor.x,
                       ambientColor.y * objColor.y,
                       ambientColor.z * objColor.z);
    
    // Light 1
    Vec3 lightDir = (lightPos - point).normalized();
    double diff1 = std::max(0.0, normal.dot(lightDir));
    Vec3 halfVec = (lightDir + viewDir).normalized();
    double spec1 = std::pow(std::max(0.0, normal.dot(halfVec)), 32.0) * specularStrength;
    
    result.x += objColor.x * diff1 * lightColor.x + spec1 * lightColor.x;
    result.y += objColor.y * diff1 * lightColor.y + spec1 * lightColor.y;
    result.z += objColor.z * diff1 * lightColor.z + spec1 * lightColor.z;
    
    // Light 2 (fill)
    Vec3 lightDir2 = (lightPos2 - point).normalized();
    double diff2 = std::max(0.0, normal.dot(lightDir2));
    Vec3 halfVec2 = (lightDir2 + viewDir).normalized();
    double spec2 = std::pow(std::max(0.0, normal.dot(halfVec2)), 32.0) * specularStrength * 0.5;
    
    result.x += objColor.x * diff2 * lightColor2.x + spec2 * lightColor2.x;
    result.y += objColor.y * diff2 * lightColor2.y + spec2 * lightColor2.y;
    result.z += objColor.z * diff2 * lightColor2.z + spec2 * lightColor2.z;
    
    return result;
}

// ============ Edge Detection via Sobel on Depth + Normal ============
void detectEdges(const std::vector<double>& depthBuf,
                 const std::vector<Vec3>& normalBuf,
                 std::vector<double>& edgeBuf) {
    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 1; x < WIDTH - 1; x++) {
            int idx = y * WIDTH + x;
            
            // Depth gradient
            double gx_d = -depthBuf[y*WIDTH+(x-1)] + depthBuf[y*WIDTH+(x+1)];
            double gy_d = -depthBuf[(y-1)*WIDTH+x] + depthBuf[(y+1)*WIDTH+x];
            double depthGrad = std::sqrt(gx_d*gx_d + gy_d*gy_d);
            
            // Normal gradient
            double gx_n = 0, gy_n = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nidx = (y+dy)*WIDTH + (x+dx);
                    double gx = (dx == 1) ? 1.0 : (dx == -1 ? -1.0 : 0);
                    double gy = (dy == 1) ? 1.0 : (dy == -1 ? -1.0 : 0);
                    if (dx == 0 && dy == 0) continue;
                    double w = (std::abs(dx) + std::abs(dy) == 2) ? 1.0 : 2.0;
                    
                    double ndiff = std::abs(normalBuf[idx].dot(normalBuf[nidx]) - 1.0);
                    gx_n += ndiff * gx * w;
                    gy_n += ndiff * gy * w;
                }
            }
            double normGrad = std::sqrt(gx_n*gx_n + gy_n*gy_n);
            
            edgeBuf[idx] = std::min(1.0, (depthGrad * 0.5 + normGrad * 1.5) * 3.0);
        }
    }
}

// ============ Hatch Pattern Rendering ============
// 6 tonal levels (0-5), angles at 0, 30, 60, 90, 120, 150 degrees
// Level 0: white (no hatching)
// Level 1: single direction (sparse)
// Level 2: single direction (dense)
// Level 3: two directions
// Level 4: three directions
// Level 5: four+ directions (near black)

// Precomputed hatch angles for each level
const double hatchAngles[6][4] = {
    {},                    // level 0: none
    {0.0},                 // level 1: horizontal
    {0.0},                 // level 2: horizontal (dense)
    {0.0, PI/3},          // level 3: 0° + 60°
    {0.0, PI/3, 2*PI/3},  // level 4: 0° + 60° + 120°
    {0.0, PI/4, PI/2, 3*PI/4} // level 5: 0° + 45° + 90° + 135°
};

const int hatchCount[6] = {0, 1, 1, 2, 3, 4};
const double hatchDensity[6] = {0.0, 0.8, 0.5, 0.65, 0.5, 0.4};
const double hatchWidth[6] = {0.0, 0.25, 0.35, 0.3, 0.28, 0.25};

// Map luminance [0,1] to hatch level [0,5] (0=dark, 5=bright)
int luminanceToLevel(double lum) {
    if (lum < 0.08) return 5;
    if (lum < 0.20) return 4;
    if (lum < 0.38) return 3;
    if (lum < 0.58) return 2;
    if (lum < 0.80) return 1;
    return 0;
}

double hatchAtPixel(int x, int y, int level, double edgeStrength) {
    if (level == 0) return 0.0; // white, no hatching
    
    double hatch = 0.0;
    int count = hatchCount[level];
    
    // Background darkness from hatch level (remaining white portion)
    double baseDarkness = level * 0.16;
    
    for (int i = 0; i < count; i++) {
        double angle = hatchAngles[level][i];
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        
        // Project pixel onto line direction
        double proj = x * cosA + y * sinA;
        
        // Periodic hatch pattern
        double density = hatchDensity[level];
        double width = hatchWidth[level];
        double period = 1.0 / density;
        
        double phase = proj / period;
        double frac = phase - std::floor(phase);
        
        // Add some wobble for hand-drawn look
        double wobble = std::sin(y * 0.3 + x * 0.2) * 0.08;
        frac += wobble;
        
        if (frac < width) {
            hatch += 1.0 / count;
        }
    }
    
    // Edge overlay: draw solid for edges
    if (edgeStrength > 0.3) {
        hatch = std::max(hatch, edgeStrength * 0.9 + 0.1);
    }
    
    return std::min(1.0, hatch + baseDarkness);
}

// ============ Main Rendering ============
int main() {
    // Scene: metallic spheres on checkerboard ground + a wall plane
    std::vector<Sphere> spheres = {
        {{0.0, 0.0, 2.5}, 1.0, {0.8, 0.2, 0.2}, 0.8},    // red sphere (center)
        {{-1.8, -0.3, 3.5}, 0.7, {0.2, 0.2, 0.9}, 0.6},   // blue sphere (left back)
        {{1.6, 0.3, 2.0}, 0.6, {0.2, 0.8, 0.2}, 0.3},     // green sphere (right front)
        {{-2.5, -0.1, 1.5}, 0.9, {0.9, 0.7, 0.2}, 0.95},  // gold sphere (left front)
        {{1.8, 0.5, 3.8}, 0.4, {0.7, 0.2, 0.7}, 0.5},    // purple sphere (right back small)
    };
    
    Plane ground = {{0, -1, 0}, {0, 1, 0}, {0.9, 0.9, 0.9}, 0.3};
    Plane backWall = {{0, 0, 5}, {0, 0, -1}, {0.8, 0.75, 0.7}, 0.1};
    
    Vec3 cameraPos(0, 0.8, -1.5);
    Vec3 cameraDir(0, -0.05, 1);
    Vec3 cameraUp(0, 1, 0);
    
    Vec3 viewDir = cameraDir.normalized();
    Vec3 right = viewDir.cross(cameraUp).normalized();
    Vec3 up = right.cross(viewDir).normalized();
    
    double fov = 60.0 * PI / 180.0;
    double halfH = std::tan(fov / 2);
    double halfW = halfH * WIDTH / HEIGHT;
    
    std::vector<Vec3> colorBuf(WIDTH * HEIGHT, Vec3(0, 0, 0));
    std::vector<double> depthBuf(WIDTH * HEIGHT, DBL_MAX);
    std::vector<Vec3> normalBuf(WIDTH * HEIGHT, Vec3(0, 0, 0));
    
    int sampledPixels = 0;
    
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            double u = (2.0 * (x + 0.5) / WIDTH - 1.0) * halfW;
            double v = (1.0 - 2.0 * (y + 0.5) / HEIGHT) * halfH;
            
            Vec3 rayDir = (viewDir + right * u + up * v).normalized();
            Vec3 rayOrigin = cameraPos;
            
            double tMin = DBL_MAX;
            Vec3 hitColor, hitNormal;
            double hitSpec = 0.2;
            bool hit = false;
            
            // Test spheres
            for (const auto& s : spheres) {
                Vec3 oc = rayOrigin - s.center;
                double b = oc.dot(rayDir);
                double c = oc.dot(oc) - s.radius * s.radius;
                double disc = b*b - c;
                
                if (disc > 0) {
                    double t = -b - std::sqrt(disc);
                    if (t > 0.001 && t < tMin) {
                        tMin = t;
                        Vec3 hitPoint = rayOrigin + rayDir * t;
                        hitNormal = (hitPoint - s.center).normalized();
                        hitColor = s.color;
                        hitSpec = s.specular;
                        hit = true;
                    }
                    // Check if second intersection is closer (inside sphere case)
                    double t2 = -b + std::sqrt(disc);
                    if (t2 > 0.001 && t2 < tMin) {
                        tMin = t2;
                        Vec3 hitPoint = rayOrigin + rayDir * t2;
                        hitNormal = (hitPoint - s.center).normalized();
                        hitColor = s.color;
                        hitSpec = s.specular;
                        hit = true;
                    }
                }
            }
            
            // Test ground plane
            {
                double denom = ground.normal.dot(rayDir);
                if (std::abs(denom) > 1e-6) {
                    double t = (ground.point - rayOrigin).dot(ground.normal) / denom;
                    if (t > 0.001 && t < tMin) {
                        tMin = t;
                        Vec3 hitPoint = rayOrigin + rayDir * t;
                        hitNormal = ground.normal;
                        hitColor = checkerColor(hitPoint);
                        hitSpec = ground.specular;
                        hit = true;
                    }
                }
            }
            
            // Test back wall
            {
                double denom = backWall.normal.dot(rayDir);
                if (std::abs(denom) > 1e-6) {
                    double t = (backWall.point - rayOrigin).dot(backWall.normal) / denom;
                    if (t > 0.001 && t < tMin) {
                        tMin = t;
                        Vec3 hitPoint = rayOrigin + rayDir * t;
                        hitNormal = backWall.normal;
                        // Wall checker like pattern for back wall
                        int u = (int)std::floor(hitPoint.x * 2);
                        int v = (int)std::floor(hitPoint.y * 2);
                        bool white = ((u + v) & 1) == 0;
                        hitColor = white ? Vec3(0.85, 0.78, 0.72) : Vec3(0.65, 0.58, 0.52);
                        hitSpec = backWall.specular;
                        hit = true;
                    }
                }
            }
            
            if (hit && tMin < 1000) {
                sampledPixels++;
                Vec3 hitPoint = rayOrigin + rayDir * tMin;
                Vec3 viewVec = (cameraPos - hitPoint).normalized();
                
                Vec3 litColor = computeLighting(hitPoint, hitNormal, viewVec, hitColor, hitSpec);
                colorBuf[y*WIDTH + x] = litColor;
                depthBuf[y*WIDTH + x] = tMin;
                normalBuf[y*WIDTH + x] = hitNormal;
            } else {
                // Sky gradient
                double skyT = (rayDir.y + 1.0) * 0.5;
                colorBuf[y*WIDTH + x] = Vec3(0.5 + 0.3 * skyT, 0.6 + 0.2 * skyT, 0.85 + 0.1 * skyT);
                depthBuf[y*WIDTH + x] = DBL_MAX;
                normalBuf[y*WIDTH + x] = Vec3(0, 1, 0);
            }
        }
    }
    
    std::cout << "Sampled pixels: " << sampledPixels << "/" << (WIDTH*HEIGHT) << std::endl;
    
    // Edge detection
    std::vector<double> edgeBuf(WIDTH * HEIGHT, 0.0);
    detectEdges(depthBuf, normalBuf, edgeBuf);
    
    // Apply cross-hatching
    std::vector<unsigned char> image(WIDTH * HEIGHT * 3);
    double totalLum = 0, totalHatch = 0;
    
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int idx = y * WIDTH + x;
            double lum = computeLuminance(colorBuf[idx]);
            int level = luminanceToLevel(lum);
            double edgeStr = edgeBuf[idx];
            
            double hatch = hatchAtPixel(x, y, level, edgeStr);
            
            totalLum += lum;
            totalHatch += hatch;
            
            // Background paper color (slightly warm white)
            double paperR = 0.97, paperG = 0.95, paperB = 0.90;
            
            // Mix: hatch determines ink amount
            double inkR = 0.05, inkG = 0.05, inkB = 0.08; // dark ink color
            
            // Add slight color tint based on object color
            double colorInfluence = 0.3;
            double tintR = colorBuf[idx].x * colorInfluence;
            double tintG = colorBuf[idx].y * colorInfluence;
            double tintB = colorBuf[idx].z * colorInfluence;
            
            double r = paperR * (1 - hatch) + (inkR * (1 - colorInfluence) + tintR) * hatch;
            double g = paperG * (1 - hatch) + (inkG * (1 - colorInfluence) + tintG) * hatch;
            double b = paperB * (1 - hatch) + (inkB * (1 - colorInfluence) + tintB) * hatch;
            
            image[idx*3 + 0] = (unsigned char)(std::min(255.0, std::max(0.0, r * 255)));
            image[idx*3 + 1] = (unsigned char)(std::min(255.0, std::max(0.0, g * 255)));
            image[idx*3 + 2] = (unsigned char)(std::min(255.0, std::max(0.0, b * 255)));
        }
    }
    
    double avgLum = totalLum / (WIDTH * HEIGHT);
    double avgHatch = totalHatch / (WIDTH * HEIGHT);
    std::cout << "Average luminance: " << avgLum << std::endl;
    std::cout << "Average hatch: " << avgHatch << std::endl;
    
    // Write PPM
    std::ofstream out("hatching_output.ppm", std::ios::binary);
    out << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    out.write(reinterpret_cast<char*>(image.data()), image.size());
    out.close();
    
    // Statistics for verification
    // Count pixels per hatch level
    int levelCounts[6] = {};
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int idx = y * WIDTH + x;
            double lum = computeLuminance(colorBuf[idx]);
            int level = luminanceToLevel(lum);
            levelCounts[level]++;
        }
    }
    
    std::cout << "\n=== Pixel Distribution per Hatch Level ===" << std::endl;
    for (int i = 0; i < 6; i++) {
        double pct = 100.0 * levelCounts[i] / (WIDTH * HEIGHT);
        std::cout << "Level " << i << ": " << levelCounts[i] << " pixels (" << pct << "%)" << std::endl;
    }
    
    // Verify all level counts are non-zero (should have some variety)
    int nonZeroLevels = 0;
    for (int i = 0; i < 6; i++) if (levelCounts[i] > 0) nonZeroLevels++;
    std::cout << "Non-zero levels: " << nonZeroLevels << "/6" << std::endl;
    
    // Edge pixel count
    int edgePixels = 0;
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        if (edgeBuf[i] > 0.3) edgePixels++;
    }
    double edgePct = 100.0 * edgePixels / (WIDTH * HEIGHT);
    std::cout << "Edge pixels (>0.3): " << edgePixels << " (" << edgePct << "%)" << std::endl;
    
    // Image pixel statistics (quantitative verification)
    double imgMean = 0, imgStd = 0;
    for (int i = 0; i < WIDTH * HEIGHT * 3; i++) {
        imgMean += image[i];
        imgStd += image[i] * image[i];
    }
    imgMean /= (WIDTH * HEIGHT * 3);
    imgStd = std::sqrt(imgStd / (WIDTH * HEIGHT * 3) - imgMean * imgMean);
    std::cout << "\nImage stats: mean=" << imgMean << " std=" << imgStd << std::endl;
    
    // Check file size
    std::ifstream check("hatching_output.ppm", std::ios::binary | std::ios::ate);
    long long fsize = check.tellg();
    check.close();
    std::cout << "File size: " << fsize << " bytes" << std::endl;
    
    return 0;
}
