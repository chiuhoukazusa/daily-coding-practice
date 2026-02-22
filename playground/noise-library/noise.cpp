#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <functional>

// ========== Perlin 噪声 ==========
class PerlinNoise {
private:
    std::vector<int> p;
    
    double fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    double lerp(double t, double a, double b) { return a + t * (b - a); }
    
    double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
public:
    PerlinNoise(unsigned int seed = 0) {
        p.resize(512);
        std::vector<int> permutation(256);
        for (int i = 0; i < 256; i++) permutation[i] = i;
        
        std::mt19937 rng(seed);
        std::shuffle(permutation.begin(), permutation.end(), rng);
        
        for (int i = 0; i < 512; i++)
            p[i] = permutation[i % 256];
    }
    
    double noise(double x, double y, double z) {
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        int Z = (int)floor(z) & 255;
        
        x -= floor(x);
        y -= floor(y);
        z -= floor(z);
        
        double u = fade(x);
        double v = fade(y);
        double w = fade(z);
        
        int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
        int B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
        
        return lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z),
                                       grad(p[BA], x - 1, y, z)),
                              lerp(u, grad(p[AB], x, y - 1, z),
                                   grad(p[BB], x - 1, y - 1, z))),
                   lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1),
                                grad(p[BA + 1], x - 1, y, z - 1)),
                        lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
                             grad(p[BB + 1], x - 1, y - 1, z - 1))));
    }
    
    // 分形布朗运动
    double fbm(double x, double y, double z, int octaves, double persistence) {
        double total = 0;
        double frequency = 1;
        double amplitude = 1;
        double maxValue = 0;
        
        for (int i = 0; i < octaves; i++) {
            total += noise(x * frequency, y * frequency, z * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2;
        }
        
        return total / maxValue;
    }
};

// ========== Simplex 噪声 ==========
class SimplexNoise {
private:
    std::vector<int> perm;
    
    static const int grad3[12][3];
    
    double dot(const int g[3], double x, double y, double z) {
        return g[0] * x + g[1] * y + g[2] * z;
    }
    
public:
    SimplexNoise(unsigned int seed = 0) {
        perm.resize(512);
        std::vector<int> p(256);
        for (int i = 0; i < 256; i++) p[i] = i;
        
        std::mt19937 rng(seed);
        std::shuffle(p.begin(), p.end(), rng);
        
        for (int i = 0; i < 512; i++)
            perm[i] = p[i % 256];
    }
    
    double noise(double xin, double yin, double zin) {
        double n0, n1, n2, n3;
        const double F3 = 1.0 / 3.0;
        const double G3 = 1.0 / 6.0;
        
        double s = (xin + yin + zin) * F3;
        int i = (int)floor(xin + s);
        int j = (int)floor(yin + s);
        int k = (int)floor(zin + s);
        
        double t = (i + j + k) * G3;
        double X0 = i - t;
        double Y0 = j - t;
        double Z0 = k - t;
        double x0 = xin - X0;
        double y0 = yin - Y0;
        double z0 = zin - Z0;
        
        int i1, j1, k1, i2, j2, k2;
        if (x0 >= y0) {
            if (y0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
            else if (x0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1; }
            else { i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1; }
        } else {
            if (y0 < z0) { i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1; }
            else if (x0 < z0) { i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1; }
            else { i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
        }
        
        double x1 = x0 - i1 + G3;
        double y1 = y0 - j1 + G3;
        double z1 = z0 - k1 + G3;
        double x2 = x0 - i2 + 2.0 * G3;
        double y2 = y0 - j2 + 2.0 * G3;
        double z2 = z0 - k2 + 2.0 * G3;
        double x3 = x0 - 1.0 + 3.0 * G3;
        double y3 = y0 - 1.0 + 3.0 * G3;
        double z3 = z0 - 1.0 + 3.0 * G3;
        
        int ii = i & 255;
        int jj = j & 255;
        int kk = k & 255;
        
        int gi0 = perm[ii + perm[jj + perm[kk]]] % 12;
        int gi1 = perm[ii + i1 + perm[jj + j1 + perm[kk + k1]]] % 12;
        int gi2 = perm[ii + i2 + perm[jj + j2 + perm[kk + k2]]] % 12;
        int gi3 = perm[ii + 1 + perm[jj + 1 + perm[kk + 1]]] % 12;
        
        double t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0;
        if (t0 < 0) n0 = 0.0;
        else {
            t0 *= t0;
            n0 = t0 * t0 * dot(grad3[gi0], x0, y0, z0);
        }
        
        double t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1;
        if (t1 < 0) n1 = 0.0;
        else {
            t1 *= t1;
            n1 = t1 * t1 * dot(grad3[gi1], x1, y1, z1);
        }
        
        double t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2;
        if (t2 < 0) n2 = 0.0;
        else {
            t2 *= t2;
            n2 = t2 * t2 * dot(grad3[gi2], x2, y2, z2);
        }
        
        double t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3;
        if (t3 < 0) n3 = 0.0;
        else {
            t3 *= t3;
            n3 = t3 * t3 * dot(grad3[gi3], x3, y3, z3);
        }
        
        return 32.0 * (n0 + n1 + n2 + n3);
    }
};

const int SimplexNoise::grad3[12][3] = {
    {1,1,0}, {-1,1,0}, {1,-1,0}, {-1,-1,0},
    {1,0,1}, {-1,0,1}, {1,0,-1}, {-1,0,-1},
    {0,1,1}, {0,-1,1}, {0,1,-1}, {0,-1,-1}
};

// ========== Worley 噪声 (Cellular/Voronoi) ==========
class WorleyNoise {
private:
    std::mt19937 rng;
    
    double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }
    
public:
    WorleyNoise(unsigned int seed = 0) : rng(seed) {}
    
    double noise(double x, double y, int numPoints = 10) {
        int cellX = (int)floor(x);
        int cellY = (int)floor(y);
        
        double minDist = 999999;
        
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = cellX + dx;
                int ny = cellY + dy;
                
                rng.seed(nx * 374761393 + ny * 668265263);
                std::uniform_real_distribution<double> dist(0, 1);
                
                for (int p = 0; p < numPoints; p++) {
                    double px = nx + dist(rng);
                    double py = ny + dist(rng);
                    double d = distance(x, y, px, py);
                    minDist = std::min(minDist, d);
                }
            }
        }
        
        return minDist;
    }
};

// ========== 渲染函数 ==========
void renderNoise(const char* filename, int width, int height, 
                 std::function<double(double, double)> noiseFn,
                 double scale = 0.01) {
    
    std::vector<unsigned char> pixels(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double value = noiseFn(x * scale, y * scale);
            value = (value + 1.0) * 0.5;  // [-1,1] → [0,1]
            value = std::clamp(value, 0.0, 1.0);
            
            unsigned char gray = (unsigned char)(value * 255);
            int idx = (y * width + x) * 3;
            pixels[idx] = gray;
            pixels[idx + 1] = gray;
            pixels[idx + 2] = gray;
        }
    }
    
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

int main() {
    const int W = 800, H = 800;
    
    PerlinNoise perlin(42);
    SimplexNoise simplex(42);
    WorleyNoise worley(42);
    
    // 1. Perlin 噪声
    renderNoise("noise_perlin.png", W, H, [&](double x, double y) {
        return perlin.noise(x, y, 0);
    }, 0.01);
    
    // 2. Perlin FBM (分形布朗运动)
    renderNoise("noise_perlin_fbm.png", W, H, [&](double x, double y) {
        return perlin.fbm(x, y, 0, 8, 0.5);
    }, 0.005);
    
    // 3. Simplex 噪声
    renderNoise("noise_simplex.png", W, H, [&](double x, double y) {
        return simplex.noise(x, y, 0);
    }, 0.01);
    
    // 4. Worley 噪声（细胞纹理）
    renderNoise("noise_worley.png", W, H, [&](double x, double y) {
        return worley.noise(x * 0.05, y * 0.05, 1) * 10 - 1;
    }, 1.0);
    
    // 5. Turbulence（湍流）
    renderNoise("noise_turbulence.png", W, H, [&](double x, double y) {
        double t = 0;
        double scale = 0.01;
        for (int i = 0; i < 6; i++) {
            t += fabs(perlin.noise(x * scale, y * scale, 0)) / scale;
            scale *= 2;
        }
        return t * 0.01 - 1;
    }, 1.0);
    
    // 6. 大理石纹理
    renderNoise("noise_marble.png", W, H, [&](double x, double y) {
        double n = perlin.fbm(x, y, 0, 6, 0.5);
        return sin(x * 0.05 + n * 5);
    }, 1.0);
    
    return 0;
}
