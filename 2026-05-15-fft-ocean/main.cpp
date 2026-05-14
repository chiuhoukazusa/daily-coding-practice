// FFT Ocean Surface Renderer
// Technique: Phillips spectrum + FFT (Cooley-Tukey) + Gerstner waves
// Renders a realistic ocean surface with per-pixel normals and Blinn-Phong lighting
// Output: fft_ocean_output.png (800x600)

#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <cassert>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ============================================================
// Math helpers
// ============================================================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x,y-o.y,z-o.z}; }
    Vec3 operator*(float t) const { return {x*t,y*t,z*t}; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>1e-8f ? Vec3(x/l,y/l,z/l) : Vec3(0,0,1); }
    Vec3 clamp01() const {
        return { std::max(0.0f,std::min(1.0f,x)),
                 std::max(0.0f,std::min(1.0f,y)),
                 std::max(0.0f,std::min(1.0f,z)) };
    }
};
inline float clamp01(float v){ return std::max(0.0f,std::min(1.0f,v)); }

// ============================================================
// Constants
// ============================================================
static const int   N      = 256;       // FFT grid size (must be power of 2)
static const float L      = 512.0f;    // Ocean patch size (meters)
static const float A      = 1.2e-3f;   // Phillips amplitude
static const float V      = 20.0f;     // Wind speed (m/s)
static const Vec2  W      = {1.0f, 1.0f}; // Wind direction (normalized)
static const float G      = 9.81f;     // Gravity

static const int   IMG_W  = 800;
static const int   IMG_H  = 600;

using Complex = std::complex<float>;

// ============================================================
// Framebuffer
// ============================================================
struct Framebuffer {
    int w, h;
    std::vector<Vec3> data;
    Framebuffer(int w,int h):w(w),h(h),data(w*h,Vec3(0,0,0)){}
    void set(int x,int y,Vec3 c){
        if(x>=0&&x<w&&y>=0&&y<h) data[y*w+x]=c;
    }
    Vec3 get(int x,int y) const {
        if(x>=0&&x<w&&y>=0&&y<h) return data[y*w+x];
        return Vec3(0,0,0);
    }
};

// ============================================================
// Phillips spectrum H0(k)
// ============================================================
float phillipsSpectrum(Vec2 k) {
    float kLen = std::sqrt(k.x*k.x + k.y*k.y);
    if(kLen < 1e-6f) return 0.0f;

    float L_  = V*V / G;  // dominant wavelength
    float k2  = kLen * kLen;
    float k4  = k2 * k2;
    float kL2 = k2 * L_ * L_;
    float Ph  = A * std::exp(-1.0f / kL2) / k4;

    // Directional spreading: (k_hat . w_hat)^2
    float kHatDotW = (k.x * W.x + k.y * W.y) / kLen;
    Ph *= kHatDotW * kHatDotW;

    // Suppress very small waves
    float l = 0.001f * L_;
    Ph *= std::exp(-k2 * l * l);

    return Ph;
}

// ============================================================
// Initialize H0 (complex amplitude spectrum)
// ============================================================
void initH0(std::vector<Complex>& h0, std::vector<Complex>& h0conj,
             std::mt19937& rng)
{
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    h0.resize(N * N);
    h0conj.resize(N * N);

    for(int m = 0; m < N; m++){
        for(int n = 0; n < N; n++){
            Vec2 k = {
                (float)(2.0 * M_PI * (n - N/2)) / L,
                (float)(2.0 * M_PI * (m - N/2)) / L
            };
            float P = phillipsSpectrum(k);
            float sqrtP = std::sqrt(P * 0.5f);
            float xi_r = gauss(rng);
            float xi_i = gauss(rng);
            h0[m*N+n] = Complex(xi_r * sqrtP, xi_i * sqrtP);
            // h0(-k)*
            Vec2 mk = {-k.x, -k.y};
            float Pm = phillipsSpectrum(mk);
            float sqrtPm = std::sqrt(Pm * 0.5f);
            float xir2 = gauss(rng);
            float xii2 = gauss(rng);
            h0conj[m*N+n] = std::conj(Complex(xir2 * sqrtPm, xii2 * sqrtPm));
        }
    }
}

// ============================================================
// Cooley-Tukey FFT (in-place, recursive-style iterative)
// ============================================================
void fft1D(std::vector<Complex>& a, bool inverse) {
    int n = (int)a.size();
    // Bit-reversal permutation
    for(int i=1,j=0; i<n; i++){
        int bit = n >> 1;
        for(; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if(i < j) std::swap(a[i], a[j]);
    }
    // Butterfly
    for(int len=2; len<=n; len<<=1){
        float ang = (inverse ? 1.0f : -1.0f) * 2.0f * (float)M_PI / len;
        Complex wlen(std::cos(ang), std::sin(ang));
        for(int i=0; i<n; i+=len){
            Complex w(1.0f, 0.0f);
            for(int j=0; j<len/2; j++){
                Complex u = a[i+j];
                Complex v = a[i+j+len/2] * w;
                a[i+j]       = u + v;
                a[i+j+len/2] = u - v;
                w *= wlen;
            }
        }
    }
    if(inverse){
        for(auto& x : a) x /= (float)n;
    }
}

void fft2D(std::vector<Complex>& grid, bool inverse) {
    std::vector<Complex> row(N);
    // FFT rows
    for(int m=0; m<N; m++){
        for(int n=0; n<N; n++) row[n] = grid[m*N+n];
        fft1D(row, inverse);
        for(int n=0; n<N; n++) grid[m*N+n] = row[n];
    }
    // FFT cols
    std::vector<Complex> col(N);
    for(int n=0; n<N; n++){
        for(int m=0; m<N; m++) col[m] = grid[m*N+n];
        fft1D(col, inverse);
        for(int m=0; m<N; m++) grid[m*N+n] = col[m];
    }
}

// ============================================================
// Compute height field H(x,t) and gradient (dx,dz)
// ============================================================
void computeOcean(float t,
                  const std::vector<Complex>& h0,
                  const std::vector<Complex>& h0conj,
                  std::vector<float>& height,
                  std::vector<float>& slopeX,
                  std::vector<float>& slopeZ)
{
    std::vector<Complex> ht(N*N);
    std::vector<Complex> dhtdx(N*N);
    std::vector<Complex> dhtdz(N*N);

    for(int m=0; m<N; m++){
        for(int n=0; n<N; n++){
            Vec2 k = {
                (float)(2.0 * M_PI * (n - N/2)) / L,
                (float)(2.0 * M_PI * (m - N/2)) / L
            };
            float kLen = std::sqrt(k.x*k.x + k.y*k.y);
            float omega = std::sqrt(G * kLen);  // dispersion

            Complex e_pos(std::cos(omega*t), std::sin(omega*t));
            Complex e_neg(std::cos(omega*t), -std::sin(omega*t));

            int idx = m*N+n;
            Complex H = h0[idx] * e_pos + h0conj[idx] * e_neg;
            ht[idx]   = H;

            // Gradient: ik * H(k,t)
            dhtdx[idx] = Complex(-H.imag() * k.x,  H.real() * k.x);  // i*kx * H
            dhtdz[idx] = Complex(-H.imag() * k.y,  H.real() * k.y);  // i*kz * H
        }
    }

    // Inverse FFT to get spatial domain
    fft2D(ht,    true);
    fft2D(dhtdx, true);
    fft2D(dhtdz, true);

    height.resize(N*N);
    slopeX.resize(N*N);
    slopeZ.resize(N*N);

    for(int i=0; i<N*N; i++){
        // Sign correction for FFT shift
        int m = i / N;
        int n = i % N;
        float sign = ((m + n) & 1) ? -1.0f : 1.0f;
        height[i] = ht[i].real()    * sign;
        slopeX[i] = dhtdx[i].real() * sign;
        slopeZ[i] = dhtdz[i].real() * sign;
    }
}

// ============================================================
// Rendering: top-down view of ocean surface with shading
// ============================================================
// Camera: looking straight down from above at y=60m
// The ocean XZ patch [0,L]x[0,L] maps to image plane

Vec3 tonemap(Vec3 c) {
    // Reinhard
    return { c.x/(1.0f+c.x), c.y/(1.0f+c.y), c.z/(1.0f+c.z) };
}

Vec3 sampleBilinear(const std::vector<float>& field, float tx, float tz) {
    // tx, tz in [0, N-1]
    int ix = (int)tx % N;
    int iz = (int)tz % N;
    if(ix < 0) ix += N;
    if(iz < 0) iz += N;
    int ix1 = (ix+1) % N;
    int iz1 = (iz+1) % N;
    float fx = tx - std::floor(tx);
    float fz = tz - std::floor(tz);
    float v00 = field[iz*N+ix];
    float v10 = field[iz*N+ix1];
    float v01 = field[iz1*N+ix];
    float v11 = field[iz1*N+ix1];
    return Vec3(
        v00*(1-fx)*(1-fz) + v10*fx*(1-fz) + v01*(1-fx)*fz + v11*fx*fz,
        0, 0
    );
}

int main() {
    std::cout << "[FFT Ocean] Initializing..." << std::endl;

    // Init RNG
    std::mt19937 rng(42);

    // Build initial spectrum
    std::vector<Complex> h0, h0conj;
    initH0(h0, h0conj, rng);

    // Time steps to render (pick t=10s for nice wave pattern)
    float t = 10.0f;

    std::cout << "[FFT Ocean] Computing FFT ocean at t=" << t << "s..." << std::endl;
    std::vector<float> height, slopeX, slopeZ;
    computeOcean(t, h0, h0conj, height, slopeX, slopeZ);

    // Stats
    float hMin = *std::min_element(height.begin(), height.end());
    float hMax = *std::max_element(height.begin(), height.end());
    float hRange = hMax - hMin;
    std::cout << "[FFT Ocean] Height range: [" << hMin << ", " << hMax << "]" << std::endl;

    // Render: map ocean grid to image
    Framebuffer fb(IMG_W, IMG_H);

    // Light direction (sun)
    Vec3 lightDir = Vec3(1.0f, 2.0f, 1.0f).norm();
    // Camera position (top-down perspective with slight angle)
    // Vec3 camPos(256.0f, 80.0f, 256.0f);  // unused in current top-down render

    for(int py = 0; py < IMG_H; py++){
        for(int px = 0; px < IMG_W; px++){
            // Map pixel to ocean patch coordinates
            float u = (float)px / IMG_W;
            float v = (float)(IMG_H - 1 - py) / IMG_H;  // flip Y so sky is up

            // World XZ position on ocean surface
            float wx = u * L;
            float wz = v * L;

            // Grid index (with tiling)
            float gx = (wx / L) * N;
            float gz = (wz / L) * N;

            // Sample height and slopes with bilinear interpolation
            int igx = (int)gx; if(igx < 0) igx = 0; if(igx >= N) igx = N-1;
            int igz = (int)gz; if(igz < 0) igz = 0; if(igz >= N) igz = N-1;

            float h   = height[igz * N + igx];
            float dhdx= slopeX[igz * N + igx];
            float dhdz= slopeZ[igz * N + igx];

            // World-space normal from gradient: N = normalize(-dh/dx, 1, -dh/dz)
            Vec3 normal = Vec3(-dhdx, 1.0f, -dhdz).norm();

            // === Blinn-Phong shading ===
            Vec3 viewDir = Vec3(0, 1, 0);  // top-down view
            Vec3 halfV = (lightDir + viewDir).norm();

            float diffuse  = std::max(0.0f, normal.dot(lightDir));
            float specular = std::pow(std::max(0.0f, normal.dot(halfV)), 64.0f);

            // === Ocean color ===
            // Deep water: dark blue
            // Crests: bright white/cyan
            // Height-based: normalize height
            float hn = hRange > 0.0f ? (h - hMin) / hRange : 0.5f;

            // Base ocean color
            Vec3 deepColor  (0.01f, 0.05f, 0.20f);  // deep blue
            Vec3 shallowColor(0.05f, 0.25f, 0.45f);  // mid blue
            Vec3 crestColor  (0.80f, 0.90f, 1.00f);  // white crest

            Vec3 baseColor;
            if(hn < 0.7f){
                float t2 = hn / 0.7f;
                baseColor = deepColor * (1-t2) + shallowColor * t2;
            } else {
                float t2 = (hn - 0.7f) / 0.3f;
                baseColor = shallowColor * (1-t2) + crestColor * t2;
            }

            // Ambient + diffuse + specular
            float ambient = 0.15f;
            Vec3 diffColor = baseColor * (ambient + diffuse * 0.6f);
            Vec3 specColor (specular * 0.8f, specular * 0.85f, specular * 1.0f);

            // Fresnel approximation (more reflective at shallow angles)
            float fresnel = 0.04f + 0.96f * std::pow(1.0f - std::abs(normal.dot(viewDir)), 5.0f);
            Vec3 skyColor(0.4f, 0.6f, 0.9f);  // sky reflection
            Vec3 color = diffColor + specColor + skyColor * (fresnel * 0.4f);

            // Slight height-based fog/depth
            // (already encoded in hn above)

            // Tonemap
            color = tonemap(color);
            color = color.clamp01();

            // Gamma correction
            color.x = std::pow(color.x, 1.0f/2.2f);
            color.y = std::pow(color.y, 1.0f/2.2f);
            color.z = std::pow(color.z, 1.0f/2.2f);

            fb.set(px, py, color);
        }
    }

    // Save image
    std::vector<uint8_t> pixels(IMG_W * IMG_H * 3);
    for(int y=0; y<IMG_H; y++){
        for(int x=0; x<IMG_W; x++){
            Vec3 c = fb.get(x, y);
            int idx = (y * IMG_W + x) * 3;
            pixels[idx+0] = (uint8_t)(c.x * 255.0f);
            pixels[idx+1] = (uint8_t)(c.y * 255.0f);
            pixels[idx+2] = (uint8_t)(c.z * 255.0f);
        }
    }

    const char* filename = "fft_ocean_output.png";
    int ok = stbi_write_png(filename, IMG_W, IMG_H, 3, pixels.data(), IMG_W*3);
    if(!ok){
        std::cerr << "❌ Failed to write PNG!" << std::endl;
        return 1;
    }
    std::cout << "✅ Saved: " << filename << std::endl;

    // Quick stats for validation
    float totalR = 0, totalG = 0, totalB = 0;
    for(int i = 0; i < IMG_W * IMG_H; i++){
        totalR += pixels[i*3+0];
        totalG += pixels[i*3+1];
        totalB += pixels[i*3+2];
    }
    float np = (float)(IMG_W * IMG_H);
    std::cout << "Pixel mean RGB: (" 
              << totalR/np << ", " << totalG/np << ", " << totalB/np << ")" << std::endl;

    return 0;
}
