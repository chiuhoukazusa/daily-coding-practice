#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

// ============================================================
// Atmosphere Scattering Renderer
// Implements single-scattering Rayleigh + Mie atmosphere model
// Renders a sky panorama at multiple sun elevations (sunrise/noon/sunset)
// ============================================================

const int WIDTH  = 512;
const int HEIGHT = 512;

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 0 ? *this * (1.f/l) : Vec3(0,0,0); }
};

// ---- Atmosphere parameters ----
const float EARTH_RADIUS    = 6360e3f;   // m
const float ATMOS_RADIUS    = 6420e3f;   // m
const float RAYLEIGH_HEIGHT = 7994.f;    // scale height (m)
const float MIE_HEIGHT      = 1200.f;    // scale height (m)
const float MIE_G           = 0.76f;     // Mie asymmetry factor

// Rayleigh scattering coefficients (wavelength-dependent, per meter)
const Vec3 BETA_R = Vec3(5.8e-6f, 13.5e-6f, 33.1e-6f); // R, G, B
// Mie scattering coefficient (wavelength-independent approximation)
const float BETA_M = 21e-6f;

// Number of integration samples
const int NUM_SAMPLES     = 16;
const int NUM_LIGHT_SAMPS = 8;

// ---- Utility ----
float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Intersect ray with sphere centered at origin, radius r
// Returns (t_min, t_max), or (-1, -1) if no hit
bool raySphereIntersect(Vec3 orig, Vec3 dir, float r, float& t0, float& t1) {
    float a = dir.dot(dir);
    float b = 2.f * orig.dot(dir);
    float c = orig.dot(orig) - r * r;
    float disc = b*b - 4*a*c;
    if (disc < 0) { t0 = t1 = -1; return false; }
    float sq = sqrtf(disc);
    t0 = (-b - sq) / (2*a);
    t1 = (-b + sq) / (2*a);
    return true;
}

// Rayleigh phase function
float phaseRayleigh(float cosTheta) {
    return 3.f / (16.f * (float)M_PI) * (1.f + cosTheta * cosTheta);
}

// Mie phase function (Henyey-Greenstein)
float phaseMie(float cosTheta, float g) {
    float g2 = g * g;
    float denom = 1.f + g2 - 2.f * g * cosTheta;
    return 3.f / (8.f * (float)M_PI) *
           (1.f - g2) * (1.f + cosTheta * cosTheta) /
           ((2.f + g2) * powf(denom, 1.5f));
}

// Optical depth along a ray from position p in direction dir, up to sphere atmos
float opticalDepthRayleigh(Vec3 p, Vec3 dir) {
    float t0, t1;
    raySphereIntersect(p, dir, ATMOS_RADIUS, t0, t1);
    float tmax = t1 > 0 ? t1 : 0;
    float tmin = t0 > 0 ? t0 : 0;
    float len  = tmax - tmin;
    float seg  = len / NUM_LIGHT_SAMPS;
    float sum  = 0;
    for (int i = 0; i < NUM_LIGHT_SAMPS; ++i) {
        Vec3 pos = p + dir * (tmin + (i + 0.5f) * seg);
        float h  = pos.length() - EARTH_RADIUS;
        if (h < 0) h = 0;
        sum += expf(-h / RAYLEIGH_HEIGHT) * seg;
    }
    return sum;
}

float opticalDepthMie(Vec3 p, Vec3 dir) {
    float t0, t1;
    raySphereIntersect(p, dir, ATMOS_RADIUS, t0, t1);
    float tmax = t1 > 0 ? t1 : 0;
    float tmin = t0 > 0 ? t0 : 0;
    float len  = tmax - tmin;
    float seg  = len / NUM_LIGHT_SAMPS;
    float sum  = 0;
    for (int i = 0; i < NUM_LIGHT_SAMPS; ++i) {
        Vec3 pos = p + dir * (tmin + (i + 0.5f) * seg);
        float h  = pos.length() - EARTH_RADIUS;
        if (h < 0) h = 0;
        sum += expf(-h / MIE_HEIGHT) * seg;
    }
    return sum;
}

// Main atmosphere integration: compute sky color in direction `rayDir` with sun at `sunDir`
Vec3 atmosphereColor(Vec3 rayDir, Vec3 sunDir) {
    // Observer on Earth's surface (just above ground)
    Vec3 orig = Vec3(0, EARTH_RADIUS + 1.f, 0);

    // Find atmosphere entry/exit
    float t0, t1;
    if (!raySphereIntersect(orig, rayDir, ATMOS_RADIUS, t0, t1))
        return Vec3(0, 0, 0);

    // If ray starts inside atmosphere (which it does), t_start = max(0, t0)
    float tStart = std::max(0.f, t0);
    float tEnd   = t1;

    // Check if ray hits Earth
    float te0, te1;
    if (raySphereIntersect(orig, rayDir, EARTH_RADIUS, te0, te1) && te1 > 0 && te0 < tEnd) {
        tEnd = std::max(0.f, te0);
    }

    float segLen = (tEnd - tStart) / NUM_SAMPLES;

    Vec3 sumR = Vec3(0, 0, 0);
    Vec3 sumM = Vec3(0, 0, 0);
    float optR = 0, optM = 0;  // accumulated optical depth from viewer

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        float t   = tStart + (i + 0.5f) * segLen;
        Vec3  pos = orig + rayDir * t;
        float h   = pos.length() - EARTH_RADIUS;
        if (h < 0) h = 0;

        float dR = expf(-h / RAYLEIGH_HEIGHT) * segLen;
        float dM = expf(-h / MIE_HEIGHT)      * segLen;
        optR += dR;
        optM += dM;

        // Check if sun is visible from this sample point (above horizon)
        float ls0, ls1;
        if (raySphereIntersect(pos, sunDir, EARTH_RADIUS, ls0, ls1) && ls1 > 0) {
            // Sun blocked by Earth
            continue;
        }

        // Optical depth toward sun
        float lightOptR = opticalDepthRayleigh(pos, sunDir);
        float lightOptM = opticalDepthMie(pos, sunDir);

        // Extinction (transmittance) from viewer to sample, sample to sun
        Vec3 tau_r = BETA_R * (optR + lightOptR);
        Vec3 tau_m = Vec3(BETA_M, BETA_M, BETA_M) * 1.1f * (optM + lightOptM);
        Vec3 tau   = tau_r + tau_m;
        Vec3 transmit = Vec3(expf(-tau.x), expf(-tau.y), expf(-tau.z));

        sumR += transmit * dR;
        sumM += transmit * dM;
    }

    float cosTheta = rayDir.dot(sunDir);
    Vec3 colorR = sumR * BETA_R * phaseRayleigh(cosTheta);
    Vec3 colorM = sumM * Vec3(BETA_M, BETA_M, BETA_M) * phaseMie(cosTheta, MIE_G);

    // Sun intensity
    const float SUN_INTENSITY = 22.f;
    return (colorR + colorM) * SUN_INTENSITY;
}

// Tone mapping: ACES approximate
Vec3 acesToneMap(Vec3 c) {
    float a = 2.51f, b = 0.03f, cs = 2.43f, d = 0.59f, e = 0.14f;
    float r = (c.x*(a*c.x+b))/(c.x*(cs*c.x+d)+e);
    float g = (c.y*(a*c.y+b))/(c.y*(cs*c.y+d)+e);
    float bv = (c.z*(a*c.z+b))/(c.z*(cs*c.z+d)+e);
    return Vec3(clamp(r,0,1), clamp(g,0,1), clamp(bv,0,1));
}

// ---- PPM writer ----
void writePPM(const char* filename, const std::vector<unsigned char>& pixels, int w, int h) {
    FILE* f = fopen(filename, "wb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); return; }
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(pixels.data(), 1, w * h * 3, f);
    fclose(f);
    printf("Saved: %s (%d x %d)\n", filename, w, h);
}

// Render sky panorama for given sun elevation angle (degrees above horizon)
// Output: HSplit canvas with [dawn, noon, sunset] side by side (3 * WIDTH x HEIGHT)
void renderPanel(std::vector<unsigned char>& canvas, int panelX, int panelWidth, int panelHeight,
                 float sunElevDeg) {
    float sunElev = sunElevDeg * (float)M_PI / 180.f;
    Vec3 sunDir   = Vec3(0, sinf(sunElev), cosf(sunElev)).normalized();

    for (int py = 0; py < panelHeight; ++py) {
        for (int px = 0; px < panelWidth; ++px) {
            // Map pixel to sky direction
            // x -> azimuth [-60, +60] deg, y -> elevation [0, +80] deg
            float az  = ((float)px / panelWidth  - 0.5f) * 120.f * (float)M_PI / 180.f;
            float el  = ((float)(panelHeight - 1 - py) / panelHeight) * 80.f * (float)M_PI / 180.f;

            Vec3 rayDir = Vec3(sinf(az) * cosf(el), sinf(el), cosf(az) * cosf(el)).normalized();

            Vec3 color = atmosphereColor(rayDir, sunDir);
            color      = acesToneMap(color);

            int idx = (py * (3 * panelWidth) + panelX * panelWidth + px) * 3;
            canvas[idx + 0] = (unsigned char)(color.x * 255.f + 0.5f);
            canvas[idx + 1] = (unsigned char)(color.y * 255.f + 0.5f);
            canvas[idx + 2] = (unsigned char)(color.z * 255.f + 0.5f);
        }
    }
}

// ---- PNG writer (minimal, using only stdlib) - actually write PPM then convert ----
int main() {
    const int PANELS = 3;
    const int TOTAL_W = WIDTH * PANELS;
    const int TOTAL_H = HEIGHT;

    std::vector<unsigned char> canvas(TOTAL_W * TOTAL_H * 3, 0);

    // Panel 0: Dawn / Sunrise (sun at 5° above horizon)
    printf("Rendering panel 0: Dawn (sun 5 deg)...\n");
    renderPanel(canvas, 0, WIDTH, HEIGHT, 5.f);

    // Panel 1: Midday (sun at 60°)
    printf("Rendering panel 1: Noon (sun 60 deg)...\n");
    renderPanel(canvas, 1, WIDTH, HEIGHT, 60.f);

    // Panel 2: Sunset (sun at -2° below horizon, still lit atmosphere)
    printf("Rendering panel 2: Sunset (sun -2 deg)...\n");
    renderPanel(canvas, 2, WIDTH, HEIGHT, -2.f);

    // Save as PPM
    writePPM("atmosphere_output.ppm", canvas, TOTAL_W, TOTAL_H);

    // Convert to PNG with ImageMagick
    int ret = system("convert atmosphere_output.ppm atmosphere_output.png 2>&1 && echo 'PNG saved' || echo 'convert failed'");
    (void)ret;

    printf("Done! Atmosphere Scattering Renderer complete.\n");
    return 0;
}
