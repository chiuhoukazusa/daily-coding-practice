/*
 * SPH Fluid Simulation
 * Date: 2026-04-09
 *
 * Implements Smoothed Particle Hydrodynamics (SPH) for 2D fluid simulation.
 * Based on Müller et al. "Particle-Based Fluid Simulation for Interactive Applications" (2003)
 *
 * Features:
 *   - SPH density estimation with poly6 kernel
 *   - Pressure force with spiky kernel gradient
 *   - Viscosity force with viscosity kernel Laplacian
 *   - Gravity + elastic boundary forces
 *   - Semi-implicit Euler integration with velocity clamping
 *   - Gaussian splat rendering per particle
 *   - Color by speed: blue(slow) → cyan → green → yellow → red(fast)
 *   - 6 frames captured at different simulation steps
 *
 * Output: sph_output.png (1200x200 strip, 6 frames side by side)
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <zlib.h>

// ─────────────────────────────────────────────
// PNG writer
// ─────────────────────────────────────────────

static uint32_t crc_table[256];
static bool crc_ready = false;

static void init_crc() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc_table[i] = c;
    }
    crc_ready = true;
}

static uint32_t calc_crc(const uint8_t* d, size_t n) {
    if (!crc_ready) init_crc();
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) c = crc_table[(c ^ d[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static void pu32(std::vector<uint8_t>& o, uint32_t v) {
    o.push_back((v>>24)&0xFF); o.push_back((v>>16)&0xFF);
    o.push_back((v>>8)&0xFF);  o.push_back(v&0xFF);
}

static void pchunk(std::vector<uint8_t>& out, const char t[4],
                   const uint8_t* d, size_t n)
{
    pu32(out, (uint32_t)n);
    size_t base = out.size();
    for (int i = 0; i < 4; i++) out.push_back((uint8_t)t[i]);
    for (size_t i = 0; i < n; i++) out.push_back(d[i]);
    pu32(out, calc_crc(out.data() + base, 4 + n));
}

static void save_png(const std::string& path, int w, int h,
                     const std::vector<uint8_t>& rgb)
{
    std::vector<uint8_t> raw;
    raw.reserve((size_t)(w * 3 + 1) * h);
    for (int y = 0; y < h; y++) {
        raw.push_back(0);
        for (int x = 0; x < w; x++) {
            int i = (y * w + x) * 3;
            raw.push_back(rgb[i]); raw.push_back(rgb[i+1]); raw.push_back(rgb[i+2]);
        }
    }
    uLongf bound = compressBound((uLong)raw.size());
    std::vector<uint8_t> comp(bound);
    compress2(comp.data(), &bound, raw.data(), (uLong)raw.size(), Z_BEST_SPEED);
    comp.resize(bound);

    std::vector<uint8_t> png;
    const uint8_t sig[] = {137,80,78,71,13,10,26,10};
    for (auto b : sig) png.push_back(b);

    uint8_t ihdr[13];
    ihdr[0]=(w>>24)&0xFF; ihdr[1]=(w>>16)&0xFF; ihdr[2]=(w>>8)&0xFF; ihdr[3]=w&0xFF;
    ihdr[4]=(h>>24)&0xFF; ihdr[5]=(h>>16)&0xFF; ihdr[6]=(h>>8)&0xFF; ihdr[7]=h&0xFF;
    ihdr[8]=8; ihdr[9]=2; ihdr[10]=ihdr[11]=ihdr[12]=0;
    pchunk(png, "IHDR", ihdr, 13);
    pchunk(png, "IDAT", comp.data(), comp.size());
    pchunk(png, "IEND", nullptr, 0);

    std::ofstream f(path, std::ios::binary);
    f.write((char*)png.data(), (std::streamsize)png.size());
}

// ─────────────────────────────────────────────
// SPH Parameters (tuned for stability)
// ─────────────────────────────────────────────

// Smoothing radius in simulation units
static const float H         = 12.0f;
static const float H2        = H * H;
static const float MASS      = 65.0f;   // particle mass
static const float REST_DENS = 1000.0f; // target rest density
static const float K_GAS     = 8.0f;   // pressure stiffness (lower = more stable)
static const float MU_VISC   = 6.0f;   // dynamic viscosity
static const float DT        = 0.003f; // time step (small for stability)
static const float GRAVITY   = 200.0f; // gravitational acceleration (sim units/s^2)
static const float DAMP      = 0.3f;   // boundary velocity damping
static const float V_MAX     = 500.0f; // velocity clamp for stability

// SPH kernel normalisation constants
static const float PI_F       = 3.14159265358979f;
static const float POLY6_C    = 4.0f   / (PI_F * std::pow(H, 8.0f));
static const float SPIKY_C    = -10.0f / (PI_F * std::pow(H, 5.0f));
static const float VISC_C     =  40.0f / (PI_F * std::pow(H, 4.0f));

// ─────────────────────────────────────────────
// Particle
// ─────────────────────────────────────────────

struct Particle {
    float x, y;
    float vx, vy;
    float fx, fy;
    float rho;   // density
    float press; // pressure
};

// ─────────────────────────────────────────────
// SPH solver step
// ─────────────────────────────────────────────

static void sph_density(std::vector<Particle>& ps) {
    for (auto& pi : ps) {
        pi.rho = 0.0f;
        for (const auto& pj : ps) {
            float dx = pj.x - pi.x;
            float dy = pj.y - pi.y;
            float r2 = dx*dx + dy*dy;
            if (r2 < H2) {
                float q = H2 - r2;
                pi.rho += MASS * POLY6_C * q * q * q;
            }
        }
        pi.rho = std::max(pi.rho, 0.001f);
        // Equation of state: tait/ideal gas
        pi.press = K_GAS * (pi.rho - REST_DENS);
    }
}

static void sph_forces(std::vector<Particle>& ps) {
    for (size_t i = 0; i < ps.size(); i++) {
        float ax = 0, ay = 0;

        for (size_t j = 0; j < ps.size(); j++) {
            if (i == j) continue;
            float dx = ps[j].x - ps[i].x;
            float dy = ps[j].y - ps[i].y;
            float r2 = dx*dx + dy*dy;
            if (r2 < H2 && r2 > 0.01f) {
                float r   = std::sqrt(r2);
                float q   = H - r;

                // Pressure force (symmetric)
                float fp  = -MASS * (ps[i].press + ps[j].press)
                            / (2.0f * ps[j].rho)
                            * SPIKY_C * q * q / r;
                ax += fp * dx;
                ay += fp * dy;

                // Viscosity force
                float fv = MU_VISC * MASS / ps[j].rho
                           * VISC_C * q;
                ax += fv * (ps[j].vx - ps[i].vx);
                ay += fv * (ps[j].vy - ps[i].vy);
            }
        }

        // Gravity (downward = +y)
        ay += GRAVITY;

        ps[i].fx = ax * ps[i].rho;  // store as actual force (density-scaled)
        ps[i].fy = ay * ps[i].rho;
    }
}

static void sph_integrate(std::vector<Particle>& ps, float dom_w, float dom_h) {
    for (auto& p : ps) {
        float ax = p.fx / p.rho;
        float ay = p.fy / p.rho;

        p.vx += DT * ax;
        p.vy += DT * ay;

        // Clamp velocity for stability
        float spd = std::sqrt(p.vx*p.vx + p.vy*p.vy);
        if (spd > V_MAX) {
            float scale = V_MAX / spd;
            p.vx *= scale;
            p.vy *= scale;
        }

        p.x += DT * p.vx;
        p.y += DT * p.vy;

        // Boundary: elastic reflection
        const float R = 1.5f;
        if (p.x < R)         { p.x = R;         p.vx = std::abs(p.vx) * DAMP; }
        if (p.x > dom_w - R) { p.x = dom_w - R; p.vx = -std::abs(p.vx) * DAMP; }
        if (p.y < R)         { p.y = R;         p.vy = std::abs(p.vy) * DAMP; }
        if (p.y > dom_h - R) { p.y = dom_h - R; p.vy = -std::abs(p.vy) * DAMP; }
    }
}

// ─────────────────────────────────────────────
// Rendering
// ─────────────────────────────────────────────

static void speed_to_color(float speed, float max_speed,
                            uint8_t& r, uint8_t& g, uint8_t& b)
{
    float t = std::min(1.0f, speed / (max_speed + 1e-6f));
    float rf=0,gf=0,bf=0;
    if      (t < 0.25f) { float s=t/0.25f;         rf=0;   gf=s;   bf=1.f; }
    else if (t < 0.50f) { float s=(t-0.25f)/0.25f; rf=0;   gf=1.f; bf=1.f-s; }
    else if (t < 0.75f) { float s=(t-0.50f)/0.25f; rf=s;   gf=1.f; bf=0; }
    else                { float s=(t-0.75f)/0.25f; rf=1.f; gf=1.f-s; bf=0; }
    r = (uint8_t)(rf*255); g = (uint8_t)(gf*255); b = (uint8_t)(bf*255);
}

static void render_frame(const std::vector<Particle>& ps,
                          std::vector<uint8_t>& canvas,
                          int img_w, int img_h,
                          int frame_ox, int frame_w,
                          float max_speed)
{
    // Background: deep navy
    for (int y = 0; y < img_h; y++) {
        for (int x = frame_ox; x < frame_ox + frame_w; x++) {
            int i = (y * img_w + x) * 3;
            canvas[i+0] = 10; canvas[i+1] = 15; canvas[i+2] = 38;
        }
    }

    // Gaussian splat per particle
    const int SR = 6;
    const float sigma2 = (float)(SR*SR) * 0.20f;

    for (const auto& p : ps) {
        float spd = std::sqrt(p.vx*p.vx + p.vy*p.vy);
        uint8_t cr, cg, cb;
        speed_to_color(spd, max_speed, cr, cg, cb);

        int px = (int)std::round(p.x) + frame_ox;
        int py = (int)std::round(p.y);

        for (int dy = -SR; dy <= SR; dy++) {
            for (int dx = -SR; dx <= SR; dx++) {
                float d2 = (float)(dx*dx + dy*dy);
                float alpha = std::exp(-d2 / sigma2);
                if (alpha < 0.01f) continue;

                int nx = px + dx, ny = py + dy;
                if (nx < frame_ox || nx >= frame_ox + frame_w) continue;
                if (ny < 0 || ny >= img_h) continue;

                int i = (ny * img_w + nx) * 3;
                canvas[i+0] = (uint8_t)std::min(255, (int)canvas[i+0] + (int)(cr * alpha));
                canvas[i+1] = (uint8_t)std::min(255, (int)canvas[i+1] + (int)(cg * alpha));
                canvas[i+2] = (uint8_t)std::min(255, (int)canvas[i+2] + (int)(cb * alpha));
            }
        }
    }

    // Frame separator
    for (int y = 0; y < img_h; y++) {
        int i = (y * img_w + frame_ox) * 3;
        canvas[i+0] = canvas[i+1] = canvas[i+2] = 50;
    }
}

// ─────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────

int main() {
    printf("=== SPH Fluid Simulation (Dam-Break) ===\n");

    // Simulation domain (pixels)
    const float DOM_W = 200.0f;
    const float DOM_H = 200.0f;

    // Output: 6 frames × 200px = 1200×200
    const int FRAME_W  = 200;
    const int FRAME_H  = 200;
    const int N_FRAMES = 6;
    const int IMG_W    = FRAME_W * N_FRAMES;
    const int IMG_H    = FRAME_H;

    // Initialize: fluid block in upper-left (dam-break)
    std::vector<Particle> ps;
    ps.reserve(200);

    const float SPACING = H * 0.7f;
    const float OX = 15.0f;
    const float OY = 5.0f;
    const int   NX = 12;
    const int   NY = 14;

    for (int ix = 0; ix < NX; ix++) {
        for (int iy = 0; iy < NY; iy++) {
            Particle p;
            // Offset alternate rows for denser packing
            p.x  = OX + ix * SPACING + (iy % 2 == 0 ? 0.f : SPACING * 0.5f);
            p.y  = OY + iy * SPACING;
            p.vx = 0.f;
            p.vy = 0.f;
            p.fx = p.fy = 0.f;
            p.rho   = REST_DENS;
            p.press = 0.f;
            if (p.x < DOM_W - 2.f && p.y < DOM_H - 2.f)
                ps.push_back(p);
        }
    }

    printf("Initialized %zu particles\n", ps.size());

    // Simulation: 1200 steps total, capture 6 frames
    const int TOTAL_STEPS = 1200;
    int caps[N_FRAMES] = {0, 120, 280, 500, 800, 1200};

    std::vector<uint8_t> canvas((size_t)(IMG_W * IMG_H * 3), 0);
    const float MAX_SPD = 200.0f; // tuned to simulation speed range

    int ci = 0;
    for (int step = 0; step <= TOTAL_STEPS; step++) {
        if (ci < N_FRAMES && step == caps[ci]) {
            float maxspd = 0;
            for (const auto& p : ps)
                maxspd = std::max(maxspd, std::sqrt(p.vx*p.vx + p.vy*p.vy));

            render_frame(ps, canvas, IMG_W, IMG_H, ci * FRAME_W, FRAME_W, MAX_SPD);
            printf("  Frame %d @ step %4d | particles=%zu | max_speed=%.1f\n",
                   ci, step, ps.size(), maxspd);
            ci++;
        }
        if (step == TOTAL_STEPS) break;

        sph_density(ps);
        sph_forces(ps);
        sph_integrate(ps, DOM_W, DOM_H);
    }

    printf("Saving sph_output.png (%dx%d)...\n", IMG_W, IMG_H);
    save_png("sph_output.png", IMG_W, IMG_H, canvas);
    printf("✅ sph_output.png saved.\n");

    return 0;
}
