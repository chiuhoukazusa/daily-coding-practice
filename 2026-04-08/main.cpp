/**
 * Particle System Simulation
 * 2026-04-08 Daily Coding Practice
 *
 * 模拟烟花爆炸粒子系统：
 * - 多次爆炸，每次产生 N 个粒子
 * - 粒子受重力、阻力影响
 * - 颜色随寿命衰减（发光→暗淡）
 * - 将若干帧合成一张图（运动轨迹）
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <string>

// ─── Image ─────────────────────────────────────────────────────────────────

struct Color {
    float r, g, b;
    Color(float r = 0, float g = 0, float b = 0) : r(r), g(g), b(b) {}
    Color operator+(const Color& o) const { return {r + o.r, g + o.g, b + o.b}; }
    Color operator*(float t)         const { return {r * t, g * t, b * t}; }
    Color& operator+=(const Color& o) { r += o.r; g += o.g; b += o.b; return *this; }
};

struct Image {
    int width, height;
    std::vector<Color> pixels;

    Image(int w, int h) : width(w), height(h), pixels(w * h, Color(0, 0, 0)) {}

    void addPixel(int x, int y, const Color& c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        pixels[y * width + x] += c;
    }

    // Additive blend with radius
    void splat(float fx, float fy, const Color& c, float radius = 1.5f) {
        int x0 = static_cast<int>(fx - radius);
        int x1 = static_cast<int>(fx + radius) + 1;
        int y0 = static_cast<int>(fy - radius);
        int y1 = static_cast<int>(fy + radius) + 1;
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float dx = fx - x, dy = fy - y;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < radius) {
                    float falloff = 1.0f - dist / radius;
                    addPixel(x, y, c * (falloff * falloff));
                }
            }
        }
    }

    void savePNG(const std::string& filename) const;
};

// ─── PNG writer (minimal, no zlib compression — store filter) ──────────────

static void writeU32BE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back((v >> 24) & 0xFF);
    buf.push_back((v >> 16) & 0xFF);
    buf.push_back((v >>  8) & 0xFF);
    buf.push_back((v      ) & 0xFF);
}

static uint32_t crc32(const uint8_t* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// zlib non-compressed (DEFLATE store) wrapper
static std::vector<uint8_t> zlibStore(const std::vector<uint8_t>& raw) {
    // zlib header
    std::vector<uint8_t> out;
    out.push_back(0x78); // CMF: deflate, window 32768
    out.push_back(0x01); // FLG: no dict, check bits

    size_t pos = 0;
    size_t remaining = raw.size();
    while (remaining > 0) {
        size_t block = std::min(remaining, (size_t)65535);
        bool last = (block == remaining);
        out.push_back(last ? 0x01 : 0x00); // BFINAL + BTYPE=00 (no compression)
        uint16_t len16 = static_cast<uint16_t>(block);
        uint16_t nlen  = ~len16;
        out.push_back( len16       & 0xFF);
        out.push_back((len16 >> 8) & 0xFF);
        out.push_back( nlen        & 0xFF);
        out.push_back((nlen  >> 8) & 0xFF);
        for (size_t i = 0; i < block; ++i) out.push_back(raw[pos + i]);
        pos += block;
        remaining -= block;
    }

    // Adler-32
    uint32_t s1 = 1, s2 = 0;
    for (uint8_t b : raw) { s1 = (s1 + b) % 65521; s2 = (s2 + s1) % 65521; }
    uint32_t adler = (s2 << 16) | s1;
    out.push_back((adler >> 24) & 0xFF);
    out.push_back((adler >> 16) & 0xFF);
    out.push_back((adler >>  8) & 0xFF);
    out.push_back( adler        & 0xFF);
    return out;
}

void Image::savePNG(const std::string& filename) const {
    // Tone-map: clamp with exposure
    auto tonemapChannel = [](float v) -> uint8_t {
        // Reinhard
        v = v / (1.0f + v);
        v = std::max(0.0f, std::min(1.0f, v));
        // Gamma 2.2
        v = std::pow(v, 1.0f / 2.2f);
        return static_cast<uint8_t>(v * 255.0f + 0.5f);
    };

    // Build raw scanlines (filter byte 0 = None before each row)
    std::vector<uint8_t> raw;
    raw.reserve((1 + width * 3) * height);
    for (int y = 0; y < height; ++y) {
        raw.push_back(0); // filter type None
        for (int x = 0; x < width; ++x) {
            const Color& c = pixels[y * width + x];
            raw.push_back(tonemapChannel(c.r));
            raw.push_back(tonemapChannel(c.g));
            raw.push_back(tonemapChannel(c.b));
        }
    }

    std::vector<uint8_t> idat = zlibStore(raw);

    std::ofstream f(filename, std::ios::binary);
    // PNG signature
    const uint8_t sig[] = {137, 80, 78, 71, 13, 10, 26, 10};
    f.write(reinterpret_cast<const char*>(sig), 8);

    auto writeChunk = [&](const char* type, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> buf;
        writeU32BE(buf, static_cast<uint32_t>(data.size()));
        f.write(reinterpret_cast<const char*>(buf.data()), 4);
        f.write(type, 4);
        if (!data.empty()) f.write(reinterpret_cast<const char*>(data.data()), data.size());
        // CRC over type + data
        std::vector<uint8_t> crcBuf;
        crcBuf.insert(crcBuf.end(), type, type + 4);
        crcBuf.insert(crcBuf.end(), data.begin(), data.end());
        buf.clear();
        writeU32BE(buf, crc32(crcBuf.data(), crcBuf.size()));
        f.write(reinterpret_cast<const char*>(buf.data()), 4);
    };

    // IHDR
    std::vector<uint8_t> ihdr;
    writeU32BE(ihdr, width);
    writeU32BE(ihdr, height);
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(2);  // color type RGB
    ihdr.push_back(0);  // compression
    ihdr.push_back(0);  // filter
    ihdr.push_back(0);  // interlace
    writeChunk("IHDR", ihdr);

    // IDAT
    writeChunk("IDAT", idat);

    // IEND
    writeChunk("IEND", {});
}

// ─── Physics ────────────────────────────────────────────────────────────────

struct Vec2 { float x, y; };

struct Particle {
    Vec2  pos;
    Vec2  vel;
    Color color;
    float life;      // 0..1, decreases each step
    float decay;     // how fast life decreases
    float size;
};

// ─── Simulation ─────────────────────────────────────────────────────────────

static std::mt19937 rng(42);

static float randf(float lo, float hi) {
    return lo + (hi - lo) * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
}

static Color hsvToRgb(float h, float s, float v) {
    h = fmod(h, 360.0f);
    int hi = static_cast<int>(h / 60.0f) % 6;
    float f = h / 60.0f - std::floor(h / 60.0f);
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);
    switch (hi) {
        case 0: return {v, t, p};
        case 1: return {q, v, p};
        case 2: return {p, v, t};
        case 3: return {p, q, v};
        case 4: return {t, p, v};
        default: return {v, p, q};
    }
}

struct Explosion {
    Vec2  center;
    int   numParticles;
    float hue;        // base hue 0..360
    float speed;      // peak speed
    float gravity;    // downward pull per step
    float drag;       // velocity multiplier per step
};

static std::vector<Particle> spawnExplosion(const Explosion& e) {
    std::vector<Particle> ps;
    ps.reserve(e.numParticles);

    for (int i = 0; i < e.numParticles; ++i) {
        float angle = randf(0.0f, 2.0f * 3.14159265f);
        float speed = randf(0.3f, 1.0f) * e.speed;
        // some particles get trails
        float decay = randf(0.008f, 0.018f);
        // vary hue slightly
        float hue = e.hue + randf(-20.0f, 20.0f);
        float sat = randf(0.7f, 1.0f);
        float val = randf(1.5f, 3.5f); // HDR brightness

        Particle p;
        p.pos   = e.center;
        p.vel   = {std::cos(angle) * speed, std::sin(angle) * speed};
        p.color = hsvToRgb(hue, sat, val);
        p.life  = 1.0f;
        p.decay = decay;
        p.size  = randf(1.0f, 2.5f);
        ps.push_back(p);
    }
    return ps;
}

// Simulate particles for `steps` with given dt, rendering onto image every `renderEvery` steps
static void simulate(std::vector<Particle>& particles,
                     Image& img,
                     int steps, float dt, int renderEvery,
                     float gravity, float drag)
{
    for (int step = 0; step < steps; ++step) {
        // Render
        if (step % renderEvery == 0) {
            for (const auto& p : particles) {
                if (p.life <= 0) continue;
                // Fade color with life
                Color c = p.color * (p.life * p.life);
                img.splat(p.pos.x, p.pos.y, c, p.size * p.life);
            }
        }
        // Step
        for (auto& p : particles) {
            if (p.life <= 0) continue;
            p.vel.y += gravity * dt;
            p.vel.x *= drag;
            p.vel.y *= drag;
            p.pos.x += p.vel.x * dt;
            p.pos.y += p.vel.y * dt;
            p.life  -= p.decay;
        }
    }
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    const int W = 800, H = 600;
    Image img(W, H);

    // Dark background
    // (black by default)

    // Define 5 explosions
    std::vector<Explosion> explosions = {
        { {400, 280}, 600, 30.0f,   4.5f, 0.03f, 0.985f },  // orange center
        { {200, 200}, 500, 200.0f,  4.0f, 0.03f, 0.985f },  // yellow-green left
        { {620, 220}, 500, 280.0f,  4.2f, 0.03f, 0.984f },  // cyan right
        { {150, 400}, 400, 0.0f,    3.8f, 0.03f, 0.987f },  // red low-left
        { {650, 380}, 450, 130.0f,  3.5f, 0.03f, 0.986f },  // green low-right
    };

    // Time-stagger the explosions slightly to avoid overlap clutter
    for (size_t i = 0; i < explosions.size(); ++i) {
        const Explosion& e = explosions[i];
        auto particles = spawnExplosion(e);

        // Each explosion simulates for 200 steps, rendering every 4 steps
        simulate(particles, img, 200, 1.0f, 4, e.gravity, e.drag);
    }

    // Add some "launch trails" — thin streaks going up into each burst center
    // Simulate rising spark before each explosion
    for (size_t i = 0; i < explosions.size(); ++i) {
        const Explosion& e = explosions[i];
        // Rising spark from bottom
        float sx = e.center.x + randf(-10, 10);
        float sy = static_cast<float>(H - 1);
        float tx = e.center.x, ty = e.center.y;
        int trailSteps = 40;
        for (int s = 0; s < trailSteps; ++s) {
            float t = s / static_cast<float>(trailSteps - 1);
            float x = sx + (tx - sx) * t;
            float y = sy + (ty - sy) * t;
            float brightness = 0.3f + 0.7f * t;
            Color c = hsvToRgb(e.hue, 0.5f, brightness * 1.5f);
            img.splat(x, y, c * 0.5f, 1.0f);
        }
    }

    const std::string outFile = "particle_output.png";
    img.savePNG(outFile);
    std::cout << "Saved: " << outFile << "\n";
    std::cout << "Image: " << W << "x" << H << " | Explosions: " << explosions.size() << "\n";
    return 0;
}
