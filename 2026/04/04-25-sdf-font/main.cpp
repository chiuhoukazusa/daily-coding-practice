/**
 * SDF Font Rendering
 * 
 * Demonstrates Signed Distance Field (SDF) font rendering technique:
 * - Generate SDF from a high-res bitmap glyph
 * - Render at multiple resolutions without blurriness
 * - Effects: outline, drop shadow, glow, smooth anti-aliasing
 *
 * Output: sdf_font_output.png (800x600, comparison + effects showcase)
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <cassert>
#include <cstring>
#include <limits>
#include <array>

// =====================================================================
// Image
// =====================================================================

struct Color {
    uint8_t r, g, b, a;
    Color(uint8_t r=0, uint8_t g=0, uint8_t b=0, uint8_t a=255)
        : r(r), g(g), b(b), a(a) {}
};

struct Image {
    int width, height;
    std::vector<Color> pixels;

    Image(int w, int h, Color bg = {30, 30, 40})
        : width(w), height(h), pixels(w * h, bg) {}

    Color& at(int x, int y) { return pixels[y * width + x]; }
    const Color& at(int x, int y) const { return pixels[y * width + x]; }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    void setPixel(int x, int y, const Color& c) {
        if (inBounds(x, y)) at(x, y) = c;
    }

    Color blend(const Color& dst, const Color& src) {
        float sa = src.a / 255.0f;
        float da = dst.a / 255.0f;
        float oa = sa + da * (1.0f - sa);
        if (oa < 1e-6f) return {0,0,0,0};
        auto ch = [&](uint8_t sc, uint8_t dc) -> uint8_t {
            return (uint8_t)std::clamp((sc * sa + dc * da * (1.0f - sa)) / oa * 1.0f, 0.0f, 255.0f);
        };
        return {ch(src.r, dst.r), ch(src.g, dst.g), ch(src.b, dst.b), (uint8_t)(oa * 255)};
    }

    void blendPixel(int x, int y, const Color& c) {
        if (!inBounds(x, y)) return;
        at(x, y) = blend(at(x, y), c);
    }

    bool savePNG(const std::string& filename) const;
};

// =====================================================================
// Minimal PNG writer (no dependencies)
// =====================================================================

static uint32_t crc_table[256];
static bool crc_init = false;

static void init_crc() {
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
    crc_init = true;
}

static uint32_t update_crc(uint32_t crc, const uint8_t* data, size_t len) {
    if (!crc_init) init_crc();
    for (size_t i = 0; i < len; i++)
        crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

static uint32_t crc32(const uint8_t* data, size_t len) {
    return update_crc(0xFFFFFFFFu, data, len) ^ 0xFFFFFFFFu;
}

// Deflate using stored blocks (no compression, valid PNG)
static std::vector<uint8_t> deflate_store(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> out;
    // zlib header
    out.push_back(0x78); out.push_back(0x01);
    
    size_t offset = 0;
    size_t remaining = data.size();
    
    // Adler-32 checksum
    uint32_t s1 = 1, s2 = 0;
    for (uint8_t b : data) { s1 = (s1 + b) % 65521; s2 = (s2 + s1) % 65521; }
    
    while (remaining > 0) {
        size_t block_size = std::min(remaining, (size_t)65535);
        bool last = (block_size == remaining);
        out.push_back(last ? 1 : 0); // BFINAL, BTYPE=00
        out.push_back(block_size & 0xFF);
        out.push_back((block_size >> 8) & 0xFF);
        out.push_back(~block_size & 0xFF);
        out.push_back((~block_size >> 8) & 0xFF);
        for (size_t i = 0; i < block_size; i++)
            out.push_back(data[offset + i]);
        offset += block_size;
        remaining -= block_size;
    }
    
    // Adler-32
    out.push_back((s2 >> 8) & 0xFF);
    out.push_back(s2 & 0xFF);
    out.push_back((s1 >> 8) & 0xFF);
    out.push_back(s1 & 0xFF);
    return out;
}

static void write_uint32_be(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back((v >> 24) & 0xFF);
    buf.push_back((v >> 16) & 0xFF);
    buf.push_back((v >> 8) & 0xFF);
    buf.push_back(v & 0xFF);
}

static void write_chunk(std::vector<uint8_t>& buf, const char* type, const std::vector<uint8_t>& data) {
    write_uint32_be(buf, (uint32_t)data.size());
    buf.push_back(type[0]); buf.push_back(type[1]);
    buf.push_back(type[2]); buf.push_back(type[3]);
    buf.insert(buf.end(), data.begin(), data.end());
    std::vector<uint8_t> crc_data(4 + data.size());
    crc_data[0]=type[0]; crc_data[1]=type[1]; crc_data[2]=type[2]; crc_data[3]=type[3];
    std::copy(data.begin(), data.end(), crc_data.begin() + 4);
    write_uint32_be(buf, crc32(crc_data.data(), crc_data.size()));
}

bool Image::savePNG(const std::string& filename) const {
    std::vector<uint8_t> raw_data;
    raw_data.reserve((width * 4 + 1) * height);
    for (int y = 0; y < height; y++) {
        raw_data.push_back(0); // filter type None
        for (int x = 0; x < width; x++) {
            const Color& c = at(x, y);
            raw_data.push_back(c.r);
            raw_data.push_back(c.g);
            raw_data.push_back(c.b);
            raw_data.push_back(c.a);
        }
    }
    
    auto compressed = deflate_store(raw_data);
    
    std::vector<uint8_t> buf;
    // PNG signature
    const uint8_t sig[] = {137, 80, 78, 71, 13, 10, 26, 10};
    buf.insert(buf.end(), sig, sig + 8);
    
    // IHDR
    std::vector<uint8_t> ihdr(13);
    ihdr[0]=(width>>24)&0xFF; ihdr[1]=(width>>16)&0xFF; ihdr[2]=(width>>8)&0xFF; ihdr[3]=width&0xFF;
    ihdr[4]=(height>>24)&0xFF; ihdr[5]=(height>>16)&0xFF; ihdr[6]=(height>>8)&0xFF; ihdr[7]=height&0xFF;
    ihdr[8]=8; ihdr[9]=6; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0; // 8-bit RGBA
    write_chunk(buf, "IHDR", ihdr);
    write_chunk(buf, "IDAT", compressed);
    write_chunk(buf, "IEND", {});
    
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) return false;
    ofs.write(reinterpret_cast<const char*>(buf.data()), buf.size());
    return ofs.good();
}

// =====================================================================
// SDF Generation
// =====================================================================

// High-resolution bitmap glyph (hand-crafted pixel art for each letter)
// Each glyph is 16x16 pixels, 1=inside, 0=outside

struct Glyph {
    static const int RES = 32;
    bool pixels[RES][RES];

    void clear() { memset(pixels, 0, sizeof(pixels)); }

    // Draw a filled circle
    void circle(float cx, float cy, float r) {
        for (int y = 0; y < RES; y++)
            for (int x = 0; x < RES; x++) {
                float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
                if (dx*dx + dy*dy <= r*r) pixels[y][x] = true;
            }
    }

    // Draw a filled rect
    void rect(int x0, int y0, int x1, int y1) {
        for (int y = std::max(0,y0); y < std::min(RES,y1); y++)
            for (int x = std::max(0,x0); x < std::min(RES,x1); x++)
                pixels[y][x] = true;
    }

    // Subtract a rect (clear)
    void clearRect(int x0, int y0, int x1, int y1) {
        for (int y = std::max(0,y0); y < std::min(RES,y1); y++)
            for (int x = std::max(0,x0); x < std::min(RES,x1); x++)
                pixels[y][x] = false;
    }
    
    // Draw ring (filled circle minus inner circle)
    void ring(float cx, float cy, float r_outer, float r_inner) {
        circle(cx, cy, r_outer);
        // subtract inner
        for (int y = 0; y < RES; y++)
            for (int x = 0; x < RES; x++) {
                float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
                if (dx*dx + dy*dy < r_inner*r_inner) pixels[y][x] = false;
            }
    }

    // Draw a thick diagonal line
    void line(float x0, float y0, float x1, float y1, float thickness) {
        float dx = x1 - x0, dy = y1 - y0;
        float len = sqrtf(dx*dx + dy*dy);
        if (len < 1e-6f) return;
        float nx = -dy/len, ny = dx/len;
        for (int y = 0; y < RES; y++)
            for (int x = 0; x < RES; x++) {
                float px = x + 0.5f - x0, py = y + 0.5f - y0;
                float proj = px * nx + py * ny;  // perpendicular distance
                float along = px * dx/len + py * dy/len;
                if (fabsf(proj) <= thickness && along >= 0 && along <= len)
                    pixels[y][x] = true;
            }
    }
};

// Generate glyphs for uppercase letters S, D, F
static Glyph makeGlyphS() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    // S shape: two arcs connected
    // Top arc (clockwise)
    g.ring(s*0.5f, s*0.28f, s*0.26f, s*0.14f);
    // Bottom arc (counter-clockwise)
    g.ring(s*0.5f, s*0.72f, s*0.26f, s*0.14f);
    // Clear right of top arc
    g.clearRect((int)(s*0.5f), 0, (int)s, (int)(s*0.28f));
    // Clear left of bottom arc
    g.clearRect(0, (int)(s*0.72f), (int)(s*0.5f), (int)s);
    // Connecting bar in middle
    g.rect((int)(s*0.24f), (int)(s*0.43f), (int)(s*0.76f), (int)(s*0.57f));
    // Left of top
    g.rect(0, (int)(s*0.10f), (int)(s*0.26f), (int)(s*0.28f));
    // Right of bottom
    g.rect((int)(s*0.74f), (int)(s*0.72f), (int)s, (int)(s*0.90f));
    return g;
}

static Glyph makeGlyphD() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    // D: vertical left stroke + right arc
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.35f), (int)(s*0.9f));
    // Right semicircle
    g.ring(s*0.35f, s*0.5f, s*0.42f, s*0.26f);
    // Clear left half of ring
    g.clearRect(0, 0, (int)(s*0.35f), (int)s);
    return g;
}

static Glyph makeGlyphF() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    // F: vertical stroke + two horizontal bars
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.35f), (int)(s*0.9f));
    // Top bar
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.85f), (int)(s*0.28f));
    // Middle bar
    g.rect((int)(s*0.15f), (int)(s*0.44f), (int)(s*0.72f), (int)(s*0.57f));
    return g;
}

// Additional letters for demo text
static Glyph makeGlyphR() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.35f), (int)(s*0.9f));
    // Top arc
    g.ring(s*0.47f, s*0.33f, s*0.32f, s*0.16f);
    g.clearRect(0, 0, (int)(s*0.35f), (int)(s*0.55f));
    g.clearRect(0, (int)(s*0.55f), (int)s, (int)s);
    // Reconnect stem
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.35f), (int)(s*0.9f));
    // Leg (diagonal)
    g.line(s*0.47f, s*0.5f, s*0.82f, s*0.88f, s*0.08f);
    return g;
}

static Glyph makeGlyphE() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.35f), (int)(s*0.9f));
    g.rect((int)(s*0.15f), (int)(s*0.1f), (int)(s*0.85f), (int)(s*0.28f));
    g.rect((int)(s*0.15f), (int)(s*0.44f), (int)(s*0.72f), (int)(s*0.57f));
    g.rect((int)(s*0.15f), (int)(s*0.72f), (int)(s*0.85f), (int)(s*0.90f));
    return g;
}

static Glyph makeGlyphN() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    g.rect((int)(s*0.12f), (int)(s*0.1f), (int)(s*0.32f), (int)(s*0.9f));
    g.rect((int)(s*0.68f), (int)(s*0.1f), (int)(s*0.88f), (int)(s*0.9f));
    g.line(s*0.22f, s*0.12f, s*0.78f, s*0.88f, s*0.10f);
    return g;
}

static Glyph makeGlyphI() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    g.rect((int)(s*0.40f), (int)(s*0.1f), (int)(s*0.60f), (int)(s*0.9f));
    g.rect((int)(s*0.18f), (int)(s*0.1f), (int)(s*0.82f), (int)(s*0.27f));
    g.rect((int)(s*0.18f), (int)(s*0.73f), (int)(s*0.82f), (int)(s*0.90f));
    return g;
}

static Glyph makeGlyphG() {
    Glyph g; g.clear();
    float s = Glyph::RES;
    g.ring(s*0.5f, s*0.5f, s*0.42f, s*0.26f);
    g.clearRect((int)(s*0.5f), 0, (int)s, (int)(s*0.52f));
    g.rect((int)(s*0.5f), (int)(s*0.44f), (int)(s*0.78f), (int)(s*0.58f));
    return g;
}

// Compute SDF from binary glyph
// Returns a float array of size [RES x RES], values in [-1, 1]
// positive = inside, negative = outside
std::vector<float> computeSDF(const Glyph& glyph, int sdf_res) {
    int R = Glyph::RES;
    std::vector<float> sdf(sdf_res * sdf_res);
    
    float scale = (float)R / sdf_res;
    float max_dist = (float)R * 0.4f; // normalize to this radius
    
    for (int sy = 0; sy < sdf_res; sy++) {
        for (int sx = 0; sx < sdf_res; sx++) {
            // Sample position in glyph space
            float gx = (sx + 0.5f) * scale;
            float gy = (sy + 0.5f) * scale;
            
            int igx = std::clamp((int)gx, 0, R-1);
            int igy = std::clamp((int)gy, 0, R-1);
            bool inside = glyph.pixels[igy][igx];
            
            // Find nearest border pixel using BFS-like sweep
            float min_dist_sq = max_dist * max_dist;
            
            int search = (int)(max_dist + 1);
            for (int dy = -search; dy <= search; dy++) {
                for (int dx = -search; dx <= search; dx++) {
                    int nx = igx + dx;
                    int ny = igy + dy;
                    if (nx < 0 || nx >= R || ny < 0 || ny >= R) continue;
                    
                    if (glyph.pixels[ny][nx] != inside) {
                        // This is a different-side pixel, compute distance to its border
                        float px = nx + 0.5f, py = ny + 0.5f;
                        float dist_sq = (gx - px)*(gx - px) + (gy - py)*(gy - py);
                        min_dist_sq = std::min(min_dist_sq, dist_sq);
                    }
                }
            }
            
            float dist = sqrtf(min_dist_sq) / max_dist;
            sdf[sy * sdf_res + sx] = inside ? dist : -dist;
        }
    }
    return sdf;
}

// =====================================================================
// SDF Rendering
// =====================================================================

struct SDFRenderParams {
    float smoothing;    // edge smoothing width
    float threshold;    // edge threshold (0.5 = hard edge)
    
    // Optional effects
    bool outline;
    float outlineWidth;
    Color outlineColor;
    
    bool shadow;
    float shadowOffsetX, shadowOffsetY;
    float shadowSoftness;
    Color shadowColor;
    
    bool glow;
    float glowWidth;
    Color glowColor;
    
    Color textColor;
    
    SDFRenderParams() 
        : smoothing(0.05f), threshold(0.5f),
          outline(false), outlineWidth(0.1f), outlineColor{0,0,0,255},
          shadow(false), shadowOffsetX(0.05f), shadowOffsetY(0.05f),
          shadowSoftness(0.15f), shadowColor{0,0,0,180},
          glow(false), glowWidth(0.2f), glowColor{255,200,100,200},
          textColor{255,255,255,255} {}
};

// Sample SDF with bilinear interpolation
float sampleSDF(const std::vector<float>& sdf, int sdf_res, float u, float v) {
    float x = u * sdf_res - 0.5f;
    float y = v * sdf_res - 0.5f;
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    int x1 = x0 + 1, y1 = y0 + 1;
    float fx = x - x0, fy = y - y0;
    
    auto get = [&](int xi, int yi) -> float {
        xi = std::clamp(xi, 0, sdf_res-1);
        yi = std::clamp(yi, 0, sdf_res-1);
        return sdf[yi * sdf_res + xi];
    };
    
    return get(x0,y0)*(1-fx)*(1-fy) + get(x1,y0)*fx*(1-fy)
         + get(x0,y1)*(1-fx)*fy    + get(x1,y1)*fx*fy;
}

// Render a single glyph's SDF onto an image
void renderGlyph(Image& img, const std::vector<float>& sdf, int sdf_res,
                  int dstX, int dstY, int dstW, int dstH,
                  const SDFRenderParams& params) {
    
    for (int py = 0; py < dstH; py++) {
        for (int px = 0; px < dstW; px++) {
            float u = (px + 0.5f) / dstW;
            float v = (py + 0.5f) / dstH;
            
            float dist = sampleSDF(sdf, sdf_res, u, v);
            
            // Drop shadow
            if (params.shadow) {
                float su = u - params.shadowOffsetX;
                float sv = v - params.shadowOffsetY;
                if (su >= 0 && su <= 1 && sv >= 0 && sv <= 1) {
                    float sdist = sampleSDF(sdf, sdf_res, su, sv);
                    float shadow_alpha = std::clamp(
                        (sdist - (0.5f - params.shadowSoftness)) / params.shadowSoftness,
                        0.0f, 1.0f);
                    if (shadow_alpha > 0) {
                        Color sc = params.shadowColor;
                        sc.a = (uint8_t)(sc.a * shadow_alpha);
                        img.blendPixel(dstX + px, dstY + py, sc);
                    }
                }
            }
            
            // Glow effect
            if (params.glow) {
                float glow_dist = dist + params.glowWidth;
                float glow_alpha = std::clamp(glow_dist / params.glowWidth, 0.0f, 1.0f);
                glow_alpha = glow_alpha * glow_alpha; // quadratic falloff
                if (glow_alpha > 0 && dist < params.threshold) {
                    Color gc = params.glowColor;
                    gc.a = (uint8_t)(gc.a * glow_alpha);
                    img.blendPixel(dstX + px, dstY + py, gc);
                }
            }
            
            // Outline
            if (params.outline) {
                float outline_dist = dist - (params.threshold - params.outlineWidth);
                float outline_alpha = std::clamp(
                    outline_dist / params.smoothing + 0.5f, 0.0f, 1.0f);
                float text_alpha = std::clamp(
                    (dist - params.threshold) / params.smoothing + 0.5f, 0.0f, 1.0f);
                outline_alpha = outline_alpha - text_alpha;
                if (outline_alpha > 0) {
                    Color oc = params.outlineColor;
                    oc.a = (uint8_t)(oc.a * outline_alpha);
                    img.blendPixel(dstX + px, dstY + py, oc);
                }
            }
            
            // Main text fill
            float alpha = std::clamp(
                (dist - params.threshold) / params.smoothing + 0.5f, 0.0f, 1.0f);
            if (alpha > 0) {
                Color tc = params.textColor;
                tc.a = (uint8_t)(tc.a * alpha);
                img.blendPixel(dstX + px, dstY + py, tc);
            }
        }
    }
}

// Render a string of glyphs
void renderText(Image& img, const std::vector<const std::vector<float>*>& sdfs,
                int sdf_res, int startX, int startY, int glyphW, int glyphH,
                int spacing, const SDFRenderParams& params) {
    for (size_t i = 0; i < sdfs.size(); i++) {
        if (!sdfs[i]) continue;
        int x = startX + (int)i * (glyphW + spacing);
        renderGlyph(img, *sdfs[i], sdf_res, x, startY, glyphW, glyphH, params);
    }
}

// =====================================================================
// Draw separators and labels
// =====================================================================

void drawRect(Image& img, int x0, int y0, int x1, int y1, Color c) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            img.setPixel(x, y, c);
}

void drawHLine(Image& img, int y, int x0, int x1, Color c) {
    for (int x = x0; x <= x1; x++) img.setPixel(x, y, c);
}

void drawVLine(Image& img, int x, int y0, int y1, Color c) {
    for (int y = y0; y <= y1; y++) img.setPixel(x, y, c);
}

// =====================================================================
// Main
// =====================================================================

int main() {
    const int W = 900, H = 660;
    Image img(W, H, Color(22, 22, 35));
    
    // Build glyph SDFs
    const int SDF_RES = 64; // internal SDF resolution
    
    Glyph glyphS = makeGlyphS();
    Glyph glyphD = makeGlyphD();
    Glyph glyphF = makeGlyphF();
    Glyph glyphR = makeGlyphR();
    Glyph glyphE = makeGlyphE();
    Glyph glyphN = makeGlyphN();
    Glyph glyphI = makeGlyphI();
    Glyph glyphG = makeGlyphG();
    
    std::cout << "Generating SDFs..." << std::endl;
    auto sdfS = computeSDF(glyphS, SDF_RES);
    auto sdfD = computeSDF(glyphD, SDF_RES);
    auto sdfF = computeSDF(glyphF, SDF_RES);
    auto sdfR = computeSDF(glyphR, SDF_RES);
    auto sdfE = computeSDF(glyphE, SDF_RES);
    auto sdfN = computeSDF(glyphN, SDF_RES);
    auto sdfI = computeSDF(glyphI, SDF_RES);
    auto sdfG = computeSDF(glyphG, SDF_RES);
    std::cout << "SDFs generated." << std::endl;
    
    // "SDF" letters
    std::vector<const std::vector<float>*> sdfWord = {&sdfS, &sdfD, &sdfF};
    // "RENDERING" letters  
    std::vector<const std::vector<float>*> renderWord = {&sdfR, &sdfE, &sdfN, &sdfD, &sdfE, &sdfR, &sdfI, &sdfN, &sdfG};
    // "SDF" single letters for demo rows
    std::vector<const std::vector<float>*> singleS = {&sdfS};
    std::vector<const std::vector<float>*> singleD = {&sdfD};
    std::vector<const std::vector<float>*> singleF = {&sdfF};
    
    // ----------------------------------------------------------------
    // Section backgrounds
    // ----------------------------------------------------------------
    
    // Top header: dark gradient
    for (int y = 0; y < 80; y++) {
        float t = y / 80.0f;
        uint8_t v = (uint8_t)(30 + t * 10);
        for (int x = 0; x < W; x++)
            img.setPixel(x, y, Color(v, v, v+15));
    }
    
    // Dividers
    Color divider(60, 60, 90);
    drawHLine(img, 79, 0, W-1, divider);
    drawHLine(img, 80, 0, W-1, Color(80,80,120));
    
    // ----------------------------------------------------------------
    // Header: Large "SDF" title with glow + outline
    // ----------------------------------------------------------------
    {
        SDFRenderParams p;
        p.textColor = {240, 240, 255, 255};
        p.smoothing = 0.04f;
        p.glow = true;
        p.glowWidth = 0.35f;
        p.glowColor = {100, 140, 255, 160};
        p.outline = true;
        p.outlineWidth = 0.12f;
        p.outlineColor = {60, 100, 200, 255};
        renderText(img, sdfWord, SDF_RES, 30, 8, 60, 62, 6, p);
    }
    
    // "RENDERING" text in header
    {
        SDFRenderParams p;
        p.textColor = {200, 215, 255, 230};
        p.smoothing = 0.05f;
        renderText(img, renderWord, SDF_RES, 250, 20, 35, 42, 3, p);
    }
    
    // Subtitle row: "FONT RENDERING TECHNIQUE"
    // Use smaller version
    {
        // Draw "SDF" with shadow effect in subtitle
        SDFRenderParams p;
        p.textColor = {180, 200, 255, 200};
        p.smoothing = 0.06f;
        p.shadow = true;
        p.shadowOffsetX = 0.06f;
        p.shadowOffsetY = 0.08f;
        p.shadowSoftness = 0.2f;
        p.shadowColor = {0, 0, 0, 120};
        renderText(img, sdfWord, SDF_RES, 600, 18, 28, 34, 3, p);
    }
    
    // ----------------------------------------------------------------
    // Section 1: Resolution comparison (top row)
    // Bitmap vs SDF at different sizes
    // ----------------------------------------------------------------
    
    int sec1_y = 90;
    Color secBg1(28, 28, 42);
    drawRect(img, 0, sec1_y, W-1, sec1_y + 180, secBg1);
    drawHLine(img, sec1_y + 180, 0, W-1, divider);
    
    // Section label (simple pixels for text)
    Color labelColor(150, 170, 220);
    
    // Draw "RESOLUTION COMPARISON" using tiny rects as pixel font
    // Draw scaled S glyphs at different sizes to demonstrate SDF quality
    
    // Left label area
    Color boxBg(35, 35, 55);
    drawRect(img, 10, sec1_y + 5, 140, sec1_y + 175, boxBg);
    
    // Glyph "S" at sizes: 16, 24, 40, 60
    {
        SDFRenderParams p;
        p.textColor = {255, 220, 100, 255};
        p.smoothing = 0.05f;
        int sizes[] = {16, 24, 40, 60};
        int xs[] = {20, 50, 82, 135};
        int ys[] = {sec1_y + 112, sec1_y + 105, sec1_y + 95, sec1_y + 85};
        for (int i = 0; i < 4; i++) {
            SDFRenderParams pi = p;
            pi.smoothing = std::max(0.02f, 0.15f / sizes[i] * 16); // less smooth = sharper at larger sizes
            renderGlyph(img, sdfS, SDF_RES, xs[i] - sizes[i]/2, ys[i], sizes[i], sizes[i], pi);
        }
        
        // Label the sizes
        // We'll use colored dots as size indicators
        Color sizeColors[] = {{255,100,100,255}, {255,180,100,255}, {100,255,180,255}, {100,180,255,255}};
        int dotSizes[] = {3, 4, 6, 8};
        for (int i = 0; i < 4; i++) {
            int bx = xs[i], by = sec1_y + 168;
            drawRect(img, bx-dotSizes[i]/2, by-dotSizes[i]/2, bx+dotSizes[i]/2, by+dotSizes[i]/2, sizeColors[i]);
        }
    }
    
    // Right: "D" and "F" at full size with various effects
    // Effect demo panels
    int panel_x[] = {160, 310, 460, 610, 750};
    int panel_w = 130;
    Color panelColors[] = {
        Color(28, 40, 55),
        Color(28, 45, 35),
        Color(45, 28, 55),
        Color(55, 40, 28),
        Color(28, 28, 50)
    };
    
    const char* effectNames[] = {
        "SMOOTH", "OUTLINE", "SHADOW", "GLOW", "COMBINED"
    };
    (void)effectNames;
    
    for (int p = 0; p < 5; p++) {
        drawRect(img, panel_x[p], sec1_y + 5, panel_x[p] + panel_w, sec1_y + 175, panelColors[p]);
        drawRect(img, panel_x[p], sec1_y + 5, panel_x[p] + panel_w, sec1_y + 6, Color(80,80,120));
    }
    
    // Panel 0: Smooth AA text "DF"
    {
        SDFRenderParams p;
        p.textColor = {240, 240, 255, 255};
        p.smoothing = 0.05f;
        renderGlyph(img, sdfD, SDF_RES, panel_x[0]+15, sec1_y+30, 45, 55, p);
        renderGlyph(img, sdfF, SDF_RES, panel_x[0]+65, sec1_y+30, 45, 55, p);
    }
    
    // Panel 1: Outline
    {
        SDFRenderParams p;
        p.textColor = {240, 240, 255, 255};
        p.smoothing = 0.04f;
        p.outline = true;
        p.outlineWidth = 0.12f;
        p.outlineColor = {100, 200, 255, 255};
        renderGlyph(img, sdfD, SDF_RES, panel_x[1]+15, sec1_y+30, 45, 55, p);
        renderGlyph(img, sdfF, SDF_RES, panel_x[1]+65, sec1_y+30, 45, 55, p);
    }
    
    // Panel 2: Drop shadow
    {
        SDFRenderParams p;
        p.textColor = {255, 230, 150, 255};
        p.smoothing = 0.04f;
        p.shadow = true;
        p.shadowOffsetX = 0.08f;
        p.shadowOffsetY = 0.10f;
        p.shadowSoftness = 0.18f;
        p.shadowColor = {0, 0, 0, 200};
        renderGlyph(img, sdfD, SDF_RES, panel_x[2]+15, sec1_y+30, 45, 55, p);
        renderGlyph(img, sdfF, SDF_RES, panel_x[2]+65, sec1_y+30, 45, 55, p);
    }
    
    // Panel 3: Glow
    {
        SDFRenderParams p;
        p.textColor = {255, 255, 255, 255};
        p.smoothing = 0.04f;
        p.glow = true;
        p.glowWidth = 0.30f;
        p.glowColor = {255, 150, 50, 210};
        renderGlyph(img, sdfD, SDF_RES, panel_x[3]+15, sec1_y+30, 45, 55, p);
        renderGlyph(img, sdfF, SDF_RES, panel_x[3]+65, sec1_y+30, 45, 55, p);
    }
    
    // Panel 4: Combined effects
    {
        SDFRenderParams p;
        p.textColor = {220, 240, 255, 255};
        p.smoothing = 0.04f;
        p.glow = true;
        p.glowWidth = 0.25f;
        p.glowColor = {80, 160, 255, 180};
        p.outline = true;
        p.outlineWidth = 0.08f;
        p.outlineColor = {200, 220, 255, 255};
        p.shadow = true;
        p.shadowOffsetX = 0.06f;
        p.shadowOffsetY = 0.09f;
        p.shadowSoftness = 0.20f;
        p.shadowColor = {0, 0, 20, 160};
        renderGlyph(img, sdfD, SDF_RES, panel_x[4]+15, sec1_y+30, 45, 55, p);
        renderGlyph(img, sdfF, SDF_RES, panel_x[4]+65, sec1_y+30, 45, 55, p);
    }
    
    // Effect labels below panels (colored stripe)
    Color effectBgColors[] = {
        Color(40,40,80,200), Color(30,60,80,200), Color(60,40,80,200),
        Color(80,50,30,200), Color(40,40,80,200)
    };
    Color effectTextColors[] = {
        Color(180,200,255), Color(100,220,255), Color(255,230,150),
        Color(255,180,80), Color(200,220,255)
    };
    // Draw colored stripe at bottom of each panel
    for (int p = 0; p < 5; p++) {
        drawRect(img, panel_x[p]+1, sec1_y+155, panel_x[p]+panel_w-1, sec1_y+174, effectBgColors[p]);
        // Draw 3 dots as visual label
        for (int d = 0; d < 3; d++) {
            int dx = panel_x[p] + 30 + d * 22;
            int dy = sec1_y + 164;
            Color dc = effectTextColors[p];
            drawRect(img, dx-3, dy-3, dx+3, dy+3, dc);
        }
    }
    
    // ----------------------------------------------------------------
    // Section 2: SDF Distance Field Visualization
    // ----------------------------------------------------------------
    
    int sec2_y = 272;
    Color secBg2(30, 25, 40);
    drawRect(img, 0, sec2_y, 300, H-1, secBg2);
    drawVLine(img, 300, sec2_y, H-1, divider);
    
    // Visualize the raw SDF as a colored gradient
    {
        int viz_x = 10, viz_y = sec2_y + 10;
        int viz_w = 130, viz_h = 130;
        
        for (int py = 0; py < viz_h; py++) {
            for (int px = 0; px < viz_w; px++) {
                float u = (float)px / viz_w;
                float v = (float)py / viz_h;
                float dist = sampleSDF(sdfS, SDF_RES, u, v);
                
                // Colorize: hot colormap for distance field
                // Negative (outside) = dark blue → blue
                // Near 0 (border) = yellow/white
                // Positive (inside) = red
                float t = std::clamp(dist * 1.5f + 0.5f, 0.0f, 1.0f);
                
                uint8_t r, g, b;
                if (t < 0.25f) {
                    r = 0; g = 0; b = (uint8_t)(t / 0.25f * 200);
                } else if (t < 0.5f) {
                    float s = (t - 0.25f) / 0.25f;
                    r = 0; g = (uint8_t)(s * 180); b = (uint8_t)(200 - s*200);
                } else if (t < 0.75f) {
                    float s = (t - 0.5f) / 0.25f;
                    r = (uint8_t)(s * 255); g = (uint8_t)(180 + s * 75); b = 0;
                } else {
                    float s = (t - 0.75f) / 0.25f;
                    r = 255; g = (uint8_t)(255 - s * 100); b = 0;
                }
                
                img.setPixel(viz_x + px, viz_y + py, Color(r, g, b));
                
                // Draw isolines (contours)
                float isoline_check = fabsf(fmodf(fabsf(dist) * 6.0f, 1.0f) - 0.5f);
                if (isoline_check < 0.05f) {
                    img.setPixel(viz_x + px, viz_y + py, Color(255, 255, 255, 100));
                }
            }
        }
        
        // Border
        drawRect(img, viz_x-1, viz_y-1, viz_x+viz_w, viz_y+viz_h, Color(80,80,120));
    }
    
    // Second SDF viz: D glyph
    {
        int viz_x = 155, viz_y = sec2_y + 10;
        int viz_w = 130, viz_h = 130;
        
        for (int py = 0; py < viz_h; py++) {
            for (int px = 0; px < viz_w; px++) {
                float u = (float)px / viz_w;
                float v = (float)py / viz_h;
                float dist = sampleSDF(sdfD, SDF_RES, u, v);
                
                // Blue-cyan-white-yellow-red colormap
                float t = std::clamp(dist * 1.5f + 0.5f, 0.0f, 1.0f);
                uint8_t r, g, b;
                if (t < 0.33f) {
                    float s = t / 0.33f;
                    r = 20; g = (uint8_t)(s * 120); b = (uint8_t)(80 + s * 120);
                } else if (t < 0.66f) {
                    float s = (t - 0.33f) / 0.33f;
                    r = (uint8_t)(s * 200); g = (uint8_t)(120 + s * 100); b = (uint8_t)(200 - s * 150);
                } else {
                    float s = (t - 0.66f) / 0.34f;
                    r = (uint8_t)(200 + s * 55); g = (uint8_t)(220 - s * 100); b = (uint8_t)(50 - s * 50);
                }
                img.setPixel(viz_x + px, viz_y + py, Color(r, g, b));
                
                float isoline_check = fabsf(fmodf(fabsf(dist) * 6.0f, 1.0f) - 0.5f);
                if (isoline_check < 0.05f)
                    img.setPixel(viz_x + px, viz_y + py, Color(255, 255, 255, 80));
            }
        }
        drawRect(img, viz_x-1, viz_y-1, viz_x+viz_w, viz_y+viz_h, Color(80,80,120));
    }
    
    // ----------------------------------------------------------------
    // Section 3: Large display with all effects
    // ----------------------------------------------------------------
    
    int sec3_y = sec2_y;
    int sec3_x = 310;
    Color secBg3(25, 25, 38);
    drawRect(img, sec3_x, sec3_y, W-1, H-1, secBg3);
    
    // Large "SDF" with combined effects
    {
        SDFRenderParams p;
        p.textColor = {230, 240, 255, 255};
        p.smoothing = 0.03f;
        p.glow = true;
        p.glowWidth = 0.35f;
        p.glowColor = {80, 130, 255, 150};
        p.outline = true;
        p.outlineWidth = 0.10f;
        p.outlineColor = {120, 180, 255, 220};
        p.shadow = true;
        p.shadowOffsetX = 0.07f;
        p.shadowOffsetY = 0.10f;
        p.shadowSoftness = 0.22f;
        p.shadowColor = {0, 10, 40, 180};
        renderText(img, sdfWord, SDF_RES, sec3_x + 20, sec3_y + 15, 82, 98, 8, p);
    }
    
    // "RENDERING" below in golden
    {
        SDFRenderParams p;
        p.textColor = {255, 210, 80, 255};
        p.smoothing = 0.04f;
        p.outline = true;
        p.outlineWidth = 0.08f;
        p.outlineColor = {180, 120, 20, 255};
        p.shadow = true;
        p.shadowOffsetX = 0.05f;
        p.shadowOffsetY = 0.07f;
        p.shadowSoftness = 0.18f;
        p.shadowColor = {0, 0, 0, 160};
        renderText(img, renderWord, SDF_RES, sec3_x + 15, sec3_y + 128, 55, 65, 4, p);
    }
    
    // Color variations row
    Color textVarColors[] = {
        Color(255, 100, 100, 255),
        Color(100, 255, 150, 255),
        Color(100, 180, 255, 255),
        Color(255, 200, 80, 255),
        Color(200, 100, 255, 255),
    };
    
    for (int ci = 0; ci < 5; ci++) {
        SDFRenderParams p;
        p.textColor = textVarColors[ci];
        p.smoothing = 0.04f;
        p.glow = true;
        p.glowWidth = 0.22f;
        p.glowColor = Color(textVarColors[ci].r/2, textVarColors[ci].g/2, textVarColors[ci].b/2, 120);
        renderText(img, sdfWord, SDF_RES, sec3_x + 20 + ci * 112, sec3_y + 210, 28, 34, 3, p);
    }
    
    // Bottom row: "SDF" at various thresholds (different weight/bold effects)
    float thresholds[] = {0.35f, 0.42f, 0.50f, 0.57f, 0.64f};
    for (int ti = 0; ti < 5; ti++) {
        SDFRenderParams p;
        p.textColor = {220, 230, 255, 240};
        p.smoothing = 0.04f;
        p.threshold = thresholds[ti];
        renderText(img, sdfWord, SDF_RES, sec3_x + 20 + ti * 112, sec3_y + 260, 25, 30, 3, p);
    }
    
    // ----------------------------------------------------------------
    // Section 4: Bottom info bar
    // ----------------------------------------------------------------
    
    int bar_y = 615;
    drawRect(img, 0, bar_y, W-1, H-1, Color(18, 18, 28));
    drawHLine(img, bar_y, 0, W-1, divider);
    
    // Small SDF glyphs in bottom bar for decoration
    {
        SDFRenderParams p;
        p.textColor = {120, 140, 200, 180};
        p.smoothing = 0.06f;
        for (int n = 0; n < 15; n++) {
            int gx = 30 + n * 58;
            int idx = n % 8;
            const std::vector<float>* ptrs[] = {&sdfS,&sdfD,&sdfF,&sdfR,&sdfE,&sdfN,&sdfI,&sdfG};
            renderGlyph(img, *ptrs[idx], SDF_RES, gx, bar_y+5, 20, 25, p);
        }
    }
    
    // ----------------------------------------------------------------
    // Save
    // ----------------------------------------------------------------
    
    std::cout << "Saving image..." << std::endl;
    if (!img.savePNG("sdf_font_output.png")) {
        std::cerr << "Failed to save PNG!" << std::endl;
        return 1;
    }
    
    std::cout << "✅ Done! Output: sdf_font_output.png (" << W << "x" << H << ")" << std::endl;
    std::cout << "  Features demonstrated:" << std::endl;
    std::cout << "  - SDF distance field generation from bitmap glyphs" << std::endl;
    std::cout << "  - Smooth anti-aliasing via threshold + gradient" << std::endl;
    std::cout << "  - Outline effect (independent SDF threshold)" << std::endl;
    std::cout << "  - Drop shadow (offset SDF sample)" << std::endl;
    std::cout << "  - Glow effect (SDF-based falloff)" << std::endl;
    std::cout << "  - Multi-size rendering from single SDF" << std::endl;
    std::cout << "  - Distance field colormap visualization with isolines" << std::endl;
    std::cout << "  - Weight/bold variation via threshold adjustment" << std::endl;
    
    return 0;
}
