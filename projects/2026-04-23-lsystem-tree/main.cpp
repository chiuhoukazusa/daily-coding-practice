// L-System Procedural Tree Renderer
// Lindenmayer System based fractal tree generation with 3D soft rasterization
// Techniques: L-System grammar rewriting, Turtle Graphics 3D, depth sorting, Phong shading
// Date: 2026-04-23

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <stack>
#include <algorithm>
#include <random>

// ==================== STB Image Write ====================
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../stb_image_write.h"
#pragma GCC diagnostic pop

// ==================== Constants ====================
static const float PI = 3.14159265358979f;

// ==================== Math Types ====================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const {
        float l = length();
        if (l < 1e-7f) return {0,1,0};
        return *this / l;
    }
};

// 3x3 rotation matrix (column-major: columns are right, up, forward)
struct Mat3 {
    Vec3 col[3]; // col[0]=right, col[1]=up, col[2]=forward
    Mat3() {
        col[0] = {1,0,0};
        col[1] = {0,1,0};
        col[2] = {0,0,1};
    }
    Vec3 apply(const Vec3& v) const {
        return col[0]*v.x + col[1]*v.y + col[2]*v.z;
    }
};

// Rotate mat around a world axis
Mat3 rotateAround(const Mat3& m, const Vec3& axis, float angle) {
    // Rodrigues rotation
    float c = cosf(angle), s = sinf(angle), mc = 1-c;
    Vec3 a = axis;
    // Build rotation matrix
    float r00 = c + a.x*a.x*mc,      r01 = a.x*a.y*mc - a.z*s, r02 = a.x*a.z*mc + a.y*s;
    float r10 = a.y*a.x*mc + a.z*s,  r11 = c + a.y*a.y*mc,     r12 = a.y*a.z*mc - a.x*s;
    float r20 = a.z*a.x*mc - a.y*s,  r21 = a.z*a.y*mc + a.x*s, r22 = c + a.z*a.z*mc;

    Mat3 result;
    for (int i=0; i<3; i++) {
        Vec3 v = m.col[i];
        result.col[i] = {
            r00*v.x + r01*v.y + r02*v.z,
            r10*v.x + r11*v.y + r12*v.z,
            r20*v.x + r21*v.y + r22*v.z
        };
    }
    return result;
}

// ==================== L-System ====================
struct LSystem {
    std::string axiom;
    char from[16];
    std::string to[16];
    int count = 0;

    void addRule(char f, const std::string& t) { from[count]=f; to[count]=t; count++; }

    std::string generate(int iters) const {
        std::string cur = axiom;
        for (int i=0; i<iters; i++) {
            std::string next; next.reserve(cur.size()*3);
            for (char c : cur) {
                bool rep = false;
                for (int j=0; j<count; j++) {
                    if (from[j]==c) { next += to[j]; rep=true; break; }
                }
                if (!rep) next += c;
            }
            cur = next;
        }
        return cur;
    }
};

// ==================== Branch ====================
struct Branch {
    Vec3 start, end;
    float r0, r1;     // start/end screen radius (pixels)
    Vec3 color;
    float viewZ;      // for depth sort
};

struct LeafQuad {
    Vec3 pos;
    float size;
    Vec3 color;
    float viewZ;
};

// ==================== Framebuffer ====================
static const int W = 900, H = 900;
static uint8_t fb[H][W][3];
static float zb[H][W];

void clearBufs() {
    for (int y=0; y<H; y++)
        for (int x=0; x<W; x++) {
            fb[y][x][0]=fb[y][x][1]=fb[y][x][2]=0;
            zb[y][x] = 1e9f;
        }
}

void putPixel(int x, int y, float z, uint8_t r, uint8_t g, uint8_t b) {
    if (x<0||x>=W||y<0||y>=H) return;
    if (z < zb[y][x]) {
        zb[y][x]=z;
        fb[y][x][0]=r; fb[y][x][1]=g; fb[y][x][2]=b;
    }
}

// ==================== Simple orthographic projection ====================
// World: X=right, Y=up, Z=toward viewer (slight angle)
// We use a slightly rotated view (isometric-ish from front-left)
struct Camera {
    Vec3 pos;        // camera position
    Vec3 right, up, forward; // orthonormal basis
    float scale;     // world units to pixels
    int cx, cy;      // screen center

    Vec2 project(const Vec3& p) const {
        Vec3 d = p - pos;
        float sx = d.dot(right)  * scale + cx;
        float sy = -d.dot(up)    * scale + cy;  // flip Y (screen Y down)
        return {sx, sy};
    }
    float projZ(const Vec3& p) const {
        return -(p - pos).dot(forward);  // negative = in front of camera
    }
};

// ==================== Draw gradient background ====================
void drawBackground() {
    for (int y=0; y<H; y++) {
        float t = (float)y / H;
        // Sky gradient: soft blue at top -> warm white/beige at horizon
        uint8_t R,G,B;
        if (t < 0.7f) {
            // sky
            float s = t / 0.7f;
            R = (uint8_t)(120 + s*100);
            G = (uint8_t)(168 + s*62);
            B = (uint8_t)(215 + s*10);
        } else {
            // ground
            float s = (t - 0.7f) / 0.3f;
            R = (uint8_t)(100 + s*30);  // brown-green
            G = (uint8_t)(115 + s*20);
            B = (uint8_t)(55 - s*15);
        }
        for (int x=0; x<W; x++) {
            fb[y][x][0]=R; fb[y][x][1]=G; fb[y][x][2]=B;
            zb[y][x]=1e9f;
        }
    }
}

// ==================== Draw filled circle (branch cross-section endpoint) ====================
void drawCircle(float cx, float cy, float z, float r, uint8_t R, uint8_t G, uint8_t B) {
    int ir = (int)ceilf(r);
    for (int oy=-ir; oy<=ir; oy++)
        for (int ox=-ir; ox<=ir; ox++) {
            float d2 = (float)(ox*ox + oy*oy);
            if (d2 <= r*r) {
                float d = sqrtf(d2);
                float edge = 1.0f - d/r * 0.35f;
                putPixel((int)(cx+ox),(int)(cy+oy),z,
                         (uint8_t)(R*edge),(uint8_t)(G*edge),(uint8_t)(B*edge));
            }
        }
}

// ==================== Draw thick line ====================
void drawThickLine(float x0, float y0, float x1, float y1, float z0, float z1,
                   float r0, float r1, Vec3 col) {
    float dx = x1-x0, dy = y1-y0;
    float len = sqrtf(dx*dx+dy*dy);
    if (len < 0.001f) return;

    // Perpendicular direction
    float px = -dy/len, py = dx/len;

    // Draw as filled trapezoid using scanline approach
    // Simple: step along the line and draw circles
    int steps = (int)(len) + 2;
    for (int i=0; i<=steps; i++) {
        float t = (float)i / steps;
        float cx_ = x0 + dx*t, cy_ = y0 + dy*t;
        float cz = z0 + (z1-z0)*t;
        float cr = r0 + (r1-r0)*t;
        if (cr < 0.5f) cr = 0.5f;

        // Lighting: simple diffuse based on angle
        float bright = 0.75f + 0.25f * (1.0f - t);  // slight highlight at start
        int ir = (int)ceilf(cr);
        for (int oy=-ir; oy<=ir; oy++)
            for (int ox=-ir; ox<=ir; ox++) {
                float d = sqrtf((float)(ox*ox+oy*oy));
                if (d <= cr) {
                    float edge = (1.0f - d/cr * 0.4f) * bright;
                    uint8_t R = (uint8_t)std::min(col.x*255*edge, 255.0f);
                    uint8_t G = (uint8_t)std::min(col.y*255*edge, 255.0f);
                    uint8_t B = (uint8_t)std::min(col.z*255*edge, 255.0f);
                    putPixel((int)(cx_+ox),(int)(cy_+oy),cz,R,G,B);
                }
            }
    }
    (void)px; (void)py;
}

// ==================== Draw leaf ====================
void drawLeaf(float cx, float cy, float cz, float r, Vec3 col) {
    int ir = (int)ceilf(r);
    for (int oy=-ir; oy<=ir; oy++)
        for (int ox=-ir; ox<=ir; ox++) {
            float d2 = (float)(ox*ox + oy*oy);
            if (d2 <= r*r) {
                float d = sqrtf(d2)/r;
                float bright = 0.6f + 0.5f*(1.0f-d);
                // slight yellow tint at center (fresh leaves)
                float yt = (1.0f-d*d)*0.25f;
                uint8_t R = (uint8_t)std::min((col.x+yt)*255*bright, 255.0f);
                uint8_t G = (uint8_t)std::min((col.y+yt*0.3f)*255*bright, 255.0f);
                uint8_t B = (uint8_t)std::min(col.z*255*bright*0.8f, 255.0f);
                putPixel((int)(cx+ox),(int)(cy+oy),cz,R,G,B);
            }
        }
}

// ==================== Turtle Interpreter ====================
struct TState {
    Vec3 pos;
    Mat3 orient;   // col[1] = heading (up direction for turtle), col[0]=right, col[2]=heading x up
    float length;
    float pixRadius;   // radius in pixels
    int generation;    // depth in recursion
};

void buildTree(const std::string& lstr, float baseLen, float basePixR,
               float angleDeg, float lenScale, float radScale,
               std::vector<Branch>& branches, std::vector<LeafQuad>& leaves,
               const Camera& cam, std::mt19937& rng)
{
    std::uniform_real_distribution<float> jit(-0.1f, 0.1f);

    std::stack<TState> stk;
    TState turtle;
    turtle.pos = {0, 0, 0};
    // Initial orientation: heading straight up (+Y)
    turtle.orient.col[0] = {1, 0, 0};   // right
    turtle.orient.col[1] = {0, 1, 0};   // heading (up)
    turtle.orient.col[2] = {0, 0, 1};   // forward (= heading x right)
    turtle.length = baseLen;
    turtle.pixRadius = basePixR;
    turtle.generation = 0;

    float angle = angleDeg * PI / 180.0f;

    // Find max generation for coloring
    int maxGen = 0; { int g=0;
        for (char c : lstr) {
            if (c=='[') g++;
            else if (c==']') g--;
            if (g>maxGen) maxGen=g;
        }
    }

    for (char c : lstr) {
        switch(c) {
        case 'F': case 'G': case 'A': case 'B': case 'C': {
            float len = turtle.length * (1.0f + jit(rng)*0.15f);
            Vec3 heading = turtle.orient.col[1];  // heading
            Vec3 newPos = turtle.pos + heading * len;

            // Color: brown trunk -> green-brown branches
            float t = (float)turtle.generation / (float)(maxGen+1);
            Vec3 col;
            if (t < 0.35f) {
                // trunk: warm brown
                col = Vec3(0.48f, 0.30f, 0.13f) + Vec3(jit(rng)*0.05f, jit(rng)*0.03f, 0);
            } else if (t < 0.65f) {
                // mid branches: brownish green
                float s = (t-0.35f)/0.3f;
                col = Vec3(0.35f-s*0.08f, 0.30f+s*0.1f, 0.12f+s*0.05f) + Vec3(jit(rng)*0.04f,jit(rng)*0.04f,0);
            } else {
                // thin: green
                col = Vec3(0.20f, 0.38f+jit(rng)*0.06f, 0.14f+jit(rng)*0.04f);
            }
            col.x = std::max(0.0f, std::min(1.0f, col.x));
            col.y = std::max(0.0f, std::min(1.0f, col.y));
            col.z = std::max(0.0f, std::min(1.0f, col.z));

            Branch br;
            br.start = turtle.pos; br.end = newPos;
            br.r0 = turtle.pixRadius; br.r1 = turtle.pixRadius * 0.9f;
            br.color = col;
            // view Z for sorting (smaller = closer to camera = draw last)
            br.viewZ = cam.projZ((turtle.pos + newPos)*0.5f);
            branches.push_back(br);

            // Sprinkle leaves on thin-to-medium branches
            if (turtle.pixRadius < basePixR * 0.55f) {
                std::uniform_real_distribution<float> prob(0,1);
                // More leaves on thinner branches
                float leafChance = 1.0f - (turtle.pixRadius / (basePixR * 0.55f)) * 0.4f;
                if (prob(rng) < leafChance) {
                    int nL = 4 + (int)(rng()%5);
                    std::uniform_real_distribution<float> lo(-0.6f, 0.6f);
                    for (int i=0; i<nL; i++) {
                        float lt = 0.15f + (float)(rng()%100)/100.0f * 0.85f;
                        Vec3 lp = turtle.pos + heading * (len*lt);
                        Vec3 right = turtle.orient.col[0];
                        Vec3 fwd   = turtle.orient.col[2];
                        lp = lp + right*(lo(rng)*len*0.55f) + fwd*(lo(rng)*len*0.55f) + heading*(lo(rng)*len*0.2f);

                        float hv = (float)(rng()%100)/100.0f;
                        LeafQuad lf;
                        lf.pos = lp;
                        lf.size = 4.0f + hv*5.0f;
                        lf.color = Vec3(0.08f+hv*0.1f, 0.33f+hv*0.32f, 0.06f+hv*0.07f);
                        lf.viewZ = cam.projZ(lp);
                        leaves.push_back(lf);
                    }
                }
            }

            turtle.pos = newPos;
            break;
        }
        case '+': {  // yaw left (rotate around heading x right = local Z)
            float a = angle + jit(rng)*0.15f;
            // rotate around local forward axis (col[2])
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[2], a);
            break;
        }
        case '-': {  // yaw right
            float a = angle + jit(rng)*0.15f;
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[2], -a);
            break;
        }
        case '&': {  // pitch forward (rotate around local right)
            float a = angle + jit(rng)*0.15f;
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[0], a);
            break;
        }
        case '^': {  // pitch backward
            float a = angle + jit(rng)*0.15f;
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[0], -a);
            break;
        }
        case '\\': {  // roll left (rotate around heading)
            float a = angle + jit(rng)*0.15f;
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[1], a);
            break;
        }
        case '/': {  // roll right
            float a = angle + jit(rng)*0.15f;
            turtle.orient = rotateAround(turtle.orient, turtle.orient.col[1], -a);
            break;
        }
        case '[': {
            stk.push(turtle);
            turtle.generation++;
            turtle.length *= lenScale;
            turtle.pixRadius *= radScale;
            break;
        }
        case ']': {
            if (!stk.empty()) {
                turtle = stk.top(); stk.pop();
            }
            break;
        }
        default: break;
        }
    }
}

// ==================== Main ====================
int main() {
    printf("L-System Procedural Tree Renderer\n");
    printf("Date: 2026-04-23\n\n");

    std::mt19937 rng(12345);

    // ---- L-System definition ----
    // Classic 3D branching tree (4 branches per node, full 3D rotation)
    // Inspired by Prusinkiewicz & Lindenmayer "The Algorithmic Beauty of Plants"
    //
    // A = apical bud: grow up, then create 4 lateral branches (+/- with rolls)
    // and continue up with new A
    LSystem tree;
    tree.axiom = "FFFFA";
    //              trunk   left      right     front     back    continue
    tree.addRule('A', "FF[&+A][&-A][&\\A][&/A]FA");

    printf("Generating L-system (6 iterations)...\n");
    std::string lstr = tree.generate(6);
    printf("L-string length: %zu\n", lstr.size());

    // ---- Setup camera ----
    // Orthographic projection with slight 3D rotation feel
    // Camera looks slightly from front-left, down-angle
    Camera cam;
    // Rotation: 15 degrees horizontal, 5 degrees vertical tilt
    float yaw = 15.0f * PI/180.0f;
    float pitch = 5.0f * PI/180.0f;
    cam.right   = {cosf(yaw), 0,          -sinf(yaw)};
    cam.up      = {sinf(yaw)*sinf(pitch), cosf(pitch), cosf(yaw)*sinf(pitch)};
    cam.forward = {sinf(yaw)*cosf(pitch), -sinf(pitch), cosf(yaw)*cosf(pitch)};
    cam.scale   = 55.0f;
    cam.cx      = W/2;
    cam.cy      = (int)(H * 0.78f);  // tree grows upward, root near bottom
    cam.pos     = {0, 0, 0};

    // ---- Build geometry ----
    std::vector<Branch> branches;
    std::vector<LeafQuad> leaves;
    buildTree(lstr, 0.28f, 9.0f, 25.0f, 0.68f, 0.64f, branches, leaves, cam, rng);
    printf("Branches: %zu, Leaves: %zu\n", branches.size(), leaves.size());

    // ---- Draw background ----
    drawBackground();

    // ---- Sort branches back-to-front ----
    std::sort(branches.begin(), branches.end(),
              [](const Branch& a, const Branch& b){ return a.viewZ < b.viewZ; });
    std::sort(leaves.begin(), leaves.end(),
              [](const LeafQuad& a, const LeafQuad& b){ return a.viewZ < b.viewZ; });

    // ---- Render branches ----
    printf("Rendering...\n");
    for (auto& br : branches) {
        Vec2 s0 = cam.project(br.start);
        Vec2 s1 = cam.project(br.end);
        float z0 = cam.projZ(br.start);
        float z1 = cam.projZ(br.end);

        // Scale radius based on projection distance (simple perspective feel)
        float r0 = br.r0, r1 = br.r1;

        drawThickLine(s0.x, s0.y, s1.x, s1.y, z0, z1, r0, r1, br.color);
        // End cap
        drawCircle(s1.x, s1.y, z1, r1, (uint8_t)(br.color.x*255*0.85f),
                   (uint8_t)(br.color.y*255*0.85f), (uint8_t)(br.color.z*255*0.85f));
    }

    // ---- Render leaves ----
    for (auto& lf : leaves) {
        Vec2 sp = cam.project(lf.pos);
        float sz = cam.projZ(lf.pos);
        drawLeaf(sp.x, sp.y, sz, lf.size, lf.color);
    }

    // ---- Save output ----
    const char* outFile = "lsystem_tree_output.png";
    if (!stbi_write_png(outFile, W, H, 3, fb, W*3)) {
        fprintf(stderr, "ERROR: failed to write %s\n", outFile);
        return 1;
    }
    printf("Saved: %s\n", outFile);

    // ---- Print stats ----
    long total = 0;
    for (int y=0; y<H; y++)
        for (int x=0; x<W; x++)
            total += fb[y][x][0] + fb[y][x][1] + fb[y][x][2];
    float mean = (float)total / (W*H*3);
    printf("Pixel mean: %.1f\n", mean);
    printf("Done!\n");
    return 0;
}
