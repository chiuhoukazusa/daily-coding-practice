/**
 * Cloth Simulation - Mass-Spring System
 * 布料模拟：质点弹簧系统
 *
 * 技术特点：
 * - Verlet积分（位置Verlet，稳定性好）
 * - 三种弹簧：结构弹簧/剪切弹簧/弯曲弹簧
 * - 重力 + 风力
 * - 球体碰撞检测与响应
 * - 约束迭代
 * - 软光栅化三面板渲染（初始/中途/末尾）
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>

// ============================================================
//  Image
// ============================================================
struct Color {
    float r, g, b;
    Color(float r=0, float g=0, float b=0): r(r), g(g), b(b) {}
    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Color operator*(float t) const { return {r*t, g*t, b*t}; }
    Color lerp(const Color& o, float t) const { return *this*(1-t) + o*t; }
};

struct Image {
    int W, H;
    std::vector<Color> pixels;
    std::vector<float> zbuf;
    Image(int w, int h): W(w), H(h), pixels(w*h), zbuf(w*h, 1e30f) {}
    Color& at(int x, int y) { return pixels[y*W+x]; }
    const Color& at(int x, int y) const { return pixels[y*W+x]; }
    void fill(const Color& c) { for (auto& p : pixels) p = c; }
    void savePPM(const std::string& path) const {
        FILE* f = fopen(path.c_str(), "wb");
        fprintf(f, "P6\n%d %d\n255\n", W, H);
        for (const auto& c : pixels) {
            uint8_t r = (uint8_t)std::min(255.0f, std::max(0.0f, c.r*255.0f));
            uint8_t g = (uint8_t)std::min(255.0f, std::max(0.0f, c.g*255.0f));
            uint8_t b = (uint8_t)std::min(255.0f, std::max(0.0f, c.b*255.0f));
            fwrite(&r,1,1,f); fwrite(&g,1,1,f); fwrite(&b,1,1,f);
        }
        fclose(f);
    }
};

// ============================================================
//  Vec3
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float len2() const { return x*x+y*y+z*z; }
    float len() const { return sqrtf(len2()); }
    Vec3 normalized() const { float l=len(); return l>1e-8f ? *this/l : Vec3(); }
};
Vec3 operator*(float t, const Vec3& v) { return v*t; }

// ============================================================
//  Particle
// ============================================================
struct Particle {
    Vec3 pos, prevPos, acc;
    bool pinned;
    Vec3 pinnedPos;
    Particle(): pos(), prevPos(), acc(), pinned(false), pinnedPos() {}
    Particle(Vec3 p, bool pin=false): pos(p), prevPos(p), acc(), pinned(pin), pinnedPos(p) {}

    void addForce(const Vec3& f) { if (!pinned) acc += f; }

    void step(float dt) {
        if (pinned) { pos=pinnedPos; prevPos=pinnedPos; acc=Vec3(); return; }
        Vec3 vel = (pos - prevPos) * 0.98f;
        Vec3 next = pos + vel + acc*(dt*dt);
        prevPos = pos;
        pos = next;
        acc = Vec3();
    }
};

// ============================================================
//  Spring
// ============================================================
struct Spring {
    int a, b;
    float restLen, stiffness;
    Spring(int a, int b, float l, float k): a(a), b(b), restLen(l), stiffness(k) {}
};

// ============================================================
//  Cloth
// ============================================================
struct Cloth {
    int rows, cols;
    std::vector<Particle> particles;
    std::vector<Spring> springs;

    int idx(int r, int c) const { return r*cols+c; }

    void init(int rows, int cols, float spacing) {
        this->rows = rows;
        this->cols = cols;
        particles.resize(rows*cols);

        // Cloth hangs from top edge, centered at origin
        // X: -half to +half, Y: from top down, Z=0
        float halfW = (cols-1)*spacing*0.5f;
        float halfH = (rows-1)*spacing*0.5f;

        for (int r=0; r<rows; r++) {
            for (int c=0; c<cols; c++) {
                float x = -halfW + c*spacing;
                float y = halfH - r*spacing;
                // Pin top-left and top-right corners + a few more top pins
                bool pin = (r==0) && (c==0 || c==cols-1 || c==cols/4 || c==cols*3/4);
                particles[idx(r,c)] = Particle(Vec3(x, y, 0), pin);
            }
        }

        auto addS = [&](int a, int b, float k) {
            float l = (particles[a].pos - particles[b].pos).len();
            springs.emplace_back(a, b, l, k);
        };
        for (int r=0; r<rows; r++) {
            for (int c=0; c<cols; c++) {
                if (c+1 < cols) addS(idx(r,c), idx(r,c+1), 0.95f);  // structural H
                if (r+1 < rows) addS(idx(r,c), idx(r+1,c), 0.95f);  // structural V
                if (r+1<rows && c+1<cols) addS(idx(r,c), idx(r+1,c+1), 0.8f); // shear
                if (r+1<rows && c-1>=0)   addS(idx(r,c), idx(r+1,c-1), 0.8f); // shear
                if (c+2 < cols) addS(idx(r,c), idx(r,c+2), 0.7f);   // bend H
                if (r+2 < rows) addS(idx(r,c), idx(r+2,c), 0.7f);   // bend V
            }
        }
    }

    void solveConstraints(int iter) {
        for (int i=0; i<iter; i++) {
            for (auto& s : springs) {
                Particle& pa = particles[s.a];
                Particle& pb = particles[s.b];
                Vec3 d = pb.pos - pa.pos;
                float l = d.len();
                if (l < 1e-8f) continue;
                float diff = (l - s.restLen) / l * 0.5f * s.stiffness;
                Vec3 corr = d * diff;
                if (!pa.pinned) pa.pos += corr;
                if (!pb.pinned) pb.pos -= corr;
            }
        }
    }

    void step(float dt, const Vec3& gravity, const Vec3& wind,
              const Vec3& sphCenter, float sphR, int cIter) {
        for (auto& p : particles) { p.addForce(gravity); p.addForce(wind); }
        for (auto& p : particles) p.step(dt);
        solveConstraints(cIter);
        // Sphere collision
        for (auto& p : particles) {
            if (p.pinned) continue;
            Vec3 d = p.pos - sphCenter;
            float dist = d.len();
            if (dist < sphR + 0.015f)
                p.pos = sphCenter + d.normalized() * (sphR + 0.015f);
        }
        // Ground
        for (auto& p : particles) {
            if (!p.pinned && p.pos.y < -3.5f) p.pos.y = -3.5f;
        }
    }
};

// ============================================================
//  Projection
// ============================================================
struct Camera {
    Vec3 eye, center, up;
    float fovY, aspect, znear;
    int W, H;
    // Precomputed
    Vec3 forward, right, actualUp;
    float tanHalf;

    Camera(Vec3 e, Vec3 c, Vec3 u, float fovY, int W, int H)
        : eye(e), center(c), up(u), fovY(fovY), aspect((float)W/H), znear(0.01f), W(W), H(H) {
        forward = (c-e).normalized();
        right = forward.cross(u).normalized();
        actualUp = right.cross(forward);
        tanHalf = tanf(fovY*0.5f*3.14159265f/180.0f);
    }

    bool project(const Vec3& world, float& sx, float& sy, float& sz) const {
        Vec3 d = world - eye;
        float ex = d.dot(right);
        float ey = d.dot(actualUp);
        float ez = d.dot(forward);
        if (ez < znear) return false;
        float px = ex / (ez * tanHalf * aspect);
        float py = ey / (ez * tanHalf);
        sx = (px+1.0f)*0.5f*W;
        sy = (1.0f-(py+1.0f)*0.5f)*H;
        sz = ez;
        return sx>=-10 && sx<W+10 && sy>=-10 && sy<H+10;
    }
};

// ============================================================
//  Drawing
// ============================================================
void drawLine(Image& img, float x0, float y0, float x1, float y1, const Color& col) {
    int ix0=(int)x0, iy0=(int)y0, ix1=(int)x1, iy1=(int)y1;
    int dx=abs(ix1-ix0), dy=abs(iy1-iy0);
    int sx=ix0<ix1?1:-1, sy=iy0<iy1?1:-1;
    int err=dx-dy;
    for (int steps=0; steps<2000; steps++) {
        if (ix0>=0&&ix0<img.W&&iy0>=0&&iy0<img.H) img.at(ix0,iy0)=col;
        if (ix0==ix1&&iy0==iy1) break;
        int e2=2*err;
        if (e2>-dy){err-=dy;ix0+=sx;}
        if (e2< dx){err+=dx;iy0+=sy;}
    }
}

void fillCircle(Image& img, float cx, float cy, float r, const Color& col) {
    int x0=(int)(cx-r-1), x1=(int)(cx+r+1);
    int y0=(int)(cy-r-1), y1=(int)(cy+r+1);
    for (int y=y0; y<=y1; y++) for (int x=x0; x<=x1; x++) {
        if (x<0||x>=img.W||y<0||y>=img.H) continue;
        float dx=x-cx, dy=y-cy;
        if (dx*dx+dy*dy<=r*r) img.at(x,y)=col;
    }
}

void drawBackground(Image& img) {
    // Sky gradient
    for (int y=0; y<img.H; y++) {
        float t=(float)y/img.H;
        Color sky = Color(0.35f,0.55f,0.85f)*(1-t) + Color(0.75f,0.82f,0.95f)*t;
        for (int x=0; x<img.W; x++) img.at(x,y)=sky;
    }
    // Ground stripe
    for (int y=(int)(img.H*0.82f); y<img.H; y++) {
        float t=(float)(y-img.H*0.82f)/(img.H*0.18f);
        Color ground = Color(0.4f,0.6f,0.35f)*(1-t) + Color(0.3f,0.5f,0.25f)*t;
        for (int x=0; x<img.W; x++) img.at(x,y)=ground;
    }
}

void renderClothAndSphere(Image& img, const Cloth& cloth, const Camera& cam,
                          const Vec3& sphCenter, float sphR, const Vec3& lightDir) {
    drawBackground(img);

    // Draw sphere
    float sx,sy,sz;
    if (cam.project(sphCenter, sx, sy, sz)) {
        float sx2=0,sy2=0,sz2=0;
        cam.project(sphCenter+Vec3(sphR,0,0), sx2,sy2,sz2);
        float screenR = fabsf(sx2-sx);
        int R=(int)(screenR+2);
        for (int dy=-R; dy<=R; dy++) for (int dx=-R; dx<=R; dx++) {
            int px=(int)sx+dx, py=(int)sy+dy;
            if (px<0||px>=img.W||py<0||py>=img.H) continue;
            float d=sqrtf((float)(dx*dx+dy*dy));
            if (d<=screenR) {
                float nx=dx/screenR, ny=-dy/screenR;
                float nz=sqrtf(std::max(0.0f,1.0f-nx*nx-ny*ny));
                Vec3 n(nx,ny,nz);
                float diff=std::max(0.0f, n.dot(lightDir.normalized()));
                float shad=0.2f+diff*0.8f;
                // Specular
                Vec3 viewDir=(sphCenter-cam.eye).normalized();
                Vec3 refl=(lightDir.normalized() - viewDir*(2.0f*lightDir.normalized().dot(viewDir))).normalized();
                float spec=powf(std::max(0.0f,refl.dot(Vec3(nx,ny,nz))),32.0f);
                Color col = Color(0.85f,0.3f,0.15f)*shad + Color(1,1,1)*spec*0.5f;
                img.at(px,py)=col;
            }
        }
    }

    // Draw cloth
    int rows=cloth.rows, cols=cloth.cols;
    // Draw every-other-row quad mesh for filled look
    for (int r=0; r<rows-1; r++) {
        for (int c=0; c<cols-1; c++) {
            // Get 4 corners
            float sx0=0,sy0=0,sz0=0, sx1=0,sy1=0,sz1=0, sx2=0,sy2=0,sz2=0, sx3=0,sy3=0,sz3=0;
            bool ok0 = cam.project(cloth.particles[cloth.idx(r,c)].pos,   sx0,sy0,sz0);
            bool ok1 = cam.project(cloth.particles[cloth.idx(r,c+1)].pos, sx1,sy1,sz1);
            bool ok2 = cam.project(cloth.particles[cloth.idx(r+1,c)].pos, sx2,sy2,sz2);
            bool ok3 = cam.project(cloth.particles[cloth.idx(r+1,c+1)].pos,sx3,sy3,sz3);
            if (!ok0||!ok1||!ok2||!ok3) continue;

            // Compute face normal for shading
            Vec3 p0=cloth.particles[cloth.idx(r,c)].pos;
            Vec3 p1=cloth.particles[cloth.idx(r,c+1)].pos;
            Vec3 p2=cloth.particles[cloth.idx(r+1,c)].pos;
            Vec3 n=(p1-p0).cross(p2-p0).normalized();
            float diff=std::max(0.1f, fabsf(n.dot(lightDir.normalized())));
            float shading=0.3f+0.7f*diff;

            // Checkerboard cloth color (blue/white)
            bool ck=((r+c)%2==0);
            Color baseCol = ck ? Color(0.15f,0.45f,0.85f) : Color(0.9f,0.9f,0.95f);
            Color col=baseCol*shading;

            // Draw two triangles (lines only - wireframe + fill effect)
            drawLine(img, sx0,sy0, sx1,sy1, col);
            drawLine(img, sx0,sy0, sx2,sy2, col);
            drawLine(img, sx1,sy1, sx3,sy3, col);
            drawLine(img, sx2,sy2, sx3,sy3, col);
        }
    }

    // Highlight pinned particles
    for (int c=0; c<cols; c++) {
        const Particle& p = cloth.particles[cloth.idx(0,c)];
        if (p.pinned) {
            float px,py,pz;
            if (cam.project(p.pos, px,py,pz))
                fillCircle(img, px,py,3.5f, Color(1.0f,0.9f,0.0f));
        }
    }
}

// Composite 3 frames side by side
Image composite3(const std::vector<Image>& frames) {
    int fw=frames[0].W, fh=frames[0].H;
    Image out(fw*3, fh);
    for (int i=0; i<3; i++) {
        const Image& src=frames[i];
        for (int y=0; y<fh; y++)
            for (int x=0; x<fw; x++)
                out.at(i*fw+x, y)=src.at(x,y);
    }
    return out;
}

// ============================================================
//  Main
// ============================================================
int main() {
    printf("=== Cloth Simulation - Mass-Spring System ===\n");
    fflush(stdout);

    const int W=480, H=360;
    const int ROWS=22, COLS=22;
    const float SPACING=0.18f;
    const int TOTAL_FRAMES=150;
    const int SUBSTEPS=12;
    const float DT=0.016f/SUBSTEPS;
    const int CITER=15;

    Vec3 gravity(0, -0.032f, 0); // tuned for visible draping
    Vec3 sphCenter(0.0f, -0.8f, 0.3f);
    float sphR = 0.65f;

    Cloth cloth;
    cloth.init(ROWS, COLS, SPACING);

    // Camera: look at cloth from front-slightly-above
    Camera cam(
        Vec3(0.0f, 0.5f, 5.5f),  // eye
        Vec3(0.0f, -0.3f, 0.0f), // center
        Vec3(0,1,0),
        52.0f, W, H
    );
    Vec3 lightDir = Vec3(1.0f, 2.0f, 1.5f).normalized();

    std::vector<Image> captured;
    int captureFrames[3] = {0, TOTAL_FRAMES/2, TOTAL_FRAMES-1};
    int ci=0;

    for (int frame=0; frame<TOTAL_FRAMES; frame++) {
        float t=(float)frame/TOTAL_FRAMES;
        // Wind: sinusoidal lateral push
        Vec3 wind(sinf(t*6.28f*2.5f)*0.0015f, 0, cosf(t*6.28f*1.8f)*0.0008f);

        for (int s=0; s<SUBSTEPS; s++)
            cloth.step(DT, gravity, wind, sphCenter, sphR, CITER);

        if (ci < 3 && frame == captureFrames[ci]) {
            Image img(W, H);
            renderClothAndSphere(img, cloth, cam, sphCenter, sphR, lightDir);
            captured.push_back(img);
            printf("  Captured frame %d/%d\n", frame+1, TOTAL_FRAMES);
            fflush(stdout);
            ci++;
        }
    }

    // Save composite
    Image out = composite3(captured);
    out.savePPM("cloth_output.ppm");
    printf("Saved: cloth_output.ppm (%dx%d)\n", out.W, out.H);
    printf("=== Done ===\n");
    return 0;
}
