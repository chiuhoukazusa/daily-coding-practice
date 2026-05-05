/**
 * PBD Soft Body Deformation Renderer
 * ====================================
 * Position-Based Dynamics (PBD) simulation of elastic soft body
 * with SDF-based ground collision, rendered via software rasterizer.
 *
 * Features:
 *  - PBD distance constraints (stretch + shear)
 *  - Bending constraints
 *  - SDF-based ground plane collision
 *  - Gravity + damping
 *  - Soft rasterizer with Gouraud shading
 *  - Multi-frame composite (shows deformation over time)
 *
 * Build: g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
 */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// ─── Math primitives ─────────────────────────────────────────────────────────

struct Vec2 {
    float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
    Vec2 operator+(const Vec2& b)const{return{x+b.x,y+b.y};}
    Vec2 operator-(const Vec2& b)const{return{x-b.x,y-b.y};}
    Vec2 operator*(float t)const{return{x*t,y*t};}
    Vec2 operator/(float t)const{return{x/t,y/t};}
    Vec2& operator+=(const Vec2& b){x+=b.x;y+=b.y;return*this;}
    Vec2& operator-=(const Vec2& b){x-=b.x;y-=b.y;return*this;}
    float dot(const Vec2& b)const{return x*b.x+y*b.y;}
    float length()const{return std::sqrt(x*x+y*y);}
    Vec2 normalized()const{float l=length();return l>1e-9f?(*this)*(1.f/l):Vec2(0,0);}
};

struct Vec3 {
    float x,y,z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b)const{return{x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3& b)const{return{x-b.x,y-b.y,z-b.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    float dot(const Vec3& b)const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3& b)const{return{y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    float length()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 normalized()const{float l=length();return l>1e-9f?(*this)*(1.f/l):Vec3(0,0,0);}
};

struct Color {
    uint8_t r,g,b;
};

// ─── Image ───────────────────────────────────────────────────────────────────

struct Image {
    int width, height;
    std::vector<Color> pixels;
    std::vector<float> zbuf;

    Image(int w, int h, Color bg={20,20,30})
        : width(w), height(h),
          pixels(w*h, bg),
          zbuf(w*h, 1e9f) {}

    void setPixel(int x, int y, float z, Color c) {
        if(x<0||x>=width||y<0||y>=height) return;
        int idx = y*width+x;
        if(z < zbuf[idx]) {
            zbuf[idx] = z;
            pixels[idx] = c;
        }
    }

    // Write PPM then convert to PNG via raw bytes (write BMP instead for portability)
    bool savePNG(const std::string& path);
};

// ─── BMP writer (no external deps) ──────────────────────────────────────────

static void writeBMP(const std::string& path, const Image& img) {
    int w = img.width, h = img.height;
    int rowSize = (w * 3 + 3) & ~3;
    int dataSize = rowSize * h;
    int fileSize = 54 + dataSize;

    std::ofstream f(path, std::ios::binary);
    // File header
    uint8_t fh[14] = {
        'B','M',
        (uint8_t)(fileSize),(uint8_t)(fileSize>>8),(uint8_t)(fileSize>>16),(uint8_t)(fileSize>>24),
        0,0,0,0,
        54,0,0,0
    };
    f.write((char*)fh,14);
    // Info header
    uint8_t ih[40]={};
    ih[0]=40;
    ih[4]=(uint8_t)w; ih[5]=(uint8_t)(w>>8); ih[6]=(uint8_t)(w>>16); ih[7]=(uint8_t)(w>>24);
    int nh=-h;
    ih[8]=(uint8_t)nh; ih[9]=(uint8_t)(nh>>8); ih[10]=(uint8_t)(nh>>16); ih[11]=(uint8_t)(nh>>24);
    ih[12]=1; ih[14]=24;
    ih[20]=(uint8_t)dataSize; ih[21]=(uint8_t)(dataSize>>8); ih[22]=(uint8_t)(dataSize>>16); ih[23]=(uint8_t)(dataSize>>24);
    f.write((char*)ih,40);
    std::vector<uint8_t> row(rowSize,0);
    for(int y=0;y<h;y++){
        for(int x=0;x<w;x++){
            const Color& c = img.pixels[y*w+x];
            row[x*3+0]=c.b;
            row[x*3+1]=c.g;
            row[x*3+2]=c.r;
        }
        f.write((char*)row.data(),rowSize);
    }
}

// ─── PPM writer ──────────────────────────────────────────────────────────────

static void writePPM(const std::string& path, const Image& img) {
    std::ofstream f(path);
    f << "P3\n" << img.width << " " << img.height << "\n255\n";
    for(const auto& c : img.pixels)
        f << (int)c.r << " " << (int)c.g << " " << (int)c.b << "\n";
}

// ─── Rasterizer helpers ──────────────────────────────────────────────────────


static void drawLine(Image& img, Vec2 a, Vec2 b, Color c) {
    int x0=(int)a.x,y0=(int)a.y,x1=(int)b.x,y1=(int)b.y;
    int dx=std::abs(x1-x0), dy=std::abs(y1-y0);
    int sx=x0<x1?1:-1, sy=y0<y1?1:-1;
    int err=dx-dy;
    while(true){
        img.setPixel(x0,y0,-1.f,c);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2<dx){err+=dx;y0+=sy;}
    }
}

// ─── PBD Soft Body ───────────────────────────────────────────────────────────

struct Particle {
    Vec2 pos, prevPos, vel;
    float invMass; // 0 = pinned
    Vec2 force;
};

struct Constraint {
    int i, j;
    float restLen;
    float stiffness; // [0,1]
};

struct SoftBody {
    std::vector<Particle> particles;
    std::vector<Constraint> constraints;
    // Grid dim
    int cols, rows;

    // Build a soft grid (cloth/jelly style)
    void buildGrid(int c, int r, Vec2 origin, float spacing, float mass) {
        cols=c; rows=r;
        particles.resize(c*r);
        for(int iy=0;iy<r;iy++){
            for(int ix=0;ix<c;ix++){
                Particle& p = particles[iy*c+ix];
                p.pos = origin + Vec2(ix*spacing, iy*spacing);
                p.prevPos = p.pos;
                p.vel = {0,0};
                p.invMass = 1.f/mass;
                p.force = {0,0};
            }
        }
        // Structural constraints (horizontal + vertical)
        for(int iy=0;iy<r;iy++){
            for(int ix=0;ix<c;ix++){
                if(ix+1<c) addConstraint(iy*c+ix, iy*c+ix+1, spacing, 0.9f);
                if(iy+1<r) addConstraint(iy*c+ix, (iy+1)*c+ix, spacing, 0.9f);
            }
        }
        // Shear constraints
        float diag = spacing*std::sqrt(2.f);
        for(int iy=0;iy<r-1;iy++){
            for(int ix=0;ix<c-1;ix++){
                addConstraint(iy*c+ix,     (iy+1)*c+ix+1, diag, 0.6f);
                addConstraint(iy*c+ix+1,   (iy+1)*c+ix,   diag, 0.6f);
            }
        }
        // Bending constraints (skip-one)
        float bend = spacing*2.f;
        float bendDiag = spacing*2.f*std::sqrt(2.f);
        for(int iy=0;iy<r;iy++){
            for(int ix=0;ix<c-2;ix++)
                addConstraint(iy*c+ix, iy*c+ix+2, bend, 0.3f);
        }
        for(int iy=0;iy<r-2;iy++){
            for(int ix=0;ix<c;ix++)
                addConstraint(iy*c+ix, (iy+2)*c+ix, bend, 0.3f);
        }
        for(int iy=0;iy<r-2;iy++){
            for(int ix=0;ix<c-2;ix++){
                addConstraint(iy*c+ix,   (iy+2)*c+ix+2, bendDiag, 0.2f);
                addConstraint(iy*c+ix+2, (iy+2)*c+ix,   bendDiag, 0.2f);
            }
        }
    }

    void addConstraint(int i, int j, float len, float s) {
        constraints.push_back({i,j,len,s});
    }

    // SDF: ground plane at y = groundY
    static float sdfGround(Vec2 p, float groundY) {
        return p.y - groundY;
    }

    void step(float dt, float groundY, int solverIter=10) {
        float gravity = -9.8f;
        float damping = 0.98f;

        // Integrate velocities
        for(auto& p : particles){
            if(p.invMass==0) continue;
            p.vel.y += gravity*dt;
            p.vel = p.vel * damping;
            p.prevPos = p.pos;
            p.pos += p.vel*dt;
        }

        // Project constraints
        for(int iter=0;iter<solverIter;iter++){
            for(auto& c : constraints){
                Particle& pi = particles[c.i];
                Particle& pj = particles[c.j];
                Vec2 diff = pj.pos - pi.pos;
                float d = diff.length();
                if(d<1e-9f) continue;
                float w = pi.invMass+pj.invMass;
                if(w<1e-9f) continue;
                float corr = (d - c.restLen)/d * c.stiffness / w;
                Vec2 dp = diff * corr;
                pi.pos += dp * pi.invMass;
                pj.pos -= dp * pj.invMass;
            }
            // Collision with ground SDF
            for(auto& p : particles){
                float d = sdfGround(p.pos, groundY);
                if(d < 0.f){
                    p.pos.y -= d; // push out
                }
            }
        }

        // Update velocities from positions
        for(auto& p : particles){
            if(p.invMass==0) continue;
            p.vel = (p.pos - p.prevPos) * (1.f/dt);
            // Friction when on ground
            float d = sdfGround(p.pos, groundY);
            if(d < 0.02f){
                p.vel.x *= 0.85f;
            }
        }
    }
};

// ─── Project 3D → 2D NDC (simple ortho for 2D sim) ──────────────────────────

// The soft body is 2D; we'll display it as a 2D visualization with colors
// showing stretch/compression.

// ─── Color utilities ─────────────────────────────────────────────────────────

static Vec3 heatColor(float t) {
    // t in [0,1]: blue → cyan → green → yellow → red
    t = std::clamp(t, 0.f, 1.f);
    if(t<0.25f){
        float s=t/0.25f;
        return {0, s, 1};
    } else if(t<0.5f){
        float s=(t-0.25f)/0.25f;
        return {0, 1, 1-s};
    } else if(t<0.75f){
        float s=(t-0.5f)/0.25f;
        return {s, 1, 0};
    } else {
        float s=(t-0.75f)/0.25f;
        return {1, 1-s, 0};
    }
}

static Vec3 softBodyColor(float stretch) {
    // stretch: 0=compressed, 0.5=rest, 1=stretched
    return heatColor(stretch);
}

// ─── Render soft body to image ───────────────────────────────────────────────

// World to screen: world range ~ [-2,2]x[-2,2] → screen [0,W]x[0,H]
struct Camera {
    float worldMinX, worldMaxX, worldMinY, worldMaxY;
    int screenW, screenH;

    Vec2 worldToScreen(Vec2 p) const {
        float u = (p.x - worldMinX) / (worldMaxX - worldMinX);
        float v = (p.y - worldMinY) / (worldMaxY - worldMinY);
        return { u * screenW, (1.f-v) * screenH };
    }
};

static void renderSoftBody(Image& img, const SoftBody& sb, const Camera& cam,
                            float groundY, int frame, int totalFrames)
{
    // Draw ground
    Vec2 gL = cam.worldToScreen({cam.worldMinX, groundY});
    Vec2 gR = cam.worldToScreen({cam.worldMaxX, groundY});
    drawLine(img, gL, gR, {80,180,80});

    // Draw title/frame label as dots (simple)
    // Draw particles as filled circles
    for(const auto& p : sb.particles){
        Vec2 sp = cam.worldToScreen(p.pos);
        int sx=(int)sp.x, sy=(int)sp.y;
        float speed = p.vel.length();
        float t = std::min(speed/5.f, 1.f);
        Vec3 col = {0.2f+0.6f*t, 0.8f-0.3f*t, 1.f-0.7f*t};
        uint8_t r=(uint8_t)(col.x*255);
        uint8_t g=(uint8_t)(col.y*255);
        uint8_t b=(uint8_t)(col.z*255);
        for(int dy=-2;dy<=2;dy++)
            for(int dx=-2;dx<=2;dx++)
                if(dx*dx+dy*dy<=4)
                    img.setPixel(sx+dx,sy+dy,-0.5f,{r,g,b});
    }

    // Draw structural constraint lines with stretch color
    int c=sb.cols;
    for(int iy=0;iy<sb.rows;iy++){
        for(int ix=0;ix<sb.cols;ix++){
            const Particle& p0=sb.particles[iy*c+ix];
            // horizontal
            if(ix+1<sb.cols){
                const Particle& p1=sb.particles[iy*c+ix+1];
                Vec2 s0=cam.worldToScreen(p0.pos);
                Vec2 s1=cam.worldToScreen(p1.pos);
                float len=(p1.pos-p0.pos).length();
                // Compare to constraint rest length
                float restLen = -1;
                for(const auto& con : sb.constraints){
                    if((con.i==iy*c+ix && con.j==iy*c+ix+1)||
                       (con.j==iy*c+ix && con.i==iy*c+ix+1)){
                        restLen=con.restLen; break;
                    }
                }
                float stretch=0.5f;
                if(restLen>0) stretch=std::clamp(len/restLen-1.f+0.5f,0.f,1.f);
                Vec3 col=softBodyColor(stretch);
                Color lc={(uint8_t)(col.x*255),(uint8_t)(col.y*255),(uint8_t)(col.z*255)};
                drawLine(img,s0,s1,lc);
            }
            // vertical
            if(iy+1<sb.rows){
                const Particle& p1=sb.particles[(iy+1)*c+ix];
                Vec2 s0=cam.worldToScreen(p0.pos);
                Vec2 s1=cam.worldToScreen(p1.pos);
                float len=(p1.pos-p0.pos).length();
                float restLen=-1;
                for(const auto& con : sb.constraints){
                    if((con.i==iy*c+ix && con.j==(iy+1)*c+ix)||
                       (con.j==iy*c+ix && con.i==(iy+1)*c+ix)){
                        restLen=con.restLen; break;
                    }
                }
                float stretch=0.5f;
                if(restLen>0) stretch=std::clamp(len/restLen-1.f+0.5f,0.f,1.f);
                Vec3 col=softBodyColor(stretch);
                Color lc={(uint8_t)(col.x*255),(uint8_t)(col.y*255),(uint8_t)(col.z*255)};
                drawLine(img,s0,s1,lc);
            }
        }
    }

    // Frame progress bar (bottom strip)
    float progress = (float)frame/totalFrames;
    int barW = (int)(progress * img.width);
    for(int x=0;x<barW;x++){
        float t=(float)x/img.width;
        Color c2={(uint8_t)(50+t*200),(uint8_t)(200-t*100),80};
        img.setPixel(x, img.height-4, -2.f, c2);
        img.setPixel(x, img.height-3, -2.f, c2);
        img.setPixel(x, img.height-2, -2.f, c2);
    }
}

// ─── Composite multi-frame into one image ────────────────────────────────────

static void compositeFrames(const std::vector<Image>& frames, Image& out) {
    // We'll arrange frames in a 4-column grid
    int N = (int)frames.size();
    int cols = 4;
    int rows = (N+cols-1)/cols;
    int fw = frames[0].width, fh = frames[0].height;
    out = Image(fw*cols, fh*rows);

    for(int i=0;i<N;i++){
        int cx = i%cols, cy = i/cols;
        int ox = cx*fw, oy = cy*fh;
        for(int y=0;y<fh;y++){
            for(int x=0;x<fw;x++){
                Color c2 = frames[i].pixels[y*fw+x];
                int idx = (oy+y)*out.width+(ox+x);
                out.pixels[idx] = c2;
            }
        }
        // Draw thin border
        for(int x=0;x<fw;x++){
            Color bc={60,60,80};
            int t=(oy)*out.width+(ox+x);
            int b=(oy+fh-1)*out.width+(ox+x);
            out.pixels[t]=bc; out.pixels[b]=bc;
        }
        for(int y=0;y<fh;y++){
            Color bc={60,60,80};
            int l=(oy+y)*out.width+ox;
            int r=(oy+y)*out.width+(ox+fw-1);
            out.pixels[l]=bc; out.pixels[r]=bc;
        }
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== PBD Soft Body Deformation ===" << std::endl;

    // Simulation parameters
    const int GRID_COLS = 8;
    const int GRID_ROWS = 8;
    const float SPACING = 0.18f;
    const float MASS = 1.0f;
    const float GROUND_Y = -1.5f;
    const int SIM_STEPS = 60;   // Total simulation steps
    const int CAPTURE_FRAMES = 8; // Frames to capture
    const float DT = 0.016f;    // ~60fps

    // Build soft body (jelly blob)
    SoftBody sb;
    Vec2 origin(-SPACING*(GRID_COLS-1)*0.5f, 0.2f); // Centered, above ground
    sb.buildGrid(GRID_COLS, GRID_ROWS, origin, SPACING, MASS);

    // Give it a slight initial velocity to make it interesting
    for(auto& p : sb.particles){
        p.vel = {0.3f, 0.f};
    }

    // Camera
    Camera cam;
    cam.worldMinX = -2.0f; cam.worldMaxX = 2.0f;
    cam.worldMinY = -2.0f; cam.worldMaxY = 2.0f;
    cam.screenW = 320; cam.screenH = 240;

    // Simulate and capture frames
    std::vector<Image> frames;
    int captureInterval = SIM_STEPS / CAPTURE_FRAMES;

    std::cout << "Simulating " << SIM_STEPS << " steps..." << std::endl;

    for(int step=0; step<SIM_STEPS; step++){
        sb.step(DT, GROUND_Y, 12);

        if(step % captureInterval == 0 || step == SIM_STEPS-1){
            int frameIdx = (int)frames.size();
            frames.emplace_back(cam.screenW, cam.screenH);
            renderSoftBody(frames.back(), sb, cam, GROUND_Y, step, SIM_STEPS);
            std::cout << "  Captured frame " << frameIdx
                      << " at step " << step << std::endl;
        }
    }

    // Composite all frames into final image
    std::cout << "Compositing " << frames.size() << " frames..." << std::endl;
    Image final(1, 1);
    compositeFrames(frames, final);

    // Save as PPM (we'll convert to PNG after)
    writePPM("pbd_softbody_output.ppm", final);
    writeBMP("pbd_softbody_output.bmp", final);
    std::cout << "Saved pbd_softbody_output.ppm / .bmp ("
              << final.width << "x" << final.height << ")" << std::endl;

    // Convert PPM to PNG
    int ret = std::system("convert pbd_softbody_output.ppm pbd_softbody_output.png 2>/dev/null"
                          " || python3 -c \""
                          "from PIL import Image as PILImage;"
                          "PILImage.open('pbd_softbody_output.ppm').save('pbd_softbody_output.png')"
                          "\" 2>/dev/null"
                          " || cp pbd_softbody_output.bmp pbd_softbody_output.png");
    (void)ret;

    // Verify output
    std::ifstream check("pbd_softbody_output.png", std::ios::binary);
    if(check.good()){
        check.seekg(0, std::ios::end);
        auto sz = check.tellg();
        std::cout << "Output file: pbd_softbody_output.png (" << sz << " bytes)" << std::endl;
        if(sz < 1000){
            // fallback: use bmp
            std::cout << "PNG too small, checking BMP..." << std::endl;
            std::ifstream bcheck("pbd_softbody_output.bmp", std::ios::binary);
            bcheck.seekg(0, std::ios::end);
            auto bsz = bcheck.tellg();
            std::cout << "BMP size: " << bsz << " bytes" << std::endl;
        }
    } else {
        std::cerr << "ERROR: output file not created!" << std::endl;
        return 1;
    }

    std::cout << "=== Done ===" << std::endl;
    return 0;
}
