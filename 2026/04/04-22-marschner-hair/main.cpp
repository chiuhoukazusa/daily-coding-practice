/*
 * Marschner Hair Rendering
 * 
 * Implements the Marschner et al. 2003 hair shading model:
 * - R: surface reflection
 * - TT: two-fold transmission (light enters and exits hair)
 * - TRT: transmission-reflection-transmission (internal reflection + caustic)
 * 
 * Each lobe is factored into:
 *   S(theta_i, phi_i, theta_r, phi_r) = M(theta_h) * N(phi) / cos^2(theta_d)
 *
 * where M is the longitudinal scattering and N is the azimuthal scattering.
 *
 * Scene: multiple hair strands lit by directional + ambient light, rendered
 * to a 800x600 PNG using a software rasterizer.
 */

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>

// ---- Math types --------------------------------------------------------
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o)       const { return x*o.x + y*o.y + z*o.z; }
    float len2()                   const { return dot(*this); }
    float len()                    const { return std::sqrt(len2()); }
    Vec3 norm()                    const { float l=len(); return (l>1e-9f)?*this/l:Vec3(0,1,0); }
};
inline Vec3 cross(const Vec3& a, const Vec3& b){
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t){ return a*(1-t)+b*t; }

// ---- Color helpers -----------------------------------------------------
inline float clamp01(float v){ return std::max(0.f, std::min(1.f, v)); }
inline Vec3  clamp01(Vec3 v){ return {clamp01(v.x), clamp01(v.y), clamp01(v.z)}; }

// Gamma encode
inline unsigned char toU8(float v){
    float g = std::pow(clamp01(v), 1.f/2.2f);
    return (unsigned char)(g*255.f+.5f);
}

// ---- Simple PNG writer (uncompressed, using lodepng-style raw DEFLATE) -
// We write a minimal PNG with zlib DEFLATE level-0 blocks for simplicity.
#include <fstream>

static uint32_t crc32_table[256];
static bool     crc32_init_done = false;
static void init_crc32(){
    for(uint32_t n=0;n<256;++n){
        uint32_t c=n;
        for(int k=0;k<8;++k) c=(c&1)?(0xEDB88320u^(c>>1)):(c>>1);
        crc32_table[n]=c;
    }
    crc32_init_done=true;
}
static uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc=0xFFFFFFFFu){
    if(!crc32_init_done) init_crc32();
    for(size_t i=0;i<len;++i) crc=crc32_table[(crc^data[i])&0xFF]^(crc>>8);
    return crc^0xFFFFFFFFu;
}
static uint32_t adler32(const uint8_t* d, size_t n){
    uint32_t a=1,b=0;
    for(size_t i=0;i<n;++i){a=(a+d[i])%65521;b=(b+a)%65521;}
    return (b<<16)|a;
}

static void write32be(std::vector<uint8_t>& v, uint32_t x){
    v.push_back((x>>24)&0xFF);v.push_back((x>>16)&0xFF);
    v.push_back((x>>8)&0xFF); v.push_back(x&0xFF);
}
static void writeChunk(std::vector<uint8_t>& png, const char type[4], const std::vector<uint8_t>& data){
    write32be(png,(uint32_t)data.size());
    size_t typeStart=png.size();
    png.insert(png.end(),type,type+4);
    png.insert(png.end(),data.begin(),data.end());
    uint32_t c=crc32(png.data()+typeStart, 4+data.size());
    write32be(png,c);
}

static bool savePNG(const char* path, const uint8_t* rgb, int W, int H){
    // Build raw image data (filter byte 0 per scanline)
    std::vector<uint8_t> raw;
    raw.reserve((1+W*3)*H);
    for(int y=0;y<H;++y){
        raw.push_back(0); // filter none
        for(int x=0;x<W;++x){
            raw.push_back(rgb[(y*W+x)*3+0]);
            raw.push_back(rgb[(y*W+x)*3+1]);
            raw.push_back(rgb[(y*W+x)*3+2]);
        }
    }
    // Deflate store (no compression): split into 65535-byte blocks
    std::vector<uint8_t> deflate;
    deflate.push_back(0x78); // zlib CMF
    deflate.push_back(0x01); // zlib FLG (no dict, level 0)
    size_t pos=0;
    while(pos<raw.size()){
        size_t blockLen=std::min((size_t)65535, raw.size()-pos);
        bool last=(pos+blockLen==raw.size());
        deflate.push_back(last?0x01:0x00); // BFINAL|BTYPE
        // LEN and NLEN (little-endian)
        deflate.push_back(blockLen&0xFF);
        deflate.push_back((blockLen>>8)&0xFF);
        deflate.push_back(~blockLen&0xFF);
        deflate.push_back((~blockLen>>8)&0xFF);
        deflate.insert(deflate.end(), raw.data()+pos, raw.data()+pos+blockLen);
        pos+=blockLen;
    }
    uint32_t adl=adler32(raw.data(),raw.size());
    write32be(deflate,adl);

    std::vector<uint8_t> png;
    // PNG signature
    static const uint8_t sig[8]={137,80,78,71,13,10,26,10};
    png.insert(png.end(),sig,sig+8);
    // IHDR
    std::vector<uint8_t> ihdr;
    write32be(ihdr,W); write32be(ihdr,H);
    ihdr.push_back(8); // bit depth
    ihdr.push_back(2); // color type: RGB
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    writeChunk(png,"IHDR",ihdr);
    // IDAT
    writeChunk(png,"IDAT",deflate);
    // IEND
    writeChunk(png,"IEND",{});

    std::ofstream f(path, std::ios::binary);
    if(!f) return false;
    f.write((char*)png.data(),png.size());
    return true;
}

// ---- Marschner Hair Model -----------------------------------------------
// Reference: Marschner et al. "Light Scattering from Human Hair Fibers" SIGGRAPH 2003
//
// Convention:
//   theta_i: angle of incidence (from tangent plane, i.e., complement of angle to tangent)
//   theta_r: angle of reflection
//   phi:     azimuthal angle difference (0 = forward scatter)
//   eta:     index of refraction (~1.55 for human hair)
//   alpha_p: longitudinal shift for lobe p (R: -alpha, TT: -alpha/2, TRT: -3*alpha/2)
//   beta_p:  longitudinal width for lobe p

static const float PI  = 3.14159265358979f;
static const float PI2 = PI*2.f;

inline float sqr(float x){ return x*x; }

// Gaussian distribution G(beta, theta)
inline float gaussianM(float beta, float x){
    return std::exp(-x*x/(2.f*beta*beta)) / (beta * std::sqrt(2.f*PI));
}

// Fresnel reflectance (dielectric, unpolarized)
static float fresnel(float cosT, float eta){
    float sinT2 = 1.f - cosT*cosT;
    if(sinT2/sqr(eta) > 1.f) return 1.f; // TIR
    float cosT2 = std::sqrt(std::max(0.f, 1.f - sinT2/sqr(eta)));
    float rs = (cosT - eta*cosT2)/(cosT + eta*cosT2);
    float rp = (eta*cosT - cosT2)/(eta*cosT + cosT2);
    return .5f*(rs*rs + rp*rp);
}

// Azimuthal scattering for each lobe (exact Marschner formulation)
// phi(p, h) = 2*p*asin(h) - 2*p*asin(h/eta') + p*pi  (simplified for each lobe)
// We compute N_p(phi) numerically by summing over caustic contributions.
static float Np(int p, float phi, float cosTD, float eta){
    float etaT = std::sqrt(eta*eta - 1.f + cosTD*cosTD) / cosTD; // eta' adjusted for tilt
    // For each p, phi(p,h) = 2p*gamma_t - 2p*arcsin(h/etaT) + p*pi
    // We invert: find all h in [-1,1] such that phi(p,h) == phi
    // phi(p,h) = 2*arcsin(h) - 2*p*(arcsin(h/etaT)) + p*pi  [Marschner eq. 5]
    // dPhi/dh = 2/sqrt(1-h^2) - 2p/(etaT*sqrt(1-(h/etaT)^2))
    // We do numerical root finding over [-1,1]
    
    float result = 0.f;
    const int STEPS = 200;
    float prevH   = -1.f + 1.f/STEPS;
    float gammaI0 = std::asin(std::max(-1.f, std::min(1.f, prevH)));
    float gammaT0 = std::asin(std::max(-1.f, std::min(1.f, prevH/etaT)));
    float phi0 = 2.f*p*gammaT0 - 2.f*gammaI0 + p*PI;
    // wrap phi0 into [-pi, pi]
    while(phi0 >  PI) phi0 -= PI2;
    while(phi0 < -PI) phi0 += PI2;
    float f0 = phi0 - phi;
    while(f0 >  PI) f0 -= PI2;
    while(f0 < -PI) f0 += PI2;

    for(int i=1; i<=STEPS; ++i){
        float h = -1.f + (2.f*i)/STEPS;
        h = std::max(-0.9999f, std::min(0.9999f, h));
        float gammaI = std::asin(h);
        float hT = h/etaT;
        hT = std::max(-0.9999f, std::min(0.9999f, hT));
        float gammaT = std::asin(hT);
        float phiVal = 2.f*p*gammaT - 2.f*gammaI + p*PI;
        while(phiVal >  PI) phiVal -= PI2;
        while(phiVal < -PI) phiVal += PI2;
        float f1 = phiVal - phi;
        while(f1 >  PI) f1 -= PI2;
        while(f1 < -PI) f1 += PI2;

        if(f0 * f1 < 0.f){
            // root in [prevH, h]
            // Estimate |dphi/dh| at midpoint
            float hm = (prevH+h)*0.5f;
            hm = std::max(-0.9999f, std::min(0.9999f, hm));
            float dphidh = 2.f/std::sqrt(1.f - hm*hm)
                         - 2.f*p / (etaT * std::sqrt(std::max(1e-6f, 1.f - (hm/etaT)*(hm/etaT))));
            // Attenuation A_p(h)
            float fr = fresnel(std::cos(std::asin(hm)), eta);
            float fp;
            if(p == 0){
                fp = fr;
            } else if(p == 1){
                fp = sqr(1.f-fr)*std::pow(std::max(0.f, 1.f-fr), (float)(p-1));
                // transmission: (1-F)^2 * F^(p-1) => for TT p=1: (1-F)^2
                fp = sqr(1.f-fr);
            } else {
                fp = sqr(1.f-fr)*sqr(fr); // TRT: (1-F)^2 * F^2  (approx)
            }
            // Absorption: hair color (golden blonde)
            Vec3 sigmaA(0.08f, 0.12f, 0.20f);
            float cosGammaT = std::cos(std::asin(hm/etaT));
            float pathLength = (p == 0) ? 0.f : 2.f*p / std::max(0.01f, cosGammaT);
            Vec3 T(std::exp(-sigmaA.x*pathLength),
                   std::exp(-sigmaA.y*pathLength),
                   std::exp(-sigmaA.z*pathLength));
            // contribute |dphi/dh|^-1 * fp * T
            float contrib = fp / std::max(1e-4f, std::abs(dphidh));
            // Return grayscale (we'll modulate by color later)
            result += contrib * (0.299f*T.x + 0.587f*T.y + 0.114f*T.z);
        }
        prevH = h; f0 = f1;
    }
    return result * 0.5f; // normalization factor
}

// Marschner BSDF for one lobe p
// Returns Vec3 RGB color contribution
static Vec3 marschnerLobe(int p, float thetaI, float thetaR, float phi,
                           float alpha, float betaM, float eta, const Vec3& hairColor){
    // Longitudinal shift
    float alphaP;
    if(p == 0)      alphaP = -alpha;
    else if(p == 1) alphaP = -alpha/2.f;
    else            alphaP = -3.f*alpha/2.f;

    // Longitudinal widths
    float betaP;
    if(p == 0)      betaP = betaM;
    else if(p == 1) betaP = betaM/2.f + 0.02f;
    else            betaP = betaM*2.f + 0.02f;

    float thetaH = (thetaR + thetaI)*0.5f;
    float thetaD = (thetaR - thetaI)*0.5f;
    float cosT2  = sqr(std::cos(thetaD));

    // M_p longitudinal scattering
    float Mp = gaussianM(betaP, thetaH - alphaP);

    // Azimuthal scattering N_p
    float cosTD = std::cos(thetaD);
    float Nval  = Np(p, phi, cosTD, eta);

    // Combined
    float S = Mp * Nval / std::max(1e-4f, cosT2);

    // Color tint based on lobe
    Vec3 color;
    if(p == 0){
        // R: specular highlight (whiter)
        color = Vec3(1.f, 1.f, 1.f);
    } else if(p == 1){
        // TT: transmission (hair color)
        color = hairColor;
    } else {
        // TRT: internal reflection caustic (warm highlight)
        color = hairColor * Vec3(1.2f, 1.0f, 0.7f);
    }

    return color * S;
}

// ---- Framebuffer -------------------------------------------------------
struct Image {
    int W, H;
    std::vector<Vec3> pixels;
    Image(int w, int h) : W(w), H(h), pixels(w*h, Vec3(0,0,0)) {}
    Vec3& at(int x, int y){ return pixels[y*W+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*W+x]; }
};

// ---- Hair Strand rendering ---------------------------------------------
// A hair strand is a cubic Bezier curve with a given width.
// We evaluate it in screen space and draw hair segments.

struct HairStrand {
    Vec3 cp[4];    // Control points in 3D
    Vec3 color;    // Base hair color
    float width;   // Hair width in world units
};

// Simple camera/projection
struct Camera {
    Vec3 pos, look, up, right;
    float fov;  // vertical fov radians
    int W, H;
};

static Vec3 project(const Camera& cam, const Vec3& p){
    Vec3 d = p - cam.pos;
    float z = d.dot(cam.look);
    float x = d.dot(cam.right);
    float y = d.dot(cam.up);
    float aspect = (float)cam.W / cam.H;
    float scale = std::tan(cam.fov*0.5f);
    float sx = (x / (z * scale * aspect) + 1.f) * 0.5f * cam.W;
    float sy = (1.f - y / (z * scale)) * 0.5f * cam.H;
    return {sx, sy, z};
}

// Cubic Bezier evaluation
static Vec3 bezier(const Vec3 cp[4], float t){
    float u = 1.f-t;
    return cp[0]*(u*u*u) + cp[1]*(3.f*u*u*t) + cp[2]*(3.f*u*t*t) + cp[3]*(t*t*t);
}
static Vec3 bezierTangent(const Vec3 cp[4], float t){
    float u = 1.f-t;
    Vec3 r = (cp[1]-cp[0])*(3.f*u*u)
           + (cp[2]-cp[1])*(6.f*u*t)
           + (cp[3]-cp[2])*(3.f*t*t);
    return r.norm();
}

// Draw a thick anti-aliased line segment on the image using Marschner shading
static void drawHairSegment(Image& img, const Camera& cam,
                             const Vec3& p0, const Vec3& p1,
                             const Vec3& tang0, const Vec3& tang1,
                             const Vec3& hairColor, float worldWidth,
                             const Vec3& lightDir, const Vec3& lightColor){
    Vec3 s0 = project(cam, p0);
    Vec3 s1 = project(cam, p1);
    if(s0.z < 0.01f || s1.z < 0.01f) return;

    float dx = s1.x - s0.x;
    float dy = s1.y - s0.y;
    float len = std::sqrt(dx*dx+dy*dy);
    if(len < 0.5f) return;

    // Project width in screen space (approximate)
    float screenWidth = std::max(1.5f, worldWidth * cam.H / (s0.z * std::tan(cam.fov*0.5f)));

    int x0 = (int)std::min(s0.x,s1.x) - (int)screenWidth - 2;
    int x1 = (int)std::max(s0.x,s1.x) + (int)screenWidth + 2;
    int y0 = (int)std::min(s0.y,s1.y) - (int)screenWidth - 2;
    int y1 = (int)std::max(s0.y,s1.y) + (int)screenWidth + 2;
    x0=std::max(0,x0); x1=std::min(img.W-1,x1);
    y0=std::max(0,y0); y1=std::min(img.H-1,y1);

    // Marschner parameters
    float alpha  = 3.f * PI / 180.f; // 3 degree longitudinal shift
    float betaM  = 0.25f;            // longitudinal width
    float eta    = 1.55f;            // IOR of hair

    for(int py=y0; py<=y1; ++py){
        for(int px=x0; px<=x1; ++px){
            // Distance from pixel to line segment
            float fx = px+0.5f, fy = py+0.5f;
            float ex = fx-s0.x, ey = fy-s0.y;
            float t = (ex*dx+ey*dy)/(dx*dx+dy*dy);
            t = std::max(0.f, std::min(1.f, t));
            float closestX = s0.x + t*dx;
            float closestY = s0.y + t*dy;
            float dist = std::sqrt(sqr(fx-closestX)+sqr(fy-closestY));

            if(dist > screenWidth+1.f) continue;
            float alpha_blend = std::max(0.f, 1.f - std::max(0.f, dist - screenWidth + 1.f));

            // Interpolate tangent
            Vec3 tang = lerp(tang0, tang1, t).norm();

            // Build local frame around hair tangent
            // thetaI: angle of light with hair tangent plane
            Vec3 lDir = lightDir.norm();
            float sinThetaI = std::abs(tang.dot(lDir));
            float thetaI    = std::asin(std::min(1.f, sinThetaI)) - PI/2.f;

            // View direction (approximate from camera)
            Vec3 viewDir = (cam.pos - lerp(p0,p1,t)).norm();
            float sinThetaR = std::abs(tang.dot(viewDir));
            float thetaR    = std::asin(std::min(1.f, sinThetaR)) - PI/2.f;

            // Azimuthal angle phi
            // Project l and v onto plane perpendicular to tangent
            Vec3 lPerp = (lDir - tang*(tang.dot(lDir))).norm();
            Vec3 vPerp = (viewDir - tang*(tang.dot(viewDir))).norm();
            float cosPhi = std::max(-1.f, std::min(1.f, lPerp.dot(vPerp)));
            float phi    = std::acos(cosPhi);
            // sign
            Vec3 cross_lv = cross(lPerp, vPerp);
            if(cross_lv.dot(tang) < 0) phi = -phi;

            // Sum Marschner lobes
            Vec3 color(0,0,0);
            float weights[3] = {0.5f, 0.8f, 0.6f};
            for(int p=0; p<3; ++p){
                Vec3 lobe = marschnerLobe(p, thetaI, thetaR, phi, alpha, betaM, eta, hairColor);
                color += lobe * lightColor * weights[p];
            }

            // Ambient
            color += hairColor * 0.06f;

            // Blend with soft additive, but cap accumulation
            Vec3& pixel = img.at(px,py);
            Vec3 incoming = color * alpha_blend;
            // Use luminance-based additive blend to prevent oversaturation
            pixel = pixel + incoming * (1.f - clamp01(pixel.x*0.3f + pixel.y*0.6f + pixel.z*0.1f)*0.8f);
        }
    }
}

// ---- Scene setup -------------------------------------------------------
static std::vector<HairStrand> buildHairStrands(){
    std::vector<HairStrand> strands;
    
    // Hair color: dark brown, blonde, auburn
    Vec3 colors[] = {
        {0.12f, 0.07f, 0.03f},  // dark brown
        {0.45f, 0.32f, 0.12f},  // dark blonde
        {0.38f, 0.18f, 0.05f},  // auburn
        {0.22f, 0.15f, 0.06f},  // medium brown
        {0.55f, 0.42f, 0.18f},  // golden blonde
    };

    // Generate hair strands in a dome shape (like a head of hair)
    // Using sinusoidal variation for natural look
    int nStrands = 120;
    for(int i=0; i<nStrands; ++i){
        float u = (float)i / nStrands;
        float angle = u * 2.f * PI;
        float r = 0.8f + 0.2f*std::sin(angle*3.f + 0.7f);
        
        // Root position on top hemisphere
        float rootX = r * std::cos(angle) * 0.9f;
        float rootY = 1.5f;  // top
        float rootZ = r * std::sin(angle) * 0.9f + 3.f; // push back
        
        // Hair grows down and outward with some curl
        float curl = 0.1f + 0.15f * std::sin(angle * 2.f + 1.3f);
        float length = 1.2f + 0.5f * std::sin(angle * 1.7f + 0.4f);
        
        HairStrand s;
        s.cp[0] = Vec3(rootX, rootY, rootZ);
        s.cp[1] = Vec3(rootX + curl*std::cos(angle+PI/4.f), 
                       rootY - length*0.35f, 
                       rootZ + curl*std::sin(angle+PI/4.f));
        s.cp[2] = Vec3(rootX + curl*2.f*std::cos(angle+PI/3.f), 
                       rootY - length*0.7f, 
                       rootZ + curl*2.f*std::sin(angle+PI/3.f));
        s.cp[3] = Vec3(rootX + curl*3.f*std::cos(angle+PI/2.f)*0.8f,
                       rootY - length, 
                       rootZ + curl*3.f*std::sin(angle+PI/2.f)*0.8f);
        
        s.color  = colors[i % 5];
        s.width  = 0.035f;
        strands.push_back(s);
    }
    
    return strands;
}

// ---- Main --------------------------------------------------------------
int main(){
    const int W = 800, H = 600;
    Image img(W, H);

    // Background: dark gradient (studio background)
    for(int y=0; y<H; ++y){
        float t = (float)y/H;
        Vec3 bg = Vec3(0.05f,0.05f,0.08f) * (1.f-t) + Vec3(0.02f,0.02f,0.03f)*t;
        for(int x=0; x<W; ++x){
            img.at(x,y) = bg;
        }
    }

    // Camera
    Camera cam;
    cam.pos   = Vec3(0.f, 0.8f, 0.f);  // slight above
    cam.look  = Vec3(0.f, 0.f, 1.f).norm();  // look at +Z
    cam.up    = Vec3(0.f, 1.f, 0.f);
    cam.right = cross(cam.look, cam.up).norm();
    // re-orthogonalize up
    cam.up    = cross(cam.right, cam.look).norm();
    cam.fov   = 60.f * PI / 180.f;
    cam.W = W; cam.H = H;

    // Lights
    struct Light { Vec3 dir; Vec3 color; };
    Light lights[] = {
        { Vec3(0.8f, 1.f, 0.5f).norm(), Vec3(1.f, 0.95f, 0.85f)*0.8f },   // key light
        { Vec3(-0.5f, 0.3f, 0.8f).norm(), Vec3(0.4f, 0.5f, 0.7f)*0.3f },  // fill light
        { Vec3(0.f, -0.5f, 0.8f).norm(), Vec3(0.3f, 0.3f, 0.35f)*0.2f },  // rim light
    };

    auto strands = buildHairStrands();

    // Render each strand
    const int SUBDIV = 20;  // segments per strand
    for(const auto& strand : strands){
        for(int seg=0; seg<SUBDIV; ++seg){
            float t0 = (float)seg/SUBDIV;
            float t1 = (float)(seg+1)/SUBDIV;
            Vec3 p0 = bezier(strand.cp, t0);
            Vec3 p1 = bezier(strand.cp, t1);
            Vec3 tang0 = bezierTangent(strand.cp, t0);
            Vec3 tang1 = bezierTangent(strand.cp, t1);

            for(const auto& light : lights){
                drawHairSegment(img, cam, p0, p1, tang0, tang1,
                                strand.color, strand.width,
                                light.dir, light.color);
            }
        }
    }

    // Tone mapping (ACES Filmic)
    for(int i=0; i<W*H; ++i){
        Vec3& c = img.pixels[i];
        auto aces = [](float x) -> float {
            const float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
            return std::max(0.f, std::min(1.f, (x*(a*x+b))/(x*(cc*x+d)+e)));
        };
        c.x = aces(c.x); c.y = aces(c.y); c.z = aces(c.z);
    }

    // Convert to u8 RGB
    std::vector<uint8_t> rgb(W*H*3);
    for(int i=0; i<W*H; ++i){
        rgb[i*3+0] = toU8(img.pixels[i].x);
        rgb[i*3+1] = toU8(img.pixels[i].y);
        rgb[i*3+2] = toU8(img.pixels[i].z);
    }

    const char* outPath = "marschner_hair_output.png";
    if(!savePNG(outPath, rgb.data(), W, H)){
        fprintf(stderr, "Failed to write PNG\n");
        return 1;
    }

    printf("Rendered %d hair strands\n", (int)strands.size());
    printf("Output: %s\n", outPath);
    return 0;
}
