#include <cmath>
#include <vector>
#include <random>
#include <limits>
#include <cstdio>
#include <algorithm>

static const double PI = 3.14159265358979323846;
static const double INF = std::numeric_limits<double>::infinity();
static const double EPS = 1e-6;

struct Vec3 {
    double x, y, z;
    Vec3(double v=0): x(v), y(v), z(v) {}
    Vec3(double x, double y, double z): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 neg() const { return {-x,-y,-z}; }
    double dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const { return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x}; }
    double len2() const { return x*x+y*y+z*z; }
    double len() const { return std::sqrt(len2()); }
    Vec3 normalized() const { double l=len(); return l>EPS?Vec3(x/l,y/l,z/l):Vec3(0); }
};

static std::mt19937_64 rng(42);
static std::uniform_real_distribution<double> dist01(0.0, 1.0);
inline double rand01() { return dist01(rng); }

Vec3 uniformSphere() {
    double u = rand01(), v = rand01();
    double theta = 2.0 * PI * u;
    double phi = std::acos(2.0 * v - 1.0);
    return Vec3(std::sin(phi)*std::cos(theta), std::cos(phi), std::sin(phi)*std::sin(theta));
}

struct Ray { Vec3 origin, dir; Ray(Vec3 o, Vec3 d): origin(o), dir(d.normalized()) {} Vec3 at(double t) const { return origin + dir*t; } };

enum class MatType { DIFFUSE, MIRROR, GLASS };
struct Material { Vec3 albedo, emission; MatType type; double ior; };
struct HitRecord { 
    double t = INF; Vec3 pos, normal; const Material* mat = nullptr; bool frontFace;
    void setNormal(const Ray& r, const Vec3& outN) {
        frontFace = r.dir.dot(outN) < 0;
        normal = frontFace ? outN : outN.neg();
    }
};

struct Sphere { Vec3 center; double radius; Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.dir.len2(), hb = oc.dot(r.dir), c = oc.len2() - radius*radius;
        double disc = hb*hb - a*c;
        if (disc < 0) return false;
        double sq = std::sqrt(disc), t = (-hb - sq) / a;
        if (t < tMin || t > tMax) { t = (-hb + sq) / a; if (t < tMin || t > tMax) return false; }
        rec.t = t; rec.pos = r.at(t); rec.setNormal(r, (rec.pos - center) * (1.0/radius)); rec.mat = &mat; return true;
    }
};
struct AARectXZ { double x0,x1,z0,z1,k; Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.y) < EPS) return false;
        double t = (k - r.origin.y) / r.dir.y;
        if (t < tMin || t > tMax) return false;
        double x = r.origin.x + t*r.dir.x, z = r.origin.z + t*r.dir.z;
        if (x < x0 || x > x1 || z < z0 || z > z1) return false;
        rec.t = t; rec.pos = r.at(t); rec.setNormal(r, Vec3(0,1,0)); rec.mat = &mat; return true;
    }
};
struct AARectXY { double x0,x1,y0,y1,k; Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.z) < EPS) return false;
        double t = (k - r.origin.z) / r.dir.z;
        if (t < tMin || t > tMax) return false;
        double x = r.origin.x + t*r.dir.x, y = r.origin.y + t*r.dir.y;
        if (x < x0 || x > x1 || y < y0 || y > y1) return false;
        rec.t = t; rec.pos = r.at(t); rec.setNormal(r, Vec3(0,0,-1)); rec.mat = &mat; return true;
    }
};
struct AARectYZ { double y0,y1,z0,z1,k; Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.x) < EPS) return false;
        double t = (k - r.origin.x) / r.dir.x;
        if (t < tMin || t > tMax) return false;
        double y = r.origin.y + t*r.dir.y, z = r.origin.z + t*r.dir.z;
        if (y < y0 || y > y1 || z < z0 || z > z1) return false;
        rec.t = t; rec.pos = r.at(t); rec.setNormal(r, Vec3(1,0,0)); rec.mat = &mat; return true;
    }
};

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<AARectXZ> xzRects;
    std::vector<AARectXY> xyRects;
    std::vector<AARectYZ> yzRects;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tmp; bool any=false; double closest=tMax;
        for (auto& s : spheres)  if (s.hit(r,tMin,closest,tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : xzRects)  if (s.hit(r,tMin,closest,tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : xyRects)  if (s.hit(r,tMin,closest,tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : yzRects)  if (s.hit(r,tMin,closest,tmp)) { any=true; closest=tmp.t; rec=tmp; }
        return any;
    }
};

int main() {
    Scene sc;
    Material white = {{0.73,0.73,0.73},{0,0,0},MatType::DIFFUSE,1.5};
    Material red = {{0.65,0.05,0.05},{0,0,0},MatType::DIFFUSE,1.5};
    Material green = {{0.12,0.45,0.15},{0,0,0},MatType::DIFFUSE,1.5};
    Material light = {{1,1,0.9},{12,12,10},MatType::DIFFUSE,1.5};
    Material glass = {{1,1,1},{0,0,0},MatType::GLASS,1.5};
    
    sc.xzRects.push_back({0,555,0,555,0,white});
    sc.xzRects.push_back({0,555,0,555,555,white});
    sc.xyRects.push_back({0,555,0,555,555,white});
    sc.yzRects.push_back({0,555,0,555,0,green});
    sc.yzRects.push_back({0,555,0,555,555,red});
    sc.xzRects.push_back({213,343,227,332,554,light});
    sc.spheres.push_back({{278,120,250},110,glass});
    
    // 测试: 数出相机可以看到的各类面
    Vec3 eye = {278,278,-800};
    Vec3 lower_left, horiz, vert;
    double vfov = 40.0, aspect = 1.0;
    double halfH = std::tan(vfov*PI/180.0/2.0);
    double halfW = aspect * halfH;
    Vec3 w = Vec3(0,0,1); // eye - target normalized = (0,0,1)
    Vec3 u_ax = Vec3(0,1,0).cross(w).normalized(); // = (-1,0,0) wait
    // eye=(278,278,-800) target=(278,278,0), eye-target=(0,0,-800), normalized=(0,0,-1)
    // w = (0,0,-1) wait: (0,0,-800).normalized() = (0,0,-1)
    Vec3 w2 = (eye - Vec3(278,278,0)).normalized();
    Vec3 u2 = Vec3(0,1,0).cross(w2).normalized();
    Vec3 v2 = w2.cross(u2);
    printf("w=(%.2f,%.2f,%.2f) u=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f)\n",
           w2.x,w2.y,w2.z, u2.x,u2.y,u2.z, v2.x,v2.y,v2.z);
    lower_left = eye + (u2.neg())*halfW + v2.neg()*halfH + w2.neg();
    horiz = u2*(2*halfW);
    vert  = v2*(2*halfH);
    
    // 测试10x10网格光线
    int hitFloor=0, hitCeil=0, hitBack=0, hitLeft=0, hitRight=0, hitGlass=0, hitLight=0, hitMiss=0;
    for (int y=0; y<100; ++y) for (int x=0; x<100; ++x) {
        double s = (x+0.5)/100.0, t = (y+0.5)/100.0;
        Vec3 dir = lower_left + horiz*s + vert*t + eye.neg();
        Ray r(eye, dir);
        HitRecord rec;
        if (sc.hit(r, EPS, INF, rec)) {
            if (rec.mat->type == MatType::GLASS) hitGlass++;
            else if (!rec.mat->emission.isZero() && rec.mat->emission.x > 1) hitLight++;
            else if (rec.pos.y < 1) hitFloor++;
            else if (rec.pos.y > 554) hitCeil++;
            else if (rec.pos.z > 554) hitBack++;
            else if (rec.pos.x < 1) hitLeft++;
            else if (rec.pos.x > 554) hitRight++;
            else hitMiss++;
        } else hitMiss++;
    }
    printf("Camera ray hits (10000 rays):\n");
    printf("  Floor: %d  Ceil: %d  Back: %d  Left: %d  Right: %d  Glass: %d  Light: %d  Miss: %d\n",
           hitFloor, hitCeil, hitBack, hitLeft, hitRight, hitGlass, hitLight, hitMiss);
    
    // 测试光子直接从光源打到地面的比例
    int floorDirect = 0;
    for (int i=0; i<10000; ++i) {
        Vec3 lpos = {213 + rand01()*(343-213), 554.0, 227 + rand01()*(332-227)};
        Vec3 ldir; do { ldir = uniformSphere(); } while (ldir.y > 0);
        Ray r(lpos, ldir);
        HitRecord h;
        if (sc.hit(r, EPS, INF, h)) {
            if (h.pos.y < 1.0) floorDirect++;
        }
    }
    printf("\nLight→Floor direct: %d/10000 (%.1f%%)\n", floorDirect, 100.0*floorDirect/10000);
    
    // 测试从eye到scene中心的直接光线
    {
        Ray center(eye, Vec3(278,278,0) - eye);
        HitRecord h;
        sc.hit(center, EPS, INF, h);
        printf("Center ray hits glass sphere? %d pos=(%.1f,%.1f,%.1f)\n",
               (int)(h.mat && h.mat->type == MatType::GLASS),
               h.pos.x, h.pos.y, h.pos.z);
    }
    
    return 0;
}
