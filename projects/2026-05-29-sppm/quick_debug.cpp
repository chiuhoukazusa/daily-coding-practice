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
    Vec3():x(0),y(0),z(0){}
    Vec3(double v):x(v),y(v),z(v){}
    Vec3(double x,double y,double z):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const{return {x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3& b) const{return {x-b.x,y-b.y,z-b.z};}
    Vec3 operator-() const{return {-x,-y,-z};}
    Vec3 operator*(double t) const{return {x*t,y*t,z*t};}
    Vec3 operator/(double t) const{return {x/t,y/t,z/t};}
    double dot(const Vec3& b) const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3& b) const{return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    double len2() const{return x*x+y*y+z*z;}
    double len() const{return std::sqrt(len2());}
    Vec3 normalized() const{double l=len();return l>EPS?*this/l:Vec3();}
};

static std::mt19937_64 rng(42);
static std::uniform_real_distribution<double> dist01(0.0,1.0);
inline double rand01(){return dist01(rng);}

Vec3 uniformSphere(){
    double u=rand01(),v=rand01();
    double theta=2*PI*u, phi=std::acos(std::max(-1.0,std::min(1.0,2*v-1)));
    return {std::sin(phi)*std::cos(theta),std::cos(phi),std::sin(phi)*std::sin(theta)};
}

Vec3 cosineHemisphere(const Vec3& n){
    double r1=rand01(),r2=rand01(),phi=2*PI*r1,sr2=std::sqrt(r2);
    double lx=std::cos(phi)*sr2, lz=std::sin(phi)*sr2, ly=std::sqrt(std::max(0.0,1-r2));
    Vec3 up = std::abs(n.y) < 0.999 ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 t=n.cross(up).normalized(), b=n.cross(t);
    return (t*lx+n*ly+b*lz).normalized();
}

struct Ray{Vec3 o,d;Ray(Vec3 o,Vec3 d):o(o),d(d.normalized()){}Vec3 at(double t)const{return o+d*t;}};

struct HitInfo{double t=INF;Vec3 pos,n;int type=0;double ior=1.5;bool front=true;};

// 平面
bool hitXZ(double y, double x0, double x1, double z0, double z1,
           const Ray& r, double tMin, HitInfo& h){
    if(std::abs(r.d.y)<EPS)return false;
    double t=(y-r.o.y)/r.d.y;
    if(t<tMin||t>h.t)return false;
    double x=r.o.x+t*r.d.x, z=r.o.z+t*r.d.z;
    if(x<x0||x>x1||z<z0||z>z1)return false;
    h.t=t;h.pos=r.at(t);
    Vec3 outN(0,1,0);
    h.front=r.d.dot(outN)<0;
    h.n=h.front?outN:-outN;
    h.type=0;return true;
}

bool hitSphere(Vec3 c, double r_s, double ior,
               const Ray& r, double tMin, HitInfo& h){
    Vec3 oc=r.o-c;
    double a=r.d.len2(),hb=oc.dot(r.d),cc=oc.len2()-r_s*r_s;
    double disc=hb*hb-a*cc;
    if(disc<0)return false;
    double sq=std::sqrt(disc),t=(-hb-sq)/a;
    if(t<tMin||t>h.t){t=(-hb+sq)/a;if(t<tMin||t>h.t)return false;}
    h.t=t;h.pos=r.at(t);
    Vec3 outN=(h.pos-c)*(1.0/r_s);
    h.front=r.d.dot(outN)<0;
    h.n=h.front?outN:-outN;
    h.type=2;h.ior=ior;return true;
}

Vec3 reflect(const Vec3& v,const Vec3& n){return v-n*(2*v.dot(n));}
bool refract(const Vec3& v,const Vec3& n,double ni,Vec3& out){
    Vec3 uv=v.normalized();double dt=uv.dot(n);
    double disc=1-ni*ni*(1-dt*dt);
    if(disc<=0)return false;
    out=(uv-n*dt)*ni-n*std::sqrt(disc);return true;
}
double schlick(double c,double ior){double r0=(1-ior)/(1+ior);r0*=r0;return r0+(1-r0)*std::pow(1-c,5);}

int main(){
    Vec3 glassCenter(278,97,250);
    double glassR=80, glassIor=1.5;
    
    // 统计：从光源发出的光子能打到地面的比例
    int N=100000;
    int hitGround=0, hitGlass=0, hitOther=0, hitNothing=0;
    double totalFlux=0;
    std::vector<Vec3> groundPositions;
    
    for(int i=0;i<N;i++){
        Vec3 lpos={213+rand01()*(343-213), 554.0, 227+rand01()*(332-227)};
        // 余弦加权向下（normal=(0,-1,0)）
        Vec3 lnormal(0,-1,0);
        // cosineHemisphere with n=(0,-1,0): up=(1,0,0) since abs(ny)=1
        Vec3 ldir = cosineHemisphere(lnormal);
        // 确认方向朝下
        if(ldir.y >= 0) ldir = {ldir.x, -std::abs(ldir.y)-0.01, ldir.z};
        ldir = ldir.normalized();
        
        Ray r(lpos,ldir);
        int depth=0;
        bool done=false;
        
        while(depth<10 && !done){
            HitInfo h;
            // 检查地面
            bool gHit = hitXZ(0, 0, 555, 0, 555, r, EPS, h);
            // 检查玻璃球
            HitInfo hg; hg.t=INF;
            bool bHit = hitSphere(glassCenter, glassR, glassIor, r, EPS, hg);
            
            // 用最近的命中
            if(bHit && hg.t < h.t){ h=hg; gHit=false; bHit=true; }
            else { bHit=false; }
            
            if(h.t >= INF){ hitNothing++; done=true; break; }
            
            if(bHit){ // 玻璃球
                hitGlass++;
                double niOverNt = h.front ? (1.0/glassIor) : glassIor;
                double cosT = std::min(-r.d.normalized().dot(h.n),1.0);
                double sinT = std::sqrt(std::max(0.0,1.0-cosT*cosT));
                double rProb = (niOverNt*sinT>1.0) ? 1.0 : schlick(cosT,glassIor);
                Vec3 side = h.front ? h.n : -h.n;
                if(rand01()<rProb){
                    r=Ray(h.pos+side*EPS, reflect(r.d,h.n).normalized());
                } else {
                    Vec3 refr;
                    refract(r.d,h.n,niOverNt,refr);
                    r=Ray(h.pos-side*EPS, refr.normalized());
                }
            } else if(gHit){ // 地面
                hitGround++;
                if(groundPositions.size()<20){
                    groundPositions.push_back(h.pos);
                }
                totalFlux += 1.0;
                done=true;
            } else {
                hitOther++; done=true;
            }
            depth++;
        }
    }
    
    printf("光子追踪统计（%d 光子）:\n",N);
    printf("  命中地面: %d (%.2f%%)\n",hitGround,100.0*hitGround/N);
    printf("  穿过玻璃: %d\n",hitGlass);
    printf("  命中其他: %d\n",hitOther);
    printf("  未命中:   %d\n",hitNothing);
    printf("\n前20个地面命中位置:\n");
    for(int i=0;i<(int)groundPositions.size();i++){
        printf("  [%d] (%.1f,%.1f,%.1f)\n",i,groundPositions[i].x,groundPositions[i].y,groundPositions[i].z);
    }
    
    // 检查 cosineHemisphere((0,-1,0)) 方向
    int downCount=0;
    Vec3 n_down(0,-1,0);
    for(int i=0;i<1000;i++){
        Vec3 d=cosineHemisphere(n_down);
        if(d.y<0) downCount++;
    }
    printf("\ncosineHemisphere((0,-1,0)) 朝下比例: %d/1000\n",downCount);
    
    return 0;
}
