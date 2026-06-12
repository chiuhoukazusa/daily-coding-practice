#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <vector>
#include <random>
#include <limits>
#include <cstdio>
#include <algorithm>
#include <unordered_map>

static const double PI = 3.14159265358979323846;
static const double EPS = 1e-6;
static const double INF = std::numeric_limits<double>::infinity();

struct Vec3 {
    double x, y, z;
    Vec3():x(0),y(0),z(0){}
    Vec3(double v):x(v),y(v),z(v){}
    Vec3(double x,double y,double z):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b)const{return {x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3& b)const{return {x-b.x,y-b.y,z-b.z};}
    Vec3 operator-()const{return {-x,-y,-z};}
    Vec3 operator*(double t)const{return {x*t,y*t,z*t};}
    Vec3 operator*(const Vec3& b)const{return {x*b.x,y*b.y,z*b.z};}
    Vec3 operator/(double t)const{return {x/t,y/t,z/t};}
    Vec3& operator+=(const Vec3& b){x+=b.x;y+=b.y;z+=b.z;return *this;}
    double dot(const Vec3& b)const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3& b)const{return{y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    double len2()const{return x*x+y*y+z*z;}
    double len()const{return std::sqrt(len2());}
    Vec3 normalized()const{double l=len();return l>EPS?*this/l:Vec3();}
    bool isZero()const{return len2()<EPS*EPS;}
};
Vec3 operator*(double t,const Vec3& v){return v*t;}

static std::mt19937_64 rng(42);
static std::uniform_real_distribution<double> d01(0.0,1.0);
inline double rand01(){return d01(rng);}

Vec3 cosineHemisphere(const Vec3& n){
    double r1=rand01(),r2=rand01(),phi=2*PI*r1,sr2=std::sqrt(r2);
    double lx=std::cos(phi)*sr2,lz=std::sin(phi)*sr2,ly=std::sqrt(std::max(0.0,1-r2));
    Vec3 up = std::abs(n.y)<0.999 ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 t=n.cross(up).normalized(),b=n.cross(t);
    return (t*lx+n*ly+b*lz).normalized();
}
Vec3 uniformSphere(){
    double u=rand01(),v=rand01(),theta=2*PI*u,phi=std::acos(std::max(-1.0,std::min(1.0,2*v-1)));
    return {std::sin(phi)*std::cos(theta),std::cos(phi),std::sin(phi)*std::sin(theta)};
}

enum class MT{DIFF,MIRROR,GLASS};
struct Mat{Vec3 a,e;MT t;double ior;};
struct HR{double t=INF;Vec3 pos,n;const Mat* m=nullptr;bool front=true;
    void setN(const Vec3& d,Vec3 on){front=d.dot(on)<0;n=front?on:-on;}};
struct Ray{Vec3 o,d;Ray(Vec3 o,Vec3 d):o(o),d(d.normalized()){}Vec3 at(double t)const{return o+d*t;}};

struct Sphere{Vec3 c;double r;Mat m;
    bool hit(const Ray& ray,double tMin,double tMax,HR& rec)const{
        Vec3 oc=ray.o-c;double a=ray.d.len2(),hb=oc.dot(ray.d),cc=oc.len2()-r*r;
        double disc=hb*hb-a*cc;if(disc<0)return false;
        double sq=std::sqrt(disc),t=(-hb-sq)/a;
        if(t<tMin||t>tMax){t=(-hb+sq)/a;if(t<tMin||t>tMax)return false;}
        rec.t=t;rec.pos=ray.at(t);rec.setN(ray.d,(rec.pos-c)*(1.0/r));rec.m=&m;return true;}};
struct RXZ{double x0,x1,z0,z1,k;Mat m;
    bool hit(const Ray& r,double tMin,double tMax,HR& rec)const{
        if(std::abs(r.d.y)<EPS)return false;
        double t=(k-r.o.y)/r.d.y;if(t<tMin||t>tMax)return false;
        double x=r.o.x+t*r.d.x,z=r.o.z+t*r.d.z;
        if(x<x0||x>x1||z<z0||z>z1)return false;
        rec.t=t;rec.pos=r.at(t);rec.setN(r.d,Vec3(0,1,0));rec.m=&m;return true;}};
struct RXY{double x0,x1,y0,y1,k;Mat m;
    bool hit(const Ray& r,double tMin,double tMax,HR& rec)const{
        if(std::abs(r.d.z)<EPS)return false;
        double t=(k-r.o.z)/r.d.z;if(t<tMin||t>tMax)return false;
        double x=r.o.x+t*r.d.x,y=r.o.y+t*r.d.y;
        if(x<x0||x>x1||y<y0||y>y1)return false;
        rec.t=t;rec.pos=r.at(t);rec.setN(r.d,Vec3(0,0,-1));rec.m=&m;return true;}};
struct RYZ{double y0,y1,z0,z1,k;Mat m;
    bool hit(const Ray& r,double tMin,double tMax,HR& rec)const{
        if(std::abs(r.d.x)<EPS)return false;
        double t=(k-r.o.x)/r.d.x;if(t<tMin||t>tMax)return false;
        double y=r.o.y+t*r.d.y,z=r.o.z+t*r.d.z;
        if(y<y0||y>y1||z<z0||z>z1)return false;
        rec.t=t;rec.pos=r.at(t);rec.setN(r.d,Vec3(1,0,0));rec.m=&m;return true;}};

struct Scene{
    std::vector<Sphere> sp;
    std::vector<RXZ> xz;std::vector<RXY> xy;std::vector<RYZ> yz;
    bool hit(const Ray& r,double tMin,double tMax,HR& rec)const{
        HR tmp;bool any=false;double cl=tMax;
        for(auto& s:sp)if(s.hit(r,tMin,cl,tmp)){any=true;cl=tmp.t;rec=tmp;}
        for(auto& s:xz)if(s.hit(r,tMin,cl,tmp)){any=true;cl=tmp.t;rec=tmp;}
        for(auto& s:xy)if(s.hit(r,tMin,cl,tmp)){any=true;cl=tmp.t;rec=tmp;}
        for(auto& s:yz)if(s.hit(r,tMin,cl,tmp)){any=true;cl=tmp.t;rec=tmp;}
        return any;}};

Vec3 reflect(const Vec3& v,const Vec3& n){return v-n*(2*v.dot(n));}
bool refract(const Vec3& v,const Vec3& n,double ni,Vec3& out){
    Vec3 uv=v.normalized();double dt=uv.dot(n),disc=1-ni*ni*(1-dt*dt);
    if(disc<=0)return false;out=(uv-n*dt)*ni-n*std::sqrt(disc);return true;}
double schlick(double c,double ior){double r0=(1-ior)/(1+ior);r0*=r0;return r0+(1-r0)*std::pow(1-c,5);}

int main(){
    Scene sc;
    Mat white={{0.73,0.73,0.73},{},MT::DIFF,1.5};
    Mat red={{0.65,0.05,0.05},{},MT::DIFF,1.5};
    Mat green={{0.12,0.45,0.15},{},MT::DIFF,1.5};
    Mat lightM={{1,1,0.9},{12,12,10},MT::DIFF,1.5};
    Mat glass={{1,1,1},{},MT::GLASS,1.5};
    Mat mirror={{0.9,0.85,0.8},{},MT::MIRROR,1.5};
    
    sc.xz.push_back({0,555,0,555,0,white});
    sc.xz.push_back({0,555,0,555,555,white});
    sc.xy.push_back({0,555,0,555,555,white});
    sc.yz.push_back({0,555,0,555,0,green});
    sc.yz.push_back({0,555,0,555,555,red});
    sc.xz.push_back({213,343,227,332,554,lightM});
    sc.sp.push_back({Vec3(278,97,250),80,glass});
    sc.sp.push_back({Vec3(100,70,120),70,mirror});
    
    Vec3 lightEmission{12,12,10};
    double lx0=213,lx1=343,lz0=227,lz1=332,ly=554;
    double lightArea=(lx1-lx0)*(lz1-lz0);
    Vec3 totalPower=lightEmission*lightArea*PI;
    
    int N=10000;
    int depthStats[11]={};
    int surfaceTypes[5]={};  // 0=floor,1=ceil,2=back,3=wall,4=glass
    std::vector<Vec3> photonPositions;
    
    double totalFluxX=0;
    int floorPhotons=0;
    
    for(int i=0;i<N;i++){
        Vec3 lpos={lx0+rand01()*(lx1-lx0),ly,lz0+rand01()*(lz1-lz0)};
        Vec3 ldir=cosineHemisphere(Vec3(0,-1,0));
        
        Ray r(lpos,ldir);
        Vec3 power=totalPower*(1.0/N);
        
        for(int d=0;d<10;d++){
            HR rec;
            if(!sc.hit(r,EPS,INF,rec)){depthStats[d]++;break;}
            const Mat* m=rec.m;
            if(!m->e.isZero())break;
            
            if(m->t==MT::DIFF){
                depthStats[d]++;
                // 沉积光子
                if(rec.pos.y<1){
                    surfaceTypes[0]++;
                    floorPhotons++;
                    totalFluxX+=power.x;
                    if(photonPositions.size()<20)photonPositions.push_back(rec.pos);
                } else if(rec.pos.y>554){surfaceTypes[1]++;}
                else if(rec.pos.z>554){surfaceTypes[2]++;}
                else{surfaceTypes[3]++;}
                
                double sp=std::max({m->a.x,m->a.y,m->a.z});
                sp=std::min(sp,0.9);
                if(rand01()>sp)break;
                power=power*m->a*(1.0/sp);
                r=Ray(rec.pos+rec.n*EPS,cosineHemisphere(rec.n));
            } else if(m->t==MT::MIRROR){
                power=power*m->a;
                r=Ray(rec.pos+rec.n*EPS,reflect(r.d,rec.n).normalized());
            } else {
                surfaceTypes[4]++;
                double niOverNt=rec.front?(1.0/m->ior):m->ior;
                double cosT=std::min(-r.d.normalized().dot(rec.n),1.0);
                double sinT=std::sqrt(std::max(0.0,1.0-cosT*cosT));
                double rProb=(niOverNt*sinT>1.0)?1.0:schlick(cosT,m->ior);
                Vec3 side=rec.front?rec.n:-rec.n;
                if(rand01()<rProb){
                    r=Ray(rec.pos+side*EPS,reflect(r.d,rec.n).normalized());
                }else{
                    Vec3 refr;refract(r.d,rec.n,niOverNt,refr);
                    r=Ray(rec.pos-side*EPS,refr.normalized());
                }
            }
        }
    }
    
    printf("光子追踪（%d光子，含完整场景）:\n",N);
    printf("  沉积在地面: %d (%.2f%%)\n",floorPhotons,100.0*floorPhotons/N);
    printf("  沉积在天花板: %d\n",surfaceTypes[1]);
    printf("  沉积在背墙: %d\n",surfaceTypes[2]);
    printf("  沉积在侧墙: %d\n",surfaceTypes[3]);
    printf("  玻璃折射次数: %d\n",surfaceTypes[4]);
    printf("  地面总通量X: %.4f\n",totalFluxX);
    printf("\n前%d个地面光子位置:\n",(int)photonPositions.size());
    for(auto& p:photonPositions)printf("  (%.1f,%.1f,%.1f)\n",p.x,p.y,p.z);
    
    return 0;
}
