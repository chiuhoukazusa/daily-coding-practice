#include <cmath>
#include <vector>
#include <cstdio>
#include <random>
#include <algorithm>

static const float H=0.2f,H2=H*H,MASS=0.02f,REST_DENS=1000.f,PI=3.14159265f;
static const float POLY6K = 4.0f/(PI*std::pow(H,8));
struct Vec2{float x,y;};
struct Particle{Vec2 pos,vel,force;float density,pressure;};
float kpoly6(float r2){if(r2>=H2)return 0;float d=H2-r2;return POLY6K*d*d*d;}

int main(){
    std::vector<Particle> ps;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> jitter(-H*0.01f,H*0.01f);
    int cols=24;float spacing=H*0.75f,ox=0.3f,oy=0.3f;
    for(int i=0;i<600;i++){
        int c=i%cols,r=i/cols;
        Particle p;
        p.pos={ox+c*spacing+(float)jitter(rng),oy+r*spacing+(float)jitter(rng)};
        p.vel={0,0};p.force={0,0};p.density=REST_DENS;p.pressure=0;
        ps.push_back(p);
    }
    // Compute density
    for(auto& pi:ps){
        pi.density=0;
        for(const auto& pj:ps){
            float dx=pi.pos.x-pj.pos.x,dy=pi.pos.y-pj.pos.y;
            pi.density+=MASS*kpoly6(dx*dx+dy*dy);
        }
    }
    float avg=0,mn=1e9f,mx=-1e9f;
    for(auto&p:ps){avg+=p.density;mn=std::min(mn,p.density);mx=std::max(mx,p.density);}
    avg/=ps.size();
    printf("Initial density: avg=%.4f min=%.4f max=%.4f (REST=%.1f)\n",avg,mn,mx,REST_DENS);
    return 0;
}
