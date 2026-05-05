// Quick diagnostic: print particle positions at each frame
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>

static const float H = 16.0f;
static const float H2 = H*H;
static const float MASS = 1.0f;
static const float REST_DENS = 300.0f;
static const float GAS_CONST = 2000.0f;
static const float VISC = 200.0f;
static const float DT = 0.0016f;
static const float GRAVITY_Y = 9000.0f;
static const float BOUND_DAMP = 0.5f;
static const float PI_F = 3.14159265f;
static const float POLY6 = 315.f/(65.f*PI_F*std::pow(H,9.f));
static const float SPIKY_GRAD = -45.f/(PI_F*std::pow(H,6.f));
static const float VISC_LAP = 45.f/(PI_F*std::pow(H,6.f));

struct P { float x,y,vx,vy,fx,fy,d,p; };

void dp(std::vector<P>& ps) {
    for (auto& pi:ps) { pi.d=0;
        for (auto& pj:ps) { float dx=pj.x-pi.x,dy=pj.y-pi.y,r2=dx*dx+dy*dy;
            if(r2<H2){float d=H2-r2; pi.d+=MASS*POLY6*d*d*d;} }
        pi.p=GAS_CONST*(pi.d-REST_DENS); }
}
void cf(std::vector<P>& ps) {
    for (size_t i=0;i<ps.size();i++) { float fpx=0,fpy=0,fvx=0,fvy=0;
        for (size_t j=0;j<ps.size();j++) { if(i==j)continue;
            float dx=ps[j].x-ps[i].x,dy=ps[j].y-ps[i].y,r2=dx*dx+dy*dy;
            if(r2<H2&&r2>1e-8f){float r=std::sqrt(r2),hnr=H-r;
                float pr=-MASS*(ps[i].p+ps[j].p)/(2*ps[j].d);
                float sp=SPIKY_GRAD*hnr*hnr;
                fpx+=pr*sp*(dx/r); fpy+=pr*sp*(dy/r);
                float vf=VISC*MASS*VISC_LAP*hnr/ps[j].d;
                fvx+=vf*(ps[j].vx-ps[i].vx); fvy+=vf*(ps[j].vy-ps[i].vy);} }
        ps[i].fx=fpx+fvx; ps[i].fy=fpy+fvy+GRAVITY_Y*ps[i].d; }
}
void intg(std::vector<P>& ps) {
    for (auto& p:ps) {
        if(p.d>1e-6f){p.vx+=DT*p.fx/p.d; p.vy+=DT*p.fy/p.d;}
        p.x+=DT*p.vx; p.y+=DT*p.vy;
        if(p.x<2){p.x=2;p.vx*=-BOUND_DAMP;} if(p.x>198){p.x=198;p.vx*=-BOUND_DAMP;}
        if(p.y<2){p.y=2;p.vy*=-BOUND_DAMP;} if(p.y>198){p.y=198;p.vy*=-BOUND_DAMP;}
    }
}

int main(){
    std::vector<P> ps;
    float sp=H*0.65f,ox=18,oy=10;
    for(int ix=0;ix<10;ix++) for(int iy=0;iy<14;iy++){
        P p; p.x=ox+ix*sp+(iy%2==0?0.f:sp*0.5f); p.y=oy+iy*sp;
        p.vx=p.vy=p.fx=p.fy=0; p.d=REST_DENS; p.p=0;
        if(p.x<198&&p.y<198) ps.push_back(p);}
    printf("Particles: %zu\n",ps.size());
    int caps[]={0,100,200,300,420,600};
    int ci=0;
    for(int step=0;step<=600;step++){
        if(ci<6&&step==caps[ci]){
            float minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
            float maxspeed=0;
            for(auto& p:ps){
                minx=std::min(minx,p.x); maxx=std::max(maxx,p.x);
                miny=std::min(miny,p.y); maxy=std::max(maxy,p.y);
                float spd=std::sqrt(p.vx*p.vx+p.vy*p.vy);
                maxspeed=std::max(maxspeed,spd);
            }
            printf("Step %3d: x=[%.1f,%.1f] y=[%.1f,%.1f] maxspeed=%.1f\n",
                step,minx,maxx,miny,maxy,maxspeed);
            ci++;
        }
        if(step==600)break;
        dp(ps); cf(ps); intg(ps);
    }
}
