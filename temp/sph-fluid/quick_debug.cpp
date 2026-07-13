#include <cmath>
#include <vector>
#include <cstdio>
#include <algorithm>

static const float H = 0.2f, H2 = H*H, MASS=0.02f;
static const float REST_DENS=1000.f, GAS_CONST=500.f, VISC=1.f, GRAVITY=9.8f;
static const float DT=0.001f;
static const float PI = 3.14159265358979f;
static const float POLY6K    = 4.0f / (PI * std::pow(H, 8));
static const float SPIKY_GRAD= -10.0f / (PI * std::pow(H, 5));
static const float VISC_LAP  =  40.0f / (PI * std::pow(H, 5));
static const float DOM_W=4.f, DOM_H=3.f;

struct Vec2{float x,y;
  Vec2(float x=0,float y=0):x(x),y(y){}
  Vec2 operator+(const Vec2&o)const{return{x+o.x,y+o.y};}
  Vec2 operator-(const Vec2&o)const{return{x-o.x,y-o.y};}
  Vec2 operator*(float s)const{return{x*s,y*s};}
  Vec2& operator+=(const Vec2&o){x+=o.x;y+=o.y;return*this;}
  float len2()const{return x*x+y*y;}
  float len()const{return sqrt(len2());}
};
struct Particle{Vec2 pos,vel,force;float density,pressure;};

float kpoly6(float r2){if(r2>=H2)return 0;float d=H2-r2;return POLY6K*d*d*d;}
Vec2 kspiky(const Vec2& rij,float r){if(r<=1e-6f||r>=H)return{0,0};float d=H-r;return rij*(SPIKY_GRAD*d*d/r);}
float kvisc(float r){if(r>=H)return 0;return VISC_LAP*(H-r);}

int main(){
    printf("POLY6K=%e SPIKY_GRAD=%e VISC_LAP=%e\n",POLY6K,SPIKY_GRAD,VISC_LAP);
    
    // Test density with 2 particles at distance 0
    printf("poly6(0)=%e\n",kpoly6(0));
    printf("Expected density for 1 neighbor at r=0: %e (need ~1000)\n", MASS*kpoly6(0));
    
    // How many neighbors fit in H radius with spacing H*0.75?
    float spacing = H*0.75f;
    float sum=0;
    for(float dx=-2*H;dx<=2*H;dx+=spacing){
        for(float dy=-2*H;dy<=2*H;dy+=spacing){
            float r2=dx*dx+dy*dy;
            sum+=kpoly6(r2);
        }
    }
    printf("Sum poly6 over %dx grid: %e\n", (int)(4*H/spacing+1), sum);
    printf("Density estimate (MASS*sum): %e (need ~1000)\n", MASS*sum);
    
    // So we need MASS to be about 1000/sum
    printf("Suggested MASS = %.4f\n", 1000.0f/sum);
    return 0;
}
