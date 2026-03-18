// 快速调试：检查球面沿垂直条带的着色连续性
// 对中心球（x=0, y=1, z=0, r=0.85）从顶部到侧面采样法线+着色值
#include <cmath>
#include <cstdio>

static const double PI = 3.14159265358979;

struct Vec3 {
    double x,y,z;
    Vec3(double x=0,double y=0,double z=0):x(x),y(y),z(z){}
    Vec3 operator+(Vec3 b)const{return{x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(Vec3 b)const{return{x-b.x,y-b.y,z-b.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    double dot(Vec3 b)const{return x*b.x+y*b.y+z*b.z;}
    double len()const{return sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{double l=len();return{x/l,y/l,z/l};}
};

inline double clamp(double v,double lo,double hi){return v<lo?lo:(v>hi?hi:v);}

// 模拟 computeLTCParams（与 main.cpp 相同）
struct LTCParams { double m11,m22,m13,amplitude; };
LTCParams computeLTCParams(double roughness, double NoV) {
    double alpha = roughness*roughness;
    LTCParams p;
    double a = alpha;
    p.m11 = 1.0 + a*(0.5 + a*(-1.0 + a*1.5));
    p.m22 = 1.0 + a*(0.5 + a*(-0.5));
    p.m13 = -a*(1.0-NoV)*0.5;
    p.amplitude = 1.0 - a*(0.5 - a*0.3);
    if (alpha < 0.25) {
        double t = alpha/0.25;
        p.amplitude = 0.25*(1-t*t)*4.0 + t*t*p.amplitude;
        // (simplified)
    }
    return p;
}

int main(){
    // 沿球面从顶部(theta=0)到侧面(theta=90度)，步长5度
    // 相机在 (0, 2.5, 6)，球心 (0,1,0)
    Vec3 camPos(0, 2.5, 6.0);
    Vec3 sphereCenter(0, 1.0, 0);
    double r = 0.85;
    double roughness = 0.1; // 最低 roughness，最容易出问题

    printf("theta | NoV  | NoL  | params.m11 | amplitude | branch\n");
    printf("------|------|------|------------|-----------|-------\n");
    for (int deg = 0; deg <= 85; deg += 5) {
        double theta = deg * PI / 180.0;
        // 法线在 XZ 平面内旋转（从顶部 Y 到侧面 Z）
        Vec3 N(0, cos(theta), sin(theta)); // 球面法线（近似）
        Vec3 P = sphereCenter + N * r;
        Vec3 V = (camPos - P).norm();
        double NoV = clamp(N.dot(V), 0.0, 1.0);
        
        // 光源方向（主光在 0,7,0）
        Vec3 lightPos(0, 7.0, 0);
        Vec3 L = (lightPos - P).norm();
        double NoL = clamp(N.dot(L), 0.0, 1.0);
        
        LTCParams params = computeLTCParams(roughness, NoV);
        const char* branch = roughness < 0.08 ? "RepPoint" : "LTC";
        
        printf("%5d | %.3f| %.3f| %.6f   | %.5f   | %s\n",
               deg, NoV, NoL, params.m11, params.amplitude, branch);
    }
    return 0;
}
