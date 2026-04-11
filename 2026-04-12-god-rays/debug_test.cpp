#include <cmath>
#include <iostream>

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 norm() const { float l = len(); return (*this)*(1.0f/l); }
};

float miePhase(float cosT, float g=0.72f) {
    float g2=g*g, denom=1+g2-2*g*cosT;
    return (1-g2)/(4*3.14159f*denom*sqrtf(denom));
}

int main() {
    // 模拟：从相机出发，沿一条不被遮挡的光线积分
    Vec3 ro = {0, 1.2f, -3.5f};
    Vec3 rd = {0.3f, -0.1f, 1.0f};
    float rdLen = rd.len(); rd = rd.norm();
    
    Vec3 LIGHT_POS = {5, 7, -5};
    Vec3 LIGHT_COLOR = {3.5f, 2.8f, 1.6f};
    
    Vec3 toSun = (LIGHT_POS - ro).norm();
    float cosT = rd.dot(toSun);
    float phase = 0.6f * miePhase(cosT, 0.72f) + 0.4f * (3.0f/(16*3.14159f))*(1+cosT*cosT);
    
    std::cout << "cosTheta = " << cosT << ", phase = " << phase << "\n";
    
    int STEPS = 96;
    float marchDist = 22.0f;
    float stepSize = marchDist / STEPS;
    float sigma_s = 0.12f, sigma_t = 0.13f;
    float transmittance = 1.0f;
    Vec3 accumulated = {0,0,0};
    
    for (int i = 0; i < STEPS; ++i) {
        float t = (i + 0.5f) * stepSize;
        Vec3 pos = ro + rd * t;
        
        // 假设不在阴影中
        float d = (LIGHT_POS - pos).len();
        float lightAtten = 50.0f / (d*d + 10.0f);
        
        Vec3 Ls = LIGHT_COLOR * (lightAtten * phase * sigma_s * transmittance * stepSize);
        accumulated.x += Ls.x;
        accumulated.y += Ls.y;
        accumulated.z += Ls.z;
        
        transmittance *= expf(-sigma_t * stepSize);
        if (transmittance < 0.002f) break;
    }
    
    std::cout << "无遮挡散射积分: (" << accumulated.x << ", " << accumulated.y << ", " << accumulated.z << ")\n";
    std::cout << "乘以5.0倍: (" << accumulated.x*5 << ", " << accumulated.y*5 << ", " << accumulated.z*5 << ")\n";
    std::cout << "期望：若值 > 0.1 则应有可见效果\n";
    
    // 再模拟一条靠近光轴的光线（前向散射方向）
    Vec3 rd2 = (LIGHT_POS - ro).norm() * (-1); // 直接朝着光源对侧
    rd2.x = -rd2.x; rd2.y = -rd2.y; rd2.z = -rd2.z;
    float cosT2 = rd2.dot(toSun);
    std::cout << "\n朝光源看方向 cosTheta=" << cosT2 << ", phase=" << miePhase(cosT2, 0.72f) << "\n";
    
    (void)rdLen;
    return 0;
}
