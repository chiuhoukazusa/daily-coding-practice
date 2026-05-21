#include <cmath>
#include <cstdio>

float acesApprox(float x){float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;return std::max(0.0f,std::min(1.0f,(x*(a*x+b))/(x*(c*x+d)+e)));}
float gamma(float v){return std::pow(std::max(0.0f,v),1.0f/2.2f);}

int main(){
    // Background color
    float bg = 0.02f;
    float mapped = acesApprox(bg);
    float g = gamma(mapped);
    printf("Background 0.02 -> aces=%.4f -> gamma=%.4f -> byte=%d\n", mapped, g, (int)(g*255+0.5f));
    
    // A typical direct light color (e.g., white wall)
    float direct = 0.5f;
    float dm = acesApprox(direct);
    float dg = gamma(dm);
    printf("Direct 0.5 -> aces=%.4f -> gamma=%.4f -> byte=%d\n", dm, dg, (int)(dg*255+0.5f));
    
    // Test: what gives byte=16?
    float target = 16.0f/255.0f;
    printf("Byte 16 = %.4f linear\n", target);
    // reverse gamma
    float linear = std::pow(target, 2.2f);
    printf("After reverse gamma: %.4f\n", linear);
    
    return 0;
}
