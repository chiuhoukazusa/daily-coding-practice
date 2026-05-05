// 快速验证：hit point 是否收到光子
#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

struct Vec3 {
    double x,y,z;
    Vec3(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    double len2()const{return x*x+y*y+z*z;}
};

// 光子分布在地板 y=-1 上
// Hit points 也在地板 y=-1 上
// 半径 0.1，在 2x2 地板上随机 10000 个 HPs，10000 个光子
int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> zdist(0.0, 2.0);
    
    int N = 1000;
    double R2 = 0.1 * 0.1;
    
    // 光子在地板
    std::vector<Vec3> photons(N);
    for (auto& p : photons) p = {dist(rng), -1.0, zdist(rng)};
    
    // Hit points 在地板
    std::vector<Vec3> hps(N);
    for (auto& h : hps) h = {dist(rng), -1.0, zdist(rng)};
    
    // 暴力查询每个 HP
    int totalFound = 0;
    int zeroFound = 0;
    for (auto& hp : hps) {
        int found = 0;
        for (auto& ph : photons) {
            Vec3 d = hp - ph;
            if (d.len2() <= R2) found++;
        }
        totalFound += found;
        if (found == 0) zeroFound++;
    }
    
    printf("N=%d, R=%.2f\n", N, std::sqrt(R2));
    printf("HP receiving 0 photons: %d/%d (%.1f%%)\n", zeroFound, N, 100.0*zeroFound/N);
    printf("Avg photons per HP: %.2f\n", (double)totalFound/N);
    // 理论期望：N * π*R² / (2*2) = 1000 * π*0.01 / 4 ≈ 7.85
    double expected = N * M_PI * R2 / (2.0 * 2.0);
    printf("Expected: %.2f\n", expected);
    
    return 0;
}
