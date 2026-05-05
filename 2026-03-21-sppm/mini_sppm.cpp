// 迷你 SPPM 测试：验证 kd树查询和密度估计
#include <cmath>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <random>

struct Vec3 {
    double x,y,z;
    Vec3(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    Vec3 operator*(const Vec3&o)const{return{x*o.x,y*o.y,z*o.z};}
    double len2()const{return x*x+y*y+z*z;}
};

struct Photon { Vec3 pos, power; };

struct KdNode { Photon ph; int left,right,axis; };

struct KdTree {
    std::vector<KdNode> nodes;
    std::vector<int> idx;
    int root=-1;
    
    void build(std::vector<Photon>& phs) {
        if(phs.empty())return;
        idx.resize(phs.size());
        for(int i=0;i<(int)phs.size();i++)idx[i]=i;
        nodes.resize(phs.size());
        for(int i=0;i<(int)phs.size();i++){nodes[i].ph=phs[i];nodes[i].left=nodes[i].right=-1;}
        root=buildR(phs,0,(int)idx.size(),0);
    }
    
    int buildR(std::vector<Photon>& phs, int lo, int hi, int d) {
        if(lo>=hi)return -1;
        int axis=d%3, mid=(lo+hi)/2;
        std::nth_element(idx.begin()+lo,idx.begin()+mid,idx.begin()+hi,
            [&](int a,int b){
                if(axis==0)return phs[a].pos.x<phs[b].pos.x;
                if(axis==1)return phs[a].pos.y<phs[b].pos.y;
                return phs[a].pos.z<phs[b].pos.z;
            });
        int ni=idx[mid];
        nodes[ni].axis=axis;
        nodes[ni].left=buildR(phs,lo,mid,d+1);
        nodes[ni].right=buildR(phs,mid+1,hi,d+1);
        return ni;
    }
    
    void query(int ni, const Vec3& pos, double r2, std::vector<int>& res)const{
        if(ni<0)return;
        const KdNode& n=nodes[ni];
        Vec3 d=pos-n.ph.pos;
        if(d.len2()<=r2)res.push_back(ni);
        double diff;
        if(n.axis==0)diff=d.x;
        else if(n.axis==1)diff=d.y;
        else diff=d.z;
        if(diff*diff<=r2){
            query(n.left,pos,r2,res);
            query(n.right,pos,r2,res);
        } else if(diff<0){
            query(n.left,pos,r2,res);
        } else {
            query(n.right,pos,r2,res);
        }
    }
};

int main() {
    // 创建 1000 个随机光子，均匀分布在 [-1,1]^3
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    
    std::vector<Photon> photons(1000);
    for(auto& ph : photons) {
        ph.pos = {dist(rng), dist(rng), dist(rng)};
        ph.power = {1,1,1};
    }
    
    KdTree kdt;
    kdt.build(photons);
    
    // 查询原点附近 r=0.2 内的光子
    Vec3 queryPos(0,0,0);
    double r2 = 0.2*0.2;
    std::vector<int> results;
    kdt.query(kdt.root, queryPos, r2, results);
    
    printf("KdTree query r=0.2: found %d photons (expected ~4π/3*r³*n ≈ %.0f)\n",
           (int)results.size(), 4.0/3.0*M_PI*0.2*0.2*0.2 * 1000 / 8.0);
    
    // 暴力验证
    int brute=0;
    for(auto& ph : photons) {
        Vec3 d = queryPos - ph.pos;
        if(d.len2() <= r2) brute++;
    }
    printf("Brute force: %d\n", brute);
    printf("Match: %s\n", (results.size()==(size_t)brute)?"✅":"❌");
    
    // 测试 hit point 处查询（模拟背墙漫反射点 z=1.0 附近）
    Vec3 wallPos(0.0, 0.0, 0.999);
    double wallR2 = 0.1*0.1;
    std::vector<int> wallRes;
    kdt.query(kdt.root, wallPos, wallR2, wallRes);
    printf("\nQuery near back wall (0,0,1) r=0.1: %d photons\n", (int)wallRes.size());
    
    // 测试 SPPM 更新
    double alpha=0.7;
    double radius2 = 0.04*0.04*4.0; // 初始半径² = (0.08)²
    double initR = std::sqrt(radius2);
    printf("\nInitial search radius: %.4f\n", initR);
    
    int photonCount=0;
    Vec3 flux={0,0,0};
    Vec3 weight={0.73,0.73,0.73}; // 白色漫反射
    Vec3 pos_hp = {0.0, 0.0, 0.999}; // 背墙 hit point
    
    for(int iter=0;iter<5;iter++){
        results.clear();
        kdt.query(kdt.root, pos_hp, radius2, results);
        int m=(int)results.size();
        int n=photonCount;
        if(n+m>0){
            Vec3 newFlux={0,0,0};
            for(int ni:results) newFlux=newFlux+photons[ni].power;
            double ratio=(n+alpha*m)/(double)(n+m);
            radius2*=ratio;
            flux=(flux+weight*newFlux)*ratio;
            photonCount+=(int)(alpha*m);
        }
        printf("Iter %d: found %d photons, radius=%.4f, flux=(%.4f,%.4f,%.4f), photonCount=%d\n",
               iter+1, m, std::sqrt(radius2), flux.x,flux.y,flux.z, photonCount);
    }
    
    return 0;
}
