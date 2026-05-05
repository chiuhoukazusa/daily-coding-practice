#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>

static const double PI = 3.14159265;

struct Vec3 {
    double x,y,z;
    Vec3(double a=0,double b=0,double c=0):x(a),y(b),z(c){}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator*(double t)const{return{x*t,y*t,z*t};}
    double len2()const{return x*x+y*y+z*z;}
    double len()const{return std::sqrt(len2());}
};

struct Photon { Vec3 pos, power, dir; };

struct KdNode {
    int photonIdx;
    int left, right;
    int splitAxis;
};

struct KdTree {
    std::vector<KdNode> nodes;
    std::vector<Photon>* pp = nullptr;
    std::vector<int> bidx;
    int root = -1;
    int nc = 0;
    
    void build(std::vector<Photon>& phs) {
        if(phs.empty()){root=-1;nc=0;return;}
        pp=&phs;
        bidx.resize(phs.size());
        std::iota(bidx.begin(),bidx.end(),0);
        nodes.resize(phs.size());
        nc=0;
        root=buildR(0,(int)bidx.size(),0);
    }
    
    int buildR(int lo,int hi,int d){
        if(lo>=hi)return -1;
        int axis=d%3,mid=(lo+hi)/2;
        std::nth_element(bidx.begin()+lo,bidx.begin()+mid,bidx.begin()+hi,
            [&](int a,int b){
                auto&pa=(*pp)[a].pos,&pb=(*pp)[b].pos;
                if(axis==0)return pa.x<pb.x;
                if(axis==1)return pa.y<pb.y;
                return pa.z<pb.z;
            });
        int ni=nc++;
        nodes[ni].photonIdx=bidx[mid];
        nodes[ni].splitAxis=axis;
        nodes[ni].left=buildR(lo,mid,d+1);
        nodes[ni].right=buildR(mid+1,hi,d+1);
        return ni;
    }
    
    void query(int ni,const Vec3& pos,double r2,std::vector<int>& res)const{
        if(ni<0||!pp)return;
        auto& n=nodes[ni];
        auto& ph=(*pp)[n.photonIdx];
        Vec3 d=pos-ph.pos;
        if(d.len2()<=r2)res.push_back(n.photonIdx);
        double diff;
        if(n.splitAxis==0)diff=d.x;
        else if(n.splitAxis==1)diff=d.y;
        else diff=d.z;
        if(diff*diff<=r2){query(n.left,pos,r2,res);query(n.right,pos,r2,res);}
        else if(diff<0)query(n.left,pos,r2,res);
        else query(n.right,pos,r2,res);
    }
};

int main(){
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> xd(-1.0,1.0);
    std::uniform_real_distribution<double> zd(0.0,2.0);
    
    // 模拟地板光子（y=-1）
    int N=5000;
    std::vector<Photon> phs(N);
    for(auto& p:phs){p.pos={xd(rng),-1.0,zd(rng)};p.power={1,1,1};p.dir={0,-1,0};}
    
    KdTree kdt;
    kdt.build(phs);
    
    // 查询地板中心 (0,-1,1)
    Vec3 qpos(0,-1.0,1.0);
    double r2=0.1*0.1;
    std::vector<int> res;
    kdt.query(kdt.root,qpos,r2,res);
    
    // 暴力验证
    int brute=0;
    for(auto& p:phs){Vec3 d=qpos-p.pos;if(d.len2()<=r2)brute++;}
    
    printf("KdTree: %d, Brute: %d, Match: %s\n",
           (int)res.size(),brute,(res.size()==(size_t)brute)?"✅":"❌");
    
    // 更大半径
    r2=0.2*0.2;
    res.clear();
    kdt.query(kdt.root,qpos,r2,res);
    brute=0;
    for(auto& p:phs){Vec3 d=qpos-p.pos;if(d.len2()<=r2)brute++;}
    printf("r=0.2: KdTree=%d, Brute=%d, Match=%s\n",
           (int)res.size(),brute,(res.size()==(size_t)brute)?"✅":"❌");
    
    // 模拟几个随机查询
    std::uniform_real_distribution<double> rx(-0.8,0.8),rz(0.2,1.8);
    int errors=0;
    for(int i=0;i<100;i++){
        Vec3 p={rx(rng),-1.0,rz(rng)};
        r2=0.15*0.15;
        res.clear();
        kdt.query(kdt.root,p,r2,res);
        brute=0;
        for(auto& ph:phs){Vec3 d=p-ph.pos;if(d.len2()<=r2)brute++;}
        if((int)res.size()!=brute)errors++;
    }
    printf("100 random queries, errors: %d\n",errors);
    
    return 0;
}
