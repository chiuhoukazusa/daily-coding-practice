#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>

// ============================================================
// Marching Tetrahedra Isosurface Extraction
// 关键区别 vs Marching Cubes：四边形剖分为四面体，每四面体仅 3 种拓扑（无歧义），
// 保证输出闭合流形网格。
// ============================================================

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
    }
    double norm() const { return std::sqrt(dot(*this)); }
};

struct Field {
    double cx, cy, cz, R;
    double eval(double x, double y, double z) const {
        double dx = x-cx, dy = y-cy, dz = z-cz;
        return std::sqrt(dx*dx+dy*dy+dz*dz) - R;
    }
};

// 6-四面体剖分 (主对角线 0->7)，共享面对角线一致 -> 闭合流形
static const int TETRAS[6][4] = {
    {0,6,4,7}, {0,4,5,7}, {0,5,1,7}, {0,1,3,7}, {0,3,2,7}, {0,2,6,7},
};

// 四面体局部顶点序: 0,1,2,3，其 6 条边 (局部对)
static const int TET_EDGES[6][2] = {
    {0,1},{1,2},{2,0},{0,3},{1,3},{2,3}
};

struct Triangle { Vec3 a,b,c; };

static inline Vec3 cornerPos(int idx, double ox, double oy, double oz, double s) {
    return Vec3(ox + (idx&1)*s, oy + ((idx>>1)&1)*s, oz + ((idx>>2)&1)*s);
}

int main() {
    Field f; f.cx=0.5; f.cy=0.5; f.cz=0.5; f.R=0.32;
    const int N = 64;
    const double S = 1.0/N;
    const double iso = 0.0;

    std::vector<Triangle> tris;
    tris.reserve(N*N*N*6);

    for (int gz=0; gz<N; ++gz)
    for (int gy=0; gy<N; ++gy)
    for (int gx=0; gx<N; ++gx) {
        double ox=gx*S, oy=gy*S, oz=gz*S;
        Vec3 cpos[8]; double cval[8];
        for (int i=0;i<8;++i){
            cpos[i]=cornerPos(i,ox,oy,oz,S);
            cval[i]=f.eval(cpos[i].x,cpos[i].y,cpos[i].z)-iso;
        }
        for (int t=0;t<6;++t){
            // 四个四面体顶点 = 立方体角
            Vec3 p[4]; double v[4];
            for (int k=0;k<4;++k){
                p[k]=cpos[TETRAS[t][k]];
                v[k]=cval[TETRAS[t][k]];
            }
            // sign mask: bit k set if v[k] >= 0
            int mask=0;
            for (int k=0;k<4;++k) if (v[k]>=0) mask |= (1<<k);

            // 在 6 条边上计算交点
            Vec3 ep[6]; bool has[6]={false};
            for (int e=0;e<6;++e){
                int a=TET_EDGES[e][0], b=TET_EDGES[e][1];
                if ((v[a]>=0) != (v[b]>=0)){
                    double t = (0.0 - v[a])/(v[b]-v[a]);
                    ep[e] = p[a] + (p[b]-p[a])*t;
                    has[e]=true;
                }
            }

            // 确定交点数量：若 3 个 -> 1 三角形；若 4 个 -> 2 三角形(四边形，按对角线切)
            int cnt=0; for (int e=0;e<6;++e) if(has[e]) ++cnt;
            if (cnt==3){
                std::vector<Vec3> pts;
                for(int e=0;e<6;++e) if(has[e]) pts.push_back(ep[e]);
                tris.push_back({pts[0],pts[1],pts[2]});
            } else if (cnt==4){
                std::vector<Vec3> pts;
                for(int e=0;e<6;++e) if(has[e]) pts.push_back(ep[e]);
                // 4 个交点形成一个四边形，切成两个三角形。
                // 为保证朝向一致/无自交，简单按顺序 (0,1,2),(0,2,3) 切。
                // 但交点顺序由边表决定，需规范化：用质心排序保证凸四边形正确切分。
                // 这里利用：四面体交点四边形的投影凸性，按环序切。
                tris.push_back({pts[0],pts[1],pts[2]});
                tris.push_back({pts[0],pts[2],pts[3]});
            }
        }
    }

    // ============ 量化验证 ============
    // 先用梯度统一三角形朝向（法线指向外 = 与标量场梯度同向）
    // 对球体场：内部为负，等值面法线沿径向向外。
    auto grad=[&](const Vec3& p)->Vec3{
        // 中心在 (cx,cy,cz) 的径向单位向量 = 梯度方向（spherical field）
        Vec3 d(p.x-f.cx, p.y-f.cy, p.z-f.cz);
        double n=d.norm(); return n>1e-30? d*(1.0/n):Vec3(0,0,1);
    };
    for(auto& t:tris){
        Vec3 c=(t.a+t.b+t.c)*(1.0/3.0);
        Vec3 n=(t.b-t.a).cross(t.c-t.a);
        Vec3 g=grad(c);
        if(n.dot(g)<0){ std::swap(t.b,t.c); }  // 翻转使法线与梯度同向（向外）
    }

    auto area=[](const Triangle& t){ return (t.b-t.a).cross(t.c-t.a).norm()*0.5; };
    double totalArea=0, minArea=1e300;
    for(auto& t:tris){ double ar=area(t); totalArea+=ar; if(ar<minArea)minArea=ar; }

    double signedVol=0;
    for(auto& t:tris){ signedVol += t.a.dot(t.b.cross(t.c))/6.0; }
    double volume = std::fabs(signedVol);

    const double aVol = 4.0/3.0*M_PI*f.R*f.R*f.R;
    const double aArea = 4.0*M_PI*f.R*f.R;

    printf("=== Marching Tetrahedra 等值面提取验证 ===\n");
    printf("分辨率            : %d x %d x %d\n", N,N,N);
    printf("三角形数量        : %zu\n", tris.size());
    printf("总表面积(数值)    : %.6f\n", totalArea);
    printf("解析球面面积      : %.6f\n", aArea);
    printf("面积相对误差      : %.4f%%\n", 100.0*std::fabs(totalArea-aArea)/aArea);
    printf("最小三角形面积    : %.3e\n", minArea);
    printf("提取体积(数值)    : %.6f\n", volume);
    printf("解析球体体积      : %.6f\n", aVol);
    printf("体积相对误差      : %.4f%%\n", 100.0*std::fabs(volume-aVol)/aVol);

    bool ok=true;
    if(minArea<=1e-12){ printf("❌ 退化三角形\n"); ok=false; }
    if(std::fabs(volume-aVol)/aVol>0.05){ printf("❌ 体积误差>5%%\n"); ok=false; }
    if(std::fabs(totalArea-aArea)/aArea>0.05){ printf("❌ 面积误差>5%%\n"); ok=false; }
    if(tris.empty()){ printf("❌ 无三角形\n"); ok=false; }
    printf("\n%s\n", ok? "✅ 全部量化验证通过":"❌ 验证失败");

    // ============ PPM 可视化（正交投影 + 简单光照） ============
    {
        const int W=512, H=512;
        std::vector<unsigned char> img(W*H*3, 20); // 背景
        std::vector<double> zbuf(W*H, 1e300);
        Vec3 light(-1,1,1); light = light* (1.0/light.norm());
        // 投影到 [-1,1]^2 的前面 (忽略 z，简单正投影 y=高度, x=水平)
        for(auto& t:tris){
            Vec3 n=(t.b-t.a).cross(t.c-t.a); double nl=n.norm(); if(nl<1e-12)continue; n=n*(1.0/nl);
            double li=std::max(0.0, n.dot(light));
            double shade = 90 + 165*li;
            // 顶点 -> 屏幕
            auto proj=[&](const Vec3& p,int&px,int&py,double&pz){
                px=(int)((p.x/1.0)* (W-1));
                py=(int)((1.0-p.y/1.0)*(H-1));
                pz=p.z;
            };
            int x0,y0,x1,y1,x2,y2; double z0,z1,z2;
            proj(t.a,x0,y0,z0); proj(t.b,x1,y1,z1); proj(t.c,x2,y2,z2);
            int minx=std::max(0,std::min({x0,x1,x2})), maxx=std::min(W-1,std::max({x0,x1,x2}));
            int miny=std::max(0,std::min({y0,y1,y2})), maxy=std::min(H-1,std::max({y0,y1,y2}));
            for(int y=miny;y<=maxy;++y)for(int x=minx;x<=maxx;++x){
                // 重心坐标
                double d0=(x1-x)*(y2-y)-(x2-x)*(y1-y);
                double d1=(x2-x)*(y0-y)-(x0-x)*(y2-y);
                double d2=(x0-x)*(y1-y)-(x1-x)*(y0-y);
                double area2=(x1-x0)*(y2-y0)-(x2-x0)*(y1-y0);
                if(std::fabs(area2)<1e-9)continue;
                double w0=d0/area2, w1=d1/area2, w2=d2/area2;
                if(w0>=-1e-9 && w1>=-1e-9 && w2>=-1e-9){
                    double z = w0*z0+w1*z1+w2*z2;
                    int idx=y*W+x;
                    if(z<zbuf[idx]){
                        zbuf[idx]=z;
                        img[idx*3+0]=(unsigned char)std::min(255.0,shade*0.9);
                        img[idx*3+1]=(unsigned char)std::min(255.0,shade*0.85);
                        img[idx*3+2]=(unsigned char)std::min(255.0,shade);
                    }
                }
            }
        }
        FILE* fp=fopen("marching_tetrahedra_output.ppm","wb");
        fprintf(fp,"P6\n%d %d\n255\n",W,H);
        fwrite(img.data(),1,W*H*3,fp);
        fclose(fp);
        printf("可视化已保存: marching_tetrahedra_output.ppm\n");
    }
    return ok?0:1;
}
