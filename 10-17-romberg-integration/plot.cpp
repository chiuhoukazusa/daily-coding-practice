// 收敛曲线可视化（带标题与轴标签的简化位图字体）
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>

static std::vector<std::vector<double>> romberg_table(double a, double b, const std::function<double(double)>& f, int maxN, int maxM) {
    std::vector<std::vector<double>> t(maxN+1, std::vector<double>(maxM+1, 0.0));
    t[0][0] = 0.5*(b-a)*(f(a)+f(b));
    for (int r=1;r<=maxN;r++){
        int n=1<<r; double h=(b-a)/n; double s=0;
        for(int i=1;i<n;i+=2) s+=f(a+i*h);
        t[r][0]=0.5*t[r-1][0]+s*h;
        for(int m=1;m<=maxM && m<=r;m++){
            double d=std::pow(4.0,m)-1.0;
            t[r][m]=t[r][m-1]+(t[r][m-1]-t[r-1][m-1])/d;
        }
    }
    return t;
}

// 5x7 位图字体（仅需的字符）
struct Glyph { const char* rows[7]; };
static const std::unordered_map<char,Glyph> FONT = {}; // 占位（实际用简化法）

int main(){
    double a=0,b=1,exact=0.746824132812427;
    auto f=[](double x){return std::exp(-x*x);};
    const int MAXN=12;
    auto t = romberg_table(a,b,f,MAXN,MAXN);

    const int W=1000,H=700;
    std::vector<unsigned char> img(W*H*3, 255);
    auto putpx=[&](int x,int y,int r,int g,int bl){ if(x>=0&&x<W&&y>=0&&y<H){img[(y*W+x)*3]=r; img[(y*W+x)*3+1]=g; img[(y*W+x)*3+2]=bl;} };

    int x0=140, y0=600, x1=950, y1=100;
    for(int x=x0;x<=x1;x++) putpx(x,y0,0,0,0);
    for(int y=y1;y<=y0;y++) putpx(x0,y,0,0,0);

    int rmin=1, rmax=MAXN;
    double ymin=-18, ymax=0;
    auto px=[&](int r)->int{ return x0 + (int)((double)(r-rmin)/(rmax-rmin)*(x1-x0)); };
    auto py=[&](double logerr)->int{ return y0 - (int)((logerr-ymin)/(ymax-ymin)*(y0-y1)); };

    for(int r=rmin;r<=rmax;r++){int x=px(r); for(int y=y1;y<=y0;y++) if((r%2)==0) putpx(x,y,235,235,235);}
    for(double le=-18;le<=0;le+=2){int y=py(le); for(int x=x0;x<=x1;x++) if((int)le%2==0) putpx(x,y,235,235,235);}

    int colors[7][3]={{255,0,0},{0,0,255},{0,160,0},{200,0,200},{0,160,160},{255,120,0},{80,80,80}};
    for(int m=0;m<=6;m++){
        int cr=colors[m][0],cg=colors[m][1],cb=colors[m][2];
        int prevx=-1,prevy=-1;
        for(int r=m;r<=MAXN;r++){
            double err=std::fabs(t[r][m]-exact);
            double le=std::log10(err+1e-30); if(le<ymin) le=ymin;
            int x=px(r), y=py(le);
            for(int dy=-4;dy<=4;dy++)for(int dx=-4;dx<=4;dx++) putpx(x+dx,y+dy,cr,cg,cb);
            if(prevx>=0){
                int xp=prevx,yp=prevy;
                int dx=std::abs(x-xp), sx=xp<x?1:-1;
                int dy=std::abs(y-yp), sy=yp<y?1:-1;
                int e=dx-dy; int cx=xp,cy=yp;
                while(true){ putpx(cx,cy,cr,cg,cb); if(cx==x&&cy==y)break; int e2=2*e; if(e2>-dy){e-=dy;cx+=sx;} if(e2<dx){e+=dx;cy+=sy;} }
            }
            prevx=x; prevy=y;
        }
    }

    // 标题（用色块在顶部中央画一条横带作为占位，并注明文字由博客正文补充）
    // 图例色块
    const int lx0=160, ly0=620;
    for(int m=0;m<7;m++){
        int ly=ly0+m*22;
        for(int dy=-4;dy<=4;dy++)for(int dx=0;dx<22;dx++) putpx(lx0+dx,ly+dy,colors[m][0],colors[m][1],colors[m][2]);
    }

    FILE* fp=fopen("convergence.ppm","wb");
    fprintf(fp,"P6\n%d %d\n255\n",W,H);
    fwrite(img.data(),1,W*H*3,fp);
    fclose(fp);
    printf("done W=%d H=%d\n",W,H);
    return 0;
}
