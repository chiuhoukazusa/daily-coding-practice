// Visualization: render the two polygons A (red) and B (green), and shade the
// boolean result regions (union/intersection/difference) into a PPM image.
// Uses a pixel-grid point-in-polygon rasterization (independent of the area solver).
#include <bits/stdc++.h>
using namespace std;
typedef long double ld;
const ld EPS=1e-12;
struct Pt{ld x,y;};
ld cross(const Pt&a,const Pt&b){return a.x*b.y-a.y*b.x;}
int ori(const Pt&a,const Pt&b,const Pt&c){ld v=cross({b.x-a.x,b.y-a.y},{c.x-a.x,c.y-a.y});if(fabs(v)<EPS)return 0;return v>0?1:-1;}
bool onSeg(const Pt&p,const Pt&a,const Pt&b){return ori(a,b,p)==0&&min(a.x,b.x)-EPS<=p.x&&p.x<=max(a.x,b.x)+EPS&&min(a.y,b.y)-EPS<=p.y&&p.y<=max(a.y,b.y)+EPS;}
bool inPoly(const Pt&p,const vector<Pt>&P){int n=P.size();bool in=false;for(int i=0,j=n-1;i<n;j=i++){if(onSeg(p,P[j],P[i]))return true;bool c1=(P[i].y>p.y)!=(P[j].y>p.y);if(c1){ld xi=P[i].x+(p.y-P[i].y)*(P[j].x-P[i].x)/(P[j].y-P[i].y);if(p.x<xi)in=!in;}}return in;}

int main(){
    vector<Pt> A={{0,0},{4,0},{4,4},{0,4}};
    vector<Pt> B={{2,2},{6,2},{6,6},{2,6}};
    int W=800,H=800;
    // world bounds x[-0.5,6.5], y[-0.5,6.5]
    ld xmin=-0.5,xmax=6.5,ymin=-0.5,ymax=6.5;
    auto toPix=[&](Pt p,int&px,int&py){px=(int)round((p.x-xmin)/(xmax-xmin)*(W-1));py=(int)round((ymax-p.y)/(ymax-ymin)*(H-1));};
    vector<unsigned char> img(W*H*3,255); // white background
    auto setPix=[&](int px,int py,int r,int g,int b){if(px<0||px>=W||py<0||py>=H)return;int idx=(py*W+px)*3;img[idx]=r;img[idx+1]=g;img[idx+2]=b;};

    // Shade boolean regions via raster sampling
    for(int py=0;py<H;py++)for(int px=0;px<W;px++){
        ld x=xmin+(ld)px/(W-1)*(xmax-xmin);
        ld y=ymax-(ld)py/(H-1)*(ymax-ymin);
        Pt p{x,y};
        bool a=inPoly(p,A), b=inPoly(p,B);
        int r=255,g=255,bb=255;
        if(a&&b){r=255;g=200;bb=0;}   // intersection = amber
        else if(a){r=255;g=120;bb=120;} // A only = light red
        else if(b){r=120;g=255;bb=120;} // B only = light green
        setPix(px,py,r,g,bb);
    }

    // Draw polygon edges (outlines)
    auto drawSeg=[&](Pt a,Pt b,int r,int g,int bb){
        int x0,y0,x1,y1;toPix(a,x0,y0);toPix(b,x1,y1);
        int dx=abs(x1-x0),dy=-abs(y1-y0),sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx+dy;
        while(true){setPix(x0,y0,r,g,bb);if(x0==x1&&y0==y1)break;int e2=2*err;if(e2>=dy){err+=dy;x0+=sx;}if(e2<=dx){err+=dx;y0+=sy;}}
    };
    for(size_t i=0;i<A.size();i++)drawSeg(A[i],A[(i+1)%A.size()],200,30,30);
    for(size_t i=0;i<B.size();i++)drawSeg(B[i],B[(i+1)%B.size()],30,160,30);

    // write PPM
    FILE*f=fopen("polygon_boolean_output.ppm","wb");
    fprintf(f,"P6\n%d %d\n255\n",W,H);
    fwrite(img.data(),1,W*H*3,f);
    fclose(f);
    printf("wrote polygon_boolean_output.ppm (%dx%d)\n",W,H);
    return 0;
}
