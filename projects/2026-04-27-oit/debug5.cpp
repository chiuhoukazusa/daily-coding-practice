#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

inline float ef(float ax,float ay,float bx,float by,float px,float py){
    return (px-ax)*(by-ay)-(py-ay)*(bx-ax);
}

int main(){
    // Red panel tri screen coords: (269,387) (389,424) (387,178)
    float sx[3]={269,389,387};
    float sy[3]={387,424,178};
    float nz[3]={0.87f,0.85f,0.82f};
    
    float e1x=sx[1]-sx[0], e1y=sy[1]-sy[0];
    float e2x=sx[2]-sx[0], e2y=sy[2]-sy[0];
    float area = e1x*e2y - e1y*e2x;
    printf("area=%.1f\n",area);
    float sf = (area>0)?1:-1;
    printf("signFactor=%.0f\n",sf);
    
    // centroid
    float cx=(sx[0]+sx[1]+sx[2])/3;
    float cy=(sy[0]+sy[1]+sy[2])/3;
    float w0=ef(sx[1],sy[1],sx[2],sy[2],cx,cy)*sf;
    float w1=ef(sx[2],sy[2],sx[0],sy[0],cx,cy)*sf;
    float w2=ef(sx[0],sy[0],sx[1],sy[1],cx,cy)*sf;
    printf("At centroid (%.0f,%.0f): w0=%.2f w1=%.2f w2=%.2f\n",cx,cy,w0,w1,w2);
    printf("Inside? %s\n", (w0>=0&&w1>=0&&w2>=0)?"YES":"NO");

    int W=800, H=600;
    int minX=(int)std::max(0.0f,std::min({sx[0],sx[1],sx[2]}));
    int maxX=(int)std::min((float)(W-1),std::max({sx[0],sx[1],sx[2]}));
    int minY=(int)std::max(0.0f,std::min({sy[0],sy[1],sy[2]}));
    int maxY=(int)std::min((float)(H-1),std::max({sy[0],sy[1],sy[2]}));
    printf("BBox: x[%d,%d] y[%d,%d], size=%d\n",minX,maxX,minY,maxY,(maxX-minX+1)*(maxY-minY+1));
    
    int hit=0, depthFail=0;
    for(int py=minY;py<=maxY;py++) for(int px=minX;px<=maxX;px++){
        float bw0=ef(sx[1],sy[1],sx[2],sy[2],(float)px,(float)py)*sf;
        float bw1=ef(sx[2],sy[2],sx[0],sy[0],(float)px,(float)py)*sf;
        float bw2=ef(sx[0],sy[0],sx[1],sy[1],(float)px,(float)py)*sf;
        if(bw0<0||bw1<0||bw2<0) continue;
        float denom=bw0+bw1+bw2;
        if(denom<1e-8f) continue;
        float z=bw0/denom*nz[0]+bw1/denom*nz[1]+bw2/denom*nz[2];
        float opaqueDepth = 1e9f; // background
        if(z>=opaqueDepth){depthFail++; continue;}
        hit++;
    }
    printf("Hit=%d depthFail=%d\n",hit,depthFail);
    return 0;
}
