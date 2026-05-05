#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>

inline float ef(float ax,float ay,float bx,float by,float px,float py){
    return (px-ax)*(by-ay)-(py-ay)*(bx-ax);
}

int main(){
    // Tri screen: (269,387) (389,424) (387,178) - CW (area=-29422)
    float sx[3]={269,389,387};
    float sy[3]={387,424,178};
    
    // Center of triangle
    float cx=(sx[0]+sx[1]+sx[2])/3;
    float cy=(sy[0]+sy[1]+sy[2])/3;
    printf("Triangle centroid: (%.1f, %.1f)\n", cx, cy);
    
    // Test centroid
    float w0=ef(sx[1],sy[1],sx[2],sy[2],cx,cy);
    float w1=ef(sx[2],sy[2],sx[0],sy[0],cx,cy);
    float w2=ef(sx[0],sy[0],sx[1],sy[1],cx,cy);
    printf("At centroid: w0=%.2f w1=%.2f w2=%.2f\n",w0,w1,w2);
    
    bool ccw = false; // area < 0
    bool inside;
    float tw0=w0,tw1=w1,tw2=w2;
    if(!ccw){inside=(w0<=0&&w1<=0&&w2<=0);}
    else{inside=(w0>=0&&w1>=0&&w2>=0);}
    printf("Inside (double-sided, ccw=false): %s\n", inside?"YES":"NO");
    
    // What if we flip: check all negative
    printf("w0<=0: %s, w1<=0: %s, w2<=0: %s\n",
           w0<=0?"Y":"N", w1<=0?"Y":"N", w2<=0?"Y":"N");

    // Also check: maybe the edgeFunc definition is causing issues
    // ef(a,b) = (p-a)x(b-a)
    // For CW winding, inside points have all edge funcs NEGATIVE
    // Let's verify with known inside point (centroid)
    printf("\n--- raw edge function ---\n");
    printf("ef(v1,v2, centroid) = %.2f (expect <0 for CW)\n", w0);
    printf("ef(v2,v0, centroid) = %.2f (expect <0 for CW)\n", w1);
    printf("ef(v0,v1, centroid) = %.2f (expect <0 for CW)\n", w2);
    
    return 0;
}
