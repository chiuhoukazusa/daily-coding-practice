#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
struct G{int w,h;vector<bool>b;G(int w_,int h_):w(w_),h(h_),b(w*h,false){}
    bool op(int x,int y)const{return x<0||x>=w||y<0||y>=h||b[y*w+x];}
    void s(int x,int y,bool v){b[y*w+x]=v;}};
struct Pt{int x,y;bool operator==(const Pt&o)const{return x==o.x&&y==o.y;}};

// Simplified: just trace what happens on a 10x10 no-obstacle grid
int main(){
    G g(10,10);
    // Jump from (0,0) diagonally (1,1) to (9,9)
    // We want to see what jump points are found
    Pt cur{0,0}; int dx=1,dy=1; Pt t{9,9};
    
    // Trace diagonal jump manually
    Pt p = cur;
    int steps = 0;
    while(true){
        int nx=p.x+dx,ny=p.y+dy;
        if(g.op(nx,ny)){cout<<"hit obstacle at ("<<nx<<","<<ny<<")\n";break;}
        p={nx,ny}; steps++;
        cout<<"Step "<<steps<<": ("<<p.x<<","<<p.y<<")\n";
        if(p==t){cout<<"Reached target!\n";break;}
        // Check for forced neighbors - none on empty grid
        // Check if horizontal/vertical jump from here finds anything
        // On empty grid, nothing is forced
        // So should just continue diagonally
    }
    cout<<"Total steps: "<<steps<<endl;
    cout<<"Jump point should be: ("<<9<<","<<9<<") goal\n";
    return 0;
}
