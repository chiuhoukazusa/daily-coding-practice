#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cstdlib>
using namespace std;

struct G { int w,h; vector<bool> b; 
  G(int w_,int h_):w(w_),h(h_),b(w*h,false){}
  bool op(int x,int y)const{return x<0||x>=w||y<0||y>=h||b[y*w+x];}
  void s(int x,int y,bool v){b[y*w+x]=v;}
};
struct Pt{int x,y;bool operator==(const Pt&o)const{return x==o.x&&y==o.y;}};
struct PH{size_t operator()(const Pt&p)const{return (size_t)p.x*1000003+(size_t)p.y;}};

int h(Pt a,Pt b){return 14*max(abs(a.x-b.x),abs(a.y-b.y))+4*min(abs(a.x-b.x),abs(a.y-b.y));}

// Modified A* with careful cost tracking
int astar_cost(const G& g, Pt s, Pt t) {
    struct N{int f,g;Pt p,pr;bool cl;};
    unordered_map<Pt,N,PH> m; auto cp=[](const N&a,const N&b){return a.f>b.f;};
    priority_queue<N,vector<N>,decltype(cp)> q(cp);
    m[s]={h(s,t),0,s,{-1,-1},false}; q.push(m[s]);
    int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1},cs[8]={14,10,14,10,10,14,10,14};
    while(!q.empty()){
        N c=q.top();q.pop();if(m[c.p].cl)continue;m[c.p].cl=true;
        if(c.p==t)return c.g;
        for(int d=0;d<8;d++){
            int nx=c.p.x+dx[d],ny=c.p.y+dy[d];if(g.op(nx,ny))continue;
            if(dx[d]!=0&&dy[d]!=0)if(g.op(c.p.x+dx[d],c.p.y)&&g.op(c.p.x,c.p.y+dy[d]))continue;
            Pt np{nx,ny};int ng=c.g+cs[d];auto it=m.find(np);
            if(it==m.end()||ng<it->second.g){m[np]={ng+h(np,t),ng,np,c.p,false};q.push(m[np]);}
        }
    }
    return -1;
}

int main(){
    // Test: obstacle-free 128x128 grid
    G g(128,128);
    Pt s{0,0},t{127,127};
    int cost = astar_cost(g,s,t);
    printf("Empty 128x128: A* cost = %d\n", cost);
    // Optimal path: 127 diagonal steps at 14 each = 1778
    printf("Expected optimum = %d\n", 127*14);
    
    // Test: 1 horizontal + 126 diagonal = 10 + 126*14 = 1774
    printf("If 1 straight + 126 diag = %d\n", 10 + 126*14);
    printf("If 127 straight + 127 straight = %d\n", 1270 + 1270);
    
    // The octile heuristic with costs 10/14 might make non-diagonal cheaper for certain paths
    // Actually 127*14 = 1778, vs 127*10+127*10 = 2540. Diagonal path IS optimal.
    return 0;
}
