#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
using namespace std;

struct G { int w,h; vector<bool> b; 
  G(int w_,int h_):w(w_),h(h_),b(w*h,false){}
  bool op(int x,int y)const{return x<0||x>=w||y<0||y>=h||b[y*w+x];}
  void s(int x,int y,bool v){b[y*w+x]=v;}
};
struct Pt{int x,y;bool operator==(const Pt&o)const{return x==o.x&&y==o.y;}};
struct PH{size_t operator()(const Pt&p)const{return (size_t)p.x*1000003+(size_t)p.y;}};

int dijkstra(const G& g, Pt s, Pt t) {
    struct N{int g;Pt p;};
    auto cmp=[](const N&a,const N&b){return a.g>b.g;};
    priority_queue<N,vector<N>,decltype(cmp)> q(cmp);
    unordered_map<Pt,int,PH> dist;
    dist[s]=0; q.push({0,s});
    int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1},cs[8]={14,10,14,10,10,14,10,14};
    int best = 1<<30;
    while(!q.empty()){
        N c=q.top();q.pop();
        if(c.g > dist[c.p]) continue;
        if(c.p==t) return c.g;
        for(int d=0;d<8;d++){
            int nx=c.p.x+dx[d],ny=c.p.y+dy[d];if(g.op(nx,ny))continue;
            if(dx[d]!=0&&dy[d]!=0)if(g.op(c.p.x+dx[d],c.p.y)&&g.op(c.p.x,c.p.y+dy[d]))continue;
            Pt np{nx,ny};int ng=c.g+cs[d];auto it=dist.find(np);
            if(it==dist.end()||ng<it->second){dist[np]=ng;q.push({ng,np});}
        }
    }
    return -1;
}

G maze(int w, int h, int seed) {
    G g(w, h);
    mt19937 r(seed);
    uniform_int_distribution<int> c(0, 99), rx(1, w-8), ry(1, h-8);
    for (int y = 1; y < h-1; y++)
        for (int x = 1; x < w-1; x++)
            g.s(x, y, c(r) < 30);
    for (int i = 0; i < 5; i++) {
        int x0 = rx(r), y0 = ry(r);
        for (int dy = 0; dy <= 3; dy++)
            for (int dx = 0; dx <= 6; dx++)
                g.s(x0+dx, y0+dy, true);
    }
    g.s(0,0,false); g.s(1,0,false); g.s(0,1,false);
    g.s(w-1,h-1,false); g.s(w-2,h-1,false); g.s(w-1,h-2,false);
    return g;
}

int main(){
    for(int seed=42; seed<1000; seed+=137){
        G g = maze(64,64,seed);
        Pt s{0,0},t{63,63};
        int dcost = dijkstra(g,s,t);
        if(dcost < 0) continue;
        printf("seed=%d: Dijkstra optimal = %d\n", seed, dcost);
    }
    return 0;
}
