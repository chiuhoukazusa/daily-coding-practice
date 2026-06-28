#include <cstdio>
#include "main_jps_functions.cpp"  // nah, let's just do inline

// Actually let me make a simpler test
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
using namespace std;

struct Grid {
    int w, h;
    vector<bool> blocked;
    Grid(int w_, int h_) : w(w_), h(h_), blocked(w * h, false) {}
    bool operator()(int x, int y) const {
        return x < 0 || x >= w || y < 0 || y >= h || blocked[y * w + x];
    }
    void set(int x, int y, bool v) { blocked[y * w + x] = v; }
};
struct Pt { int x,y; bool operator==(const Pt&o)const{return x==o.x&&y==o.y;} };
struct PtHash { size_t operator()(const Pt&p)const{return (size_t)p.x*1000003+(size_t)p.y;} };

int main() {
    // Simple 5x5 grid with no obstacles: verify both find same cost
    Grid g(5,5);
    Pt s{0,0}, t{4,4};
    
    // Manual A*:
    struct N { int f,g; Pt p,par; bool closed; };
    unordered_map<Pt,N,PtHash> m;
    auto cmp=[](const N&a,const N&b){return a.f>b.f;};
    priority_queue<N,vector<N>,decltype(cmp)> q(cmp);
    m[s]={0,0,s,{-1,-1},false}; q.push(m[s]);
    int cost=0;
    while(!q.empty()){
        N c=q.top();q.pop();if(m[c.p].closed)continue;m[c.p].closed=true;
        if(c.p==t){cost=c.g;break;}
        int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1},cst[8]={14,10,14,10,10,14,10,14};
        for(int d=0;d<8;d++){
            int nx=c.p.x+dx[d],ny=c.p.y+dy[d];if(g(nx,ny))continue;
            if(dx[d]!=0&&dy[d]!=0){if(g(c.p.x+dx[d],c.p.y)&&g(c.p.x,c.p.y+dy[d]))continue;}
            Pt np{nx,ny};int ng=c.g+cst[d];
            auto it=m.find(np);if(it==m.end()||ng<it->second.g){m[np]={ng+abs(4-nx)*10+abs(4-ny)*10,ng,np,c.p,false};q.push(m[np]);}
        }
    }
    cout << "A* optimal cost: " << cost << " (expect 4*14 = 56 for diagonal path)" << endl;
    cout << "But with 10-cost for straight and 14 for diag..." << endl;
    return 0;
}
