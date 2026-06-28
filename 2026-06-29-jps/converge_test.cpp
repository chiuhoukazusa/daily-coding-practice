#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
using namespace std;
// same structs as main.cpp...
struct G { int w,h; vector<bool> b; 
  G(int w_,int h_):w(w_),h(h_),b(w*h,false){}
  bool op(int x,int y)const{return x<0||x>=w||y<0||y>=h||b[y*w+x];}
  void s(int x,int y,bool v){b[y*w+x]=v;}
};
struct Pt{int x,y;bool operator==(const Pt&o)const{return x==o.x&&y==o.y;}};
struct PH{size_t operator()(const Pt&p)const{return (size_t)p.x*1000003+(size_t)p.y;}};
int heur(Pt a,Pt b){return 10*max(abs(a.x-b.x),abs(a.y-b.y))+4*min(abs(a.x-b.x),abs(a.y-b.y));}

int astar_cost(const G& g, Pt s, Pt t){
    struct N{int f,g;Pt p,pr;bool cl;};unordered_map<Pt,N,PH> m;
    auto cp=[](const N&a,const N&b){return a.f>b.f;};
    priority_queue<N,vector<N>,decltype(cp)> q(cp);m[s]={heur(s,t),0,s,{-1,-1},false};q.push(m[s]);
    int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1},cs[8]={14,10,14,10,10,14,10,14};
    while(!q.empty()){N c=q.top();q.pop();if(m[c.p].cl)continue;m[c.p].cl=true;
        if(c.p==t)return c.g;
        for(int d=0;d<8;d++){int nx=c.p.x+dx[d],ny=c.p.y+dy[d];if(g.op(nx,ny))continue;
            if(dx[d]!=0&&dy[d]!=0)if(g.op(c.p.x+dx[d],c.p.y)&&g.op(c.p.x,c.p.y+dy[d]))continue;
            Pt np{nx,ny};int ng=c.g+cs[d];auto it=m.find(np);
            if(it==m.end()||ng<it->second.g){m[np]={ng+heur(np,t),ng,np,c.p,false};q.push(m[np]);}}
    }return -1;
}

// Identical JPS with fix
bool forced(const G& g, Pt p, int dx, int dy) {
    if(dx!=0&&dy==0)return(!g.op(p.x,p.y-1)&&g.op(p.x-dx,p.y-1))||(!g.op(p.x,p.y+1)&&g.op(p.x-dx,p.y+1));
    if(dx==0&&dy!=0)return(!g.op(p.x-1,p.y)&&g.op(p.x-1,p.y-dy))||(!g.op(p.x+1,p.y)&&g.op(p.x+1,p.y-dy));
    return false;
}
Pt jump(const G& g, Pt c, int dx, int dy, Pt t) {
    int nx=c.x+dx,ny=c.y+dy;if(g.op(nx,ny))return{-1,-1};
    Pt np{nx,ny};if(np==t)return np;if(forced(g,np,dx,dy))return np;
    if(dx!=0&&dy!=0){if(jump(g,np,dx,0,t).x!=-1)return np;if(jump(g,np,0,dy,t).x!=-1)return np;}
    return jump(g,np,dx,dy,t);
}
struct Succ{Pt p;int cost;};
void successors(const G& g, Pt par, Pt cur, Pt t, vector<Succ>& out) {
    int dx=(cur.x>par.x)?1:((cur.x<par.x)?-1:0),dy=(cur.y>par.y)?1:((cur.y<par.y)?-1:0);
    if(dx!=0&&dy!=0){Pt jp;if((jp=jump(g,cur,dx,dy,t)).x!=-1)out.push_back({jp,max(abs(jp.x-cur.x),abs(jp.y-cur.y))*14});
        if((jp=jump(g,cur,dx,0,t)).x!=-1)out.push_back({jp,abs(jp.x-cur.x)*10});
        if((jp=jump(g,cur,0,dy,t)).x!=-1)out.push_back({jp,abs(jp.y-cur.y)*10});
    }else if(dx!=0){Pt jp;if((jp=jump(g,cur,dx,0,t)).x!=-1)out.push_back({jp,abs(jp.x-cur.x)*10});
        if(!g.op(cur.x,cur.y-1)&&g.op(cur.x-dx,cur.y-1))if((jp=jump(g,cur,dx,-1,t)).x!=-1){int d=max(abs(jp.x-cur.x),abs(jp.y-cur.y));out.push_back({jp,d*14});}
        if(!g.op(cur.x,cur.y+1)&&g.op(cur.x-dx,cur.y+1))if((jp=jump(g,cur,dx,1,t)).x!=-1){int d=max(abs(jp.x-cur.x),abs(jp.y-cur.y));out.push_back({jp,d*14});}
    }else{Pt jp;if((jp=jump(g,cur,0,dy,t)).x!=-1)out.push_back({jp,abs(jp.y-cur.y)*10});
        if(!g.op(cur.x-1,cur.y)&&g.op(cur.x-1,cur.y-dy))if((jp=jump(g,cur,-1,dy,t)).x!=-1){int d=max(abs(jp.x-cur.x),abs(jp.y-cur.y));out.push_back({jp,d*14});}
        if(!g.op(cur.x+1,cur.y)&&g.op(cur.x+1,cur.y-dy))if((jp=jump(g,cur,1,dy,t)).x!=-1){int d=max(abs(jp.x-cur.x),abs(jp.y-cur.y));out.push_back({jp,d*14});}
    }
}
int jps_cost(const G& g, Pt s, Pt t) {
    struct N{int f,g;Pt p,par;bool closed;};unordered_map<Pt,N,PH> m;
    auto cmp=[](const N&a,const N&b){return a.f>b.f;};
    priority_queue<N,vector<N>,decltype(cmp)> q(cmp);m[s]={heur(s,t),0,s,{-1,-1},false};q.push(m[s]);
    const int d8x[8]={-1,0,1,-1,1,-1,0,1},d8y[8]={-1,-1,-1,0,0,1,1,1};
    while(!q.empty()){N c=q.top();q.pop();if(m[c.p].closed)continue;m[c.p].closed=true;
        if(c.p==t)return c.g;vector<Succ> succs;
        if(c.par.x==-1){for(int d=0;d<8;d++){int dx=d8x[d],dy=d8y[d];if(g.op(c.p.x+dx,c.p.y+dy))continue;
            if(dx!=0&&dy!=0)if(g.op(c.p.x+dx,c.p.y)&&g.op(c.p.x,c.p.y+dy))continue;
            Pt jp=jump(g,c.p,dx,dy,t);if(jp.x!=-1){int dist=max(abs(jp.x-c.p.x),abs(jp.y-c.p.y));
            succs.push_back({jp,(dx!=0&&dy!=0)?14*dist:10*dist});}}
        }else{successors(g,c.par,c.p,t,succs);}
        for(auto& sc:succs){int ng=c.g+sc.cost;auto it=m.find(sc.p);
            if(it==m.end()||ng<it->second.g){m[sc.p]={ng+heur(sc.p,t),ng,sc.p,c.p,false};q.push(m[sc.p]);}}}
    return -1;
}

G maze(int w,int h,int seed){G g(w,h);mt19937 r(seed);uniform_int_distribution<int>c(0,99),rx(1,w-8),ry(1,h-8);
    for(int y=1;y<h-1;y++)for(int x=1;x<w-1;x++)g.s(x,y,c(r)<30);
    for(int i=0;i<5;i++){int x0=rx(r),y0=ry(r);for(int dy=0;dy<=3;dy++)for(int dx=0;dx<=6;dx++)g.s(x0+dx,y0+dy,true);}
    g.s(0,0,false);g.s(1,0,false);g.s(0,1,false);g.s(w-1,h-1,false);g.s(w-2,h-1,false);g.s(w-1,h-2,false);return g;}

int main(){
    int match=0, total=0;
    for(int seed=42;seed<2000;seed++){
        G g=maze(64,64,seed);Pt s{0,0},t{63,63};
        int ac=astar_cost(g,s,t),jc=jps_cost(g,s,t);
        if(ac<0||jc<0)continue;
        total++;
        if(ac!=jc)printf("seed=%d: A*=%d JPS=%d diff=%d\n",seed,ac,jc,jc-ac);
        else match++;
    }
    printf("\nMatch: %d/%d = %.1f%%\n",match,total,100.0*match/total);
    return 0;
}
