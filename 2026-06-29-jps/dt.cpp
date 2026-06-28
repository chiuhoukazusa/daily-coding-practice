#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
using namespace std;
struct G { int w,h; vector<bool> b; G(int w_,int h_):w(w_),h(h_),b(w*h,false){}
  bool operator()(int x,int y)const{return x<0||x>=w||y<0||y>=h||b[y*w+x];}
  void s(int x,int y,bool v){b[y*w+x]=v;}
};
struct Pt{int x,y;bool operator==(const Pt&o)const{return x==o.x&&y==o.y;}};
struct PH{size_t operator()(const Pt&p)const{return (size_t)p.x*1000003+(size_t)p.y;}};
int main(){
    G g(5,5); Pt s{0,0},t{4,4};
    struct N{int f,g;Pt p,pr;bool cl;};
    unordered_map<Pt,N,PH> m; auto cp=[](const N&a,const N&b){return a.f>b.f;};
    priority_queue<N,vector<N>,decltype(cp)> q(cp);
    auto h=[](Pt a,Pt b){return 14*max(abs(a.x-b.x),abs(a.y-b.y))+4*min(abs(a.x-b.x),abs(a.y-b.y));};
    m[s]={h(s,t),0,s,{-1,-1},false}; q.push(m[s]);
    int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1},cs[8]={14,10,14,10,10,14,10,14};
    int cost=-1;
    while(!q.empty()){N c=q.top();q.pop();if(m[c.p].cl)continue;m[c.p].cl=true;
        if(c.p==t){cost=c.g;break;}
        for(int d=0;d<8;d++){int nx=c.p.x+dx[d],ny=c.p.y+dy[d];if(g(nx,ny))continue;
            if(dx[d]!=0&&dy[d]!=0)if(g(c.p.x+dx[d],c.p.y)&&g(c.p.x,c.p.y+dy[d]))continue;
            Pt np{nx,ny};int ng=c.g+cs[d];auto it=m.find(np);if(it==m.end()||ng<it->second.g){m[np]={ng+h(np,t),ng,np,c.p,false};q.push(m[np]);}
        }
    }
    printf("A* cost from (0,0) to (4,4) with no obstacles: %d\n",cost);
    // octile straight-line: 4*14 = 56
    printf("Expected minimum: 56 (4 diagonal steps at 14 each)\n");
    return 0;
}
