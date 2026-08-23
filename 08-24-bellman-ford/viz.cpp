// PPM visualization: render shortest-path distances as a heatmap grid
#include <bits/stdc++.h>
using namespace std;
const long long INF = 4e18;
struct Edge{int u,v;long long w;};
pair<vector<long long>,bool> bf(int n,const vector<Edge>&e,int s){
    vector<long long>d(n,INF);d[s]=0;
    for(int p=0;p<n-1;p++){bool c=0;for(auto&x:e)if(d[x.u]!=INF&&d[x.u]+x.w<d[x.v]){d[x.v]=d[x.u]+x.w;c=1;}if(!c)break;}
    for(auto&x:e)if(d[x.u]!=INF&&d[x.u]+x.w<d[x.v])return {d,true};
    return {d,false};
}
int main(){
    mt19937 rng(20260824);
    int N=48,n=N*N; 
    vector<Edge>e;
    vector<long long>pot(n);
    for(int i=0;i<n;i++)pot[i]=(long long)(rng()%4001)-2000;
    set<pair<int,int>>used;
    auto idx=[&](int r,int c){return r*N+c;};
    for(int r=0;r<N;r++)for(int c=0;c<N;c++){
        int di[]={0,1,0,-1,1,1,-1,-1},dj[]={1,0,-1,0,1,-1,1,-1};
        for(int k=0;k<8;k++){int nr=r+di[k],nc=c+dj[k];if(nr<0||nr>=N||nc<0||nc>=N)continue;
            int u=idx(r,c),v=idx(nr,nc);if(used.count({u,v}))continue;used.insert({u,v});
            e.push_back({u,v,pot[v]-pot[u]+(long long)(rng()%20)+1});
        }
    }
    auto res=bf(n,e,0);
    long long maxd=0;for(auto d:res.first)if(d!=INF)maxd=max(maxd,d);
    // heatmap via min-max normalize (reachable only)
    auto f=res.first;
    long long mind=INF;for(auto d:f)if(d!=INF)mind=min(mind,d);
    vector<unsigned char>img(n*3);
    int reach=0;
    for(int i=0;i<n;i++){
        unsigned char r=0,g=0,b=0;
        if(f[i]!=INF){reach++;
            double t=maxd==mind?0.0:(double)(f[i]-mind)/(maxd-mind);
            // blue->red heatmap
            r=(unsigned char)(255*t);g=0;b=(unsigned char)(255*(1-t));
        }
        img[i*3]=r;img[i*3+1]=g;img[i*3+2]=b;
    }
    printf("P3\n%d %d\n255\n",N,N);
    for(int i=0;i<n;i++)printf("%d %d %d\n",img[i*3],img[i*3+1],img[i*3+2]);
    fprintf(stderr,"reachable=%d/%d mind=%lld maxd=%lld\n",reach,n,mind,maxd);
    return 0;
}
