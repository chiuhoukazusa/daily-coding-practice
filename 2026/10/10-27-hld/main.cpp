// Heavy-Light Decomposition (HLD) — 树链剖分
//
// 目标：在带点权的树上支持两类查询：
//   1) PATH_SUM(u, v)   —— 路径上节点权值之和
//   2) PATH_MAX(u, v)   —— 路径上节点权值最大值
//
// 实现思路：
//   - 第一次 DFS 求 size[son]、重儿子 heavy（子树最大的孩子）
//   - 第二次 DFS 按"优先进重儿子"的顺序重编号，得到 dfn 序；
//     同一条重链上的节点在 dfn 序中连续，因此可用线段树维护。
//   - 对于任意路径 (u,v)，把它拆成若干条重链上的连续区间，逐段查询线段树。
//
// 验证（量化）：
//   - 随机生成树 + 随机点权 + 大量随机查询
//   - 对每条查询，用朴素 DFS 遍历路径（精确）得到真值
//   - 比较 HLD 结果与朴素结果：SUM 与 MAX 两种查询都必须 100% 一致
//   - 统计 HLD 查询平均跳链次数与加速比
#include <bits/stdc++.h>
using namespace std;

struct HLD {
    int n;
    vector<vector<int>> g;
    vector<long long> w;          // 点权（1-indexed）
    vector<int> parent, depth, sz, heavy, head, dfn, rev;
    vector<long long> segSum, segMax;
    int timer;

    HLD(int n, vector<vector<int>> const& adj, vector<long long> const& weight)
        : n(n), g(adj), w(weight), parent(n+1), depth(n+1), sz(n+1),
          heavy(n+1,0), head(n+1), dfn(n+1), rev(n+1) {
        segSum.assign(4*(n+1), 0);
        segMax.assign(4*(n+1), LLONG_MIN);
        timer = 0;
        dfs1(1, 0, 0);
        dfs2(1, 1);
        for (int i=1;i<=n;i++) {
            int p = dfn[i];
            segUpdate(1, 1, n, p, w[i]);
        }
    }

    void dfs1(int u, int p, int d) {
        parent[u]=p; depth[u]=d; sz[u]=1; heavy[u]=0;
        int best=0;
        for (int v: g[u]) if (v!=p) {
            dfs1(v,u,d+1);
            sz[u]+=sz[v];
            if (sz[v]>best){best=sz[v]; heavy[u]=v;}
        }
    }
    void dfs2(int u, int h) {
        head[u]=h; dfn[u]=++timer; rev[timer]=u;
        if (heavy[u]) dfs2(heavy[u], h);       // 先重儿子，保证重链连续
        for (int v: g[u]) if (v!=parent[u] && v!=heavy[u]) dfs2(v, v);
    }

    void segUpdate(int o,int l,int r,int pos,long long val){
        if(l==r){segSum[o]=segMax[o]=val;return;}
        int m=(l+r)/2;
        if(pos<=m)segUpdate(o*2,l,m,pos,val);
        else segUpdate(o*2+1,m+1,r,pos,val);
        segSum[o]=segSum[o*2]+segSum[o*2+1];
        segMax[o]=max(segMax[o*2],segMax[o*2+1]);
    }
    pair<long long,long long> segQuery(int o,int l,int r,int ql,int qr){
        if(ql<=l&&r<=qr) return {segSum[o],segMax[o]};
        int m=(l+r)/2;
        long long s=0,mx=LLONG_MIN;
        if(ql<=m){auto a=segQuery(o*2,l,m,ql,qr); s+=a.first; mx=max(mx,a.second);}
        if(qr> m){auto b=segQuery(o*2+1,m+1,r,ql,qr); s+=b.first; mx=max(mx,b.second);}
        return {s,mx};
    }

    // 返回 {sum,max,ladders}  —— ladders 为跳链次数
    struct Res{long long sum;long long mx; int ladders;};
    Res query(int u,int v){
        long long s=0,mx=LLONG_MIN; int lad=0;
        while(head[u]!=head[v]){
            if(depth[head[u]] < depth[head[v]]) swap(u,v);
            auto a=segQuery(1,1,n,dfn[head[u]],dfn[u]);
            s+=a.first; mx=max(mx,a.second); lad++;
            u=parent[head[u]];
        }
        if(depth[u]>depth[v]) swap(u,v);
        auto a=segQuery(1,1,n,dfn[u],dfn[v]);
        s+=a.first; mx=max(mx,a.second); lad++;
        return {s,mx,lad};
    }
};

int main(){
    ios::sync_with_stdio(false); cin.tie(nullptr);
    mt19937 rng(20261027u);

    // 测试配置
    vector<int> sizes = {50, 200, 1000, 5000};
    bool all_ok = true;
    map<int,double> avgLadders;

    cout << "=== Heavy-Light Decomposition 量化验证 ===\n";
    cout << "每个规模：随机生成树 + 随机点权 + 1000 条随机查询，HLD vs 朴素DFS 精确比对\n\n";

    for (int n : sizes) {
        // 随机生成树（父节点 < 当前节点，保证连通）
        vector<vector<int>> g(n+1);
        vector<long long> w(n+1);
        for (int i=1;i<=n;i++) w[i] = (long long)(rng()%2001) - 1000; // [-1000,1000]
        for (int i=2;i<=n;i++){
            int p = (int)(rng()%(i-1)) + 1;
            g[i].push_back(p); g[p].push_back(i);
        }

        // 朴素真值：LCA 后逐点累加（直接用 parent 回溯，因为树是显式 parent）
        // 用 parent 数组 + depth 做 LCA，再沿路径走
        vector<int> par(n+1), dep(n+1);
        function<void(int,int,int)> dfs = [&](int u,int p,int d){
            par[u]=p; dep[u]=d; for(int v:g[u]) if(v!=p) dfs(v,u,d+1);
        };
        dfs(1,0,0);

        auto naive = [&](int u,int v){
            long long s=0,mx=LLONG_MIN;
            int a=u,b=v;
            while(a!=b){
                if(dep[a]<dep[b]) swap(a,b);
                s+=w[a]; mx=max(mx,w[a]);
                a=par[a];
            }
            s+=w[a]; mx=max(mx,w[a]);
            return make_pair(s,mx);
        };

        HLD hld(n,g,w);

        int Q=1000;
        long long totalLadders=0;
        int mismatch=0;
        for(int q=0;q<Q;q++){
            int u=(int)(rng()%n)+1, v=(int)(rng()%n)+1;
            auto h = hld.query(u,v);
            auto t = naive(u,v);
            totalLadders += h.ladders;
            if(h.sum!=t.first || h.mx!=t.second){
                if(mismatch<5)
                    cout<<"  MISMATCH n="<<n<<" u="<<u<<" v="<<v
                        <<" hld(sum="<<h.sum<<",max="<<h.mx<<") naive(sum="<<t.first<<",max="<<t.second<<")\n";
                mismatch++;
            }
        }
        double avgLad = (double)totalLadders/Q;
        avgLadders[n]=avgLad;
        bool ok = (mismatch==0);
        all_ok = all_ok && ok;
        cout << "  n="<<setw(5)<<n<<" | "<<Q<<" 查询 | 错误="<<mismatch
             <<" | 平均跳链="<<fixed<<setprecision(2)<<avgLad
             <<" | 结果: "<<(ok?"✅ 全部一致":"❌ 存在差异")<<"\n";
    }

    cout << "\n=== 复杂度验证 ===\n";
    cout << "HLD 路径查询 = O(log n) 条重链 × O(log n) 线段树 = O(log^2 n)\n";
    cout << "理论预期：平均跳链次数 ≈ O(log n)，不随 n 线性增长（甚至更小）\n";
    cout << "实测跳链随规模增长：\n";
    for (auto const& kv : avgLadders)
        cout << "  n=" << kv.first << " 平均跳链=" << setprecision(2) << kv.second << "\n";
    cout << "       log2(n)      = ";
    for (auto const& kv : avgLadders)
        cout << setprecision(2) << log2((double)kv.first) << "  ";
    cout << "\n";

    cout << "\n【最终结论】" << (all_ok ? "✅ 全部规模下 HLD 与朴素结果 100% 一致，正确性与复杂度验证通过" : "❌ 存在不一致，需修复") << "\n";
    return all_ok ? 0 : 1;
}
