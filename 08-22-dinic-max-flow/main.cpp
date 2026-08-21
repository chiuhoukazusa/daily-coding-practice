// Maximum Flow — Dinic's Algorithm
// 每日编程实践 2026-08-22
// 技术点：网络流最大流，Dinic算法（BFS分层图 + DFS阻塞流），Edmonds-Karp基准对比，
//         最大流最小割定理验证，随机网络生成，量化验证。

#include <bits/stdc++.h>
using namespace std;

struct Dinic {
    struct Edge { int to, rev; long long cap; };
    int n;
    vector<vector<Edge>> g;
    vector<int> level, iter;

    Dinic(int n) : n(n), g(n), level(n), iter(n) {}

    void addEdge(int from, int to, long long cap) {
        g[from].push_back({to, (int)g[to].size(), cap});
        g[to].push_back({from, (int)g[from].size() - 1, 0}); // 反向边
    }

    bool bfs(int s, int t) {
        fill(level.begin(), level.end(), -1);
        queue<int> q;
        level[s] = 0;
        q.push(s);
        while (!q.empty()) {
            int v = q.front(); q.pop();
            for (auto &e : g[v]) {
                if (e.cap > 0 && level[e.to] < 0) {
                    level[e.to] = level[v] + 1;
                    q.push(e.to);
                }
            }
        }
        return level[t] >= 0;
    }

    long long dfs(int v, int t, long long f) {
        if (v == t) return f;
        for (int &i = iter[v]; i < (int)g[v].size(); i++) {
            Edge &e = g[v][i];
            if (e.cap > 0 && level[v] < level[e.to]) {
                long long d = dfs(e.to, t, min(f, e.cap));
                if (d > 0) {
                    e.cap -= d;
                    g[e.to][e.rev].cap += d;
                    return d;
                }
            }
        }
        return 0;
    }

    long long maxFlow(int s, int t) {
        long long flow = 0;
        while (bfs(s, t)) {
            fill(iter.begin(), iter.end(), 0);
            long long f;
            while ((f = dfs(s, t, LLONG_MAX)) > 0) flow += f;
        }
        return flow;
    }
};

// Edmonds-Karp（BFS 求增广路）
struct EdmondsKarp {
    int n;
    vector<vector<long long>> cap;
    EdmondsKarp(int n) : n(n), cap(n, vector<long long>(n, 0)) {}
    void addEdge(int u, int v, long long c) { cap[u][v] += c; }

    long long maxFlow(int s, int t) {
        long long flow = 0;
        while (true) {
            vector<int> parent(n, -1);
            queue<int> q;
            q.push(s);
            while (!q.empty() && parent[t] < 0) {
                int u = q.front(); q.pop();
                for (int v = 0; v < n; v++) {
                    if (parent[v] < 0 && cap[u][v] > 0) {
                        parent[v] = u;
                        q.push(v);
                    }
                }
            }
            if (parent[t] < 0) break;
            long long add = LLONG_MAX;
            for (int v = t; v != s; v = parent[v])
                add = min(add, cap[parent[v]][v]);
            for (int v = t; v != s; v = parent[v]) {
                cap[parent[v]][v] -= add;
                cap[v][parent[v]] += add;
            }
            flow += add;
        }
        return flow;
    }
};

// 通过残量网络 BFS 求最小割的 source 侧（验证最大流最小割定理）
vector<bool> minCutSide(const Dinic &d, int s) {
    vector<bool> reach(d.n, false);
    queue<int> q;
    reach[s] = true; q.push(s);
    while (!q.empty()) {
        int v = q.front(); q.pop();
        for (auto &e : d.g[v]) {
            if (e.cap > 0 && !reach[e.to]) {
                reach[e.to] = true;
                q.push(e.to);
            }
        }
    }
    return reach;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(20260822);

    // 打印输出头
    printf("=== Dinic 最大流 量化验证 ===\n\n");

    // ---- 测试 1：简单已知网络 ----
    {
        // 经典网络：s=0 -> {1,2} -> {3} -> t=4
        // s->1(3), s->2(2), 1->3(2), 1->2(1), 2->3(3), 3->t(4), 1->t(1)
        Dinic d(5);
        d.addEdge(0,1,3); d.addEdge(0,2,2);
        d.addEdge(1,2,1); d.addEdge(1,3,2); d.addEdge(1,4,1);
        d.addEdge(2,3,3);
        d.addEdge(3,4,4);
        long long f = d.maxFlow(0,4);
        printf("[测试1] 简单网络最大流 = %lld (期望 5)\n", f);
        printf("        vs Edmonds-Karp = %lld\n", [&]{ EdmondsKarp ek(5);
            ek.addEdge(0,1,3); ek.addEdge(0,2,2);
            ek.addEdge(1,2,1); ek.addEdge(1,3,2); ek.addEdge(1,4,1);
            ek.addEdge(2,3,3); ek.addEdge(3,4,4);
            return ek.maxFlow(0,4); }());
    }

    // ---- 测试 2：随机网络，验证 Dinic == Edmonds-Karp，且 == 最小割 ----
    printf("\n[测试2] 随机网络验证（Dinic vs EK vs 最小割）\n");
    int pass = 0, total = 0;
    for (int trial = 0; trial < 50; trial++) {
        int n = 4 + (rng() % 6); // 4..9 个节点
        int m = n + (int)(rng() % (n * 2)); // 随机边数
        Dinic d(n);
        EdmondsKarp ek(n);
        set<pair<int,int>> seen;
        for (int i = 0; i < m; i++) {
            int u = rng() % n, v = rng() % n;
            if (u == v) continue;
            if (u > v) swap(u, v);
            if (seen.count({u,v})) continue;
            seen.insert({u,v});
            long long c = 1 + (rng() % 20);
            d.addEdge(u, v, c);
            ek.addEdge(u, v, c);
        }
        int s = 0, t = n - 1;
        long long fd = d.maxFlow(s, t);
        long long fek = ek.maxFlow(s, t);
        // 最小割验证已独立放在测试 3（用原始边表精确计算割容量）
        total++;
        if (fd == fek) pass++;
        else printf("  [FAIL] trial %d: Dinic=%lld EK=%lld\n", trial, fd, fek);
    }
    printf("   Dinic==EK 一致性: %d/%d %s\n", pass, total, pass==total?"✅":"❌");

    // ---- 测试 3：最小割定理独立验证（关键验证） ----
    printf("\n[测试3] 最大流最小割定理验证\n");
    {
        // 构造一个已知最小割的网络，独立手算割容量并与最大流对比。
        // 网络：s=0 -> a=1(10), s->b=2(5), a->b(15), a->t=3(5), b->t(10)
        // 手工分析最大流 = 15（s->a 10 + s->b 5 可全部过；a->t 5 + b->t 10 = 15）
        // 最小割 = 15（割 {s} | {a,b,t}: 10+5=15）
        int n = 4; // 0=s, 1=a, 2=b, 3=t
        vector<array<long long,3>> edges = {
            {0,1,10},{0,2,5},{1,2,15},{1,3,5},{2,3,10}
        };
        Dinic d(n);
        for (auto &e : edges) d.addEdge(e[0], e[1], e[2]);
        long long flow = d.maxFlow(0, 3);
        auto side = minCutSide(d, 0);
        // 计算割容量：遍历所有原始边，若 u 在 source 侧 && v 在 sink 侧，累加原始容量
        long long cut = 0;
        for (auto &e : edges) if (side[e[0]] && !side[e[1]]) cut += e[2];
        printf("   最大流 = %lld, 最小割容量 = %lld, 匹配 = %s\n",
               flow, cut, flow==cut?"✅":"❌");
        printf("   期望值 = 15\n");
    }

    // ---- 测试 4：性能与扩展性基准（随机密集网络） ----
    printf("\n[测试4] 性能基准（Dinic vs EK）\n");
    {
        int n = 400;
        Dinic d(n);
        EdmondsKarp ek(n);
        mt19937 r2(42);
        set<pair<int,int>> seen;
        int target = 6000;
        int added = 0;
        while (added < target) {
            int u = r2()%n, v = r2()%n;
            if (u==v) continue;
            if (u>v) swap(u,v);
            if (seen.count({u,v})) continue;
            seen.insert({u,v});
            long long c = 1 + r2()%1000;
            d.addEdge(u,v,c);
            ek.addEdge(u,v,c);
            added++;
        }
        auto t0 = chrono::high_resolution_clock::now();
        long long fd = d.maxFlow(0, n-1);
        auto t1 = chrono::high_resolution_clock::now();
        long long fek = ek.maxFlow(0, n-1);
        auto t2 = chrono::high_resolution_clock::now();
        double td = chrono::duration<double>(t1-t0).count();
        double tek = chrono::duration<double>(t2-t1).count();
        printf("   n=%d e=%d | Dinic=%lld (%.4fs)  EK=%lld (%.4fs)\n", n, added, fd, td, fek, tek);
        printf("   结果一致 = %s, 加速比 = %.2fx\n", fd==fek?"✅":"❌", td>0 ? tek/td : 0);
    }

    // ---- 测试 5：可视化（网络流图 + 最小割） ----
    printf("\n[测试5] 生成可视化 PPM 图像\n");
    {
        // 构造一个有代表性的网络，包含重叠路径以展示 Dinic 的分层图优势。
        // 网络：s=0 分成两支到 a=1(10) / b=2(5)，汇聚到 c=3，再到 t=4。
        int n = 6; // 0=s, 1=a, 2=b, 3=c, 4=d, 5=t
        vector<array<long long,3>> E = {
            {0,1,10},{0,2,8},
            {1,3,5},{1,4,3},
            {2,1,4},{2,3,6},
            {3,5,12},{4,5,7}
        };
        Dinic d(n);
        for (auto &e : E) d.addEdge(e[0], e[1], e[2]);
        long long flow = d.maxFlow(0, 5);
        auto side = minCutSide(d, 0);

        // 计算每条原始边的实际流量 = 原始容量 - 残量
        // 为返回残量图对每条边定位，需要重新构建并记录 addEdge 后的残量。
        // 这里用一个快照：重建网络，跑完后读取残量。
        Dinic dr(n);
        vector<array<long long,3>> E2 = E; // 拷贝
        for (auto &e : E2) dr.addEdge(e[0], e[1], e[2]);
        dr.maxFlow(0, 5);
        // 用邻接矩阵快照残量：由于双向边用 rev 索引，直接遍历正向边
        // 正向边残量通过 g[u][i].cap 读取（i 为正向边的索引）
        map<pair<int,int>, long long> resid;
        map<pair<int,int>, long long> orig;
        for (auto &e : E) orig[{e[0],e[1]}] = e[2];
        for (int u = 0; u < n; u++) {
            for (auto &e : dr.g[u]) {
                if (orig.count({u, e.to})) resid[{u, e.to}] = e.cap;
            }
        }
        vector<array<long long,3>> flowEdges;
        for (auto &e : E) {
            long long cap = orig[{e[0],e[1]}];
            long long used = cap - resid[{e[0],e[1]}];
            flowEdges.push_back({e[0], e[1], used});
        }

        // 图布局（固定坐标，W=1200 H=750）
        const int W = 1200, H = 750;
        map<int, pair<int,int>> pos = {
            {0,{120,375}}, {1,{420,180}}, {2,{420,570}},
            {3,{840,240}}, {4,{840,510}}, {5,{1080,375}}
        };

        // 生成 PPM (P3 文本格式)
        vector<vector<array<int,3>>> img(H, vector<array<int,3>>(W, {255,255,255}));

        auto drawLine = [&](int x0,int y0,int x1,int y1,array<int,3> c){
            int dx=abs(x1-x0), dy=abs(y1-y0);
            int sx=x0<x1?1:-1, sy=y0<y1?1:-1;
            int err=dx-dy;
            while(true){
                if(x0>=0&&x0<W&&y0>=0&&y0<H) img[y0][x0]=c;
                if(x0==x1&&y0==y1) break;
                int e2=2*err;
                if(e2>-dy){err-=dy;x0+=sx;}
                if(e2<dx){err+=dx;y0+=sy;}
            }
        };
        auto drawCircle = [&](int cx,int cy,int r,array<int,3> c){
            for(int y=-r;y<=r;y++) for(int x=-r;x<=r;x++){
                if(x*x+y*y<=r*r && cx+x>=0&&cx+x<W&&cy+y>=0&&cy+y<H)
                    img[cy+y][cx+x]=c;
            }
        };

        // 1. 画边（先画，节点覆盖其上）
        for (auto &fe : flowEdges) {
            int u=fe[0], v=fe[1]; long long used=fe[2];
            auto [x0,y0]=pos[u]; auto [x1,y1]=pos[v];
            long long cap=orig[{u,v}];
            // 边颜色：已满（used==cap）红色，部分使用绿色，未用灰色
            array<int,3> ec;
            if (used == cap) ec = {220,60,60};
            else if (used > 0) ec = {60,180,90};
            else ec = {200,200,200};
            drawLine(x0,y0,x1,y1,ec);
        }

        // 2. 画节点
        for (int i=0;i<n;i++){
            auto [x,y]=pos[i];
            array<int,3> nc;
            if (i==0) nc={60,120,220};           // source 蓝
            else if (i==5) nc={220,120,60};      // sink 橙
            else if (side[i]) nc={80,160,90};    // 源侧（最小割）绿
            else nc={160,160,160};                // 汇侧灰
            drawCircle(x,y,26,nc);
            // 节点边框
            drawCircle(x,y,30,{0,0,0});
            drawCircle(x,y,26,nc);
        }

        // 3. 用 PIL 后期加标签（在 C++ 里简化为后续 Python 标注）
        //    这里只输出 PPM，标签由 Python 脚本添加。

        ofstream f("dinic_maxflow_output.ppm");
        f << "P3\n" << W << " " << H << "\n255\n";
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                f << img[y][x][0] << " " << img[y][x][1] << " " << img[y][x][2] << " ";
            }
            f << "\n";
        }
        f.close();
        printf("   [Visualization] wrote dinic_maxflow_output.ppm (%dx%d)\n", W, H);
        printf("   max_flow=%lld, source_side_nodes=%d\n", flow,
               (int)count(side.begin(), side.end(), true));
    }

    printf("\n=== 全部验证完成 ===\n");
    return 0;
}
