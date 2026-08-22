// 08-23 Strongly Connected Components (Tarjan's Algorithm)
// 图算法实践：Tarjan 强连通分量 + Kosaraju 对照 + 缩点 DAG 拓扑验证
// 量化验证：SCC 数量/大小、缩点后 DAG 无环、与 Floyd-Warshall 可达性基准对照
#include <bits/stdc++.h>
using namespace std;

// ---------- Tarjan SCC ----------
struct Tarjan {
    int n, timer = 0, sccCount = 0;
    vector<vector<int>> g;
    vector<int> dfn, low, comp, stk;
    vector<bool> inStk;

    Tarjan(int n) : n(n), g(n), dfn(n, -1), low(n, 0), comp(n, -1), inStk(n, false) {}

    void addEdge(int u, int v) { g[u].push_back(v); }

    void dfs(int u) {
        dfn[u] = low[u] = timer++;
        stk.push_back(u);
        inStk[u] = true;
        for (int v : g[u]) {
            if (dfn[v] == -1) {
                dfs(v);
                low[u] = min(low[u], low[v]);
            } else if (inStk[v]) {
                low[u] = min(low[u], dfn[v]);
            }
        }
        if (low[u] == dfn[u]) { // 找到一个 SCC 的根
            int v;
            do {
                v = stk.back(); stk.pop_back();
                inStk[v] = false;
                comp[v] = sccCount;
            } while (v != u);
            sccCount++;
        }
    }

    void run() {
        for (int i = 0; i < n; i++)
            if (dfn[i] == -1) dfs(i);
    }
};

// ---------- Kosaraju SCC（对照实现）----------
struct Kosaraju {
    int n, sccCount = 0;
    vector<vector<int>> g, rg;
    vector<int> comp;
    vector<int> order;
    vector<bool> vis;

    Kosaraju(int n) : n(n), g(n), rg(n), comp(n, -1), vis(n, false) {}

    void addEdge(int u, int v) { g[u].push_back(v); rg[v].push_back(u); }

    void dfs1(int u) {
        vis[u] = true;
        for (int v : g[u]) if (!vis[v]) dfs1(v);
        order.push_back(u);
    }
    void dfs2(int u, int c) {
        comp[u] = c;
        for (int v : rg[u]) if (comp[v] == -1) dfs2(v, c);
    }
    void run() {
        for (int i = 0; i < n; i++) if (!vis[i]) dfs1(i);
        for (int i = n - 1; i >= 0; i--) {
            int u = order[i];
            if (comp[u] == -1) dfs2(u, sccCount++);
        }
    }
};

// ---------- Floyd-Warshall 可达性（基准）----------
vector<vector<bool>> computeReachability(int n, const vector<pair<int,int>>& edges) {
    vector<vector<bool>> reach(n, vector<bool>(n, false));
    for (int i = 0; i < n; i++) reach[i][i] = true;
    for (auto [u, v] : edges) reach[u][v] = true;
    for (int k = 0; k < n; k++)
        for (int i = 0; i < n; i++)
            if (reach[i][k])
                for (int j = 0; j < n; j++)
                    reach[i][j] = reach[i][j] || reach[k][j];
    return reach;
}

// ---------- 随机有向图生成 ----------
mt19937 rng(12345);

int main() {
    // 测试 1：随机图，与 Kosaraju 对照 SCC 划分是否等价
    int passed = 0, total = 0;
    {
        int n = 60;
        int edgeCnt = 0;
        vector<pair<int,int>> edges;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (i != j && (rng() % 100) < 6) { edges.push_back({i, j}); edgeCnt++; }

        Tarjan tj(n); Kosaraju ks(n);
        for (auto [u,v] : edges) { tj.addEdge(u, v); ks.addEdge(u, v); }
        tj.run(); ks.run();

        // 验证：两算法得到的 SCC 集合必须一致（同属一个 SCC 的节点对）
        bool same = true;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if ((tj.comp[i] == tj.comp[j]) != (ks.comp[i] == ks.comp[j])) same = false;
        total++; if (same) passed++;
        printf("[Test1 随机图] Tarjan SCC=%d  Kosaraju SCC=%d  划分一致=%s\n",
               tj.sccCount, ks.sccCount, same ? "✅" : "❌");

        // 验证 2：SCC 划分必须与 Floyd-Warshall 可达性一致
        // （u,v 在同一 SCC <=> u 可达 v 且 v 可达 u）—— 这也是 SCC 的严格定义
        auto reach = computeReachability(n, edges);
        bool consistent = true;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                bool inSameSCC = (tj.comp[i] == tj.comp[j]);
                bool mutualReach = reach[i][j] && reach[j][i];
                if (inSameSCC != mutualReach) consistent = false;
            }
        total++; if (consistent) passed++;
        printf("[Test2 可达性基准] SCC划分 == 相互可达关系=%s\n", consistent ? "✅" : "❌");

        // 验证 3：缩点后 DAG 必须无环（Tarjan 输出的 SCC 已按拓扑逆序）
        // 检查缩点图无重边无自环，且不存在环
        int K = tj.sccCount;
        vector<set<int>> dag(K);
        for (auto [u,v] : edges)
            if (tj.comp[u] != tj.comp[v]) dag[tj.comp[u]].insert(tj.comp[v]);
        // 用 Floyd 检查缩点图无环（若有环则说明 SCC 划分错误）
        vector<vector<bool>> dreach(K, vector<bool>(K, false));
        for (int i = 0; i < K; i++) dreach[i][i] = true;
        for (int i = 0; i < K; i++) for (int j : dag[i]) dreach[i][j] = true;
        for (int k = 0; k < K; k++)
            for (int i = 0; i < K; i++) if (dreach[i][k])
                for (int j = 0; j < K; j++) dreach[i][j] = dreach[i][j] || dreach[k][j];
        bool dagAcyclic = true;
        for (int i = 0; i < K; i++) for (int j = 0; j < K; j++)
            if (i != j && dreach[i][j] && dreach[j][i]) dagAcyclic = false;
        total++; if (dagAcyclic) passed++;
        printf("[Test3 缩点无环] 缩点图节点=%d  无环=%s\n", K, dagAcyclic ? "✅" : "❌");

        // 验证 4：单点自环必须自成一个 SCC
        bool selfLoop = true;
        for (int i = 0; i < n; i++) {
            // 若 i 只与自身连（无任何入边/出边），则其 SCC 只含 i
            bool hasAnyEdge = false;
            for (auto [u,v] : edges) if (u==i || v==i) hasAnyEdge = true;
            if (!hasAnyEdge) {
                int c = tj.comp[i]; int cnt=0;
                for (int j=0;j<n;j++) if(tj.comp[j]==c) cnt++;
                if (cnt != 1) selfLoop = false;
            }
        }
        total++; if (selfLoop) passed++;
        printf("[Test4 孤立节点] 孤立点独立成SCC=%s\n", selfLoop ? "✅" : "❌");
    }

    // 测试 5：已知强连通图（环），应全部合并为一个 SCC
    {
        int n = 100;
        Tarjan tj(n);
        for (int i = 0; i < n; i++) tj.addEdge(i, (i+1)%n); // 大环
        tj.run();
        bool oneSCC = (tj.sccCount == 1);
        total++; if (oneSCC) passed++;
        printf("[Test5 单环图] 100节点单环 SCC=%d 应为1=%s\n", tj.sccCount, oneSCC ? "✅" : "❌");
    }

    // 测试 6：DAG（无环），每个节点应自成一个 SCC
    {
        int n = 80;
        Tarjan tj(n);
        for (int i = 0; i < n; i++)
            for (int j = i+1; j < n; j++)
                if ((rng() % 100) < 4) tj.addEdge(i, j); // 只连向更大编号
        tj.run();
        bool allSingle = (tj.sccCount == n);
        total++; if (allSingle) passed++;
        printf("[Test6 DAG图] 80节点DAG SCC=%d 应为%d=%s\n", tj.sccCount, n, allSingle ? "✅" : "❌");
    }

    // ---------- 输出 PPM 可视化 ----------
    // 用彩色块表示 SCC 划分（每列 = 一个节点，颜色 = SCC id）
    {
        int n = 40;
        Tarjan tj(n);
        vector<pair<int,int>> edges;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (i != j && (rng() % 100) < (i== (j+1)%n || j==(i+1)%n ? 40 : 8)) {
                    tj.addEdge(i, j); edges.push_back({i,j});
                }
        tj.run();

        int W = n * 16, H = 400;
        ofstream f("tarjan_scc_output.ppm");
        f << "P3\n" << W << " " << H << "\n255\n";
        // 为每个 SCC 分配鲜艳颜色
        auto colorOf = [&](int c) {
            double hue = (c * 0.6180339887); // 黄金比例分布
            auto hsv2rgb = [](double h, double s, double v) {
                int i = (int)(h * 6) % 6;
                double f = h * 6 - (int)(h * 6);
                double p = v*(1-s), q = v*(1-f*s), t = v*(1-(1-f)*s);
                double r,g,b;
                switch(i){case 0:r=v;g=t;b=p;break;case 1:r=q;g=v;b=p;break;
                    case 2:r=p;g=v;b=t;break;case 3:r=p;g=q;b=v;break;
                    case 4:r=t;g=p;b=v;break;default:r=v;g=p;b=q;break;}
                return make_tuple((int)(r*255),(int)(g*255),(int)(b*255));
            };
            auto [rr,gg,bb] = hsv2rgb(fmod(hue,1.0), 0.8, 0.9);
            return make_tuple(rr,gg,bb);
        };
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int node = x / 16;
                int c = tj.comp[node];
                auto [r,g,b] = colorOf(c);
                // 顶部留一条黑色隔离带表示 SCC 边界
                if (y < 8 && node > 0 && tj.comp[node] != tj.comp[node-1]) { r=g=b=0; }
                f << r << " " << g << " " << b << " ";
            }
            f << "\n";
        }
        f.close();

        // 量化验证图像
        ifstream in("tarjan_scc_output.ppm");
        string magic; int w,h,mx; in>>magic>>w>>h>>mx;
        vector<int> px(w*h*3); for(auto&v:px) in>>v;
        double mean=0, var=0; for(int v:px) mean+=v; mean/=px.size();
        for(int v:px) var+=(v-mean)*(v-mean);
        var/=px.size(); double std=sqrt(var);
        printf("\n[图像量化] 尺寸=%dx%d 均值=%.1f 标准差=%.1f\n", w, h, mean, std);
        bool imgOk = (mean>10 && mean<245 && std>5);
        printf("[图像量化] 像素统计=%s\n", imgOk ? "✅" : "❌");
        total++; if (imgOk) passed++;
    }

    printf("\n=== 结果汇总: %d/%d 通过 ===\n", passed, total);
    if (passed != total) { printf("❌ 存在失败测试\n"); return 1; }
    printf("✅ 全部验证通过\n");
    return 0;
}
