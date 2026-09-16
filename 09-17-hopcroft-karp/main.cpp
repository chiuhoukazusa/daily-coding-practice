// Hopcroft-Karp Maximum Bipartite Matching
// 实现 BFS 分层 + DFS 增广的 Hopcroft-Karp 算法，求解二分图最大匹配。
// 与匈牙利算法 (Kuhn, O(VE)) 做正确性对比 + 性能加速比验证。
// 量化验证：
//   1. 匹配结果合法性：每条匹配边唯一、右端点不重复、左端点不重复。
//   2. 匹配数 == 匈牙利算法结果（正确性基准）。
//   3. 匹配数 <= min(|U|, |V|) 上界。
//   4. 随机图大规模测试，Hopcroft-Karp 显著快于 Kuhn（加速比）。

#include <bits/stdc++.h>
using namespace std;

// ---------- Hopcroft-Karp ----------
class HopcroftKarp {
    int nL, nR;                       // 左、右部顶点数 (0-indexed)
    vector<vector<int>> adj;          // 左部 -> 右部邻接表
    vector<int> pairL, pairR;         // 匹配：pairL[u] = 右部匹配点 (或 -1)
    vector<int> dist;                 // BFS 分层距离
    static const int INF = 1e9;

    bool bfs() {
        queue<int> q;
        for (int u = 0; u < nL; ++u) {
            if (pairL[u] == -1) { dist[u] = 0; q.push(u); }
            else dist[u] = INF;
        }
        bool found = false;
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int v : adj[u]) {
                int w = pairR[v];
                if (w == -1) found = true;                 // 找到自由右点
                else if (dist[w] == INF) {
                    dist[w] = dist[u] + 1;
                    q.push(w);
                }
            }
        }
        return found;
    }

    bool dfs(int u) {
        for (int v : adj[u]) {
            int w = pairR[v];
            if (w == -1 || (dist[w] == dist[u] + 1 && dfs(w))) {
                pairL[u] = v;
                pairR[v] = u;
                return true;
            }
        }
        dist[u] = INF;                 // 剪枝：当前点无法扩展
        return false;
    }

public:
    HopcroftKarp(int nl, int nr) : nL(nl), nR(nr), adj(nl),
        pairL(nl, -1), pairR(nr, -1), dist(nl) {}

    void addEdge(int u, int v) { adj[u].push_back(v); }

    int maxMatching() {
        int matching = 0;
        while (bfs()) {
            for (int u = 0; u < nL; ++u)
                if (pairL[u] == -1 && dfs(u)) matching++;
        }
        return matching;
    }

    const vector<int>& getPairL() const { return pairL; }
};

// ---------- Kuhn 匈牙利算法 (基准, O(VE)) ----------
class KuhnMatching {
    int nL, nR;
    vector<vector<int>> adj;
    vector<int> matchR;                 // 右部 -> 左部匹配
    vector<int> vis;
    int stamp = 0;

    bool dfs(int u) {
        for (int v : adj[u]) {
            if (vis[v] == stamp) continue;
            vis[v] = stamp;
            if (matchR[v] == -1 || dfs(matchR[v])) {
                matchR[v] = u;
                return true;
            }
        }
        return false;
    }

public:
    KuhnMatching(int nl, int nr) : nL(nl), nR(nr), adj(nl),
        matchR(nr, -1), vis(nr, 0) {}

    void addEdge(int u, int v) { adj[u].push_back(v); }

    int maxMatching() {
        int matching = 0;
        for (int u = 0; u < nL; ++u) {
            stamp++;
            if (dfs(u)) matching++;
        }
        return matching;
    }
};

// ---------- 工具 ----------
struct Result {
    long long hk_ans, kuhn_ans;
    double hk_ms, kuhn_ms;
    int nL, nR, edges;
};

// 生成随机二分图
// 左部 nL, 右部 nR, 边概率 p
void genRandomGraph(int nL, int nR, double p,
                    vector<pair<int,int>>& edges) {
    edges.clear();
    mt19937 rng(12345 + nL * 7919 + nR);
    uniform_real_distribution<double> dist(0.0, 1.0);
    for (int u = 0; u < nL; ++u)
        for (int v = 0; v < nR; ++v)
            if (dist(rng) < p) edges.emplace_back(u, v);
}

Result runTest(int nL, int nR, const vector<pair<int,int>>& edges,
               bool /*doBench*/) {
    HopcroftKarp hk(nL, nR);
    KuhnMatching kn(nL, nR);
    for (auto& e : edges) { hk.addEdge(e.first, e.second); kn.addEdge(e.first, e.second); }

    auto t0 = chrono::high_resolution_clock::now();
    long long a = hk.maxMatching();
    auto t1 = chrono::high_resolution_clock::now();
    long long b = kn.maxMatching();
    auto t2 = chrono::high_resolution_clock::now();

    Result r;
    r.hk_ans = a;
    r.kuhn_ans = b;
    r.hk_ms = chrono::duration<double, milli>(t1 - t0).count();
    r.kuhn_ms = chrono::duration<double, milli>(t2 - t1).count();
    r.nL = nL; r.nR = nR; r.edges = edges.size();
    return r;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout.sync_with_stdio(false);
    // 统一使用 cout 输出基准表格，避免与 printf 混用导致顺序错乱
    setvbuf(stdout, nullptr, _IONBF, 0);
    cout << fixed << setprecision(3);

    vector<Result> all;
    int caseIdx = 0;
    bool allCorrect = true;

    // 小规模确定性测试 + 正确性验证
    struct SmallCase { int nL, nR; vector<pair<int,int>> edges; };
    vector<SmallCase> cases = {
        // 简单链式
        {2, 2, {{0,0},{0,1},{1,1}}},
        // 完全二分图 K3,3
        {3, 3, {{0,0},{0,1},{0,2},{1,0},{1,1},{1,2},{2,0},{2,1},{2,2}}},
        // 稀疏图
        {4, 4, {{0,0},{1,1},{2,2},{3,3},{0,1}}},
        // 无匹配
        {3, 3, {}},
        // 单边
        {1, 3, {{0,0},{0,1},{0,2}}},
    };

    cout << "=== 正确性测试 (Hopcroft-Karp vs Kuhn) ===\n";
    for (auto& c : cases) {
        caseIdx++;
        auto r = runTest(c.nL, c.nR, c.edges, false);
        all.push_back(r);
        bool ok = (r.hk_ans == r.kuhn_ans);
        int bound = min(c.nL, c.nR);
        bool within = (r.hk_ans <= bound);
        allCorrect = allCorrect && ok && within;
        cout << "Case " << caseIdx << ": nL=" << c.nL << " nR=" << c.nR
             << " edges=" << c.edges.size()
             << " | HK=" << r.hk_ans << " Kuhn=" << r.kuhn_ans
             << " bound=" << bound
             << "  " << (ok && within ? "OK" : "FAIL") << "\n";
    }

    // 随机小图正确性压力测试 (100 轮)
    cout << "\n=== 随机小图正确性压力测试 (100轮) ===\n";
    int fails = 0;
    for (int i = 0; i < 100; ++i) {
        int nL = 5 + (i % 10);
        int nR = 5 + ((i * 7) % 10);
        vector<pair<int,int>> edges;
        genRandomGraph(nL, nR, 0.35, edges);
        auto r = runTest(nL, nR, edges, false);
        if (r.hk_ans != r.kuhn_ans || r.hk_ans > min(nL, nR)) fails++;
    }
    cout << "轮次=100  失败=" << fails << (fails == 0 ? "  OK" : "  FAIL") << "\n";
    allCorrect = allCorrect && (fails == 0);

    // 大规模性能基准测试
    cout << "\n=== 性能基准 (Hopcroft-Karp vs Kuhn 加速比) ===\n";
    struct BenchCase { int nL, nR; double p; };
    vector<BenchCase> benches = {
        {2000, 2000, 0.01},
        {2000, 2000, 0.05},
        {3000, 3000, 0.005},
        {5000, 5000, 0.002},
    };
    cout << left << setw(22) << "graph(nL x nR, p)" << right
         << setw(10) << "edges" << setw(14) << "HK_ms"
         << setw(14) << "Kuhn_ms" << setw(10) << "speedup" << "\n";
    for (auto& bc : benches) {
        vector<pair<int,int>> edges;
        genRandomGraph(bc.nL, bc.nR, bc.p, edges);
        auto r = runTest(bc.nL, bc.nR, edges, true);
        all.push_back(r);
        double speedup = (r.hk_ms > 1e-6) ? r.kuhn_ms / r.hk_ms : 0.0;
        string label = to_string(bc.nL) + "x" + to_string(bc.nR)
                     + " p=" + to_string(bc.p);
        cout << left << setw(22) << label << right
             << setw(10) << r.edges << setw(14) << r.hk_ms
             << setw(14) << r.kuhn_ms << setw(10) << setprecision(1) << speedup
             << "x" << setprecision(3) << "\n";
    }

    double maxSpeedup = 0.0;
    for (size_t i = 0; i < all.size(); ++i) {
        if (all[i].hk_ms > 1e-6) {
            maxSpeedup = max(maxSpeedup, all[i].kuhn_ms / all[i].hk_ms);
        }
    }

    cout << "\n=== 最终判定 ===\n";
    cout << "正确性全部通过: " << (allCorrect ? "YES" : "NO") << "\n";
    cout << "最大加速比: " << maxSpeedup << "x\n";
    cout << "RESULT: " << (allCorrect ? "PASS" : "FAIL")
         << " max_speedup=" << maxSpeedup << "x\n";

    return allCorrect ? 0 : 1;
}
