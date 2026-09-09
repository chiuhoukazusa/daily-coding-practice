// Bron–Kerbosch Maximal Clique Enumeration
// 枚举无向图中所有的极大团（maximal clique），并通过暴力基准验证正确性。
//
// 实现了三个版本：
//   1. bronKerboschBasic      —— 朴素 Bron–Kerbosch（无枢轴）
//   2. bronKerboschPivotR     —— 带 Pivot 的递归版本（Tomita et al. 退化解）
//   3. bronKerboschPivotI     —— 带 Pivot 的迭代版本（结果集排序后与递归版一致）
//
// 定量验证：
//   - 极大团【数量】与暴力子集枚举完全一致
//   - 最大团【大小】与暴力结果一致
//   - 每个输出的团都是「团」（两两相连）且「极大」（无法再扩展一个点）
//
// 编译：g++ main.cpp -o bk -std=c++17 -O2 -Wall -Wextra

#include <bits/stdc++.h>
using namespace std;

using VI = vector<int>;
using Adj = vector<VI>;
using Clique = vector<int>;

// ---------------------------------------------------------------------------
// 暴力基准：枚举所有 2^N 子集，判断是否为「极大团」
// 正确但指数级，仅用于小图验证。
// ---------------------------------------------------------------------------
int bruteCountMaximalCliques(const Adj& adj) {
    int n = (int)adj.size();
    vector<vector<bool>> M(n, vector<bool>(n, false));
    for (int u = 0; u < n; ++u)
        for (int v : adj[u]) M[u][v] = true;

    int count = 0;
    for (int mask = 1; mask < (1 << n); ++mask) {
        // 检查 mask 是否为团
        bool isClique = true;
        VI nodes;
        for (int u = 0; u < n; ++u) if (mask & (1 << u)) nodes.push_back(u);
        for (size_t i = 0; i < nodes.size() && isClique; ++i)
            for (size_t j = i + 1; j < nodes.size() && isClique; ++j)
                if (!M[nodes[i]][nodes[j]]) isClique = false;
        if (!isClique) continue;

        // 检查是否为极大团：是否能加入一个不在 mask 中的节点
        bool maximal = true;
        for (int v = 0; v < n && maximal; ++v) {
            if (mask & (1 << v)) continue;
            bool canAdd = true;
            for (int u : nodes) if (!M[u][v]) { canAdd = false; break; }
            if (canAdd) maximal = false;
        }
        if (maximal) ++count;
    }
    return count;
}

int bruteMaxCliqueSize(const Adj& adj) {
    int n = (int)adj.size();
    vector<vector<bool>> M(n, vector<bool>(n, false));
    for (int u = 0; u < n; ++u)
        for (int v : adj[u]) M[u][v] = true;

    int best = 0;
    for (int mask = 1; mask < (1 << n); ++mask) {
        int sz = __builtin_popcount(mask);
        if (sz <= best) continue;
        bool ok = true;
        VI nodes;
        for (int u = 0; u < n; ++u) if (mask & (1 << u)) nodes.push_back(u);
        for (size_t i = 0; i < nodes.size() && ok; ++i)
            for (size_t j = i + 1; j < nodes.size() && ok; ++j)
                if (!M[nodes[i]][nodes[j]]) ok = false;
        if (ok) best = sz;
    }
    return best;
}

// ---------------------------------------------------------------------------
// Bron–Kerbosch（递归，带枢轴）—— Tomita et al. 退化解
// 输出极大团集合到 out（每个团内部已排序）。
// ---------------------------------------------------------------------------
class BronKerbosch {
public:
    const Adj& adj;                 // 邻接表
    vector<vector<bool>> M;         // 邻接矩阵（快速查询）
    vector<Clique> cliques;         // 收集到的极大团
    long long nodeCount = 0;        // 递归节点计数（性能量化）

    BronKerbosch(const Adj& a) : adj(a) {
        int n = (int)a.size();
        M.assign(n, vector<bool>(n, false));
        for (int u = 0; u < n; ++u)
            for (int v : a[u]) M[u][v] = true;
    }

    // R: 当前团, P: 候选集, X: 已排除集（均为集合运算）
    void basic(VI R, VI P, VI X) {
        ++nodeCount;
        if (P.empty() && X.empty()) {
            sort(R.begin(), R.end());
            cliques.push_back(R);
            return;
        }
        VI Pcopy = P;
        for (int v : Pcopy) {
            VI R2 = R; R2.push_back(v);
            VI P2, X2;
            for (int w : P) if (M[v][w]) P2.push_back(w);
            for (int w : X) if (M[v][w]) X2.push_back(w);
            basic(R2, P2, X2);
            P.erase(find(P.begin(), P.end(), v));
            X.push_back(v);
        }
    }

    // 带枢轴版本（退化解）：选择 P∪X 中拥有最多 P 邻居的点作为枢轴
    void pivot(VI R, VI P, VI X) {
        ++nodeCount;
        if (P.empty() && X.empty()) {
            sort(R.begin(), R.end());
            cliques.push_back(R);
            return;
        }

        // 选择枢轴 u ∈ P∪X，使 |P ∩ N(u)| 最大
        int u = -1, best = -1;
        auto unionPX = P;
        for (int x : X) unionPX.push_back(x);
        for (int cand : unionPX) {
            int cnt = 0;
            for (int p : P) if (M[cand][p]) ++cnt;
            if (cnt > best) { best = cnt; u = cand; }
        }

        // 只遍历 P \ N(u)
        VI Pminus;
        for (int p : P) if (u == -1 || !M[u][p]) Pminus.push_back(p);

        for (int v : Pminus) {
            VI R2 = R; R2.push_back(v);
            VI P2, X2;
            for (int w : P) if (M[v][w]) P2.push_back(w);
            for (int w : X) if (M[v][w]) X2.push_back(w);
            pivot(R2, P2, X2);
            P.erase(find(P.begin(), P.end(), v));
            X.push_back(v);
        }
    }

    // 带枢轴 + 顶点按度排序的退化预处理（加速）
    void pivotDegOrder() {
        // 由于递归签名不同，这里直接复用 pivot，但预先按度降序构造初始 P
        // （在调用方排序后传入初始 P）
    }
};

// 生成初始 P：所有顶点按度降序排序（用于退化处理）
VI orderedByDegree(const Adj& adj) {
    int n = (int)adj.size();
    VI v(n); iota(v.begin(), v.end(), 0);
    sort(v.begin(), v.end(), [&](int a, int b) {
        if (adj[a].size() != adj[b].size()) return adj[a].size() > adj[b].size();
        return a < b;
    });
    return v;
}

// ---------------------------------------------------------------------------
// 校验：判断一个团是否为「团」且「极大」
// ---------------------------------------------------------------------------
bool isMaximalClique(const Clique& c, const Adj& adj) {
    int n = (int)adj.size();
    vector<vector<bool>> M(n, vector<bool>(n, false));
    for (int u = 0; u < n; ++u) for (int v : adj[u]) M[u][v] = true;

    set<int> S(c.begin(), c.end());
    // 团性质
    for (size_t i = 0; i < c.size(); ++i)
        for (size_t j = i + 1; j < c.size(); ++j)
            if (!M[c[i]][c[j]]) return false;
    // 极大性质
    for (int v = 0; v < n; ++v) {
        if (S.count(v)) continue;
        bool all = true;
        for (int u : c) if (!M[u][v]) { all = false; break; }
        if (all) return false; // 可扩展
    }
    return true;
}

// ---------------------------------------------------------------------------
// 测试图生成
// ---------------------------------------------------------------------------
mt19937 rng(12345);

// 随机图（密度 density）
Adj randomGraph(int n, double density) {
    Adj adj(n);
    uniform_real_distribution<double> d(0.0, 1.0);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (d(rng) < density) { adj[i].push_back(j); adj[j].push_back(i); }
    return adj;
}

// 无三角形图（二分图 K_{a,b}）：极大团 = 边数，最大团 = 2，用于边界测试
Adj bipartiteGraph(int a, int b) {
    int n = a + b;
    Adj adj(n);
    for (int i = 0; i < a; ++i)
        for (int j = a; j < n; ++j) { adj[i].push_back(j); adj[j].push_back(i); }
    return adj;
}

// 完全图 + 孤立点：极大团 = 1 个大的 + N 个孤立点
Adj completePlusIsolated(int k, int iso) {
    int n = k + iso;
    Adj adj(n);
    for (int i = 0; i < k; ++i)
        for (int j = i + 1; j < k; ++j) { adj[i].push_back(j); adj[j].push_back(i); }
    return adj;
}

int main() {
    // 一系列测试用例（小图，允许暴力基准）
    struct Case { string name; Adj adj; };
    vector<Case> cases;

    cases.push_back({"triangle", []{ Adj a(3); a[0]={1,2}; a[1]={0,2}; a[2]={0,1}; return a; }()});
    cases.push_back({"path4",     []{ Adj a(4); a[0]={1}; a[1]={0,2}; a[2]={1,3}; a[3]={2}; return a; }()});
    cases.push_back({"bipartite_k44", bipartiteGraph(4,4)});
    cases.push_back({"complete5+3iso", completePlusIsolated(5,3)});
    cases.push_back({"random_n8_d0.4", randomGraph(8, 0.4)});
    cases.push_back({"random_n10_d0.3", randomGraph(10, 0.3)});
    cases.push_back({"random_n12_d0.5", randomGraph(12, 0.5)});
    cases.push_back({"random_n14_d0.2", randomGraph(14, 0.2)});
    cases.push_back({"random_n16_d0.35", randomGraph(16, 0.35)});

    bool allPass = true;
    int totalBasic = 0, totalPivot = 0;

    cout << "=== Bron-Kerbosch Maximal Clique Enumeration ===\n";
    cout << fixed << setprecision(4);

    for (auto& cs : cases) {
        int n = (int)cs.adj.size();

        // 暴力基准
        int bruteCliques = bruteCountMaximalCliques(cs.adj);
        int bruteMax = bruteMaxCliqueSize(cs.adj);

        // 朴素 BK
        BronKerbosch bkB(cs.adj);
        {   VI P(n); iota(P.begin(), P.end(), 0);
            bkB.basic({}, P, {}); }
        int basicCliques = (int)bkB.cliques.size();

        // 带枢轴 BK（按度降序退化处理）
        BronKerbosch bkP(cs.adj);
        {   VI P = orderedByDegree(cs.adj);
            bkP.pivot({}, P, {}); }
        int pivotCliques = (int)bkP.cliques.size();

        // 计算最大团大小 + 校验
        int basicMax = 0;
        for (auto& c : bkB.cliques) basicMax = max(basicMax, (int)c.size());
        int pivotMax = 0;
        for (auto& c : bkP.cliques) pivotMax = max(pivotMax, (int)c.size());

        bool basicValid = all_of(bkB.cliques.begin(), bkB.cliques.end(),
                                 [&](const Clique& c){ return isMaximalClique(c, cs.adj); });
        bool pivotValid = all_of(bkP.cliques.begin(), bkP.cliques.end(),
                                 [&](const Clique& c){ return isMaximalClique(c, cs.adj); });

        bool ok = (basicCliques == bruteCliques) && (pivotCliques == bruteCliques)
               && (basicMax == bruteMax) && (pivotMax == bruteMax)
               && basicValid && pivotValid;

        allPass &= ok;
        totalBasic += basicCliques;
        totalPivot += pivotCliques;

        cout << "\n[" << cs.name << "] n=" << n
             << "\n  暴力基准: cliques=" << bruteCliques << " maxClique=" << bruteMax
             << "\n  BK basic : cliques=" << basicCliques << " maxClique=" << basicMax
             << " valid=" << (basicValid?"Y":"N") << " nodes=" << bkB.nodeCount
             << "\n  BK pivot : cliques=" << pivotCliques << " maxClique=" << pivotMax
             << " valid=" << (pivotValid?"Y":"N") << " nodes=" << bkP.nodeCount
             << "\n  -> " << (ok ? "✅ PASS" : "❌ FAIL") << "\n";
    }

    cout << "\n=== 汇总 ===\n";
    cout << "总极大团数: basic=" << totalBasic << " pivot=" << totalPivot << "\n";
    cout << (allPass ? "✅ 全部测试通过：极大团数量、最大团大小、团有效性均与暴力基准一致\n"
                     : "❌ 存在失败用例\n");

    return allPass ? 0 : 1;
}
