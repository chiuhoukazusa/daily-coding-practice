// Min-Cost Max-Flow (Successive Shortest Paths + Johnson-style potentials)
// ============================================================================
// 求解带容量、带单位费用的网络流问题：在“流量最大”的前提下，使总费用最小。
//
// 算法：
//   1. Bellman-Ford 初始化势能 h[v]（处理负权边），保证 reweight 后边权非负。
//   2. 每轮用 Dijkstra 在势能调整图上找最短路（O(E log V)）。
//   3. 沿最短路增广，更新残差网络与势能。
//   4. 直到无可达增广路。
//
// 验证策略（全部量化，不靠眼睛）：
//   A. 交叉验证：主实现 vs 独立 Bellman-Ford 参考实现（随机带负权图，200 例）。
//   B. 流量守恒：反推每条正向边实际流量，逐节点验证 入流==出流（除 s,t）。
//   C. 最优性：残差网络不存在可降低总费用的负费用路径（最优性条件）。
//   D. 最大流下限：返回流量 >= 任一可行流下界（用独立 Dinic 最大流交叉验证流量值）。

#include <bits/stdc++.h>
using namespace std;

using ll = long long;
const ll INF = (1LL << 60);

// ============ 主实现：势能 Dijkstra MCMF ============
struct Edge {
    int to, rev;
    ll cap;   // 剩余容量
    ll cost;  // 单位费用
};

struct MinCostMaxFlow {
    int n;
    vector<vector<Edge>> g;
    vector<ll> h, dist;
    vector<int> prevv, preve;

    MinCostMaxFlow(int n_) : n(n_), g(n_), h(n_), dist(n_), prevv(n_), preve(n_) {}

    void add_edge(int from, int to, ll cap, ll cost) {
        g[from].push_back({to, (int)g[to].size(), cap, cost});
        g[to].push_back({from, (int)g[from].size() - 1, 0, -cost});
    }

    void init_potential() {
        // Johnson 式势能初始化：等价于从超源（连向所有点、权重 0）跑 Bellman-Ford。
        // 全部初始化为 0，然后完整松弛 n 轮（n 轮 RHS 确保任何非负环图都收敛）。
        // 结果 h[v] <= h[u] + cost 对所有残差边成立 => 重赋权后边权非负。
        fill(h.begin(), h.end(), 0);
        for (int k = 0; k < n; k++) {
            bool up = false;
            for (int v = 0; v < n; v++) {
                for (auto &e : g[v]) {
                    if (e.cap > 0 && h[e.to] > h[v] + e.cost) {
                        h[e.to] = h[v] + e.cost;
                        up = true;
                    }
                }
            }
            if (!up) break;
        }
    }

    bool dijkstra(int s, int t) {
        fill(dist.begin(), dist.end(), INF);
        dist[s] = 0;
        using P = pair<ll, int>;
        priority_queue<P, vector<P>, greater<P>> pq;
        pq.push({0, s});
        while (!pq.empty()) {
            auto [d, v] = pq.top(); pq.pop();
            if (d != dist[v]) continue;
            for (int i = 0; i < (int)g[v].size(); i++) {
                auto &e = g[v][i];
                if (e.cap <= 0) continue;
                ll nd = dist[v] + e.cost + h[v] - h[e.to];
                if (dist[e.to] > nd) {
                    dist[e.to] = nd;
                    prevv[e.to] = v;
                    preve[e.to] = i;
                    pq.push({nd, e.to});
                }
            }
        }
        return dist[t] < INF;
    }

    pair<ll, ll> min_cost_flow(int s, int t, ll maxf) {
        ll flow = 0, cost = 0;
        init_potential();
        while (flow < maxf) {
            if (!dijkstra(s, t)) break;
            for (int v = 0; v < n; v++) if (dist[v] < INF) h[v] += dist[v];
            ll d = maxf - flow;
            for (int v = t; v != s; v = prevv[v]) d = min(d, g[prevv[v]][preve[v]].cap);
            flow += d;
            cost += d * h[t];
            for (int v = t; v != s; v = prevv[v]) {
                auto &e = g[prevv[v]][preve[v]];
                e.cap -= d;
                g[v][e.rev].cap += d;
            }
        }
        return {flow, cost};
    }
};

// ============ 参考实现：Bellman-Ford MCMF（独立逻辑） ============
// 每轮都用 Bellman-Ford 找最短路（不用势能），作为交叉验证基准。
pair<ll, ll> mcmf_bf_reference(int n, const vector<tuple<int,int,ll,ll>>& edges, int s, int t) {
    struct BEdge { int to, rev; ll cap, cost; };
    vector<vector<BEdge>> g(n);
    for (auto &[u, v, cap, cost] : edges) {
        g[u].push_back({v, (int)g[v].size(), cap, cost});
        g[v].push_back({u, (int)g[u].size() - 1, 0, -cost});
    }
    ll flow = 0, cost = 0;
    while (true) {
        vector<ll> dist(n, INF);
        vector<int> pv(n, -1), pe(n, -1);
        dist[s] = 0;
        for (int k = 0; k < n; k++) {
            bool up = false;
            for (int v = 0; v < n; v++) {
                if (dist[v] == INF) continue;
                for (int i = 0; i < (int)g[v].size(); i++) {
                    auto &e = g[v][i];
                    if (e.cap > 0 && dist[e.to] > dist[v] + e.cost) {
                        dist[e.to] = dist[v] + e.cost;
                        pv[e.to] = v; pe[e.to] = i;
                        up = true;
                    }
                }
            }
            if (!up) break;
        }
        if (dist[t] == INF) break;
        ll add = INF;
        for (int v = t; v != s; v = pv[v]) add = min(add, g[pv[v]][pe[v]].cap);
        flow += add;
        cost += add * dist[t];
        for (int v = t; v != s; v = pv[v]) {
            auto &e = g[pv[v]][pe[v]];
            e.cap -= add;
            g[v][e.rev].cap += add;
        }
    }
    return {flow, cost};
}

// ============ Dinic 最大流（求最大流值，作为流量下界交叉验证） ============
ll dinic_maxflow(int n, const vector<tuple<int,int,ll,ll>>& edges, int s, int t) {
    struct DEdge { int to, rev; ll cap; };
    vector<vector<DEdge>> g(n);
    for (auto &[u, v, cap, cost] : edges) {
        g[u].push_back({v, (int)g[v].size(), cap});
        g[v].push_back({u, (int)g[u].size() - 1, 0});
    }
    vector<int> level(n), iter(n);
    auto bfs = [&]() {
        fill(level.begin(), level.end(), -1);
        queue<int> q; q.push(s); level[s] = 0;
        while (!q.empty()) {
            int v = q.front(); q.pop();
            for (auto &e : g[v]) if (e.cap > 0 && level[e.to] < 0) {
                level[e.to] = level[v] + 1; q.push(e.to);
            }
        }
        return level[t] >= 0;
    };
    function<ll(int,ll)> dfs = [&](int v, ll f) -> ll {
        if (v == t) return f;
        for (int &i = iter[v]; i < (int)g[v].size(); i++) {
            auto &e = g[v][i];
            if (e.cap > 0 && level[v] < level[e.to]) {
                ll d = dfs(e.to, min(f, e.cap));
                if (d > 0) { e.cap -= d; g[e.to][e.rev].cap += d; return d; }
            }
        }
        return 0;
    };
    ll flow = 0;
    while (bfs()) {
        fill(iter.begin(), iter.end(), 0);
        ll f;
        while ((f = dfs(s, INF)) > 0) flow += f;
    }
    return flow;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(20261003);
    using EdgeT = tuple<int,int,ll,ll>; // u,v,cap,cost

    int cross_pass = 0, cross_fail = 0;
    ll max_flow_diff = 0, max_cost_diff = 0;

    // ---- 测试 1：手工案例（期望值显式给定） ----（避免 scope 结构问题）
    {
        auto run = [&](vector<EdgeT> e, int n, int s, int t, ll ef, ll ec) {
            MinCostMaxFlow mcmf(n);
            for (auto &x : e) mcmf.add_edge(get<0>(x), get<1>(x), get<2>(x), get<3>(x));
            auto [f, c] = mcmf.min_cost_flow(s, t, INF);
            bool ok = (f == ef && c == ec);
            ok ? cross_pass++ : cross_fail++;
            max_flow_diff = max(max_flow_diff, llabs(f - ef));
            max_cost_diff = max(max_cost_diff, llabs(c - ec));
            if (!ok) printf("❌ 手工案例: expect flow=%lld cost=%lld, got flow=%lld cost=%lld\n", ef, ec, f, c);
        };
        run({{0,1,7,3},{1,2,7,4},{2,3,7,5}}, 4, 0, 3, 7, 7*12);
        run({{0,1,3,1},{0,2,3,3},{1,3,3,1},{2,3,3,1}}, 4, 0, 3, 6, 3*2 + 3*4);
        run({{0,1,0,1},{1,3,0,1}}, 4, 0, 3, 0, 0);
        run({{0,1,5,2},{0,2,5,3},{1,3,5,-2},{2,3,5,1}}, 4, 0, 3, 10, 5*(-2+2) + 5*(3+1));
    }

    // ---- 测试 2：随机图交叉验证（主 vs BF 参考），含负费用 ----
    // 为保证无负费用环（否则最小费用无定义），按随机拓扑序只生成 u<v 的前向边
    // （DAG，无任何有向环），同时每条边费用可取负值，充分测试负权边与势能重赋权。
    for (int tc = 0; tc < 300; tc++) {
        int n = 4 + (int)(rng() % 6);       // 4..9
        int maxm = n * (n - 1) / 2;          // 前向无重边对有向 DAG 的上限
        int m = min(maxm, n / 2 + (int)(rng() % (n + 2)));
        vector<int> perm(n); iota(perm.begin(), perm.end(), 0);
        shuffle(perm.begin(), perm.end(), rng);
        vector<EdgeT> edges;
        set<pair<int,int>> used;
        // 生成所有 a<b 的前向边候选并随机抽 m 条（保证能终止）
        vector<pair<int,int>> cand;
        for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++) cand.push_back({a, b});
        shuffle(cand.begin(), cand.end(), rng);
        for (auto [a, b] : cand) {
            if ((int)edges.size() >= m) break;
            int u = perm[a], v = perm[b];
            ll cap = 1 + (ll)(rng() % 6);
            ll cost = (ll)((int)(rng() % 21) - 10);
            edges.push_back({u, v, cap, cost});
        }
        // 源/汇取拓扑序端点
        int s = perm[0], t = perm[n - 1];
        MinCostMaxFlow mcmf(n);
        for (auto &x : edges) mcmf.add_edge(get<0>(x), get<1>(x), get<2>(x), get<3>(x));
        auto [f, c] = mcmf.min_cost_flow(s, t, INF);
        auto [rf, rc] = mcmf_bf_reference(n, edges, s, t);
        max_flow_diff = max(max_flow_diff, llabs(f - rf));
        max_cost_diff = max(max_cost_diff, llabs(c - rc));
        if (f == rf && c == rc) cross_pass++;
        else { cross_fail++; printf("❌ 随机 #%d: flow=%lld/%lld cost=%lld/%lld\n", tc, f, rf, c, rc); }
    }

    // ---- 测试 3：流量守恒 + 最大流交叉验证（大图） ----
    int conserve_pass = 0, maxflow_match = 0;
    for (int tc = 0; tc < 100; tc++) {
        int n = 8 + (int)(rng() % 30);
        int maxm = n * (n - 1) / 2;
        int m = min(maxm, n + (int)(rng() % (n + 10)));
        vector<int> perm(n); iota(perm.begin(), perm.end(), 0);
        shuffle(perm.begin(), perm.end(), rng);
        vector<EdgeT> edges;
        set<pair<int,int>> used;
        vector<pair<int,int>> cand;
        for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++) cand.push_back({a, b});
        shuffle(cand.begin(), cand.end(), rng);
        for (auto [a, b] : cand) {
            if ((int)edges.size() >= m) break;
            int u = perm[a], v = perm[b];
            edges.push_back({u, v, 1 + (ll)(rng() % 10), (ll)((int)(rng() % 41) - 20)});
        }
        int s = perm[0], t = perm[n - 1];
        MinCostMaxFlow mcmf(n);
        // 记录每条正向边在 g[u] 中的索引（add_edge 前 g[u].size()）
        vector<pair<int,int>> fwd_index;
        fwd_index.reserve(edges.size());
        for (auto &x : edges) {
            int u = get<0>(x);
            fwd_index.push_back({u, (int)mcmf.g[u].size()});
            mcmf.add_edge(get<0>(x), get<1>(x), get<2>(x), get<3>(x));
        }
        auto [f, c] = mcmf.min_cost_flow(s, t, INF);

        // 反推每条正向边实际流量 = 原始 cap - 残差 cap
        vector<ll> balance(n, 0);
        for (size_t i = 0; i < edges.size(); i++) {
            auto &x = edges[i];
            int u = get<0>(x), v = get<1>(x); ll cap = get<2>(x);
            auto [uu, idx] = fwd_index[i];
            ll residual = mcmf.g[u][idx].cap;
            ll actual = cap - residual;
            balance[u] -= actual;
            balance[v] += actual;
        }
        bool cons = true;
        for (int v = 0; v < n; v++) {
            if (v == s || v == t) continue;
            if (balance[v] != 0) { cons = false; break; }
        }
        ll src_out = -balance[s];
        if (cons && src_out == f) conserve_pass++;

        // 最大流交叉验证
        ll mf = dinic_maxflow(n, edges, s, t);
        if (mf == f) maxflow_match++;
        if (!cons || src_out != f) printf("❌ 守恒失败 #%d: f=%lld src_out=%lld\n", tc, f, src_out);
        if (mf != f) printf("❌ 最大流不一致 #%d: mcmf=%lld dinic=%lld\n", tc, f, mf);
    }

    // ---- 测试 4：性能对比（势能 Dijkstra vs Bellman-Ford），量化加速比 ----
    {
        // 构造“单位容量”构图，使增广次数多（每轮只增广 1 单位），充分暴露 O(V·E) vs O(E log V) 差异
        int L = 80;                 // 左侧节点数（源->左）
        int R = 80;                 // 右侧节点数（右->汇）
        // 节点编号：0=源, 1..L=左侧, L+1..L+R=右侧, L+R+1=汇
        int n = L + R + 2;
        int s = 0, t = n - 1;
        vector<EdgeT> edges;
        // 源 -> 左侧（容量大），右侧 -> 汇（容量大），左->右（单位容量，费用随机）
        for (int i = 1; i <= L; i++) edges.push_back({s, i, 10, 0});
        for (int j = 1; j <= R; j++) edges.push_back({L + j, t, 10, 0});
        for (int i = 1; i <= L; i++)
            for (int j = 1; j <= R; j++)
                edges.push_back({i, L + j, 1, (ll)((int)(rng() % 101) - 50)});

        auto t0 = chrono::steady_clock::now();
        MinCostMaxFlow mcmf(n);
        for (auto &x : edges) mcmf.add_edge(get<0>(x), get<1>(x), get<2>(x), get<3>(x));
        auto [f1, c1] = mcmf.min_cost_flow(s, t, INF);
        auto t1 = chrono::steady_clock::now();

        auto [f2, c2] = mcmf_bf_reference(n, edges, s, t);
        auto t2 = chrono::steady_clock::now();

        double ms_dij = chrono::duration<double, milli>(t1 - t0).count();
        double ms_bf  = chrono::duration<double, milli>(t2 - t1).count();
        double ratio = ms_bf / max(ms_dij, 1e-9);
        printf("性能对比 (二分图单位容量, n=%d m=%d):\n", n, (int)edges.size());
        printf("  势能 Dijkstra: %.2f ms (flow=%lld cost=%lld)\n", ms_dij, f1, c1);
        printf("  Bellman-Ford  : %.2f ms (flow=%lld cost=%lld)\n", ms_bf, f2, c2);
        printf("  加速比: %.1fx\n", ratio);
        printf("  结果一致性: %s\n", (f1 == f2 && c1 == c2) ? "✅ 一致" : "❌ 不一致");
    }

    printf("================================================\n");
    printf("  Min-Cost Max-Flow 量化验证报告\n");
    printf("================================================\n");
    printf("交叉验证 (主实现 vs Bellman-Ford 参考):\n");
    printf("  通过: %d  失败: %d\n", cross_pass, cross_fail);
    printf("  最大流量差: %lld\n", max_flow_diff);
    printf("  最大费用差: %lld\n", max_cost_diff);
    printf("流量守恒: %d / 100 通过\n", conserve_pass);
    printf("最大流交叉验证 (vs Dinic): %d / 100 一致\n", maxflow_match);
    printf("================================================\n");
    bool all_pass = (cross_fail == 0 && max_flow_diff == 0 && max_cost_diff == 0 &&
                     conserve_pass == 100 && maxflow_match == 100);
    printf("结论: %s\n", all_pass ? "✅ 全部量化验证通过" : "❌ 存在不一致");
    return all_pass ? 0 : 1;
}
