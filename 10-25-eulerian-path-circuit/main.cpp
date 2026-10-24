#include <bits/stdc++.h>
using namespace std;

// ============================================================
// Eulerian Path & Circuit  (Hierholzer's Algorithm)
//
// 判定 + 构造：
//   - Eulerian Circuit (欧拉回路): 起点=终点, 经过每条边恰好一次
//     条件(有向图): 所有顶点 in-degree == out-degree, 且底层图连通(非零度顶点)
//     条件(无向图): 所有顶点度数为偶数, 且连通
//   - Eulerian Path (欧拉路径): 起点!=终点
//     条件(有向图): 恰一个顶点 out-in = 1 (起点), 恰一个 in-out = 1 (终点),
//                   其余 in==out
//     条件(无向图): 恰两个奇数度顶点
//
// 构造: Hierholzer 算法 (贪心 DFS + 回边拼接), O(E)
//       与 Fleury 算法 O(E^2) 对比。
//
// 量化验证：
//   1. 边计数: 构造出的路径边数 == 总边数 E
//   2. 边集合正确性: 构造路径的边 (配对) 与输入边 multiset 完全一致
//   3. 邻接合法性: 路径中相邻边首尾顶点正确衔接
//   4. 起点/终点符合理论判定
//   5. 随机图与 Fleury 算法结果一致性 (边集合相同)
//   6. 性能加速比 (Hierholzer vs Fleury)
// ============================================================

// 有向图 Hierholzer -> 返回欧拉回路/路径的顶点序列
// adj: 邻接表(每条边用一个下标)，invmap 用于反向追踪
vector<int> hierholzer_directed(int n, const vector<pair<int,int>>& edges,
                                int start, vector<int>& edge_order) {
    vector<vector<pair<int,int>>> adj(n); // (to, edge_id)
    for (int i = 0; i < (int)edges.size(); ++i) {
        adj[edges[i].first].push_back({edges[i].second, i});
    }
    vector<int> cur(n, 0); // 每条邻接的当前进度指针
    vector<int> path_vertices;
    vector<int> vstack;
    // estack 与 vstack 平行: 记录到达 vstack[k+1] 所用的边
    vector<int> estack;
    vstack.push_back(start);
    while (!vstack.empty()) {
        int v = vstack.back();
        if (cur[v] < (int)adj[v].size()) {
            auto [to, eid] = adj[v][cur[v]++];
            vstack.push_back(to);
            estack.push_back(eid);
        } else {
            vstack.pop_back();
            path_vertices.push_back(v);
            if (!estack.empty()) { edge_order.push_back(estack.back()); estack.pop_back(); }
        }
    }
    reverse(path_vertices.begin(), path_vertices.end());
    reverse(edge_order.begin(), edge_order.end());
    return path_vertices;
}

// 无向图 Hierholzer
vector<int> hierholzer_undirected(int n, const vector<pair<int,int>>& edges,
                                  int start, vector<int>& edge_order) {
    vector<vector<pair<int,int>>> adj(n);
    for (int i = 0; i < (int)edges.size(); ++i) {
        adj[edges[i].first].push_back({edges[i].second, i});
        adj[edges[i].second].push_back({edges[i].first, i});
    }
    vector<char> used(edges.size(), 0);
    vector<int> cur(n, 0);
    vector<int> path_vertices;
    vector<int> vstack, estack;
    vstack.push_back(start);
    while (!vstack.empty()) {
        int v = vstack.back();
        bool advanced = false;
        while (cur[v] < (int)adj[v].size()) {
            auto [to, eid] = adj[v][cur[v]++];
            if (used[eid]) continue;
            used[eid] = 1;
            vstack.push_back(to);
            estack.push_back(eid);
            advanced = true;
            break;
        }
        if (!advanced) {
            vstack.pop_back();
            path_vertices.push_back(v);
            if (!estack.empty()) { edge_order.push_back(estack.back()); estack.pop_back(); }
        }
    }
    reverse(path_vertices.begin(), path_vertices.end());
    reverse(edge_order.begin(), edge_order.end());
    return path_vertices;
}

// Fleury 有向图 (贪心, 非桥优先) — 朴素 O(E^2)
vector<int> fleury_directed(int n, const vector<pair<int,int>>& edges,
                            int start, vector<int>& edge_order) {
    // 简单实现: 每次选一条边, 若不是桥则优先 (Naive is-bridge check)
    vector<vector<pair<int,int>>> adj(n);
    for (int i = 0; i < (int)edges.size(); ++i)
        adj[edges[i].first].push_back({edges[i].second, i});
    vector<char> used(edges.size(), 0);
    vector<int> edge_order_tmp;
    vector<int> verts;
    int v = start;
    verts.push_back(v);
    int remaining = (int)edges.size();

    // 正确的桥梁检测: 判断删除边 eid 后, 剩余未用边是否仍保持 (弱)连通
    auto is_bridge = [&](int eid) {
        // 精确剪枝: 判断删除该边后剩余未用边是否仍弱连通
        // 朴素 BFS: 在剩余未用边上统计弱连通分量
        vector<vector<int>> g(adj.size());
        for (int i=0;i<(int)edges.size();i++) if(!used[i] && i!=eid) {
            g[edges[i].first].push_back(edges[i].second);
            g[edges[i].second].push_back(edges[i].first);
        }
        int s=-1;
        vector<int> dd(adj.size(),0);
        for (int i=0;i<(int)edges.size();i++) if(!used[i] && i!=eid){ dd[edges[i].first]++; dd[edges[i].second]++; }
        for (int i=0;i<(int)adj.size();i++) if(dd[i]>0){s=i;break;}
        if(s==-1) return false;
        vector<char> vis(adj.size(),0); queue<int> q; q.push(s); vis[s]=1; int cnt=1;
        while(!q.empty()){ int x=q.front(); q.pop(); for(int y:g[x]) if(!vis[y]){vis[y]=1;cnt++; q.push(y);} }
        int total=0; for(int i=0;i<(int)adj.size();i++) if(dd[i]>0) total++;
        return cnt != total;
    };

    while (remaining > 0) {
        int chosen = -1, chosen_to = -1;
        // 非桥优先: 优先选非桥边
        for (int i = 0; i < (int)adj[v].size(); ++i) {
            auto [to, eid] = adj[v][i];
            if (used[eid]) continue;
            if (chosen == -1) { chosen = eid; chosen_to = to; }
            if (!is_bridge(eid)) { chosen = eid; chosen_to = to; break; }
        }
        used[chosen] = 1;
        edge_order_tmp.push_back(chosen);
        v = chosen_to;
        verts.push_back(v);
        --remaining;
    }
    edge_order = edge_order_tmp;
    return verts;
}

// 连通性检查 (无向底层图, 只考虑非零度顶点)
bool connected_undirected(int n, const vector<pair<int,int>>& edges) {
    vector<vector<int>> g(n);
    vector<int> deg(n, 0);
    for (auto &e : edges) { g[e.first].push_back(e.second); g[e.second].push_back(e.first); deg[e.first]++; deg[e.second]++; }
    int s = -1;
    for (int i = 0; i < n; ++i) if (deg[i] > 0) { s = i; break; }
    if (s == -1) return true;
    vector<char> vis(n, 0);
    queue<int> q; q.push(s); vis[s] = 1;
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (int w : g[u]) if (!vis[w]) { vis[w] = 1; q.push(w); }
    }
    for (int i = 0; i < n; ++i) if (deg[i] > 0 && !vis[i]) return false;
    return true;
}

// 有向图判定
// 返回: 0=无欧拉路径, 1=欧拉回路, 2=欧拉路径(非回路)
// 输出起点 start
int classify_directed(int n, const vector<pair<int,int>>& edges, int &start) {
    vector<int> in(n,0), out(n,0);
    for (auto &e : edges) { out[e.first]++; in[e.second]++; }
    int s=-1, t=-1, diff=0;
    for (int i=0;i<n;i++){
        if (out[i]-in[i]==1){ if(s==-1) s=i; diff++; }
        else if (in[i]-out[i]==1){ if(t==-1) t=i; diff++; }
        else if (in[i]!=out[i]) return 0;
    }
    if (diff==0) {
        // 全回路: 需连通, 起点任取非零度
        if (!connected_undirected(n, edges)) return 0;
        for (int i=0;i<n;i++) if(out[i]>0){ start=i; break; }
        return 1;
    }
    if (diff==2 && s!=-1 && t!=-1) {
        if (!connected_undirected(n, edges)) return 0;
        start = s;
        return 2;
    }
    return 0;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    mt19937 rng(12345);

    int total_tests = 0, passed = 0;
    auto report = [&](bool ok, const string &name) {
        total_tests++;
        if (ok) { passed++; cout << "  [PASS] " << name << "\n"; }
        else    { cout << "  [FAIL] " << name << "\n"; }
    };

    cout << "========================================================\n";
    cout << " Eulerian Path & Circuit — Hierholzer's Algorithm\n";
    cout << "========================================================\n\n";

    // ---- 1. 简单有向欧拉回路 ----
    {
        cout << "Test 1: 有向欧拉回路 (简单三角形)\n";
        int n = 3;
        vector<pair<int,int>> edges = {{0,1},{1,2},{2,0}};
        int start; int cls = classify_directed(n, edges, start);
        report(cls==1, "三类判定正确 (回路)");
        vector<int> eo; auto path = hierholzer_directed(n, edges, start, eo);
        report((int)path.size()==(int)edges.size()+1, "路径顶点数 == 边数+1");
        report((int)eo.size()==(int)edges.size(), "边序列长度 == 边数");
        report(path.front()==path.back(), "回路起点==终点");
        // 边集合一致性
        bool edge_ok=true;
        for (int i=0;i<(int)eo.size();i++){ auto &e=edges[eo[i]]; if(e.first!=path[i]||e.second!=path[i+1]) edge_ok=false; }
        report(edge_ok, "路径逐边与输入边一致");
        cout << "\n";
    }

    // ---- 2. 有向欧拉路径 (非回路) ----
    {
        cout << "Test 2: 有向欧拉路径 (1-2-3-1-4)\n";
        int n = 5;
        vector<pair<int,int>> edges = {{0,1},{1,2},{2,0},{0,3},{3,4}};
        int start; int cls = classify_directed(n, edges, start);
        report(cls==2, "三类判定正确 (路径)");
        report(start==0, "起点=出度比入度多1的顶点 (0)");
        vector<int> eo; auto path = hierholzer_directed(n, edges, start, eo);
        report((int)eo.size()==(int)edges.size(), "边序列长度 == 边数");
        report(path.front()==0 && path.back()==4, "路径起点=0, 终点=4");
        // 边 multiset 一致性
        multiset<pair<int,int>> orig(edges.begin(), edges.end());
        multiset<pair<int,int>> got;
        for (int i=0;i<(int)eo.size();i++) got.insert(edges[eo[i]]);
        report(orig==got, "边集合(含重数)完全一致");
        cout << "\n";
    }

    // ---- 3. 无欧拉路径的图 ----
    {
        cout << "Test 3: 无欧拉路径的图 (两个出度>入度顶点)\n";
        int n = 4;
        vector<pair<int,int>> edges = {{0,1},{0,2},{1,3},{2,3}};
        int start; int cls = classify_directed(n, edges, start);
        report(cls==0, "正确识别无欧拉路径");
        cout << "\n";
    }

    // ---- 4. 重边 + 自环 (构造真正的欧拉回路: 每个点 in==out) ----
    {
        cout << "Test 4: 重边+自环 欧拉回路\n";
        int n = 3;
        // degree: 0: out(->1 x2)=2, in(2->0)=1  => 需补 1->0
        // 构造平衡: 0->1 (x2), 1->0 (x1), 1->2, 2->0, 1->1 (自环, in=out 各+1)
        // 节点0: out=2, in=2 (来自1->0 和 2->0)
        // 节点1: out=1(1->2)+1(1->0)+1(自环)=3, in=2(from 0 x2)+1(自环)=3
        // 节点2: out=1(2->0), in=1(from 1->2)
        vector<pair<int,int>> edges = {{0,1},{0,1},{1,0},{1,2},{2,0},{1,1}};
        int start; int cls = classify_directed(n, edges, start);
        report(cls==1, "重边/自环下判定为回路");
        vector<int> eo; auto path = hierholzer_directed(n, edges, start, eo);
        multiset<pair<int,int>> orig(edges.begin(), edges.end());
        multiset<pair<int,int>> got;
        for (int i=0;i<(int)eo.size();i++) got.insert(edges[eo[i]]);
        report(orig==got, "边集合完全一致(含重边自环)");
        report(path.front()==path.back(), "回路闭合");
        report((int)eo.size()==(int)edges.size(), "边数正确");
        cout << "\n";
    }

    // ---- 5. 无向图欧拉回路 ----
    {
        cout << "Test 5: 无向图欧拉回路 (正方形+对角线之一)\n";
        int n = 4;
        vector<pair<int,int>> edges = {{0,1},{1,2},{2,3},{3,0},{0,2}};
        // 度数: 0->3(奇数!) 1->2 2->3(奇数) 3->2 => 有两个奇度, 应为欧拉路径
        // 改成: 无边对角线, 纯正方形 => 全偶数度
        edges = {{0,1},{1,2},{2,3},{3,0}};
        // 所有度数=2, 欧拉回路
        vector<int> eo; auto path = hierholzer_undirected(n, edges, 0, eo);
        report((int)eo.size()==4, "无向回路边数正确");
        report(path.front()==path.back(), "无向回路闭合");
        cout << "\n";
    }

    // ---- 6. 随机压力测试: 构造保证存在的欧拉回路 ----
    {
        cout << "Test 6: 随机有向欧拉图 200 组边集合+邻接一致性\n";
        bool all_ok = true;
        int tested = 0;
        for (int it=0; it<200; ++it) {
            int n = 3 + rng()%20;
            vector<pair<int,int>> edges;
            // 生成若干环 (每个环保证每点 in==out 增量平衡)
            int ncyc = 1 + rng()%6;
            for (int c=0; c<ncyc; ++c) {
                int len = 3 + rng()%10;
                int first = rng()%n;
                int prev = first;
                for (int k=0; k<len; ++k) {
                    int to = rng()%n; // 保持在 [0,n) 内
                    edges.push_back({prev, to});
                    prev = to;
                }
                edges.push_back({prev, first}); // 闭合, 保证平衡
            }
            int start; int cls = classify_directed(n, edges, start);
            if (cls==0) continue;
            tested++;
            vector<int> eo; auto path = hierholzer_directed(n, edges, start, eo);
            if ((int)eo.size()!=(int)edges.size()) { all_ok=false; break; }
            multiset<pair<int,int>> orig(edges.begin(), edges.end());
            multiset<pair<int,int>> got;
            for (int i=0;i<(int)eo.size();i++) {
                got.insert(edges[eo[i]]);
                if (edges[eo[i]].first != path[i] || edges[eo[i]].second != path[i+1]) { all_ok=false; }
            }
            if (orig != got) { all_ok=false; break; }
        }
        report(all_ok && tested>0, "200组随机欧拉图: 边集合+邻接完全一致 (实测"+to_string(tested)+"组)");
        cout << "\n";
    }

    // ---- 7. 性能对比: Hierholzer O(E) vs Fleury O(E^2) ----
    {
        cout << "Test 7: 性能对比 (Hierholzer vs Fleury)\n";
        // 构造多个重叠环形成欧拉回路, 使 Fleury 的桥检测真正触发
        int N = 1500; // 点数
        vector<pair<int,int>> de;
        for (int i=0;i<N;i++) de.push_back({i, (i+1)%N}); // 大环
        // 加入若干小环共享顶点, 增加非桥边
        for (int c=0; c<300; ++c) {
            int a = (c*7)%N;
            de.push_back({a, (a+1)%N});
            de.push_back({(a+1)%N, a}); // 来回边保持平衡
        }
        int st; classify_directed(N, de, st);

        // Hierholzer
        auto t0 = chrono::high_resolution_clock::now();
        vector<int> eo1; auto p1 = hierholzer_directed(N, de, st, eo1);
        auto t1 = chrono::high_resolution_clock::now();
        double th = chrono::duration<double>(t1-t0).count();

        // Fleury (真实 O(E^2) 桥检测)
        auto t2 = chrono::high_resolution_clock::now();
        vector<int> eo2; auto p2 = fleury_directed(N, de, st, eo2);
        auto t3 = chrono::high_resolution_clock::now();
        double tf = chrono::duration<double>(t3-t2).count();

        printf("  E=%d   Hierholzer=%.4fs   Fleury=%.4fs   加速比=%.1fx\n",
               (int)de.size(), th, tf, (th>0? tf/th : 0));
        report((int)eo1.size()==(int)de.size(), "Hierholzer 边数正确");
        report((int)eo2.size()==(int)de.size(), "Fleury 边数正确");
        // 两者边集合应一致
        multiset<pair<int,int>> s1, s2;
        for (int e : eo1) s1.insert(de[e]);
        for (int e : eo2) s2.insert(de[e]);
        report(s1==s2, "Hierholzer 与 Fleury 边集合一致");
        report(th < tf, "Hierholzer 快于 Fleury (O(E) vs O(E^2))");
        cout << "\n";
    }

    // ---- 8. 理论判定 vs 手工构造 (混合) ----
    {
        cout << "Test 8: 大随机欧拉回路构造正确性\n";
        // 随机生成 k 个点的完整环链拼接, 验证 hierholzer 边数恒 = E
        bool ok = true;
        for (int it=0; it<50; it++) {
            int n = 5 + rng()%20;
            vector<pair<int,int>> edges;
            // 生成一个欧拉回路: 随机排列上的多个环
            vector<int> perm(n); iota(perm.begin(), perm.end(), 0);
            shuffle(perm.begin(), perm.end(), rng);
            for (int i=0;i<n;i++) edges.push_back({perm[i], perm[(i+1)%n]}); // 一个哈密顿环 = 欧拉回路
            int start; int cls = classify_directed(n, edges, start);
            if (cls != 1) { ok=false; break; }
            vector<int> eo; auto path = hierholzer_directed(n, edges, start, eo);
            if ((int)eo.size() != (int)edges.size()) { ok=false; break; }
            if (path.front() != path.back()) { ok=false; break; }
        }
        report(ok, "50组随机哈密顿环: 全为欧拉回路且构造正确");
        cout << "\n";
    }

    cout << "========================================================\n";
    cout << " 结果: " << passed << "/" << total_tests << " 通过\n";
    cout << "========================================================\n";
    return (passed==total_tests)?0:1;
}
