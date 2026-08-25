// Topological Sort: Kahn's Algorithm + DFS Post-Order, with cycle detection.
// Quantitatively validates:
//   1) Ordering is a valid topological order (all edges u->v satisfy pos[u] < pos[v])
//   2) Kahn == DFS result length and validity
//   3) Cycle detection correctness on graphs with/without cycles
//   4) Randomized DAG stress test (many trials)

#include <bits/stdc++.h>
using namespace std;

// ---------- Kahn's Algorithm (BFS, in-degree based) ----------
// Returns topological order; empty vector if graph has a cycle.
vector<int> kahn(int n, const vector<pair<int,int>>& edges) {
    vector<vector<int>> adj(n);
    vector<int> indeg(n, 0);
    for (auto& e : edges) {
        adj[e.first].push_back(e.second);
        indeg[e.second]++;
    }
    queue<int> q;
    for (int i = 0; i < n; i++) if (indeg[i] == 0) q.push(i);
    vector<int> order;
    order.reserve(n);
    while (!q.empty()) {
        int u = q.front(); q.pop();
        order.push_back(u);
        for (int v : adj[u]) {
            if (--indeg[v] == 0) q.push(v);
        }
    }
    if ((int)order.size() != n) return {}; // cycle detected
    return order;
}

// ---------- DFS Post-Order ----------
// Returns reverse-postorder topological order; empty if cycle.
vector<int> dfs_topo(int n, const vector<pair<int,int>>& edges) {
    vector<vector<int>> adj(n);
    for (auto& e : edges) adj[e.first].push_back(e.second);
    vector<int> color(n, 0); // 0=white 1=gray 2=black
    vector<int> order;
    bool hasCycle = false;
    function<void(int)> dfs = [&](int u) {
        color[u] = 1;
        for (int v : adj[u]) {
            if (color[v] == 1) hasCycle = true;        // back edge -> cycle
            else if (color[v] == 0) dfs(v);
        }
        color[u] = 2;
        order.push_back(u);
    };
    for (int i = 0; i < n; i++) if (color[i] == 0) dfs(i);
    if (hasCycle) return {};
    reverse(order.begin(), order.end());
    return order;
}

// ---------- Validation helpers ----------
// Check that order is a valid topological ordering of edges.
bool isValidTopo(const vector<int>& order, const vector<pair<int,int>>& edges) {
    if (order.empty()) return false;
    unordered_map<int,int> pos;
    for (int i = 0; i < (int)order.size(); i++) pos[order[i]] = i;
    if ((int)pos.size() != (int)order.size()) return false; // duplicates
    for (auto& e : edges) {
        if (pos[e.first] >= pos[e.second]) return false;
    }
    return true;
}

bool hasCycle(int n, const vector<pair<int,int>>& edges) {
    return kahn(n, edges).empty();
}

// Generate a random DAG: edges only point from lower index to higher index.
vector<pair<int,int>> randomDAG(mt19937& rng, int n, int maxEdges) {
    vector<pair<int,int>> edges;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if ((int)edges.size() >= maxEdges) break;
            if (rng() % 100 < 25) edges.push_back({i, j});
        }
        if ((int)edges.size() >= maxEdges) break;
    }
    return edges;
}

int main() {
    // ---------- Fixed test cases ----------
    struct Case { string name; int n; vector<pair<int,int>> edges; bool expectCycle; };
    vector<Case> cases = {
        {"chain",        5, {{0,1},{1,2},{2,3},{3,4}}, false},
        {"diamond",      6, {{0,1},{0,2},{1,3},{2,3},{3,4},{3,5}}, false},
        {"course-sched", 6, {{1,0},{2,0},{3,1},{3,2}}, false}, // classic 207
        {"disconnected", 6, {{0,1},{2,3},{4,5}}, false},
        {"self-loop",    3, {{0,1},{1,2},{1,1}}, true},
        {"triangle",     3, {{0,1},{1,2},{2,0}}, true},
        {"two-cycle",    4, {{0,1},{1,0},{2,3}}, true},
    };

    int pass = 0, totalChecks = 0;

    for (auto& c : cases) {
        auto k = kahn(c.n, c.edges);
        auto d = dfs_topo(c.n, c.edges);
        bool kCyc = k.empty();
        bool dCyc = d.empty();

        totalChecks++;
        if (kCyc == c.expectCycle) pass++;
        totalChecks++;
        if (dCyc == c.expectCycle) pass++;

        if (!c.expectCycle) {
            totalChecks++;
            if (isValidTopo(k, c.edges)) pass++;
            totalChecks++;
            if (isValidTopo(d, c.edges)) pass++;
            totalChecks++;
            if (k.size() == (size_t)c.n && d.size() == (size_t)c.n) pass++;
        }
        printf("[%s] kahn_cyc=%d dfs_cyc=%d expected_cyc=%d  kahn_len=%zu dfs_len=%zu\n",
               c.name.c_str(), kCyc, dCyc, c.expectCycle, k.size(), d.size());
    }

    // ---------- Random DAG stress test ----------
    mt19937 rng(12345);
    int trials = 2000;
    int dPass = 0;
    for (int t = 0; t < trials; t++) {
        int n = 2 + (int)(rng() % 30);
        auto edges = randomDAG(rng, n, 200);
        auto k = kahn(n, edges);
        auto d = dfs_topo(n, edges);
        if (!k.empty() && !d.empty() &&
            isValidTopo(k, edges) && isValidTopo(d, edges) &&
            k.size() == (size_t)n && d.size() == (size_t)n) {
            dPass++;
        }
    }
    printf("\nRandomDAG stress: %d/%d trials produced valid topo orders\n", dPass, trials);
    bool stressOK = (dPass == trials);
    // Random graph with a guaranteed cycle -> must be detected.
    // Build a DAG, pick a pair i<j that has a path i->j (guaranteed reachability),
    // then add the back edge j->i to create a real cycle.
    int cycDetected = 0, cycTrials = 500;
    for (int t = 0; t < cycTrials; t++) {
        int n = 3 + (int)(rng() % 30);
        auto edges = randomDAG(rng, n, 200);
        // Build reachability adjacency from the DAG edges (i<j by construction).
        vector<vector<bool>> reach(n, vector<bool>(n, false));
        for (auto& e : edges) reach[e.first][e.second] = true;
        for (int k = 0; k < n; k++)
            for (int i = 0; i < n; i++)
                if (reach[i][k])
                    for (int j = 0; j < n; j++)
                        if (reach[k][j]) reach[i][j] = true;
        // Find i<j with reach[i][j], add back edge j->i => guaranteed cycle.
        int bi = -1, bj = -1;
        for (int i = 0; i < n && bi < 0; i++)
            for (int j = i + 1; j < n; j++)
                if (reach[i][j]) { bi = i; bj = j; break; }
        if (bi < 0) { // no path — force a guaranteed 2-node cycle
            edges.push_back({n-1, n-2});
            edges.push_back({n-2, n-1});
        } else {
            edges.push_back({bj, bi});
        }
        if (hasCycle(n, edges)) cycDetected++;
    }
    printf("Cycle detection: %d/%d planted-cycle graphs correctly flagged\n", cycDetected, cycTrials);
    printf("\nTOTAL CHECKS: %d, PASSED: %d\n", totalChecks, pass);
    printf("stressOK=%d cycOK=%d\n", stressOK ? 1 : 0, cycDetected == cycTrials ? 1 : 0);

    bool allOK = (pass == totalChecks) && stressOK && (cycDetected == cycTrials);
    printf("\nRESULT: %s\n", allOK ? "ALL_PASS" : "FAIL");
    return allOK ? 0 : 1;
}
