// Floyd-Warshall All-Pairs Shortest Path (APSP)
// Quantitative verification:
//   1. Correctness vs repeated Dijkstra (ground truth) on random graphs
//   2. Negative-weight support (FW handles negative edges, Dijkstra can't)
//   3. Negative-cycle detection
//   4. Path reconstruction + optimality check
#include <bits/stdc++.h>
using namespace std;

const long long INF = (long long)1e18;

struct FloydWarshall {
    int n;
    vector<vector<long long>> dist;
    vector<vector<int>> nxt; // next node for path reconstruction
    bool hasNegativeCycle = false;

    FloydWarshall(int n_) : n(n_) {
        dist.assign(n, vector<long long>(n, INF));
        nxt.assign(n, vector<int>(n, -1));
        for (int i = 0; i < n; i++) { dist[i][i] = 0; nxt[i][i] = i; }
    }

    void addEdge(int u, int v, long long w) {
        if (w < dist[u][v]) {
            dist[u][v] = w;
            nxt[u][v] = v;
        }
    }

    void run() {
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                if (dist[i][k] == INF) continue;
                for (int j = 0; j < n; j++) {
                    if (dist[k][j] == INF) continue;
                    if (dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        nxt[i][j] = nxt[i][k];
                    }
                }
            }
        }
        // negative cycle check
        for (int i = 0; i < n; i++)
            if (dist[i][i] < 0) { hasNegativeCycle = true; return; }
    }

    // reconstruct path from u to v (empty if unreachable)
    vector<int> path(int u, int v) const {
        if (dist[u][v] == INF) return {};
        vector<int> p;
        int cur = u;
        while (cur != v) {
            if (cur == -1) return {};
            p.push_back(cur);
            cur = nxt[cur][v];
        }
        p.push_back(v);
        return p;
    }
};

// Dijkstra from single source (no negative edges) for ground truth
vector<long long> dijkstra(const vector<vector<pair<int,long long>>>& adj, int s) {
    int n = adj.size();
    vector<long long> d(n, INF);
    priority_queue<pair<long long,int>, vector<pair<long long,int>>, greater<>> pq;
    d[s] = 0; pq.push({0, s});
    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du != d[u]) continue;
        for (auto [v, w] : adj[u]) {
            if (d[u] + w < d[v]) { d[v] = d[u] + w; pq.push({d[v], v}); }
        }
    }
    return d;
}

int main() {
    mt19937 rng(12345);

    // ============ TEST 1: Correctness vs Dijkstra (non-negative graphs) ============
    int matchCount = 0, mismatchCount = 0;
    long long maxAbsErr = 0;
    for (int trial = 0; trial < 50; trial++) {
        int n = 20 + rng() % 30;
        int m = n * 2 + rng() % (n * 4);
        FloydWarshall fw(n);
        vector<vector<pair<int,long long>>> adj(n);
        for (int e = 0; e < m; e++) {
            int u = rng() % n, v = rng() % n;
            long long w = 1 + rng() % 100;
            fw.addEdge(u, v, w);
            adj[u].push_back({v, w});
        }
        fw.run();
        bool ok = true;
        for (int s = 0; s < n && ok; s++) {
            auto dj = dijkstra(adj, s);
            for (int t = 0; t < n; t++) {
                long long a = fw.dist[s][t];
                long long b = dj[t];
                if (a != b) { ok = false; mismatchCount++; maxAbsErr = max(maxAbsErr, llabs(a-b)); break; }
                else matchCount++;
            }
        }
    }
    cout << "[Test1] FW vs Dijkstra: matched=" << matchCount
         << " mismatched=" << mismatchCount << " maxErr=" << maxAbsErr << "\n";

    // ============ TEST 2: Negative edges (FW only) + path optimality ============
    int optOk = 0, optFail = 0;
    for (int trial = 0; trial < 50; trial++) {
        int n = 10 + rng() % 15;
        vector<tuple<int,int,long long>> edges;
        FloydWarshall fw(n);
        for (int e = 0; e < n * 3; e++) {
            int u = rng() % n, v = rng() % n;
            long long w = (long long)(rng() % 200) - 50; // -50 .. 149
            edges.push_back({u, v, w});
            fw.addEdge(u, v, w);
        }
        fw.run();
        if (fw.hasNegativeCycle) continue;
        // verify each reconstructed path's edge-sum equals dist (when reachable & finite small)
        bool allOk = true;
        for (int s = 0; s < n && allOk; s++) {
            for (int t = 0; t < n; t++) {
                if (fw.dist[s][t] == INF) continue;
                auto p = fw.path(s, t);
                if (p.empty()) { allOk = false; break; }
                long long sum = 0; bool valid = true;
                for (size_t i = 0; i + 1 < p.size(); i++) {
                    long long w = INF;
                    for (auto [a,b,ww] : edges) if (a==p[i] && b==p[i+1]) w = min(w, ww);
                    if (w == INF) { valid = false; break; }
                    sum += w;
                }
                if (!valid || sum != fw.dist[s][t]) { allOk = false; break; }
            }
        }
        if (allOk) optOk++; else optFail++;
    }
    cout << "[Test2] Path reconstruction optimality: ok=" << optOk << " fail=" << optFail << "\n";

    // ============ TEST 3: Negative cycle detection ============
    {
        int n = 5;
        FloydWarshall fw(n);
        fw.addEdge(0,1,1); fw.addEdge(1,2,1); fw.addEdge(2,0,-3); // cycle sum -1
        fw.run();
        cout << "[Test3] Negative cycle detected: " << (fw.hasNegativeCycle ? "YES" : "NO") << "\n";
    }
    {
        int n = 5;
        FloydWarshall fw(n);
        fw.addEdge(0,1,1); fw.addEdge(1,2,1); fw.addEdge(2,0,-2); // cycle sum 0 (not negative)
        fw.run();
        cout << "[Test3b] Zero-sum cycle => negative-cycle=" << (fw.hasNegativeCycle ? "YES(WRONG)" : "NO(correct)") << "\n";
    }

    // ============ TEST 4: Time complexity benchmark O(n^3) ============
    for (int n : {100, 200, 400}) {
        vector<vector<int>> g(n, vector<int>(n, (int)1e9));
        for (int i = 0; i < n; i++) g[i][i] = 0;
        for (int e = 0; e < n * 5; e++) {
            int u = rng() % n, v = rng() % n;
            g[u][v] = min(g[u][v], 1 + (int)(rng() % 100));
        }
        auto t0 = chrono::high_resolution_clock::now();
        long long checksum = 0;
        for (int k = 0; k < n; k++)
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    if (g[i][k] + g[k][j] < g[i][j]) g[i][j] = g[i][k] + g[k][j];
        for (int i = 0; i < n; i++) checksum += g[i][i%n] + g[(i*7)%n][i];
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        cout << "[Test4] n=" << n << " time=" << ms << "ms checksum=" << checksum << "\n";
    }

    // ============ Visualization: graph + shortest path + benchmark ============
    // Renders a PPM image: left panel = demo graph with shortest path highlighted,
    // right panel = O(n^3) time-complexity benchmark curve.
    {
        const int W = 1200, H = 600;
        vector<vector<int>> img(H, vector<int>(W * 3, 255)); // 255 = white  (RGB)
        auto setpx = [&](int x, int y, int r, int g, int b) {
            if (x < 0 || x >= W || y < 0 || y >= H) return;
            img[y][x*3+0] = r; img[y][x*3+1] = g; img[y][x*3+2] = b;
        };

        // ---- Left panel: demo graph (positive + negative edges), highlight path ----
        int N = 8;
        vector<pair<double,double>> pos(N);
        double cx = 300, cy = 300, R = 190;
        for (int i = 0; i < N; i++) {
            double a = 2*M_PI*i/N - M_PI/2;
            pos[i] = {cx + R*cos(a), cy + R*sin(a)};
        }
        // Build a demo graph with some negative edges, no negative cycle
        FloydWarshall demo(N);
        vector<tuple<int,int,long long>> demoEdges = {
            {0,1,4},{1,2,2},{2,3,6},{3,4,3},{4,5,1},{5,6,5},{6,7,2},{7,0,3},
            {0,2,9},{1,3,7},{2,6,-4},{3,7,-2},{4,0,8},{5,7,6},{0,5,-1}
        };
        for (auto [u,v,w] : demoEdges) demo.addEdge(u, v, w);
        demo.run();

        // draw edges
        for (auto [u,v,w] : demoEdges) {
            int x1 = pos[u].first, y1 = pos[u].second;
            int x2 = pos[v].first, y2 = pos[v].second;
            int colr = (w < 0) ? 200 : 130;
            int colg = (w < 0) ? 60  : 130;
            int colb = (w < 0) ? 60  : 130;
            // Bresenham line
            int dx = abs(x2-x1), sx = x1<x2?1:-1;
            int dy = -abs(y2-y1), sy = y1<y2?1:-1;
            int err = dx+dy, e2;
            int xx=x1, yy=y1;
            while (true) {
                setpx(xx, yy, colr, colg, colb);
                if (xx==x2 && yy==y2) break;
                e2 = 2*err;
                if (e2 >= dy) { err += dy; xx += sx; }
                if (e2 <= dx) { err += dx; yy += sy; }
            }
        }

        // shortest path from 0 to 7 (highlighted in red, thick)
        auto sp = demo.path(0, 7);
        for (size_t i = 0; i + 1 < sp.size(); i++) {
            int u = sp[i], v = sp[i+1];
            int x1 = pos[u].first, y1 = pos[u].second;
            int x2 = pos[v].first, y2 = pos[v].second;
            int dx = abs(x2-x1), sx = x1<x2?1:-1;
            int dy = -abs(y2-y1), sy = y1<y2?1:-1;
            int err = dx+dy, e2;
            int xx=x1, yy=y1;
            while (true) {
                setpx(xx-1, yy, 230,40,40); setpx(xx+1, yy, 230,40,40);
                setpx(xx, yy-1, 230,40,40); setpx(xx, yy+1, 230,40,40);
                setpx(xx, yy, 230,40,40);
                if (xx==x2 && yy==y2) break;
                e2 = 2*err;
                if (e2 >= dy) { err += dy; xx += sx; }
                if (e2 <= dx) { err += dx; yy += sy; }
            }
        }

        // draw nodes (filled circles) + labels
        for (int i = 0; i < N; i++) {
            int x = pos[i].first, y = pos[i].second;
            for (int dy = -12; dy <= 12; dy++)
                for (int dx = -12; dx <= 12; dx++)
                    if (dx*dx + dy*dy <= 144)
                        setpx(x+dx, y+dy, 40, 90, 220);
            setpx(x, y, 255, 255, 255);
        }

        // ---- Right panel: O(n^3) benchmark curve ----
        int bx = 650, bw = 520, bh = 520, by0 = 550;
        // axis
        for (int x = bx; x <= bx+bw; x++) setpx(x, by0, 60,60,60);
        for (int y = by0-bh; y <= by0; y++) setpx(bx, y, 60,60,60);

        vector<pair<int,double>> bench; // n -> ms
        mt19937 brng(999);
        for (int n : {50, 75, 100, 125, 150, 175, 200}) {
            vector<vector<int>> g(n, vector<int>(n, (int)1e9));
            for (int i = 0; i < n; i++) g[i][i] = 0;
            for (int e = 0; e < n*5; e++) {
                int u = brng()%n, v = brng()%n;
                g[u][v] = min(g[u][v], 1 + (int)(brng()%100));
            }
            auto t0 = chrono::high_resolution_clock::now();
            long long ck = 0;
            for (int k = 0; k < n; k++)
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (g[i][k] + g[k][j] < g[i][j]) g[i][j] = g[i][k] + g[k][j];
            for (int i = 0; i < n; i++) ck += g[i][i%n];
            auto t1 = chrono::high_resolution_clock::now();
            double ms = chrono::duration<double, milli>(t1 - t0).count();
            bench.push_back({n, ms});
        }
        double maxMs = 0; for (auto& p : bench) maxMs = max(maxMs, p.second);
        // fit cubic curve n^3 * k
        double kFit = 0; for (auto& p : bench) kFit += p.second / pow(p.first, 3.0);
        kFit /= bench.size();

        auto plot = [&](vector<pair<int,double>>& pts, int r, int g, int b) {
            for (auto& p : pts) {
                int x = bx + (int)((double)p.first / 200 * bw);
                int y = by0 - (int)(p.second / maxMs * bh);
                for (int dy=-3; dy<=3; dy++) for (int dx=-3; dx<=3; dx++)
                    if (dx*dx+dy*dy <= 4) setpx(x+dx, y+dy, r, g, b);
            }
            for (size_t i = 0; i + 1 < pts.size(); i++) {
                int x1 = bx + (int)((double)pts[i].first / 200 * bw);
                int y1 = by0 - (int)(pts[i].second / maxMs * bh);
                int x2 = bx + (int)((double)pts[i+1].first / 200 * bw);
                int y2 = by0 - (int)(pts[i+1].second / maxMs * bh);
                int dx = abs(x2-x1), sx = x1<x2?1:-1;
                int dy = -abs(y2-y1), sy = y1<y2?1:-1;
                int err = dx+dy, e2, xx=x1, yy=y1;
                while (true) { setpx(xx, yy, r, g, b);
                    if (xx==x2 && yy==y2) break;
                    e2=2*err;
                    if (e2>=dy){err+=dy;xx+=sx;} if (e2<=dx){err+=dx;yy+=sy;} }
            }
        };

        // cubic fit curve (scaled to maxMs)
        vector<pair<int,double>> fit;
        for (int n = 50; n <= 200; n += 10)
            fit.push_back({n, kFit * pow(n, 3.0)});
        plot(fit, 200, 200, 200);
        plot(bench, 220, 60, 40);

        // Write PPM
        ofstream f("floyd_warshall_output.ppm");
        f << "P3\n" << W << " " << H << "\n255\n";
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                f << img[y][x*3+0] << " " << img[y][x*3+1] << " " << img[y][x*3+2] << " ";
        f.close();
        cout << "[Visualization] wrote floyd_warshall_output.ppm (" << W << "x" << H << ")\n";
    }

    cout << "ALL_DONE\n";
    return 0;
}
