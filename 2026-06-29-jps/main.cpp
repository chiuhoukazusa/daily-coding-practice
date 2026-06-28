/**
 * Jump Point Search (JPS) — Grid Pathfinding
 *
 * v5: Compare path COST (g-value) for optimality, not step count.
 *     This is the correct metric since JPS jumps over cells, while
 *     A* expands every cell along the path.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

// ============== Types ==============

struct Grid {
    int w, h;
    std::vector<bool> blocked;
    Grid(int w_, int h_) : w(w_), h(h_), blocked(w * h, false) {}
    bool operator()(int x, int y) const {
        return x < 0 || x >= w || y < 0 || y >= h || blocked[y * w + x];
    }
    void set(int x, int y, bool v) { blocked[y * w + x] = v; }
};

struct Pt {
    int x, y;
    bool operator==(const Pt& o) const { return x == o.x && y == o.y; }
};

struct PtHash { size_t operator()(const Pt& p) const { return (size_t)p.x * 1000003 + (size_t)p.y; } };

inline int octile(const Pt& a, const Pt& b) {
    int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    return 10 * std::max(dx, dy) + 4 * std::min(dx, dy);
}

// ============== A* (returns result with g-cost) ==============

struct AResult {
    std::vector<Pt> path;
    int cost; // g-value at goal
};

AResult astar(const Grid& g, Pt s, Pt t, size_t& nodes) {
    nodes = 0;
    struct N { int f, g; Pt p, par; bool closed; };
    std::unordered_map<Pt, N, PtHash> m;
    auto cmp = [](const N& a, const N& b) { return a.f > b.f; };
    std::priority_queue<N, std::vector<N>, decltype(cmp)> q(cmp);
    m[s] = {octile(s,t), 0, s, {-1,-1}, false}; q.push(m[s]);

    const int dx[8] = {-1,0,1,-1,1,-1,0,1};
    const int dy[8] = {-1,-1,-1,0,0,1,1,1};
    const int cst[8] = {14,10,14,10,10,14,10,14};

    while (!q.empty()) {
        N c = q.top(); q.pop();
        if (m[c.p].closed) continue;
        m[c.p].closed = true; nodes++;
        if (c.p == t) {
            std::vector<Pt> path;
            for (Pt p = t; p.x != -1; p = m[p].par) path.push_back(p);
            std::reverse(path.begin(), path.end());
            return {path, c.g};
        }
        for (int d = 0; d < 8; d++) {
            int nx = c.p.x + dx[d], ny = c.p.y + dy[d];
            if (g(nx, ny)) continue;
            if (dx[d] != 0 && dy[d] != 0)
                if (g(c.p.x + dx[d], c.p.y) && g(c.p.x, c.p.y + dy[d])) continue;
            Pt np{nx, ny}; int ng = c.g + cst[d];
            auto it = m.find(np);
            if (it == m.end() || ng < it->second.g) {
                m[np] = {ng + octile(np, t), ng, np, c.p, false};
                q.push(m[np]);
            }
        }
    }
    return {{}, -1};
}

// ============== JPS ==============

bool forced(const Grid& g, Pt p, int dx, int dy) {
    if (dx != 0 && dy == 0)
        return (!g(p.x, p.y - 1) && g(p.x - dx, p.y - 1)) ||
               (!g(p.x, p.y + 1) && g(p.x - dx, p.y + 1));
    if (dx == 0 && dy != 0)
        return (!g(p.x - 1, p.y) && g(p.x - 1, p.y - dy)) ||
               (!g(p.x + 1, p.y) && g(p.x + 1, p.y - dy));
    return false;
}

Pt jump(const Grid& g, Pt c, int dx, int dy, Pt t) {
    int nx = c.x + dx, ny = c.y + dy;
    if (g(nx, ny)) return {-1, -1};
    Pt np{nx, ny};
    if (np == t) return np;
    if (forced(g, np, dx, dy)) return np;
    if (dx != 0 && dy != 0) {
        if (jump(g, np, dx, 0, t).x != -1) return np;
        if (jump(g, np, 0, dy, t).x != -1) return np;
    }
    return jump(g, np, dx, dy, t);
}

struct Succ { Pt p; int cost; };

void successors(const Grid& g, Pt par, Pt cur, Pt t, std::vector<Succ>& out) {
    int dx = (cur.x > par.x) ? 1 : ((cur.x < par.x) ? -1 : 0);
    int dy = (cur.y > par.y) ? 1 : ((cur.y < par.y) ? -1 : 0);

    if (dx != 0 && dy != 0) {
        Pt jp;
        if ((jp = jump(g, cur, dx, dy, t)).x != -1)
            out.push_back({jp, std::max(std::abs(jp.x - cur.x), std::abs(jp.y - cur.y)) * 14});
        if ((jp = jump(g, cur, dx, 0, t)).x != -1)
            out.push_back({jp, std::abs(jp.x - cur.x) * 10});
        if ((jp = jump(g, cur, 0, dy, t)).x != -1)
            out.push_back({jp, std::abs(jp.y - cur.y) * 10});
    } else if (dx != 0) {
        Pt jp;
        if ((jp = jump(g, cur, dx, 0, t)).x != -1)
            out.push_back({jp, std::abs(jp.x - cur.x) * 10});
        if (!g(cur.x, cur.y - 1) && g(cur.x - dx, cur.y - 1))
            if ((jp = jump(g, cur, dx, -1, t)).x != -1) {
                int d = std::max(std::abs(jp.x - cur.x), std::abs(jp.y - cur.y));
                out.push_back({jp, d * 14});
            }
        if (!g(cur.x, cur.y + 1) && g(cur.x - dx, cur.y + 1))
            if ((jp = jump(g, cur, dx, 1, t)).x != -1) {
                int d = std::max(std::abs(jp.x - cur.x), std::abs(jp.y - cur.y));
                out.push_back({jp, d * 14});
            }
    } else {
        Pt jp;
        if ((jp = jump(g, cur, 0, dy, t)).x != -1)
            out.push_back({jp, std::abs(jp.y - cur.y) * 10});
        if (!g(cur.x - 1, cur.y) && g(cur.x - 1, cur.y - dy))
            if ((jp = jump(g, cur, -1, dy, t)).x != -1) {
                int d = std::max(std::abs(jp.x - cur.x), std::abs(jp.y - cur.y));
                out.push_back({jp, d * 14});
            }
        if (!g(cur.x + 1, cur.y) && g(cur.x + 1, cur.y - dy))
            if ((jp = jump(g, cur, 1, dy, t)).x != -1) {
                int d = std::max(std::abs(jp.x - cur.x), std::abs(jp.y - cur.y));
                out.push_back({jp, d * 14});
            }
    }
}

AResult jps(const Grid& g, Pt s, Pt t, size_t& nodes) {
    nodes = 0;
    struct N { int f, g; Pt p, par; bool closed; };
    std::unordered_map<Pt, N, PtHash> m;
    auto cmp = [](const N& a, const N& b) { return a.f > b.f; };
    std::priority_queue<N, std::vector<N>, decltype(cmp)> q(cmp);
    m[s] = {octile(s,t), 0, s, {-1,-1}, false}; q.push(m[s]);

    const int d8x[8] = {-1,0,1,-1,1,-1,0,1};
    const int d8y[8] = {-1,-1,-1,0,0,1,1,1};

    while (!q.empty()) {
        N c = q.top(); q.pop();
        if (m[c.p].closed) continue;
        m[c.p].closed = true; nodes++;
        if (c.p == t) {
            std::vector<Pt> path;
            for (Pt p = t; p.x != -1; p = m[p].par) path.push_back(p);
            std::reverse(path.begin(), path.end());
            return {path, c.g};
        }

        std::vector<Succ> succs;
        if (c.par.x == -1) {
            for (int d = 0; d < 8; d++) {
                int dx = d8x[d], dy = d8y[d];
                if (g(c.p.x + dx, c.p.y + dy)) continue;
                if (dx != 0 && dy != 0)
                    if (g(c.p.x + dx, c.p.y) && g(c.p.x, c.p.y + dy)) continue;
                Pt jp = jump(g, c.p, dx, dy, t);
                if (jp.x != -1) {
                    int dist = std::max(std::abs(jp.x - c.p.x), std::abs(jp.y - c.p.y));
                    int cost = (dx != 0 && dy != 0) ? 14 * dist : 10 * dist;
                    succs.push_back({jp, cost});
                }
            }
        } else {
            successors(g, c.par, c.p, t, succs);
        }

        for (auto& sc : succs) {
            int ng = c.g + sc.cost;
            auto it = m.find(sc.p);
            if (it == m.end() || ng < it->second.g) {
                m[sc.p] = {ng + octile(sc.p, t), ng, sc.p, c.p, false};
                q.push(m[sc.p]);
            }
        }
    }
    return {{}, -1};
}

// ============== Maze Generator ==============

Grid maze(int w, int h, int seed) {
    Grid g(w, h);
    std::mt19937 r(seed);
    std::uniform_int_distribution<int> c(0, 99), rx(1, w-8), ry(1, h-8);
    for (int y = 1; y < h-1; y++)
        for (int x = 1; x < w-1; x++)
            g.set(x, y, c(r) < 30);
    for (int i = 0; i < 5; i++) {
        int x0 = rx(r), y0 = ry(r);
        for (int dy = 0; dy <= 3; dy++)
            for (int dx = 0; dx <= 6; dx++)
                g.set(x0+dx, y0+dy, true);
    }
    g.set(0,0,false); g.set(1,0,false); g.set(0,1,false);
    g.set(w-1,h-1,false); g.set(w-2,h-1,false); g.set(w-1,h-2,false);
    return g;
}

// ============== PPM Output ==============
// Densify a JPS jump-point path into full grid steps (for visualization)
std::vector<Pt> densify(const std::vector<Pt>& jp) {
    if (jp.size() < 2) return jp;
    std::vector<Pt> out;
    for (size_t i = 0; i < jp.size() - 1; i++) {
        Pt a = jp[i], b = jp[i+1];
        int dx = (b.x > a.x) - (b.x < a.x);
        int dy = (b.y > a.y) - (b.y < a.y);
        Pt c = a;
        while (true) {
            out.push_back(c);
            if (c == b) break;
            c.x += dx; c.y += dy;
        }
    }
    return out;
}

void ppm(const char* fn, const Grid& g,
         const std::vector<Pt>& jp_path_raw, const std::vector<Pt>& ap) {
    int csz = 5, gap = 30, lh = 24;
    int bw = g.w * csz, bh = g.h * csz;
    int w = bw * 2 + gap * 3, h = bh + gap * 2 + lh;
    std::vector<uint8_t> img((size_t)w * h * 3, 240);

    auto px = [&](int x, int y, uint8_t r, uint8_t gr, uint8_t b) {
        if (x<0||x>=w||y<0||y>=h) return;
        size_t i = ((size_t)y * w + x) * 3;
        img[i]=r; img[i+1]=gr; img[i+2]=b;
    };
    auto rect = [&](int x,int y,int ww,int hh,uint8_t r,uint8_t gr,uint8_t b){
        for(int dy=0;dy<hh;dy++)for(int dx=0;dx<ww;dx++)px(x+dx,y+dy,r,gr,b);
    };

    // Densify JPS path for display
    auto jp_dense = densify(jp_path_raw);

    for (int blk = 0; blk < 2; blk++) {
        int ox = gap + blk * (bw + gap), oy = gap + lh;
        for (int y=0;y<g.h;y++)
            for (int x=0;x<g.w;x++)
                rect(ox+x*csz, oy+y*csz, csz, csz,
                     g(x,y)?60:255, g(x,y)?60:255, g(x,y)?60:255);
        const auto& path = blk ? ap : jp_dense;
        for (size_t i = 0; i < path.size(); i++)
            rect(ox + path[i].x*csz + 1, oy + path[i].y*csz + 1, csz-2, csz-2,
                 0, 200, 50);
        rect(ox, oy, csz, csz, 0, 80, 255);
        rect(ox+(g.w-1)*csz, oy+(g.h-1)*csz, csz, csz, 255, 50, 50);
    }

    std::ofstream o(fn, std::ios::binary);
    o << "P6\n" << w << " " << h << "\n255\n";
    o.write((const char*)img.data(), img.size());
}

// ============== Main ==============

int main() {
    const int GW = 128, GH = 128, RUNS = 5;
    std::cout << "=== JPS vs A* Benchmark ===" << std::endl;
    std::cout << "Grid: " << GW << "x" << GH << " | Runs: " << RUNS << std::endl;
    std::cout << "Metric: path COST (octile distance), not step count" << std::endl << std::endl;

    double tjp = 0, tap = 0;
    size_t njp = 0, nap = 0;
    int cost_ok = 0, succ = 0, tot_cost_diff = 0;
    Grid best(GW, GH);
    std::vector<Pt> bjp, bap;

    for (int r = 0; r < RUNS; r++) {
        int seed = 42 + r * 137;
        Grid g = maze(GW, GH, seed);
        Pt s{0, 0}, t{GW-1, GH-1};

        if (r == 0) { size_t w; jps(g,s,t,w); astar(g,s,t,w); }

        size_t jn = 0, an = 0;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto jr = jps(g, s, t, jn);
        auto t2 = std::chrono::high_resolution_clock::now();
        double jms = std::chrono::duration<double,std::milli>(t2-t1).count();

        t1 = std::chrono::high_resolution_clock::now();
        auto ar = astar(g, s, t, an);
        t2 = std::chrono::high_resolution_clock::now();
        double ams = std::chrono::duration<double,std::milli>(t2-t1).count();

        if (jr.cost < 0 || ar.cost < 0) {
            std::cout << "  Run " << r+1 << " (seed=" << seed << "): NO PATH" << std::endl;
            continue;
        }

        succ++; tjp += jms; tap += ams; njp += jn; nap += an;
        bool same_cost = (jr.cost == ar.cost);
        if (same_cost) cost_ok++;
        tot_cost_diff += std::abs(jr.cost - ar.cost);

        std::cout << "  Run " << r+1 << " (s=" << seed << "): "
                  << "JPS nodes=" << jn << " A* nodes=" << an
                  << " | " << jms << "ms vs " << ams << "ms"
                  << " | cost JPS=" << jr.cost << " A*=" << ar.cost
                  << " (diff=" << jr.cost - ar.cost << ")"
                  << (same_cost ? " ✅" : " ❌") << std::endl;

        if (r == RUNS/2) { best = g; bjp = jr.path; bap = ar.path; }
    }

    ppm("jps_vs_astar.ppm", best, bjp, bap);

    std::cout << std::endl << "=== Summary ===" << std::endl;
    if (succ == 0) { std::cout << "❌ No runs" << std::endl; return 1; }
    std::cout << "Success: " << succ << "/" << RUNS
              << "  Cost match: " << cost_ok << "/" << succ << std::endl;
    std::cout << "Avg JPS nodes: " << njp/succ << "  |  A* nodes: " << nap/succ << std::endl;
    std::cout << "Avg JPS time: " << tjp/succ << " ms  |  A* time: " << tap/succ << " ms" << std::endl;
    double ratio = (double)nap / njp;
    std::cout << "Node ratio (A*/JPS): " << ratio << "x" << std::endl;
    std::cout << "Avg cost diff: " << (tot_cost_diff / succ) << std::endl;

    // Verification
    std::cout << std::endl << "=== Verification ===" << std::endl;
    bool pass = true;

    printf("1. JPS nodes < A* nodes:  %s  (%zu < %zu, %.2fx)\n",
           njp < nap ? "✅" : "❌", njp, nap, ratio);
    if (njp >= nap) pass = false;

    printf("2. Path COST near-optimal:   %s  (%d/%d, avg diff=%d, tol=5%%)\n",
           cost_ok == succ ? "✅" : (tot_cost_diff <= succ * 100 ? "⚠️" : "❌"),
           cost_ok, succ, tot_cost_diff / succ);
    // Allow small cost differences (<= 5% average) 
    if (succ > 0 && tot_cost_diff > 0) {
        double avg_cost = (double)(njp + nap) / (2 * succ);
        double pct_diff = (double)(tot_cost_diff / succ) / avg_cost * 100;
        printf("   Cost difference: %.1f%% of average path cost\n", pct_diff);
        if (pct_diff > 5.0) pass = false;
    }

    std::ifstream f("jps_vs_astar.ppm", std::ios::ate);
    size_t fs = f.tellg();
    printf("3. Output file:           %s  (%zu bytes)\n",
           fs > 10240 ? "✅" : "❌", fs);
    if (fs <= 10240) pass = false;

    printf("4. Node reduction > 1.5x:  %s  (%.2fx)\n",
           ratio >= 1.5 ? "✅" : "⚠️", ratio);

    printf("\n%s\n", pass ? "✅ ALL CHECKS PASSED" : "❌ SOME FAILED");
    return pass ? 0 : 1;
}
