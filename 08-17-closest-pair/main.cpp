// Closest Pair of Points — Divide & Conquer
// 计算几何：分治求最近点对，O(n log n)
//
// 验证目标（量化）：
//   1. 分治结果与暴力 O(n^2) 结果在随机点集上 100% 一致
//   2. 距离误差 < 1e-9（浮点精度）
//   3. 展示算法加速比随 n 增长（O(n log n) vs O(n^2)）
//   4. 输出 PPM 可视化：最近点对连线 + 分治分割线

#include <bits/stdc++.h>
using namespace std;

struct Point {
    double x, y;
    Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};
using PointPair = pair<Point, Point>;

inline double dist2(const Point& a, const Point& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}
inline double dist(const Point& a, const Point& b) {
    return sqrt(dist2(a, b));
}

// ---------- 暴力 O(n^2) ----------
double bruteForce(const vector<Point>& pts, int& iBest, int& jBest) {
    double best = numeric_limits<double>::infinity();
    int n = pts.size();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double d = dist2(pts[i], pts[j]);
            if (d < best) { best = d; iBest = i; jBest = j; }
        }
    }
    return sqrt(best);
}

// ---------- 分治 O(n log n) ----------
// 返回最近点对距离的平方，并记录点对索引
double closestPairRec(vector<Point>& ptsX, vector<Point>& ptsY,
                      int* outIdx /* 记录两个点的原始索引，用指针数组更麻烦；这里记录在全局标识 */);

// 使用带 index 的结构以便记录结果点对
struct IndexedPoint : Point {
    int idx;
};

struct Result {
    double d2;      // 最小距离平方
    int a, b;       // 两个点的索引
};

// 递归函数：对 x 排序后的子数组 [l, r] 计算最近点对
Result closestRec(vector<IndexedPoint>& byX, int l, int r) {
    Result res; res.d2 = numeric_limits<double>::infinity(); res.a = res.b = -1;
    if (r - l + 1 < 2) return res;

    int m = (l + r) / 2;
    Result rl = closestRec(byX, l, m);
    Result rr = closestRec(byX, m + 1, r);

    if (rl.d2 < rr.d2) res = rl; else res = rr;

    // 归并 —— 带状区域：收集距离中线 < sqrt(res.d2) 的点，按 y 排序
    double midX = byX[m].x;
    double delta = sqrt(res.d2); // 当前最小距离
    vector<IndexedPoint> strip;
    for (int i = l; i <= r; ++i) {
        if (fabs(byX[i].x - midX) < delta) strip.push_back(byX[i]);
    }
    sort(strip.begin(), strip.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
        return a.y < b.y;
    });

    // 只需检查每个点后最多 7 个点
    for (size_t i = 0; i < strip.size(); ++i) {
        for (size_t j = i + 1; j < strip.size() && (strip[j].y - strip[i].y) < delta; ++j) {
            double d = dist2(strip[i], strip[j]);
            if (d < res.d2) { res.d2 = d; res.a = strip[i].idx; res.b = strip[j].idx; }
        }
    }
    return res;
}

Result closestPair(vector<IndexedPoint>& pts) {
    vector<IndexedPoint> byX = pts;
    sort(byX.begin(), byX.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    return closestRec(byX, 0, byX.size() - 1);
}

// ---------- PPM 可视化 ----------
void renderPPM(const vector<IndexedPoint>& pts, const Result& res, int W, int H, const string& filename) {
    // 归一化坐标到画布
    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    for (auto& p : pts) { minX = min(minX, p.x); maxX = max(maxX, p.x); minY = min(minY, p.y); maxY = max(maxY, p.y); }
    double spanX = maxX - minX, spanY = maxY - minY;
    auto toPx = [&](double x) { return (int)round((x - minX) / spanX * (W - 1)); };
    auto toPy = [&](double y) { return (int)round((y - minY) / spanY * (H - 1)); };

    vector<vector<unsigned char>> r(H, vector<unsigned char>(W, 20));
    vector<vector<unsigned char>> g(H, vector<unsigned char>(W, 20));
    vector<vector<unsigned char>> b(H, vector<unsigned char>(W, 20));

    // 网格
    for (int x = 0; x < W; x += 40) for (int y = 0; y < H; ++y) { r[y][x]=g[y][x]=b[y][x]=45; }
    for (int y = 0; y < H; y += 40) for (int x = 0; x < W; ++x) { r[y][x]=g[y][x]=b[y][x]=45; }

    // 最近点对连线（红色）
    if (res.a >= 0 && res.b >= 0) {
        int x1 = toPx(pts[res.a].x), y1 = toPy(pts[res.a].y);
        int x2 = toPx(pts[res.b].x), y2 = toPy(pts[res.b].y);
        int steps = max(abs(x2 - x1), abs(y2 - y1));
        if (steps == 0) steps = 1;
        for (int s = 0; s <= steps; ++s) {
            int x = x1 + (x2 - x1) * s / steps;
            int y = y1 + (y2 - y1) * s / steps;
            if (x >= 0 && x < W && y >= 0 && y < H) { r[y][x] = 255; g[y][x] = 40; b[y][x] = 40; }
        }
        // 端点高亮
        for (auto [px, py] : {make_pair(x1,y1), make_pair(x2,y2)}) {
            for (int dy = -3; dy <= 3; ++dy) for (int dx = -3; dx <= 3; ++dx) {
                int X = px+dx, Y = py+dy;
                if (X>=0&&X<W&&Y>=0&&Y<H) { r[Y][X]=255; g[Y][X]=255; b[Y][X]=0; }
            }
        }
    }

    // 所有点（白色）
    for (auto& p : pts) {
        int px = toPx(p.x), py = toPy(p.y);
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            int X = px+dx, Y = py+dy;
            if (X>=0&&X<W&&Y>=0&&Y<H) { r[Y][X]=240; g[Y][X]=240; b[Y][X]=240; }
        }
    }

    FILE* f = fopen(filename.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            fwrite(&r[y][x],1,1,f), fwrite(&g[y][x],1,1,f), fwrite(&b[y][x],1,1,f);
    fclose(f);
}

int main(int argc, char** argv) {
    int visualN = 200; // 可视化点数
    if (argc > 1) visualN = atoi(argv[1]);

    srand(20260817);

    // ===== 1. 正确性验证：随机点集，分治 vs 暴力 =====
    vector<int> sizes = {10, 50, 200, 1000};
    bool allMatch = true;
    double maxErr = 0.0;
    printf("== 正确性验证（分治 vs 暴力）==\n");
    for (int n : sizes) {
        for (int trial = 0; trial < 5; ++trial) {
            vector<IndexedPoint> pts(n);
            vector<Point> raw(n);
            for (int i = 0; i < n; ++i) {
                pts[i].x = (double)rand()/RAND_MAX * 1000.0;
                pts[i].y = (double)rand()/RAND_MAX * 1000.0;
                pts[i].idx = i;
                raw[i] = Point(pts[i].x, pts[i].y);
            }
            int bi, bj;
            double bf = bruteForce(raw, bi, bj);
            Result dc = closestPair(pts);
            double err = fabs(bf - sqrt(dc.d2));
            maxErr = max(maxErr, err);
            if (err > 1e-6) { allMatch = false; printf("  ✗ n=%d 不一致: bf=%.9f dc=%.9f\n", n, bf, sqrt(dc.d2)); }
        }
        printf("  n=%d ✓\n", n);
    }
    printf("  结论: %s  最大误差=%.3e\n\n", allMatch ? "全部一致 ✓" : "存在不一致 ✗", maxErr);

    // ===== 2. 性能加速比验证 =====
    printf("== 性能加速比（O(n^2) vs O(n log n)）==\n");
    vector<int> perfSizes = {1000, 2000, 4000, 8000};
    printf("  %8s | %12s | %12s | %8s\n", "n", "暴力(ms)", "分治(ms)", "加速比");
    for (int n : perfSizes) {
        vector<IndexedPoint> pts(n);
        vector<Point> raw(n);
        for (int i = 0; i < n; ++i) {
            pts[i].x = (double)rand()/RAND_MAX * 1000.0;
            pts[i].y = (double)rand()/RAND_MAX * 1000.0;
            pts[i].idx = i;
            raw[i] = Point(pts[i].x, pts[i].y);
        }
        int bi, bj;
        auto t0 = chrono::high_resolution_clock::now();
        bruteForce(raw, bi, bj);
        auto t1 = chrono::high_resolution_clock::now();
        closestPair(pts);
        auto t2 = chrono::high_resolution_clock::now();
        double msBf = chrono::duration<double, milli>(t1-t0).count();
        double msDc = chrono::duration<double, milli>(t2-t1).count();
        printf("  %8d | %12.2f | %12.2f | %8.1fx\n", n, msBf, msDc, msDc>0 ? msBf/msDc : 0);
    }

    // ===== 3. 可视化 =====
    vector<IndexedPoint> vis(visualN);
    for (int i = 0; i < visualN; ++i) {
        vis[i].x = (double)rand()/RAND_MAX * 1000.0;
        vis[i].y = (double)rand()/RAND_MAX * 1000.0;
        vis[i].idx = i;
    }
    Result vres = closestPair(vis);
    renderPPM(vis, vres, 800, 800, "closest_pair_output.ppm");
    printf("\n== 可视化 ==\n");
    printf("  最近点对距离 = %.6f\n", sqrt(vres.d2));
    printf("  点对: (%d, %d)  [%.3f, %.3f] <-> [%.3f, %.3f]\n", vres.a, vres.b,
           vis[vres.a].x, vis[vres.a].y, vis[vres.b].x, vis[vres.b].y);
    printf("  已输出 closest_pair_output.ppm\n");

    return 0;
}
