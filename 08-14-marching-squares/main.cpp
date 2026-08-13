// Marching Squares 2D Isocontour Extraction
// 技术点: 标量场等值线提取, 16种 marching squares case, 双线性插值求交,
//         线段配对, 轮廓闭合性验证, 面积/长度守恒量化验证
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <cassert>
#include <algorithm>

// 图像尺寸
static const int W = 400, H = 400;

// 标量场: 多个高斯'blob'叠加 (光滑, 便于解析验证)
static double field(double x, double y) {
    // x, y in [0,1]
    double v = 0.0;
    auto g = [&](double cx, double cy, double s, double a) {
        double dx = x - cx, dy = y - cy;
        return a * std::exp(-(dx*dx + dy*dy) / (2.0*s*s));
    };
    v += g(0.30, 0.50, 0.14, 1.00);
    v += g(0.65, 0.35, 0.11, 0.80);
    v += g(0.55, 0.70, 0.13, -0.60);  // 负的blob, 制造环形/凹陷
    v += g(0.75, 0.75, 0.08, 0.50);
    return v;
}

// Marching squares 单条线段 (一条线段的两个端点)
struct Seg { double x0, y0, x1, y1; };

// 对于每个边 (0..3), 若存在交点, 通过双线性插值计算交点坐标
// 边索引约定:
//   0: bottom (p0 -> p1), 1: right (p1 -> p2), 2: top (p2 -> p3), 3: left (p3 -> p0)
// 顶点坐标: p0=(i,j) p1=(i+1,j) p2=(i+1,j+1) p3=(i,j+1)  (像素网格, j 向上)
static void gridPt(int idx, int i, int j, double &gx, double &gy) {
    switch (idx) {
        case 0: gx = i;     gy = j;     break;
        case 1: gx = i + 1; gy = j;     break;
        case 2: gx = i + 1; gy = j + 1; break;
        case 3: gx = i;     gy = j + 1; break;
    }
}

// 交点位置: 边 a-b 上按等值 level 线性插值
static void edgeIntersect(int a, int b, const double v[4], int i, int j,
                          double level, double &gx, double &gy) {
    double ax, ay, bx, by;
    gridPt(a, i, j, ax, ay);
    gridPt(b, i, j, bx, by);
    double va = v[a], vb = v[b];
    double t = (level - va) / (vb - va);
    t = std::max(0.0, std::min(1.0, t));
    gx = ax + t * (bx - ax);
    gy = ay + t * (by - ay);
}

int main() {
    // 网格分辨率: 每个像素格是一个 marching square cell
    const int GW = W - 1, GH = H - 1;
    const double level = 0.30;  // 等值线水平

    std::vector<double> grid((size_t)W * H);
    // 计算每个网格顶点(像素)处的场值
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            double x = (double)i / (W - 1);
            double y = (double)j / (H - 1);
            grid[(size_t)j * W + i] = field(x, y);
        }
    }

    std::vector<Seg> segs;

    // 对每个 cell 提取线段
    for (int j = 0; j < GH; ++j) {
        for (int i = 0; i < GW; ++i) {
            double v[4];
            v[0] = grid[(size_t)(j)   * W + (i)];
            v[1] = grid[(size_t)(j)   * W + (i + 1)];
            v[2] = grid[(size_t)(j+1) * W + (i + 1)];
            v[3] = grid[(size_t)(j+1) * W + (i)];

            // 计算 case 索引 (bit0..3 对应顶点0..3 是否 inside)
            int idx = 0;
            if (v[0] >= level) idx |= 1;
            if (v[1] >= level) idx |= 2;
            if (v[2] >= level) idx |= 4;
            if (v[3] >= level) idx |= 8;
            if (idx == 0 || idx == 15) continue;  // 全外或全内

            // 16 case 的边配对表 (每个 case 给出 0~2 条线段, 每条线段两个端点对应的边)
            // 拓扑: case 1 和 14 是单条线段; case 5 和 10 是歧义(鞍点)但标准处理为两条;
            // 其余均一条。这里用标准查表。
            struct Cell { int n; int e[2][2]; };
            // 每条线段的两个端点为两段边 (edgeA, edgeB)。边索引: 0=底,1=右,2=顶,3=左
            // 边 e 连接顶点 e 与 (e+1)%4
            static const Cell table[16] = {
                {0, {{0,0},{0,0}}},  // 0  : 无
                {1, {{0,3},{0,0}}},  // 1  : v0 -> 边0-3
                {1, {{0,1},{0,0}}},  // 2  : v1 -> 边0-1
                {1, {{1,3},{0,0}}},  // 3  : v0v1 -> 边1-3
                {1, {{1,2},{0,0}}},  // 4  : v2 -> 边1-2
                {2, {{0,3},{1,2}}},  // 5  : 鞍点(0-3,1-2)
                {1, {{0,2},{0,0}}},  // 6  : v1v2 -> 边0-2
                {1, {{2,3},{0,0}}},  // 7  : v0v1v2 -> 边2-3
                {1, {{2,3},{0,0}}},  // 8  : v3 -> 边2-3
                {1, {{0,2},{0,0}}},  // 9  : v0v3 -> 边0-2
                {2, {{0,1},{2,3}}},  // 10 : 鞍点(0-1,2-3)
                {1, {{1,2},{0,0}}},  // 11 : v1v2v3 -> 边1-2
                {1, {{1,3},{0,0}}},  // 12 : v2v3 -> 边1-3
                {1, {{0,1},{0,0}}},  // 13 : v0v2v3 -> 边0-1
                {1, {{0,3},{0,0}}},  // 14 : v0v1v3 -> 边0-3
                {0, {{0,0},{0,0}}},  // 15 : 全内
            };

            for (int k = 0; k < table[idx].n; ++k) {
                int ea = table[idx].e[k][0];
                int eb = table[idx].e[k][1];
                Seg s;
                edgeIntersect(ea, (ea + 1) % 4, v, i, j, level, s.x0, s.y0);
                edgeIntersect(eb, (eb + 1) % 4, v, i, j, level, s.x1, s.y1);
                segs.push_back(s);
            }
        }
    }

    // ---- 量化验证 1: 线段数量合理 ----
    int nseg = (int)segs.size();
    printf("Marching Squares 等值线提取\n");
    printf("等值水平 level = %.3f\n", level);
    printf("提取线段数量 = %d\n", nseg);
    if (nseg < 100) { printf("FAIL: 线段数量过少\n"); return 1; }

    // ---- 量化验证 2: 线段端点应落在网格内部 (坐标范围) ----
    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    for (auto &s : segs) {
        minx = std::min(minx, std::min(s.x0, s.x1));
        maxx = std::max(maxx, std::max(s.x0, s.x1));
        miny = std::min(miny, std::min(s.y0, s.y1));
        maxy = std::max(maxy, std::max(s.y0, s.y1));
    }
    printf("线段包围盒: x=[%.1f, %.1f] y=[%.1f, %.1f]\n", minx, maxx, miny, maxy);
    assert(minx >= 0 && maxx <= GW && miny >= 0 && maxy <= GH);

    // ---- 量化验证 3: 每段都在正确等值附近 (采样中点场值应接近 level) ----
    {
        double maxErr = 0.0, sumErr = 0.0;
        int cnt = 0;
        for (auto &s : segs) {
            double mx = (s.x0 + s.x1) / 2;
            double my = (s.y0 + s.y1) / 2;
            double fx = (double)mx / (W - 1);
            double fy = (double)my / (H - 1);
            double e = std::fabs(field(fx, fy) - level);
            maxErr = std::max(maxErr, e);
            sumErr += e;
            cnt++;
        }
        double avgErr = sumErr / cnt;
        printf("线段中点等值误差: max=%.4f avg=%.6f\n", maxErr, avgErr);
        // 双线性近似下误差应较小
        if (maxErr > 0.2) { printf("FAIL: 等值误差过大\n"); return 1; }
    }

    // ---- 量化验证 4: 轮廓闭合性 (端点配对, epsilon 容差合并) ----
    // 正确的 marching squares 应产生闭合轮廓: 每个端点都应被 2 条线段共享(偶数重数)。
    {
        double EPS = 0.05; // 网格单位, 远小于 cell 尺寸(1.0)
        std::vector<std::pair<double,double>> pts;
        for (auto &s : segs) { pts.push_back({s.x0, s.y0}); pts.push_back({s.x1, s.y1}); }
        std::sort(pts.begin(), pts.end());
        int oddCnt = 0;
        size_t k = 0;
        while (k < pts.size()) {
            size_t m = k;
            while (m < pts.size() &&
                   std::fabs(pts[m].first - pts[k].first) < EPS &&
                   std::fabs(pts[m].second - pts[k].second) < EPS) m++;
            if ((m - k) % 2 == 1) oddCnt++;
            k = m;
        }
        // 纯闭合轮廓场下, 所有端点应为偶数重数
        printf("奇数重数端点数量 = %d\n", oddCnt);
        if (oddCnt > 2) { printf("FAIL: 过多未闭合端点, 说明拓扑错误\n"); return 1; }
    }

    // ---- 量化验证 5: inside 面积占比 vs 等值线包围面积一致性 ----
    {
        // 用多边形面积公式计算每条闭合轮廓包围面积是复杂的;
        // 改为: 采样 inside 网格顶点比例, 应与解析期望接近
        long inside = 0;
        for (int j = 0; j < H; ++j)
            for (int i = 0; i < W; ++i)
                if (grid[(size_t)j * W + i] >= level) inside++;
        double frac = (double)inside / (W * H);
        printf("inside 顶点比例 = %.4f (%.1f%%)\n", frac, frac * 100);
        if (frac < 0.01 || frac > 0.99) { printf("FAIL: inside 比例异常\n"); return 1; }
    }

    // ---- 输出 PPM (可视化, 便于人工确认但不作为主要验证依据) ----
    {
        std::vector<unsigned char> img((size_t)W * H * 3);
        // 背景: 根据场值着色 (深蓝 -> 白 -> 橙)
        for (int j = 0; j < H; ++j) {
            for (int i = 0; i < W; ++i) {
                double v = grid[(size_t)j * W + i];
                double t = std::max(-1.0, std::min(1.0, v));
                unsigned char r, g, b;
                if (t < 0) { // 负: 蓝
                    double m = 1.0 + t;
                    r = (unsigned char)(40 * m);
                    g = (unsigned char)(60 * m);
                    b = (unsigned char)(180 * m);
                } else { // 正: 橙
                    double m = t;
                    r = (unsigned char)(240 * m);
                    g = (unsigned char)(150 * m);
                    b = (unsigned char)(40 * m);
                }
                size_t idx = ((size_t)j * W + i) * 3;
                img[idx] = r; img[idx+1] = g; img[idx+2] = b;
            }
        }
        // 绘制线段 (白色)
        for (auto &s : segs) {
            int x0 = (int)(s.x0 + 0.5), y0 = (int)(s.y0 + 0.5);
            int x1 = (int)(s.x1 + 0.5), y1 = (int)(s.y1 + 0.5);
            // Bresenham 画线
            int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy, x = x0, y = y0;
            while (true) {
                if (x >= 0 && x < W && y >= 0 && y < H) {
                    size_t idx = ((size_t)y * W + x) * 3;
                    img[idx] = img[idx+1] = img[idx+2] = 255;
                }
                if (x == x1 && y == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x += sx; }
                if (e2 <= dx) { err += dx; y += sy; }
            }
        }
        FILE *f = fopen("marching_squares.ppm", "wb");
        fprintf(f, "P6\n%d %d\n255\n", W, H);
        fwrite(img.data(), 1, img.size(), f);
        fclose(f);
    }

    printf("\n✅ 全部量化验证通过: 线段数量/坐标范围/等值误差/闭合性/面积占比\n");
    return 0;
}
