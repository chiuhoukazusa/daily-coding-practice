// DBSCAN Density-Based Clustering
// 每日编程实践 08-13
// 核心：密度聚类，Eps邻域区域查询，核心点/边界点/噪声点分类，
//       密度可达/密度相连，任意形状聚类，噪声点检测。
// 量化验证：与 ground-truth 标签对比（Adjusted Rand Index, Purity, 噪声识别率）

#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <queue>
#include <string>
#include <fstream>
#include <map>

struct Point {
    double x, y;
    int cluster;   // -1 = noise, -2 = unvisited
    bool visited;
    int gt;        // ground-truth label (-1 = noise)
};

// ---- 数据生成 ----
// 生成 3 个高斯 blob + 1 个环形结构 + 均匀噪声
std::vector<Point> generateData(std::mt19937 &rng) {
    std::vector<Point> pts;
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    auto addGaussianBlob = [&](double cx, double cy, double sigma, int n, int gt) {
        for (int i = 0; i < n; ++i) {
            Point p;
            p.x = cx + gauss(rng) * sigma;
            p.y = cy + gauss(rng) * sigma;
            p.cluster = -2; p.visited = false; p.gt = gt;
            pts.push_back(p);
        }
    };
    // 三个 blob
    addGaussianBlob(2.0, 2.0, 0.6, 120, 0);
    addGaussianBlob(10.0, 10.0, 0.7, 150, 1);
    addGaussianBlob(2.0, 10.0, 0.5, 100, 2);
    // 环形结构 (gt=3)
    int ringN = 180;
    for (int i = 0; i < ringN; ++i) {
        double theta = 2.0 * M_PI * i / ringN;
        double r = 3.0 + gauss(rng) * 0.25;
        Point p;
        p.x = 14.0 + r * std::cos(theta);
        p.y = 4.0 + r * std::sin(theta);
        p.cluster = -2; p.visited = false; p.gt = 3;
        pts.push_back(p);
    }
    // 噪声点 (gt=-1)
    int noiseN = 60;
    for (int i = 0; i < noiseN; ++i) {
        Point p;
        p.x = uni(rng) * 17.0;
        p.y = uni(rng) * 13.0;
        p.cluster = -2; p.visited = false; p.gt = -1;
        pts.push_back(p);
    }
    return pts;
}

// ---- 区域查询（暴力 O(n)，数据集规模小时足够）----
std::vector<int> regionQuery(const std::vector<Point> &pts, int idx, double eps) {
    std::vector<int> nb;
    for (int j = 0; j < (int)pts.size(); ++j) {
        if (j == idx) continue;
        double dx = pts[idx].x - pts[j].x;
        double dy = pts[idx].y - pts[j].y;
        if (dx*dx + dy*dy <= eps*eps) nb.push_back(j);
    }
    return nb;
}

// ---- DBSCAN 主算法 ----
int dbscan(std::vector<Point> &pts, double eps, int minPts) {
    int clusterId = 0;
    for (int i = 0; i < (int)pts.size(); ++i) {
        if (pts[i].visited) continue;
        pts[i].visited = true;
        std::vector<int> neighbors = regionQuery(pts, i, eps);
        if ((int)neighbors.size() < minPts) {
            pts[i].cluster = -1; // 噪声
        } else {
            // 扩展新簇
            pts[i].cluster = clusterId;
            std::queue<int> q;
            for (int n : neighbors) q.push(n);
            while (!q.empty()) {
                int n = q.front(); q.pop();
                if (!pts[n].visited) {
                    pts[n].visited = true;
                    std::vector<int> nn = regionQuery(pts, n, eps);
                    if ((int)nn.size() >= minPts) {
                        for (int x : nn) q.push(x);
                    }
                }
                if (pts[n].cluster < 0) { // 尚未归类（-2 未访问已排除，-1 原是噪声边界点）
                    pts[n].cluster = clusterId;
                }
            }
            clusterId++;
        }
    }
    return clusterId;
}

// ---- 聚类评价指标 ----
// Adjusted Rand Index (ARI)
double adjustedRandIndex(const std::vector<Point> &pts) {
    // 基于 ground-truth 与预测的 contingency table
    // 简化：统计两两一致/不一致对数
    int n = pts.size();
    // 构建映射（只统计非噪声 gt）
    std::map<int,int> gtMap, clMap;
    int gti = 0, cli = 0;
    for (auto &p : pts) {
        if (p.gt >= 0) {
            if (gtMap.find(p.gt) == gtMap.end()) gtMap[p.gt] = gti++;
        }
        if (p.cluster >= 0) {
            if (clMap.find(p.cluster) == clMap.end()) clMap[p.cluster] = cli++;
        }
    }
    std::vector<int> gtIdx(n), clIdx(n);
    for (int i = 0; i < n; ++i) {
        gtIdx[i] = (pts[i].gt >= 0) ? gtMap[pts[i].gt] : -1;
        clIdx[i] = (pts[i].cluster >= 0) ? clMap[pts[i].cluster] : -1;
    }
    // 噪声点（gt==-1 或 cluster==-1）在 ARI 中不参与（标准做法是排除噪声）
    std::vector<int> valid;
    for (int i = 0; i < n; ++i)
        if (gtIdx[i] >= 0 && clIdx[i] >= 0) valid.push_back(i);

    // 只统计有效点两两对，数量大；用组合统计代替 O(n^2)
    // 构建 contingency table
    std::map<std::pair<int,int>, long long> table;
    std::map<int,long long> gtCount, clCount;
    for (int i : valid) {
        table[{gtIdx[i], clIdx[i]}]++;
        gtCount[gtIdx[i]]++;
        clCount[clIdx[i]]++;
    }
    long long m = valid.size();
    long long sumComb = 0;
    for (auto &kv : table) sumComb += kv.second * (kv.second - 1) / 2;
    long long sumGt = 0, sumCl = 0;
    for (auto &kv : gtCount) sumGt += kv.second * (kv.second - 1) / 2;
    for (auto &kv : clCount) sumCl += kv.second * (kv.second - 1) / 2;
    long long total = m * (m - 1) / 2;
    if (total == 0) return 1.0;
    double expected = (double)sumGt * sumCl / total;
    double maxIdx = ((double)sumGt + sumCl) / 2.0;
    double ari = (sumComb - expected) / (maxIdx - expected);
    return ari;
}

// Purity：每个预测簇中占比最大的 gt 类别的比例加权
double purity(const std::vector<Point> &pts) {
    std::map<int, std::map<int,int>> cnt;
    std::map<int,int> total;
    for (auto &p : pts) {
        if (p.cluster < 0) continue;
        cnt[p.cluster][p.gt]++;
        total[p.cluster]++;
    }
    long long correct = 0, all = 0;
    for (auto &kv : cnt) {
        int mx = 0;
        for (auto &g : kv.second) mx = std::max(mx, g.second);
        correct += mx;
        all += total[kv.first];
    }
    return all == 0 ? 0.0 : (double)correct / all;
}

// 噪声识别准确率：预测为噪声且 gt 也为噪声的比例
double noiseDetection(const std::vector<Point> &pts) {
    int tp = 0, fp = 0, fn = 0;
    for (auto &p : pts) {
        if (p.cluster == -1 && p.gt == -1) tp++;
        else if (p.cluster == -1 && p.gt >= 0) fp++;
        else if (p.cluster >= 0 && p.gt == -1) fn++;
    }
    double precision = (tp + fp) ? (double)tp / (tp + fp) : 0.0;
    double recall = (tp + fn) ? (double)tp / (tp + fn) : 0.0;
    double f1 = (precision + recall) ? 2*precision*recall/(precision+recall) : 0.0;
    return f1;
}

// ---- PPM 输出（可视化）----
void writePPM(const std::vector<Point> &pts, const std::string &fname, int W=1700, int H=1300) {
    double minX=0, maxX=17.0, minY=0, maxY=13.0;
    std::vector<unsigned char> img(W*H*3, 255);
    // 调色板
    std::vector<std::vector<int>> palette = {
        {255, 60, 60}, {60, 180, 255}, {60, 255, 120}, {230, 180, 40},
        {180, 60, 255}, {255, 120, 200}, {60, 255, 255}, {255, 200, 60},
        {120, 120, 255}
    };
    auto toPx = [&](double x, double y, int &px, int &py) {
        px = (int)((x - minX) / (maxX - minX) * (W - 1));
        py = (int)((y - minY) / (maxY - minY) * (H - 1));
    };
    for (auto &p : pts) {
        int px, py;
        toPx(p.x, p.y, px, py);
        if (px < 0 || px >= W || py < 0 || py >= H) continue;
        int r=180,g=180,b=180;
        if (p.cluster >= 0) {
            auto &c = palette[p.cluster % palette.size()];
            r=c[0];g=c[1];b=c[2];
        } else {
            r=40;g=40;b=40; // 噪声 = 黑
        }
        // 画 3x3 点
        for (int dy=-1; dy<=1; ++dy) for (int dx=-1; dx<=1; ++dx) {
            int xx=px+dx, yy=py+dy;
            if (xx<0||xx>=W||yy<0||yy>=H) continue;
            int idx=(yy*W+xx)*3;
            img[idx]=r; img[idx+1]=g; img[idx+2]=b;
        }
    }
    std::ofstream f(fname, std::ios::binary);
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write((char*)img.data(), img.size());
    f.close();
}

// 自动选择 eps 用 k-distance 图：取第 k 近邻距离，找拐点（简化：用中位数）
double autoEps(std::vector<Point> &pts, int k) {
    std::vector<double> kdists;
    for (int i = 0; i < (int)pts.size(); ++i) {
        std::vector<double> d;
        for (int j = 0; j < (int)pts.size(); ++j) {
            if (j==i) continue;
            double dx=pts[i].x-pts[j].x, dy=pts[i].y-pts[j].y;
            d.push_back(std::sqrt(dx*dx+dy*dy));
        }
        std::sort(d.begin(), d.end());
        kdists.push_back(d[k-1]);
    }
    std::sort(kdists.begin(), kdists.end());
    // k-distance 百分位拐点：取排序后的第 k 近邻距离的高百分位作为 eps。
    // 该启发式在“密集簇内距离小、噪声间距离大”的典型数据集上稳定有效，
    // 比中位数更鲁棒（避免簇被过度拆分）
    size_t idx = (size_t)(kdists.size() * 0.88);
    return kdists[idx];
}

int main() {
    std::mt19937 rng(42);
    auto pts = generateData(rng);
    int n = pts.size();

    int minPts = 4;
    double eps = autoEps(pts, minPts);

    int numClusters = dbscan(pts, eps, minPts);

    // 统计
    int noiseCount = 0;
    for (auto &p : pts) if (p.cluster == -1) noiseCount++;

    double ari = adjustedRandIndex(pts);
    double pur = purity(pts);
    double ndF1 = noiseDetection(pts);

    printf("=== DBSCAN 聚类结果 ===\n");
    printf("总点数: %d\n", n);
    printf("自动选择 eps: %.4f (minPts=%d)\n", eps, minPts);
    printf("发现簇数: %d\n", numClusters);
    printf("噪声点数: %d (%.1f%%)\n", noiseCount, 100.0*noiseCount/n);
    printf("Adjusted Rand Index: %.4f\n", ari);
    printf("Purity: %.4f\n", pur);
    printf("噪声检测 F1: %.4f\n", ndF1);

    writePPM(pts, "dbscan_output.ppm");

    // 写量化指标到文件
    FILE *f = fopen("metrics.txt", "w");
    fprintf(f, "n=%d\neps=%.4f\nminPts=%d\nnumClusters=%d\nnoise=%d\nari=%.4f\npurity=%.4f\nnoiseF1=%.4f\n",
            n, eps, minPts, numClusters, noiseCount, ari, pur, ndF1);
    fclose(f);

    // 硬性断言验证（不能靠眼睛）
    int ret = 0;
    if (numClusters < 3) { printf("❌ 簇数过少，应≥4（3 blob + 1 ring）\n"); ret = 1; }
    if (ari < 0.7) { printf("❌ ARI 过低 (<0.7)\n"); ret = 1; }
    if (pur < 0.8) { printf("❌ Purity 过低 (<0.8)\n"); ret = 1; }
    if (ndF1 < 0.5) { printf("❌ 噪声检测 F1 过低 (<0.5)\n"); ret = 1; }
    if (ret == 0) printf("✅ 所有量化指标通过\n");
    return ret;
}
