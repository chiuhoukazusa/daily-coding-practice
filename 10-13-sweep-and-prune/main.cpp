// Sweep & Prune Broad-Phase Collision Detection
// 2026-10-13 每日编程实践
//
// 目标：实现宽相位碰撞检测算法 Sweep and Prune (SaP, 也叫 Sort & Sweep)，
//       在大量轴对齐包围盒(AABB)中快速找出所有潜在碰撞对，
//       并与暴力 O(n^2) 全对(all-pairs)检测做正确性与性能对比。
//
// 核心思路：
//   - 对每个 AABB 在 X 轴上投影得到区间 [minX, maxX]。
//   - 把所有区间的端点按坐标排序（扫描线），维护"活动列表"：
//     遇到 minX 端点 -> 该盒子进入活动列表，并与活动列表内所有盒子配对；
//     遇到 maxX 端点 -> 该盒子离开活动列表。
//   - 若两个盒子在 X 轴重叠，再检验它们在 Y 轴(以及 Z 轴)是否也重叠，
//     只有所有轴都重叠才构成一个真实的碰撞对(候选对)。
//     这一步压缩了绝大多数无需配对的情况，是性能来源。
//
// 量化验证（不能靠眼睛）：
//   1. 正确性：SaP 找出的碰撞对集合 与 暴力 O(n^2) 全对检测 完全一致（对撞集合相等）。
//   2. 性能加速比：测量 SaP 与暴力法在多个规模下(AABB 数量递增)的耗时比。
//   3. 碰撞率/候选对压缩率：统计 SaP 在 X 轴筛除后剩余的候选对数量，
//      与理论全对 C(n,2) 相比的压缩倍数。
//   4. 生成 PPM 图像可视化场景与碰撞对（仅作辅助，验证仍以数值为准）。

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

struct AABB {
    double minX, minY, maxX, maxY;
};

// 暴力全对检测：返回所有碰撞对 (i, j)，i < j
std::vector<std::pair<int,int>> bruteForce(const std::vector<AABB>& boxes) {
    std::vector<std::pair<int,int>> pairs;
    const int n = (int)boxes.size();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const AABB &a = boxes[i], &b = boxes[j];
            bool hit = a.minX < b.maxX && a.maxX > b.minX &&
                       a.minY < b.maxY && a.maxY > b.minY;
            if (hit) pairs.emplace_back(i, j);
        }
    }
    return pairs;
}

struct Endpoint {
    double value;
    int id;
    bool isMin; // true = minX(进入), false = maxX(离开)
};

// Sweep & Prune：先按 X 扫描得到候选对，再精确判定 Y 轴重叠
// 返回实际碰撞对集合(已排序去重)与候选对数量
struct SaPResult {
    std::vector<std::pair<int,int>> pairs;
    size_t candidateCount = 0;
};

SaPResult sweepAndPrune(const std::vector<AABB>& boxes) {
    std::vector<Endpoint> endpts;
    const int n = (int)boxes.size();
    endpts.reserve(2 * n);
    for (int i = 0; i < n; ++i) {
        endpts.push_back({boxes[i].minX, i, true});
        endpts.push_back({boxes[i].maxX, i, false});
    }
    // 按坐标排序：同值时 min 端点优先(进入) 保证边界接触也能配对
    std::sort(endpts.begin(), endpts.end(), [](const Endpoint& a, const Endpoint& b) {
        if (a.value != b.value) return a.value < b.value;
        return a.isMin > b.isMin; // true(min) 排在 false(max) 前
    });

    SaPResult res;
    std::vector<int> active; // 活动列表中的盒子 id
    active.reserve(n);

    for (const auto& e : endpts) {
        if (e.isMin) {
            // 进入：与活动列表中所有盒子形成候选对
            for (int id : active) {
                const AABB &a = boxes[id], &b = boxes[e.id];
                bool yOverlap = a.minY < b.maxY && a.maxY > b.minY;
                if (yOverlap) {
                    int lo = id < e.id ? id : e.id;
                    int hi = id < e.id ? e.id : id;
                    res.pairs.emplace_back(lo, hi);
                }
            }
            res.candidateCount += active.size();
            active.push_back(e.id);
        } else {
            // 离开：从活动列表移除
            auto it = std::find(active.begin(), active.end(), e.id);
            if (it != active.end()) active.erase(it);
        }
    }

    // 因为候选对可能从多个端点产生重复(同一对盒子 X 重叠区间一致但只配对一次，
    // 实际上 SaP 中每对只会在后进入者进入时配对一次，不会重复)。
    // 保险起见排序去重。
    std::sort(res.pairs.begin(), res.pairs.end());
    res.pairs.erase(std::unique(res.pairs.begin(), res.pairs.end()), res.pairs.end());
    return res;
}

// 生成随机 AABB：在 [0, world) 内随机中心，随机半宽，造成一定碰撞率
std::vector<AABB> generateBoxes(int n, double world, double size, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> center(0.0, world);
    std::uniform_real_distribution<double> half(1.0, size);
    std::vector<AABB> boxes(n);
    for (int i = 0; i < n; ++i) {
        double cx = center(rng), cy = center(rng);
        double hx = half(rng), hy = half(rng);
        boxes[i] = {cx - hx, cy - hy, cx + hx, cy + hy};
    }
    return boxes;
}

// 单次计时
double timeIt(bool useSaP, const std::vector<AABB>& boxes) {
    auto t0 = std::chrono::high_resolution_clock::now();
    if (useSaP) sweepAndPrune(boxes);
    else        bruteForce(boxes);
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

void writePPM(const std::vector<AABB>& boxes,
              const std::vector<std::pair<int,int>>& pairs,
              const std::string& filename, int imgSize) {
    double world = 0;
    for (auto& b : boxes) world = std::max(world, b.maxX);
    world = std::max(world, 0.0);
    double scale = (imgSize - 2) / world;

    // 图层：底(白) + 盒子(灰) + 碰撞边(红)
    std::vector<unsigned char> img(imgSize * imgSize * 3, 255);
    auto setPx = [&](int x, int y, int r, int g, int b) {
        if (x < 0 || y < 0 || x >= imgSize || y >= imgSize) return;
        int idx = (y * imgSize + x) * 3;
        img[idx] = r; img[idx+1] = g; img[idx+2] = b;
    };
    auto drawBox = [&](const AABB& b, int r, int g, int bl) {
        int x0 = (int)(b.minX * scale) + 1;
        int x1 = (int)(b.maxX * scale) + 1;
        int y0 = (int)(b.minY * scale) + 1;
        int y1 = (int)(b.maxY * scale) + 1;
        for (int x = x0; x <= x1; ++x) { setPx(x, y0, r, g, bl); setPx(x, y1, r, g, bl); }
        for (int y = y0; y <= y1; ++y) { setPx(x0, y, r, g, bl); setPx(x1, y, r, g, bl); }
    };
    for (auto& b : boxes) drawBox(b, 60, 60, 60);
    for (auto& p : pairs) {
        drawBox(boxes[p.first], 220, 20, 20);
        drawBox(boxes[p.second], 220, 20, 20);
    }
    FILE* f = fopen(filename.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", imgSize, imgSize);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);
}

int main() {
    // ----- 正确性验证 -----
    printf("=== Sweep & Prune 正确性验证（与暴力 O(n^2) 全对检测对比）===\n");
    bool allMatch = true;
    const double world = 200.0;
    for (int test = 0; test < 30; ++test) {
        int n = 50 + (test * 137) % 300;
        double size = 4.0 + (test % 5) * 8.0; // 不同盒子尺寸 -> 不同碰撞率
        auto boxes = generateBoxes(n, world, size, 20261013ULL + test);

        auto bf = bruteForce(boxes);
        auto sap = sweepAndPrune(boxes);

        bool match = (bf.size() == sap.pairs.size()) &&
                     std::equal(bf.begin(), bf.end(), sap.pairs.begin());
        if (!match) {
            allMatch = false;
            printf("  [FAIL] test=%d n=%d size=%.1f  bf=%zu sap=%zu\n",
                   test, n, size, bf.size(), sap.pairs.size());
        }
    }
    printf("  30 组随机测试碰撞对集合完全一致: %s\n", allMatch ? "✅ PASS" : "❌ FAIL");

    // ----- 候选对压缩率验证 -----
    printf("\n=== 候选对压缩率（X 轴扫除 vs 理论全对 C(n,2)）===\n");
    {
        int n = 2000;
        auto boxes = generateBoxes(n, world, 6.0, 777ULL);
        auto sap = sweepAndPrune(boxes);
        long long totalPairs = (long long)n * (n - 1) / 2;
        printf("  盒子数 n = %d\n", n);
        printf("  理论全对 C(n,2)  = %lld\n", totalPairs);
        printf("  SaP 候选对(仅X轴重叠) = %zu\n", sap.candidateCount);
        printf("  实际碰撞对(多轴判定后) = %zu\n", sap.pairs.size());
        double compression = (double)totalPairs / (double)sap.candidateCount;
        printf("  X 轴压缩倍数 = %.2fx\n", compression);
        if (compression < 10.0) {
            printf("  ❌ 压缩比过低，宽相位没有起效\n");
            return 1;
        }
        printf("  ✅ 压缩比 > 10x，宽相位有效\n");
    }

    // ----- 性能加速比验证 -----
    printf("\n=== 性能加速比（SaP vs 暴力）===\n");
    std::vector<int> sizes = {1000, 2000, 4000, 8000, 16000};
    double minSpeedup = 1e9;
    for (int n : sizes) {
        auto boxes = generateBoxes(n, world, 6.0, 999ULL + n);
        // 预热 + 多次取平均
        double tBrute = 0, tSap = 0;
        int reps = (n <= 4000) ? 5 : 3;
        for (int r = 0; r < reps; ++r) { tBrute += timeIt(false, boxes); tSap += timeIt(true, boxes); }
        tBrute /= reps; tSap /= reps;
        double speedup = tBrute / tSap;
        minSpeedup = std::min(minSpeedup, speedup);
        printf("  n=%6d  暴力=%.1fus  SaP=%.1fus  加速比=%.2fx\n", n, tBrute, tSap, speedup);
    }
    printf("  最小加速比 = %.2fx\n", minSpeedup);
    if (minSpeedup < 2.0) {
        printf("  ❌ 加速比不足 2x，宽相位意义有限\n");
        return 1;
    }
    printf("  ✅ 加速比 > 2x，宽相位性能验证通过\n");

    // ----- 输出可视化（辅助）-----
    auto visBoxes = generateBoxes(60, world, 10.0, 4242ULL);
    auto visSap = sweepAndPrune(visBoxes);
    writePPM(visBoxes, visSap.pairs, "sweep_and_prune_output.ppm", 800);
    printf("\n已生成可视化 PPM: sweep_and_prune_output.ppm\n");

    printf("\n=== 全部量化验证通过 ✅ ===\n");
    return 0;
}
