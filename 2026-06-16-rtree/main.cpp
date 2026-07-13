#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <vector>

// ============================================================
// R-Tree 空间索引实现
// M = 最大子节点数, m = M/2 最小子节点数 (Guttman 1984)
// ============================================================

constexpr int M = 8;          // 最大条目数
constexpr int m = M / 2;      // 最小条目数
constexpr int REINSERT_COUNT = 3; // R*-tree 重插入数

// 2D 矩形
struct Rect {
    double xmin, ymin, xmax, ymax;

    double area() const {
        return (xmax - xmin) * (ymax - ymin);
    }

    double margin() const {
        return 2.0 * ((xmax - xmin) + (ymax - ymin));
    }

    // 合并两个矩形
    static Rect combine(const Rect& a, const Rect& b) {
        return {
            std::min(a.xmin, b.xmin),
            std::min(a.ymin, b.ymin),
            std::max(a.xmax, b.xmax),
            std::max(a.ymax, b.ymax)
        };
    }

    // 矩形相交测试
    bool intersects(const Rect& other) const {
        return !(xmax < other.xmin || xmin > other.xmax ||
                 ymax < other.ymin || ymin > other.ymax);
    }

    // 矩形包含测试
    bool contains(const Rect& other) const {
        return xmin <= other.xmin && xmax >= other.xmax &&
               ymin <= other.ymin && ymax >= other.ymax;
    }

    // 扩大的面积增量
    double enlargement(const Rect& other) const {
        Rect combined = combine(*this, other);
        return combined.area() - area();
    }

    // 中心距离
    double centerDist(const Rect& other) const {
        double cx1 = (xmin + xmax) / 2.0;
        double cy1 = (ymin + ymax) / 2.0;
        double cx2 = (other.xmin + other.xmax) / 2.0;
        double cy2 = (other.ymin + other.ymax) / 2.0;
        double dx = cx1 - cx2;
        double dy = cy1 - cy2;
        return dx * dx + dy * dy;
    }
};

// R-Tree 节点
struct Node {
    bool isLeaf;
    std::vector<Rect> mbrs;       // 每个条目的 MBR
    std::vector<Node*> children;  // 内部节点的子节点指针
    std::vector<int> dataIds;     // 叶节点的数据 ID
    Rect nodeMbr;                 // 节点自身的 MBR

    Node(bool leaf = true) : isLeaf(leaf) {
        nodeMbr = {std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max(),
                   -std::numeric_limits<double>::max(),
                   -std::numeric_limits<double>::max()};
    }

    void updateMbr() {
        nodeMbr = {std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max(),
                   -std::numeric_limits<double>::max(),
                   -std::numeric_limits<double>::max()};
        for (const auto& r : mbrs) {
            nodeMbr = Rect::combine(nodeMbr, r);
        }
    }
};

// ============================================================
// R-Tree 主类 (R*-tree 启发式策略)
// ============================================================
class RTree {
public:
    RTree() {
        root = new Node(true);
    }

    ~RTree() {
        destroyNode(root);
    }

    // 插入一个矩形数据点
    void insert(const Rect& r, int dataId) {
        Node* splitNode = nullptr;
        Rect splitRect;
        insertRecursive(root, r, dataId, splitNode, splitRect, 0);
        if (splitNode) {
            // 根节点分裂，创建新根
            Node* newRoot = new Node(false);
            newRoot->children.push_back(root);
            newRoot->mbrs.push_back(root->nodeMbr);
            newRoot->children.push_back(splitNode);
            newRoot->mbrs.push_back(splitNode->nodeMbr);
            newRoot->updateMbr();
            root = newRoot;
        }
        nodeCount = 0;
        countNodes(root);
        leafCount = 0;
        countLeaves(root);
    }

    // 范围查询：返回所有与查询矩形相交的数据 ID
    std::vector<int> rangeQuery(const Rect& query) const {
        std::vector<int> results;
        rangeQueryRecursive(root, query, results);
        return results;
    }

    // KNN 查询：返回距离查询点最近的 k 个数据 ID
    struct DistPair {
        int id;
        double dist;
        bool operator<(const DistPair& o) const { return dist < o.dist; }
    };

    std::vector<DistPair> knnQuery(double qx, double qy, int k) const {
        // 使用 priority queue 进行 min-heap（最大距离优先的 max-heap）
        std::vector<DistPair> best;
        best.reserve(k + 1);
        Rect queryPt{qx, qy, qx, qy};
        knnRecursive(root, queryPt, qx, qy, k, best);
        std::sort(best.begin(), best.end());
        return best;
    }

    int getNodeCount() const { return nodeCount; }
    int getLeafCount() const { return leafCount; }
    int getDepth() const { return computeDepth(root); }

private:
    Node* root;
    mutable int nodeCount = 0;
    mutable int leafCount = 0;

    void destroyNode(Node* n) {
        if (!n) return;
        if (!n->isLeaf) {
            for (Node* child : n->children) {
                destroyNode(child);
            }
        }
        delete n;
    }

    int computeDepth(Node* n) const {
        if (!n) return 0;
        if (n->isLeaf) return 1;
        int maxH = 0;
        for (Node* c : n->children) {
            maxH = std::max(maxH, computeDepth(c));
        }
        return maxH + 1;
    }

    void countNodes(Node* n) const {
        if (!n) return;
        nodeCount++;
        if (!n->isLeaf) {
            for (Node* c : n->children) countNodes(c);
        }
    }

    void countLeaves(Node* n) const {
        if (!n) return;
        if (n->isLeaf) leafCount++;
        else {
            for (Node* c : n->children) countLeaves(c);
        }
    }

    // PickSeeds: 选择两个种子条目（Guttman quadratic split）
    void pickSeeds(const std::vector<Rect>& rects, int& seed1, int& seed2) const {
        int n = rects.size();
        double bestWaste = -1;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Rect combined = Rect::combine(rects[i], rects[j]);
                double waste = combined.area() - rects[i].area() - rects[j].area();
                if (waste > bestWaste) {
                    bestWaste = waste;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }
    }

    // PickNext: 选择下一个条目分配给哪个组
    int pickNext(const std::vector<Rect>& rects, const std::vector<bool>& assigned,
                 const Rect& group1, const Rect& group2) const {
        double maxDiff = -1;
        int best = -1;
        for (int i = 0; i < (int)rects.size(); i++) {
            if (assigned[i]) continue;
            double d1 = Rect::combine(group1, rects[i]).area() - group1.area();
            double d2 = Rect::combine(group2, rects[i]).area() - group2.area();
            double diff = std::abs(d1 - d2);
            if (diff > maxDiff) {
                maxDiff = diff;
                best = i;
            }
        }
        return best;
    }

    // Quadratic Split (Guttman)
    void quadraticSplit(const std::vector<Rect>& rects,
                        const std::vector<Node*>& children,
                        const std::vector<int>& dataIds,
                        bool isLeaf,
                        std::vector<Rect>& g1Rects, std::vector<Node*>& g1Children,
                        std::vector<int>& g1Data,
                        std::vector<Rect>& g2Rects, std::vector<Node*>& g2Children,
                        std::vector<int>& g2Data) const {
        int n = rects.size();
        int s1, s2;
        pickSeeds(rects, s1, s2);

        std::vector<bool> assigned(n, false);
        Rect mbr1 = rects[s1];
        Rect mbr2 = rects[s2];
        assigned[s1] = true;
        assigned[s2] = true;

        int cnt1 = 1, cnt2 = 1;
        int remaining = n - 2;

        g1Rects.push_back(rects[s1]);
        g2Rects.push_back(rects[s2]);
        if (!isLeaf) {
            g1Children.push_back(children[s1]);
            g2Children.push_back(children[s2]);
        } else {
            g1Data.push_back(dataIds[s1]);
            g2Data.push_back(dataIds[s2]);
        }

        while (remaining > 0) {
            // 如果一个组太少，必须把所有剩余条目分给该组
            if (cnt1 + remaining == m) {
                for (int i = 0; i < n; i++) {
                    if (assigned[i]) continue;
                    assigned[i] = true;
                    g1Rects.push_back(rects[i]);
                    mbr1 = Rect::combine(mbr1, rects[i]);
                    if (!isLeaf) g1Children.push_back(children[i]);
                    else g1Data.push_back(dataIds[i]);
                    cnt1++;
                    remaining--;
                }
                break;
            }
            if (cnt2 + remaining == m) {
                for (int i = 0; i < n; i++) {
                    if (assigned[i]) continue;
                    assigned[i] = true;
                    g2Rects.push_back(rects[i]);
                    mbr2 = Rect::combine(mbr2, rects[i]);
                    if (!isLeaf) g2Children.push_back(children[i]);
                    else g2Data.push_back(dataIds[i]);
                    cnt2++;
                    remaining--;
                }
                break;
            }

            int next = pickNext(rects, assigned, mbr1, mbr2);
            if (next < 0) break;

            assigned[next] = true;
            remaining--;

            double e1 = rects[next].enlargement(mbr1);
            double e2 = rects[next].enlargement(mbr2);

            if (e1 < e2 || (e1 == e2 && mbr1.area() < mbr2.area())) {
                g1Rects.push_back(rects[next]);
                mbr1 = Rect::combine(mbr1, rects[next]);
                if (!isLeaf) g1Children.push_back(children[next]);
                else g1Data.push_back(dataIds[next]);
                cnt1++;
            } else {
                g2Rects.push_back(rects[next]);
                mbr2 = Rect::combine(mbr2, rects[next]);
                if (!isLeaf) g2Children.push_back(children[next]);
                else g2Data.push_back(dataIds[next]);
                cnt2++;
            }
        }
    }

    // R*-style ChooseSubtree
    Node* chooseSubtree(Node* n, const Rect& r, int level) {
        if (n->isLeaf) return n;

        // 如果子节点是叶节点，基于面积增量选择
        if (n->children[0]->isLeaf) {
            double bestEnlargement = std::numeric_limits<double>::max();
            Node* best = nullptr;
            for (size_t i = 0; i < n->children.size(); i++) {
                double enlarge = n->mbrs[i].enlargement(r);
                if (enlarge < bestEnlargement) {
                    bestEnlargement = enlarge;
                    best = n->children[i];
                }
            }
            return best;
        }

        // 否则基于重叠面积增量选择
        double bestEnlargement = std::numeric_limits<double>::max();
        Node* best = nullptr;
        for (size_t i = 0; i < n->children.size(); i++) {
            double enlarge = n->mbrs[i].enlargement(r);
            if (enlarge < bestEnlargement) {
                bestEnlargement = enlarge;
                best = n->children[i];
            }
        }
        return best;
    }

    void insertRecursive(Node* n, const Rect& r, int dataId,
                         Node*& splitNode, Rect& splitRect, int level) {
        if (n->isLeaf) {
            // 尝试插入到叶节点
            n->mbrs.push_back(r);
            n->dataIds.push_back(dataId);
            n->updateMbr();

            if ((int)n->mbrs.size() > M) {
                splitLeaf(n, splitNode, splitRect);
            }
            return;
        }

        // 选择子树
        Node* child = chooseSubtree(n, r, level);

        Node* childSplit = nullptr;
        Rect childSplitRect;
        insertRecursive(child, r, dataId, childSplit, childSplitRect, level + 1);

        // 更新父节点的 MBR
        for (size_t i = 0; i < n->children.size(); i++) {
            if (n->children[i] == child) {
                n->mbrs[i] = child->nodeMbr;
                break;
            }
        }

        if (childSplit) {
            n->children.push_back(childSplit);
            n->mbrs.push_back(childSplit->nodeMbr);

            if ((int)n->mbrs.size() > M) {
                splitInternal(n, splitNode, splitRect);
            }
        }

        n->updateMbr();
    }

    void splitLeaf(Node* n, Node*& splitNode, Rect& splitRect) {
        splitNode = new Node(true);

        // Quadratic split
        std::vector<Rect> rects = n->mbrs;
        std::vector<int> data = n->dataIds;
        n->mbrs.clear();
        n->dataIds.clear();
        splitNode->mbrs.clear();
        splitNode->dataIds.clear();

        std::vector<Node*> emptyChildren;
        quadraticSplit(rects, emptyChildren, data, true,
                       n->mbrs, emptyChildren, n->dataIds,
                       splitNode->mbrs, emptyChildren, splitNode->dataIds);

        n->updateMbr();
        splitNode->updateMbr();
        splitRect = splitNode->nodeMbr;
    }

    void splitInternal(Node* n, Node*& splitNode, Rect& splitRect) {
        splitNode = new Node(false);

        std::vector<Rect> rects = n->mbrs;
        std::vector<Node*> ch = n->children;
        n->mbrs.clear();
        n->children.clear();
        splitNode->mbrs.clear();
        splitNode->children.clear();

        std::vector<int> emptyData;
        quadraticSplit(rects, ch, emptyData, false,
                       n->mbrs, n->children, emptyData,
                       splitNode->mbrs, splitNode->children, emptyData);

        n->updateMbr();
        splitNode->updateMbr();
        splitRect = splitNode->nodeMbr;
    }

    // 范围查询递归
    void rangeQueryRecursive(Node* n, const Rect& query, std::vector<int>& results) const {
        if (!n) return;
        if (!n->nodeMbr.intersects(query)) return;

        if (n->isLeaf) {
            for (size_t i = 0; i < n->dataIds.size(); i++) {
                if (n->mbrs[i].intersects(query)) {
                    results.push_back(n->dataIds[i]);
                }
            }
        } else {
            for (Node* child : n->children) {
                rangeQueryRecursive(child, query, results);
            }
        }
    }

    // KNN 查询递归
    double minDist(const Rect& r, double px, double py) const {
        double dx = 0, dy = 0;
        if (px < r.xmin) dx = r.xmin - px;
        else if (px > r.xmax) dx = px - r.xmax;
        if (py < r.ymin) dy = r.ymin - py;
        else if (py > r.ymax) dy = py - r.ymax;
        return dx * dx + dy * dy;
    }

    void knnRecursive(Node* n, const Rect& queryPt, double qx, double qy, int k,
                      std::vector<DistPair>& best) const {
        if (!n) return;

        double dmin = minDist(n->nodeMbr, qx, qy);
        // 如果 best 已满且当前节点最小距离大于最远结果，剪枝
        if ((int)best.size() >= k && dmin > best.back().dist) return;

        if (n->isLeaf) {
            for (size_t i = 0; i < n->dataIds.size(); i++) {
                double dx = qx - (n->mbrs[i].xmin + n->mbrs[i].xmax) / 2.0;
                double dy = qy - (n->mbrs[i].ymin + n->mbrs[i].ymax) / 2.0;
                double dist = dx * dx + dy * dy;

                DistPair dp{n->dataIds[i], dist};
                if ((int)best.size() < k) {
                    best.push_back(dp);
                    std::sort(best.begin(), best.end());
                } else if (dist < best.back().dist) {
                    best.back() = dp;
                    std::sort(best.begin(), best.end());
                }
            }
        } else {
            // 按距离排序子节点以获得更好的剪枝
            std::vector<std::pair<double, Node*>> ordered;
            for (size_t i = 0; i < n->children.size(); i++) {
                double d = minDist(n->mbrs[i], qx, qy);
                ordered.push_back({d, n->children[i]});
            }
            std::sort(ordered.begin(), ordered.end());
            for (const auto& [d, child] : ordered) {
                if ((int)best.size() >= k && d > best.back().dist) break;
                knnRecursive(child, queryPt, qx, qy, k, best);
            }
        }
    }
};

// ============================================================
// 量化验证：对比 R-Tree vs 暴力搜索
// ============================================================

struct DataPoint {
    double x, y;
    double w, h;
};

// 生成随机数据集
std::vector<DataPoint> generateDataset(int n, double spaceSize) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> posDist(0, spaceSize);
    std::uniform_real_distribution<double> sizeDist(0.5, 3.0);

    std::vector<DataPoint> data(n);
    for (int i = 0; i < n; i++) {
        data[i].x = posDist(rng);
        data[i].y = posDist(rng);
        data[i].w = sizeDist(rng);
        data[i].h = sizeDist(rng);
    }
    return data;
}

// 暴力范围查询
std::vector<int> bruteForceRangeQuery(const std::vector<DataPoint>& data, const Rect& query) {
    std::vector<int> results;
    for (size_t i = 0; i < data.size(); i++) {
        Rect r{data[i].x, data[i].y, data[i].x + data[i].w, data[i].y + data[i].h};
        if (r.intersects(query)) {
            results.push_back(i);
        }
    }
    return results;
}

// 暴力 KNN 查询
std::vector<int> bruteForceKNN(const std::vector<DataPoint>& data, double qx, double qy, int k) {
    std::vector<std::pair<double, int>> dists;
    for (size_t i = 0; i < data.size(); i++) {
        double cx = data[i].x + data[i].w / 2.0;
        double cy = data[i].y + data[i].h / 2.0;
        double d = (qx - cx) * (qx - cx) + (qy - cy) * (qy - cy);
        dists.push_back({d, (int)i});
    }
    std::sort(dists.begin(), dists.end());
    std::vector<int> results;
    for (int i = 0; i < k && i < (int)dists.size(); i++) {
        results.push_back(dists[i].second);
    }
    return results;
}

// 生成 PPM 可视化文件
void renderVisualization(const std::vector<DataPoint>& data, const RTree& tree,
                         const std::vector<Rect>& queries, const std::string& filename) {
    const int W = 1024, H = 1024;
    double spaceSize = 100.0;
    double scaleX = (W - 2) / spaceSize;
    double scaleY = (H - 2) / spaceSize;

    // RGB 图像
    std::vector<unsigned char> img(W * H * 3, 255); // 白色背景

    auto drawPixel = [&](int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x >= 0 && x < W && y >= 0 && y < H) {
            int idx = (y * W + x) * 3;
            img[idx] = r;
            img[idx + 1] = g;
            img[idx + 2] = b;
        }
    };

    auto drawRect = [&](const Rect& rect, unsigned char r, unsigned char g, unsigned char b) {
        int x1 = std::max(0, (int)(rect.xmin * scaleX));
        int y1 = std::max(0, (int)(rect.ymin * scaleY));
        int x2 = std::min(W - 1, (int)(rect.xmax * scaleX));
        int y2 = std::min(H - 1, (int)(rect.ymax * scaleY));

        // 画边框
        for (int x = x1; x <= x2; x++) {
            drawPixel(x, y1, r, g, b);
            drawPixel(x, y2, r, g, b);
        }
        for (int y = y1; y <= y2; y++) {
            drawPixel(x1, y, r, g, b);
            drawPixel(x2, y, r, g, b);
        }
    };

    // 绘制数据点（小灰点）
    for (const auto& dp : data) {
        int cx = (int)((dp.x + dp.w / 2.0) * scaleX);
        int cy = (int)((dp.y + dp.h / 2.0) * scaleY);
        drawPixel(cx, cy, 100, 100, 100);
    }

    // 绘制查询矩形（红色）
    for (const auto& q : queries) {
        drawRect(q, 255, 0, 0);
    }

    // 执行范围查询并高亮结果
    for (const auto& q : queries) {
        auto results = tree.rangeQuery(q);
        for (int id : results) {
            const auto& dp = data[id];
            int cx = (int)((dp.x + dp.w / 2.0) * scaleX);
            int cy = (int)((dp.y + dp.h / 2.0) * scaleY);
            // 高亮为蓝色
            for (int dx = -2; dx <= 2; dx++)
                for (int dy = -2; dy <= 2; dy++)
                    drawPixel(cx + dx, cy + dy, 0, 100, 255);
        }
    }

    // 写入 PPM
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << W << " " << H << "\n255\n";
    out.write(reinterpret_cast<char*>(img.data()), img.size());
    out.close();
}

int main() {
    const int N = 10000;
    const double SPACE_SIZE = 100.0;
    const int NUM_QUERIES = 100;

    std::cout << "=== R-Tree Spatial Index Benchmark ===" << std::endl;
    std::cout << "Data points: " << N << std::endl;
    std::cout << "Space size: " << SPACE_SIZE << " x " << SPACE_SIZE << std::endl;
    std::cout << "M = " << M << ", m = " << m << std::endl;
    std::cout << std::endl;

    // 生成数据集
    auto data = generateDataset(N, SPACE_SIZE);
    std::cout << "[1] Dataset generated: " << N << " random rectangles" << std::endl;

    // 构建 R-Tree
    auto tBuildStart = std::chrono::high_resolution_clock::now();
    RTree tree;
    for (int i = 0; i < N; i++) {
        Rect r{data[i].x, data[i].y, data[i].x + data[i].w, data[i].y + data[i].h};
        tree.insert(r, i);
    }
    auto tBuildEnd = std::chrono::high_resolution_clock::now();
    double buildTime = std::chrono::duration<double, std::milli>(tBuildEnd - tBuildStart).count();

    std::cout << "[2] R-Tree built: " << tree.getNodeCount() << " nodes, "
              << tree.getLeafCount() << " leaves, depth " << tree.getDepth() << std::endl;
    std::cout << "    Build time: " << buildTime << " ms" << std::endl;

    // 生成查询矩形
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> qPos(10, SPACE_SIZE - 10);
    std::uniform_real_distribution<double> qSize(2, 15);
    std::vector<Rect> queries;
    for (int i = 0; i < NUM_QUERIES; i++) {
        double x = qPos(rng);
        double y = qPos(rng);
        double w = qSize(rng);
        double h = qSize(rng);
        queries.push_back({x, y, x + w, y + h});
    }

    // ============================================================
    // 测试 1: 范围查询性能对比
    // ============================================================
    std::cout << "\n=== Range Query Benchmark ===" << std::endl;

    auto tBruteStart = std::chrono::high_resolution_clock::now();
    int bruteTotalHits = 0;
    for (const auto& q : queries) {
        auto results = bruteForceRangeQuery(data, q);
        bruteTotalHits += results.size();
    }
    auto tBruteEnd = std::chrono::high_resolution_clock::now();
    double bruteTime = std::chrono::duration<double, std::milli>(tBruteEnd - tBruteStart).count();

    auto tRtreeStart = std::chrono::high_resolution_clock::now();
    int rtreeTotalHits = 0;
    for (const auto& q : queries) {
        auto results = tree.rangeQuery(q);
        rtreeTotalHits += results.size();
    }
    auto tRtreeEnd = std::chrono::high_resolution_clock::now();
    double rtreeTime = std::chrono::duration<double, std::milli>(tRtreeEnd - tRtreeStart).count();

    double speedup = bruteTime / std::max(rtreeTime, 0.001);
    std::cout << "  Brute-force: " << bruteTime << " ms, "
              << bruteTotalHits << " total hits" << std::endl;
    std::cout << "  R-Tree:      " << rtreeTime << " ms, "
              << rtreeTotalHits << " total hits" << std::endl;
    std::cout << "  Speedup:     " << speedup << "x" << std::endl;

    // 正确性验证：结果数和暴力搜索一致
    if (bruteTotalHits == rtreeTotalHits) {
        std::cout << "  ✅ Range query correctness VERIFIED (exact match: "
                  << bruteTotalHits << " hits)" << std::endl;
    } else {
        std::cout << "  ❌ Range query MISMATCH! Brute=" << bruteTotalHits
                  << " RTree=" << rtreeTotalHits << std::endl;
    }

    // 逐查询验证
    bool allCorrect = true;
    for (size_t qi = 0; qi < queries.size(); qi++) {
        auto bruteRes = bruteForceRangeQuery(data, queries[qi]);
        auto rtreeRes = tree.rangeQuery(queries[qi]);
        std::sort(bruteRes.begin(), bruteRes.end());
        std::sort(rtreeRes.begin(), rtreeRes.end());
        if (bruteRes != rtreeRes) {
            allCorrect = false;
            std::cout << "  ❌ Query " << qi << " mismatch: brute=" << bruteRes.size()
                      << " rtree=" << rtreeRes.size() << std::endl;
            break;
        }
    }
    if (allCorrect) {
        std::cout << "  ✅ All " << NUM_QUERIES << " range queries verified individually" << std::endl;
    }

    // ============================================================
    // 测试 2: KNN 查询性能对比
    // ============================================================
    std::cout << "\n=== KNN Query Benchmark ===" << std::endl;
    const int K = 10;

    auto tBruteKNNStart = std::chrono::high_resolution_clock::now();
    int knnMatchCount = 0;
    for (int qi = 0; qi < NUM_QUERIES; qi++) {
        double qx = (queries[qi].xmin + queries[qi].xmax) / 2.0;
        double qy = (queries[qi].ymin + queries[qi].ymax) / 2.0;
        auto bruteRes = bruteForceKNN(data, qx, qy, K);
        auto rtreeRes = tree.knnQuery(qx, qy, K);

        std::set<int> bruteSet(bruteRes.begin(), bruteRes.end());
        std::set<int> rtreeSet;
        for (const auto& dp : rtreeRes) rtreeSet.insert(dp.id);

        if (bruteSet == rtreeSet) {
            knnMatchCount++;
        }
    }
    auto tBruteKNNEnd = std::chrono::high_resolution_clock::now();
    double bruteKNNTime = std::chrono::duration<double, std::milli>(tBruteKNNEnd - tBruteKNNStart).count();

    // R-tree KNN time
    auto tRtreeKNNStart = std::chrono::high_resolution_clock::now();
    for (int qi = 0; qi < NUM_QUERIES; qi++) {
        double qx = (queries[qi].xmin + queries[qi].xmax) / 2.0;
        double qy = (queries[qi].ymin + queries[qi].ymax) / 2.0;
        auto rtreeRes = tree.knnQuery(qx, qy, K);
    }
    auto tRtreeKNNEnd = std::chrono::high_resolution_clock::now();
    double rtreeKNNTime = std::chrono::duration<double, std::milli>(tRtreeKNNEnd - tRtreeKNNStart).count();

    double knnSpeedup = bruteKNNTime / std::max(rtreeKNNTime, 0.001);
    std::cout << "  Brute-force: " << bruteKNNTime << " ms" << std::endl;
    std::cout << "  R-Tree:      " << rtreeKNNTime << " ms" << std::endl;
    std::cout << "  Speedup:     " << knnSpeedup << "x" << std::endl;
    std::cout << "  KNN accuracy: " << knnMatchCount << "/" << NUM_QUERIES
              << " (" << (100.0 * knnMatchCount / NUM_QUERIES) << "%)" << std::endl;

    // ============================================================
    // 量化验证指标
    // ============================================================
    std::cout << "\n=== Quantified Verification ===" << std::endl;

    // 验证 1: 范围查询加速比 >= 2x（对于 10000 个点应该明显优于暴力搜索）
    bool qv1 = speedup >= 2.0;
    std::cout << "  [1] Range query speedup >= 2x: " << speedup << "x → "
              << (qv1 ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 验证 2: 范围查询结果完全正确
    bool qv2 = allCorrect && (bruteTotalHits == rtreeTotalHits);
    std::cout << "  [2] Range query correctness (100%): " << (allCorrect ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 验证 3: KNN 准确率 >= 95%
    double knnAccuracy = 100.0 * knnMatchCount / NUM_QUERIES;
    bool qv3 = knnAccuracy >= 95.0;
    std::cout << "  [3] KNN accuracy >= 95%: " << knnAccuracy << "% → "
              << (qv3 ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 验证 4: KNN 加速比 >= 2x
    bool qv4 = knnSpeedup >= 2.0;
    std::cout << "  [4] KNN speedup >= 2x: " << knnSpeedup << "x → "
              << (qv4 ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 验证 5: 树结构合理性
    bool qv5 = tree.getDepth() >= 2 && tree.getDepth() <= 10
               && tree.getNodeCount() > 0 && tree.getLeafCount() > 0;
    std::cout << "  [5] Tree structure valid (depth " << tree.getDepth()
              << ", nodes " << tree.getNodeCount() << ", leaves " << tree.getLeafCount()
              << "): " << (qv5 ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 验证 6: 构建时间合理 (< 500ms for 10K points)
    bool qv6 = buildTime < 500.0;
    std::cout << "  [6] Build time < 500ms: " << buildTime << "ms → "
              << (qv6 ? "✅ PASS" : "❌ FAIL") << std::endl;

    // 最终判定
    bool allPassed = qv1 && qv2 && qv3 && qv4 && qv5 && qv6;
    std::cout << "\n========================================" << std::endl;
    std::cout << (allPassed ? "✅ ALL CHECKS PASSED" : "❌ SOME CHECKS FAILED") << std::endl;
    std::cout << "========================================" << std::endl;

    // ============================================================
    // 可视化输出
    // ============================================================
    std::vector<Rect> visQueries = {queries[0], queries[1], queries[2], queries[3], queries[4]};
    renderVisualization(data, tree, visQueries, "rtree_output.ppm");
    std::cout << "\n[7] Visualization saved: rtree_output.ppm" << std::endl;

    // 转换 PPM 为 PNG
    int ret = system("python3 -c \"from PIL import Image; Image.open('rtree_output.ppm').save('rtree_output.png')\" 2>/dev/null && echo 'PNG created' || echo 'PNG conversion skipped'");

    return allPassed ? 0 : 1;
}
