#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// ============================================================
// 2D Point
// ============================================================
struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}

    double dist2(const Point& o) const {
        double dx = x - o.x, dy = y - o.y;
        return dx * dx + dy * dy;
    }
};

// ============================================================
// KD-Tree Node
// ============================================================
struct KDNode {
    Point pt;
    KDNode* left;
    KDNode* right;
    int axis;          // 0 = x-axis split, 1 = y-axis split

    KDNode(const Point& p, int ax)
        : pt(p), left(nullptr), right(nullptr), axis(ax) {}
};

// ============================================================
// KD-Tree Build (recursive, median split)
// ============================================================
KDNode* build_kdtree(std::vector<Point>& pts, int depth, int start, int end) {
    if (start >= end) return nullptr;

    int axis = depth % 2;
    int mid = (start + end) / 2;

    // Partial sort: partition around the median in current axis
    std::nth_element(
        pts.begin() + start, pts.begin() + mid, pts.begin() + end,
        [axis](const Point& a, const Point& b) {
            return (axis == 0) ? (a.x < b.x) : (a.y < b.y);
        });

    KDNode* node = new KDNode(pts[mid], axis);
    node->left  = build_kdtree(pts, depth + 1, start, mid);
    node->right = build_kdtree(pts, depth + 1, mid + 1, end);
    return node;
}

KDNode* construct_kdtree(std::vector<Point>& pts) {
    return build_kdtree(pts, 0, 0, pts.size());
}

// ============================================================
// KNN Search using KD-Tree (k-nearest-neighbors)
// ============================================================
// Min-heap entry: (negative squared distance, pointer) to get max-heap behavior
struct HeapEntry {
    double dist2;
    const Point* pt;
    bool operator<(const HeapEntry& o) const {
        return dist2 < o.dist2;  // max-heap: largest distance on top
    }
};

void kdtree_knn_rec(
    KDNode* node, const Point& query, int k,
    std::priority_queue<HeapEntry>& best)
{
    if (!node) return;

    double d2 = query.dist2(node->pt);

    if ((int)best.size() < k) {
        best.push({d2, &node->pt});
    } else if (d2 < best.top().dist2) {
        best.pop();
        best.push({d2, &node->pt});
    }

    int axis = node->axis;
    double diff = (axis == 0) ? (query.x - node->pt.x) : (query.y - node->pt.y);
    double diff2 = diff * diff;

    // Decide which side to search first
    KDNode* near = (diff < 0) ? node->left : node->right;
    KDNode* far  = (diff < 0) ? node->right : node->left;

    kdtree_knn_rec(near, query, k, best);

    // Check if we need to search the far side
    // If the distance to the splitting plane < current worst distance, must check far
    if ((int)best.size() < k || diff2 < best.top().dist2) {
        kdtree_knn_rec(far, query, k, best);
    }
}

std::vector<Point> kdtree_knn(KDNode* root, const Point& query, int k) {
    std::priority_queue<HeapEntry> best;
    kdtree_knn_rec(root, query, k, best);

    std::vector<Point> result;
    result.reserve(best.size());
    while (!best.empty()) {
        result.push_back(*best.top().pt);
        best.pop();
    }
    std::reverse(result.begin(), result.end());  // closest first
    return result;
}

// ============================================================
// Brute-force KNN (for verification)
// ============================================================
std::vector<Point> brute_force_knn(
    const std::vector<Point>& pts, const Point& query, int k)
{
    std::vector<std::pair<double, const Point*>> dists;
    dists.reserve(pts.size());
    for (const auto& p : pts) {
        dists.push_back({query.dist2(p), &p});
    }
    std::partial_sort(dists.begin(), dists.begin() + k, dists.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<Point> result(k);
    for (int i = 0; i < k; ++i) {
        result[i] = *dists[i].second;
    }
    return result;
}

// ============================================================
// Free KD-Tree
// ============================================================
void free_kdtree(KDNode* node) {
    if (!node) return;
    free_kdtree(node->left);
    free_kdtree(node->right);
    delete node;
}

// ============================================================
// PPM Image Output
// ============================================================
void save_ppm(
    const std::string& filename,
    const std::vector<Point>& data,
    const std::vector<Point>& queries,
    const std::vector<std::vector<Point>>& knn_results,
    int width, int height)
{
    std::vector<uint8_t> img(width * height * 3, 30);  // dark bg

    // Draw data points as small dots (white)
    double min_x = 0, max_x = 1000, min_y = 0, max_y = 1000;
    for (const auto& p : data) {
        int px = (int)((p.x - min_x) / (max_x - min_x) * (width - 1));
        int py = (int)((1.0 - (p.y - min_y) / (max_y - min_y)) * (height - 1));
        if (px >= 0 && px < width && py >= 0 && py < height) {
            int idx = (py * width + px) * 3;
            img[idx]     = std::min(255, img[idx] + 60);
            img[idx + 1] = std::min(255, img[idx + 1] + 60);
            img[idx + 2] = std::min(255, img[idx + 2] + 60);
        }
    }

    // Draw queries as red cross markers (3x3)
    for (const auto& q : queries) {
        int cx = (int)((q.x - min_x) / (max_x - min_x) * (width - 1));
        int cy = (int)((1.0 - (q.y - min_y) / (max_y - min_y)) * (height - 1));
        for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx) {
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int idx = (ny * width + nx) * 3;
                    img[idx]     = 255;
                    img[idx + 1] = 30;
                    img[idx + 2] = 30;
                }
            }
    }

    // Draw KNN results for each query with connecting lines (green)
    for (size_t qi = 0; qi < queries.size(); ++qi) {
        int qpx = (int)((queries[qi].x - min_x) / (max_x - min_x) * (width - 1));
        int qpy = (int)((1.0 - (queries[qi].y - min_y) / (max_y - min_y)) * (height - 1));

        for (const auto& r : knn_results[qi]) {
            int rpx = (int)((r.x - min_x) / (max_x - min_x) * (width - 1));
            int rpy = (int)((1.0 - (r.y - min_y) / (max_y - min_y)) * (height - 1));
            // Simple line drawing (Bresenham)
            int dx = abs(rpx - qpx), sx = qpx < rpx ? 1 : -1;
            int dy = -abs(rpy - qpy), sy = qpy < rpy ? 1 : -1;
            int err = dx + dy, e2;
            int cx = qpx, cy = qpy;
            while (true) {
                if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                    int idx = (cy * width + cx) * 3;
                    img[idx + 1] = std::min(255, img[idx + 1] + 80);
                }
                if (cx == rpx && cy == rpy) break;
                e2 = 2 * err;
                if (e2 >= dy) { err += dy; cx += sx; }
                if (e2 <= dx) { err += dx; cy += sy; }
            }
            // Highlight the neighbor point
            for (int dy2 = -1; dy2 <= 1; ++dy2)
                for (int dx2 = -1; dx2 <= 1; ++dx2) {
                    int nx = rpx + dx2, ny = rpy + dy2;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int idx = (ny * width + nx) * 3;
                        img[idx]     = 180;
                        img[idx + 1] = 255;
                        img[idx + 2] = 180;
                    }
                }
        }
    }

    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << width << " " << height << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), img.size());
    f.close();
}

// ============================================================
// MAIN
// ============================================================
int main() {
    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(0.0, 1000.0);

    const int N = 50000;
    const int Q = 20;
    const int K = 7;

    // Generate random points
    std::vector<Point> points(N);
    for (int i = 0; i < N; ++i) {
        points[i] = Point(dist(rng), dist(rng));
    }

    // Generate query points
    std::vector<Point> queries(Q);
    for (int i = 0; i < Q; ++i) {
        queries[i] = Point(dist(rng), dist(rng));
    }

    // ============ Build KD-Tree ============
    clock_t build_start = clock();
    std::vector<Point> pts_copy = points;
    KDNode* root = construct_kdtree(pts_copy);
    clock_t build_end = clock();
    double build_ms = 1000.0 * (build_end - build_start) / CLOCKS_PER_SEC;

    // ============ KD-Tree KNN ============
    clock_t kdtree_start = clock();
    std::vector<std::vector<Point>> kdtree_results(Q);
    std::vector<double> kdtree_dists(Q);
    for (int i = 0; i < Q; ++i) {
        kdtree_results[i] = kdtree_knn(root, queries[i], K);
        double max_d2 = 0;
        for (const auto& p : kdtree_results[i]) {
            max_d2 = std::max(max_d2, queries[i].dist2(p));
        }
        kdtree_dists[i] = std::sqrt(max_d2);
    }
    clock_t kdtree_end = clock();
    double kdtree_ms = 1000.0 * (kdtree_end - kdtree_start) / CLOCKS_PER_SEC;

    // ============ Brute-Force KNN ============
    clock_t brute_start = clock();
    std::vector<std::vector<Point>> brute_results(Q);
    for (int i = 0; i < Q; ++i) {
        brute_results[i] = brute_force_knn(points, queries[i], K);
    }
    clock_t brute_end = clock();
    double brute_ms = 1000.0 * (brute_end - brute_start) / CLOCKS_PER_SEC;

    // ============ Verification: KD-Tree vs Brute-Force ============
    int match_errors = 0;
    double max_dist_error = 0.0;
    for (int i = 0; i < Q; ++i) {
        // Sort both by index comparison; just compare the K closest sets
        for (int j = 0; j < K; ++j) {
            double d2_kd   = queries[i].dist2(kdtree_results[i][j]);
            double d2_bf   = queries[i].dist2(brute_results[i][j]);
            double err_pct = std::abs(std::sqrt(d2_kd) - std::sqrt(d2_bf))
                             / std::max(1e-9, std::sqrt(d2_bf));
            max_dist_error = std::max(max_dist_error, err_pct);
            if (err_pct > 1e-6) {
                match_errors++;
            }
        }
    }

    // ============ Print Results ============
    std::cout << "=== KD-Tree KNN Search Results ===" << std::endl;
    std::cout << "Points: " << N << " | Queries: " << Q << " | K: " << K << std::endl;
    std::cout << std::endl;
    std::cout << "Build time:  " << build_ms << " ms" << std::endl;
    std::cout << "KD-Tree KNN: " << kdtree_ms << " ms (" << kdtree_ms/Q << " ms/query)" << std::endl;
    std::cout << "Brute-Force: " << brute_ms << " ms (" << brute_ms/Q << " ms/query)" << std::endl;
    std::cout << "Speedup:     " << (brute_ms / std::max(1e-9, kdtree_ms)) << "x" << std::endl;
    std::cout << std::endl;
    std::cout << "Match errors (dist > 1e-6 relative): " << match_errors
              << " / " << (Q * K) << std::endl;
    std::cout << "Max distance error: " << max_dist_error << std::endl;
    std::cout << std::endl;
    std::cout << "NEAREST_NEIGHBOR_DISTANCES:";
    for (double d : kdtree_dists) std::cout << " " << d;
    std::cout << std::endl;

    // ============ Save PPM Visualization ============
    save_ppm("kdtree_output.ppm", points, queries, kdtree_results, 800, 800);
    std::cout << "Visualization saved to kdtree_output.ppm" << std::endl;

    // ============ Quantitative Assertions ============
    // Must have no mismatched neighbors
    if (match_errors > 0) {
        std::cerr << "FAIL: KD-Tree KNN results differ from brute-force!" << std::endl;
        return 1;
    }

    // KD-Tree should be faster for 50K points
    if (kdtree_ms >= brute_ms) {
        std::cerr << "FAIL: KD-Tree is not faster than brute-force!" << std::endl;
        return 1;
    }

    // Build time should be reasonable
    if (build_ms > 500) {
        std::cerr << "FAIL: KD-Tree build time too slow: " << build_ms << " ms" << std::endl;
        return 1;
    }

    // Query distances should be reasonable (within [0, 1414] in 1000x1000 grid)
    for (double d : kdtree_dists) {
        if (d < 0 || d > 1500) {
            std::cerr << "FAIL: KNN distance out of range: " << d << std::endl;
            return 1;
        }
    }

    free_kdtree(root);
    std::cout << "\nALL CHECKS PASSED" << std::endl;
    return 0;
}
