/**
 * 每日编程实践 - 2026-06-21
 * K-Means Clustering 可视化
 * 
 * 主题: 聚类算法 | K-Means | Lloyd迭代
 * 
 * 功能:
 * 1. 随机生成二维点云
 * 2. K-Means初始化（随机选择k个中心点）
 * 3. Lloyd迭代：分配→更新→收敛
 * 4. 量化验证：SSE（Sum of Squared Errors）随迭代下降
 * 5. 输出PPM图像，每个cluster不同颜色
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <array>

struct Point2D {
    double x, y;
};

struct ClusterResult {
    int cluster_id;
    double distance;
};

// 随机数生成器
std::mt19937 rng(42);  // 固定种子保证可复现

// 生成随机点云（多个潜在高斯簇）
std::vector<Point2D> generate_points(int num_clusters, int points_per_cluster, double spread = 0.8) {
    std::vector<Point2D> points;
    std::uniform_real_distribution<double> center_dist(-5.0, 5.0);
    
    // 为每个簇生成中心
    std::vector<Point2D> true_centers(num_clusters);
    for (int c = 0; c < num_clusters; c++) {
        true_centers[c] = {center_dist(rng), center_dist(rng)};
    }
    
    // 为每个簇生成点
    std::normal_distribution<double> gauss(0.0, spread);
    for (int c = 0; c < num_clusters; c++) {
        for (int p = 0; p < points_per_cluster; p++) {
            points.push_back({
                true_centers[c].x + gauss(rng),
                true_centers[c].y + gauss(rng)
            });
        }
    }
    
    // 随机打乱
    std::shuffle(points.begin(), points.end(), rng);
    
    std::cout << "Generated " << points.size() << " points in " << num_clusters << " clusters\n";
    std::cout << "True centers:\n";
    for (int c = 0; c < num_clusters; c++) {
        std::cout << "  Cluster " << c << ": (" << true_centers[c].x << ", " << true_centers[c].y << ")\n";
    }
    
    return points;
}

// 计算SSE（Sum of Squared Errors）- 聚类质量的量化指标
double compute_sse(const std::vector<Point2D>& points,
                   const std::vector<Point2D>& centers,
                   const std::vector<int>& assignments) {
    double sse = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
        double dx = points[i].x - centers[assignments[i]].x;
        double dy = points[i].y - centers[assignments[i]].y;
        sse += dx * dx + dy * dy;
    }
    return sse;
}

// 计算Silhouette Score（另一种量化指标，越高越好）
double compute_silhouette(const std::vector<Point2D>& points,
                          const std::vector<Point2D>& centers,
                          const std::vector<int>& assignments,
                          int k) {
    if (k <= 1) return 0.0;
    
    double total_score = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
        int a_cluster = assignments[i];
        
        // 计算 a(i): i到同簇内其他点的平均距离
        double a_sum = 0.0;
        int a_count = 0;
        for (size_t j = 0; j < points.size(); j++) {
            if (i != j && assignments[j] == a_cluster) {
                double dx = points[i].x - points[j].x;
                double dy = points[i].y - points[j].y;
                a_sum += std::sqrt(dx * dx + dy * dy);
                a_count++;
            }
        }
        double a_i = (a_count > 0) ? a_sum / a_count : 0.0;
        
        // 计算 b(i): i到最近的其他簇的平均距离
        double b_i = std::numeric_limits<double>::max();
        for (int c = 0; c < k; c++) {
            if (c == a_cluster) continue;
            double b_sum = 0.0;
            int b_count = 0;
            for (size_t j = 0; j < points.size(); j++) {
                if (assignments[j] == c) {
                    double dx = points[i].x - points[j].x;
                    double dy = points[i].y - points[j].y;
                    b_sum += std::sqrt(dx * dx + dy * dy);
                    b_count++;
                }
            }
            if (b_count > 0) {
                double avg_b = b_sum / b_count;
                if (avg_b < b_i) b_i = avg_b;
            }
        }
        
        double s_i = (std::max(a_i, b_i) > 0) ? (b_i - a_i) / std::max(a_i, b_i) : 0.0;
        total_score += s_i;
    }
    
    return total_score / points.size();
}

// K-Means Lloyd迭代
struct KMeansResult {
    std::vector<Point2D> centers;
    std::vector<int> assignments;
    std::vector<double> sse_history;
    int iterations;
    bool converged;
};

KMeansResult kmeans_lloyd(const std::vector<Point2D>& points, int k,
                           int max_iterations = 100, double tolerance = 1e-6) {
    KMeansResult result;
    
    // Step 1: 随机初始化中心点（从数据点中随机选）
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    
    result.centers.resize(k);
    for (int c = 0; c < k; c++) {
        result.centers[c] = points[indices[c]];
    }
    
    std::cout << "\nInitial centers (random from data):\n";
    for (int c = 0; c < k; c++) {
        std::cout << "  Center " << c << ": (" << result.centers[c].x << ", " << result.centers[c].y << ")\n";
    }
    
    result.assignments.assign(points.size(), 0);
    double prev_sse = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < max_iterations; iter++) {
        // Step 2: 分配步骤 —— 每个点分配到最近的中心
        int changes = 0;
        for (size_t i = 0; i < points.size(); i++) {
            double min_dist = std::numeric_limits<double>::max();
            int best_cluster = 0;
            for (int c = 0; c < k; c++) {
                double dx = points[i].x - result.centers[c].x;
                double dy = points[i].y - result.centers[c].y;
                double dist = dx * dx + dy * dy;
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = c;
                }
            }
            if (result.assignments[i] != best_cluster) {
                changes++;
                result.assignments[i] = best_cluster;
            }
        }
        
        // Step 3: 更新步骤 —— 重新计算中心点
        std::vector<Point2D> new_centers(k, {0.0, 0.0});
        std::vector<int> counts(k, 0);
        for (size_t i = 0; i < points.size(); i++) {
            int c = result.assignments[i];
            new_centers[c].x += points[i].x;
            new_centers[c].y += points[i].y;
            counts[c]++;
        }
        for (int c = 0; c < k; c++) {
            if (counts[c] > 0) {
                new_centers[c].x /= counts[c];
                new_centers[c].y /= counts[c];
            } else {
                new_centers[c] = result.centers[c]; // 空簇保持不变
            }
        }
        
        // 计算SSE
        double sse = compute_sse(points, new_centers, result.assignments);
        result.sse_history.push_back(sse);
        
        // 检查收敛
        double sse_delta = prev_sse - sse;
        double center_shift = 0.0;
        for (int c = 0; c < k; c++) {
            double dx = new_centers[c].x - result.centers[c].x;
            double dy = new_centers[c].y - result.centers[c].y;
            center_shift += dx * dx + dy * dy;
        }
        center_shift = std::sqrt(center_shift);
        
        result.centers = new_centers;
        
        std::cout << "Iteration " << (iter + 1) << ": SSE=" << sse 
                  << " (delta=" << sse_delta << "), changes=" << changes
                  << ", center_shift=" << center_shift << "\n";
        
        if (changes == 0 || sse_delta < tolerance || center_shift < tolerance) {
            result.iterations = iter + 1;
            result.converged = true;
            break;
        }
        
        prev_sse = sse;
        
        if (iter == max_iterations - 1) {
            result.iterations = max_iterations;
            result.converged = false;
        }
    }
    
    return result;
}

// PPM图像输出
void render_clusters(const std::vector<Point2D>& points,
                     const std::vector<int>& assignments,
                     const std::vector<Point2D>& centers,
                     int k, const std::string& filename,
                     int width = 800, int height = 800) {
    std::ofstream out(filename);
    out << "P3\n" << width << " " << height << "\n255\n";
    
    // 为每个簇分配颜色
    std::vector<std::array<int, 3>> colors(k);
    // 使用HSV转RGB的颜色方案确保区分度
    for (int c = 0; c < k; c++) {
        double hue = (double)c / k * 360.0;
        double s = 0.85, v = 0.9;
        // HSV to RGB
        double h = hue / 60.0;
        int hi = (int)h % 6;
        double f = h - (int)h;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);
        double r, g, b;
        switch (hi) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            case 5: r = v; g = p; b = q; break;
        }
        colors[c] = {(int)(r * 255), (int)(g * 255), (int)(b * 255)};
    }
    
    // 计算数据范围
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    // 稍微扩展边界
    double pad = 0.1 * (max_x - min_x);
    min_x -= pad; max_x += pad;
    double pad_y = 0.1 * (max_y - min_y);
    min_y -= pad_y; max_y += pad_y;
    
    // 创建画布（黑色背景）
    std::vector<std::array<int, 3>> canvas(width * height, {20, 20, 30});
    
    // 绘制点
    int point_radius = 3;
    for (size_t i = 0; i < points.size(); i++) {
        int px = (int)((points[i].x - min_x) / (max_x - min_x) * (width - 1));
        int py = (int)((1.0 - (points[i].y - min_y) / (max_y - min_y)) * (height - 1));
        int c = assignments[i];
        
        // 画一个圆点
        for (int dy = -point_radius; dy <= point_radius; dy++) {
            for (int dx = -point_radius; dx <= point_radius; dx++) {
                if (dx * dx + dy * dy <= point_radius * point_radius) {
                    int cx = px + dx;
                    int cy = py + dy;
                    if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                        canvas[cy * width + cx] = colors[c];
                    }
                }
            }
        }
    }
    
    // 绘制中心点（白色大星形）
    int center_radius = 5;
    for (int c = 0; c < k; c++) {
        int cx = (int)((centers[c].x - min_x) / (max_x - min_x) * (width - 1));
        int cy = (int)((1.0 - (centers[c].y - min_y) / (max_y - min_y)) * (height - 1));
        
        // 画菱形
        for (int dy = -center_radius; dy <= center_radius; dy++) {
            for (int dx = -center_radius; dx <= center_radius; dx++) {
                if (std::abs(dx) + std::abs(dy) <= center_radius) {
                    int px = cx + dx;
                    int py = cy + dy;
                    if (px >= 0 && px < width && py >= 0 && py < height) {
                        canvas[py * width + px] = {255, 255, 255}; // 白色中心
                    }
                }
            }
        }
    }
    
    // 写入PPM
    for (auto& pixel : canvas) {
        out << pixel[0] << " " << pixel[1] << " " << pixel[2] << " ";
    }
    out.close();
    std::cout << "Rendered to: " << filename << "\n";
}

// 生成随机初始化SSE（用于对比）
double random_sse(const std::vector<Point2D>& points, int k) {
    std::vector<Point2D> random_centers(k);
    std::uniform_real_distribution<double> dist(-6.0, 6.0);
    for (int c = 0; c < k; c++) {
        random_centers[c] = {dist(rng), dist(rng)};
    }
    
    // 分配
    std::vector<int> assign(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        for (int c = 0; c < k; c++) {
            double dx = points[i].x - random_centers[c].x;
            double dy = points[i].y - random_centers[c].y;
            double d = dx * dx + dy * dy;
            if (d < min_dist) {
                min_dist = d;
                assign[i] = c;
            }
        }
    }
    return compute_sse(points, random_centers, assign);
}

int main() {
    const int K = 5;
    const int POINTS_PER_CLUSTER = 60;
    
    // Step 1: 生成数据
    std::cout << "=== K-Means Clustering ===" << std::endl;
    std::cout << "K = " << K << ", Points per cluster = " << POINTS_PER_CLUSTER << "\n\n";
    
    auto points = generate_points(K, POINTS_PER_CLUSTER);
    
    // Baseline: 随机中心点的SSE
    double baseline_sse = random_sse(points, K);
    std::cout << "\nBaseline SSE (random centers): " << baseline_sse << "\n";
    
    // Step 2-3: K-Means迭代
    auto result = kmeans_lloyd(points, K);
    
    // 最终SSE
    double final_sse = result.sse_history.back();
    double improvement = (baseline_sse - final_sse) / baseline_sse * 100.0;
    
    // Step 4: 量化验证
    std::cout << "\n=== Quantification Results ===" << std::endl;
    std::cout << "Iterations: " << result.iterations << std::endl;
    std::cout << "Converged: " << (result.converged ? "YES" : "NO") << std::endl;
    std::cout << "Baseline SSE (random init):  " << baseline_sse << std::endl;
    std::cout << "Final SSE (after K-Means):  " << final_sse << std::endl;
    std::cout << "Improvement: " << improvement << "%" << std::endl;
    
    // Silhouette Score
    double silhouette = compute_silhouette(points, result.centers, result.assignments, K);
    std::cout << "Silhouette Score: " << silhouette << " (range [-1,1], higher is better)" << std::endl;
    
    // SSE下降曲线
    std::cout << "\nSSE Convergence History:\n";
    for (size_t i = 0; i < result.sse_history.size(); i++) {
        double delta = (i == 0) ? 0 : result.sse_history[i-1] - result.sse_history[i];
        std::cout << "  Iter " << (i+1) << ": SSE=" << result.sse_history[i] 
                  << " (delta=" << delta << ")\n";
    }
    
    // 验证：SSE必须单调不增
    bool sse_monotonic = true;
    for (size_t i = 1; i < result.sse_history.size(); i++) {
        if (result.sse_history[i] > result.sse_history[i-1] + 1e-10) {
            std::cout << "WARNING: SSE increased at iteration " << (i+1) << "!\n";
            sse_monotonic = false;
        }
    }
    std::cout << "SSE Monotonicity: " << (sse_monotonic ? "PASS" : "FAIL") << std::endl;
    
    // 验证：改善幅度显著
    bool significant = improvement > 30.0;
    std::cout << "Significant Improvement (>30%): " << (significant ? "PASS" : "FAIL") << std::endl;
    
    // 验证：Silhouette > 0（簇间分离好于簇内聚集）
    bool good_clustering = silhouette > 0.3;
    std::cout << "Good Clustering (Silhouette > 0.3): " << (good_clustering ? "PASS" : "FAIL") << std::endl;
    
    // 验证：收敛迭代次数合理
    bool reasonable_iters = result.iterations <= 30;
    std::cout << "Reasonable Iterations (<=30): " << (reasonable_iters ? "PASS" : "FAIL") << std::endl;
    
    // 最终裁断
    bool all_pass = sse_monotonic && significant && good_clustering && reasonable_iters;
    std::cout << "\n=== FINAL VERDICT: " << (all_pass ? "ALL PASS" : "SOME FAILURES") << " ===" << std::endl;
    
    // 输出可视化
    std::string base = "/root/.openclaw/workspace/daily-coding-practice/2026-06-21-kmeans-clustering/";
    render_clusters(points, result.assignments, result.centers, K, base + "kmeans_output.ppm");
    
    // 保存结果JSON
    std::ofstream json(base + "results.json");
    json << "{\n";
    json << "  \"date\": \"2026-06-21\",\n";
    json << "  \"topic\": \"K-Means Clustering\",\n";
    json << "  \"k\": " << K << ",\n";
    json << "  \"points\": " << points.size() << ",\n";
    json << "  \"iterations\": " << result.iterations << ",\n";
    json << "  \"converged\": " << (result.converged ? "true" : "false") << ",\n";
    json << "  \"baseline_sse\": " << baseline_sse << ",\n";
    json << "  \"final_sse\": " << final_sse << ",\n";
    json << "  \"improvement_pct\": " << improvement << ",\n";
    json << "  \"silhouette_score\": " << silhouette << ",\n";
    json << "  \"sse_monotonic\": " << (sse_monotonic ? "true" : "false") << ",\n";
    json << "  \"verdict\": \"" << (all_pass ? "ALL_PASS" : "SOME_FAILURES") << "\",\n";
    json << "  \"sse_history\": [";
    for (size_t i = 0; i < result.sse_history.size(); i++) {
        if (i > 0) json << ", ";
        json << result.sse_history[i];
    }
    json << "]\n";
    json << "}\n";
    json.close();
    
    std::cout << "Results saved to: " << base << "results.json\n";
    
    return all_pass ? 0 : 1;
}
