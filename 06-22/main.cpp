// DBSCAN Density-Based Spatial Clustering of Applications with Noise
// 2026-06-22 Daily Coding Practice
// Features: ϵ-neighborhood query, core/border/noise point classification,
//   arbitrary-shape cluster discovery, PPM visualization with color-coded clusters.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <random>
#include <cassert>

// ─── 2D Point ──────────────────────────────────────────────────────────────

struct Point {
    double x, y;
    Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

static inline double dist2(const Point& a, const Point& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// ─── DBSCAN ─────────────────────────────────────────────────────────────────

// Labels: -1 = UNDEFINED, -2 = NOISE
static const int UNDEFINED = -1;
static const int NOISE     = -2;

// Returns indices of points within eps of point i
std::vector<int> region_query(const std::vector<Point>& pts, int i, double eps2) {
    std::vector<int> neighbors;
    for (int j = 0; j < (int)pts.size(); ++j) {
        if (j == i) continue;
        if (dist2(pts[i], pts[j]) <= eps2)
            neighbors.push_back(j);
    }
    return neighbors;
}

// Expands cluster starting from a core point
void expand_cluster(const std::vector<Point>& pts,
                    std::vector<int>& labels,
                    int p, int cluster_id, double eps2, int min_pts) {
    std::vector<int> seeds = region_query(pts, p, eps2);
    labels[p] = cluster_id;

    for (size_t si = 0; si < seeds.size(); ++si) {
        int q = seeds[si];
        if (labels[q] == NOISE) {
            // Change noise to border point
            labels[q] = cluster_id;
        }
        if (labels[q] != UNDEFINED) continue;
        labels[q] = cluster_id;

        std::vector<int> neighbors = region_query(pts, q, eps2);
        if ((int)neighbors.size() >= min_pts - 1) { // includes the point itself
            // Core point: add its neighbors as new seeds
            for (int nb : neighbors) {
                if (labels[nb] == UNDEFINED || labels[nb] == NOISE)
                    seeds.push_back(nb);
            }
        }
    }
}

struct DBSCANResult {
    std::vector<int> labels;          // cluster id for each point
    int n_clusters;                   // number of clusters found
    int n_noise;                      // number of noise points
    int n_core;                       // number of core points
    std::vector<std::vector<double>> cluster_centroids;
};

DBSCANResult dbscan(const std::vector<Point>& pts, double eps, int min_pts) {
    int n = (int)pts.size();
    double eps2 = eps * eps;
    std::vector<int> labels(n, UNDEFINED);

    int cluster_id = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != UNDEFINED) continue;

        auto neighbors = region_query(pts, i, eps2);
        if ((int)neighbors.size() < min_pts - 1) {
            labels[i] = NOISE;
        } else {
            expand_cluster(pts, labels, i, cluster_id, eps2, min_pts);
            ++cluster_id;
        }
    }

    DBSCANResult res;
    res.labels      = labels;
    res.n_clusters  = cluster_id;

    // Count noise and core points
    res.n_noise = 0;
    for (int i = 0; i < n; ++i)
        if (labels[i] == NOISE) ++res.n_noise;

    // Count core points (those with >= min_pts neighbors)
    res.n_core = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] < 0) continue;
        auto neighbors = region_query(pts, i, eps2);
        if ((int)neighbors.size() >= min_pts - 1) ++res.n_core;
    }

    // Compute cluster centroids
    res.cluster_centroids.resize(cluster_id, {0, 0});
    std::vector<int> counts(cluster_id, 0);
    for (int i = 0; i < n; ++i) {
        int c = labels[i];
        if (c < 0) continue;
        res.cluster_centroids[c][0] += pts[i].x;
        res.cluster_centroids[c][1] += pts[i].y;
        counts[c]++;
    }
    for (int c = 0; c < cluster_id; ++c) {
        if (counts[c] > 0) {
            res.cluster_centroids[c][0] /= counts[c];
            res.cluster_centroids[c][1] /= counts[c];
        }
    }

    return res;
}

// ─── PPM Output ─────────────────────────────────────────────────────────────

// Generate distinct colors for clusters
void cluster_color(int id, int /* n_clusters */, unsigned char& r, unsigned char& g, unsigned char& b) {
    if (id < 0) { r = g = b = 100; return; } // noise = gray
    // Use golden angle to spread hues
    double hue = fmod(id * 0.618033988749895, 1.0);
    double sat = 0.8, val = 0.9;
    // HSV → RGB
    double c = val * sat;
    double x = c * (1 - fabs(fmod(hue * 6, 2) - 1));
    double m = val - c;
    double rp, gp, bp;
    int h6 = (int)(hue * 6);
    if (h6 == 0)      { rp=c; gp=x; bp=0; }
    else if (h6 == 1) { rp=x; gp=c; bp=0; }
    else if (h6 == 2) { rp=0; gp=c; bp=x; }
    else if (h6 == 3) { rp=0; gp=x; bp=c; }
    else if (h6 == 4) { rp=x; gp=0; bp=c; }
    else              { rp=c; gp=0; bp=x; }
    r = (unsigned char)((rp+m)*255);
    g = (unsigned char)((gp+m)*255);
    b = (unsigned char)((bp+m)*255);
}

void write_ppm(const char* filename, const std::vector<Point>& pts,
               const std::vector<int>& labels, int n_clusters,
               int width, int height,
               double xmin, double xmax, double ymin, double ymax) {
    std::vector<unsigned char> img(width * height * 3, 255); // white bg

    double xscale = (width - 1)  / (xmax - xmin);
    double yscale = (height - 1) / (ymax - ymin);

    // Draw points (larger for core points)
    for (size_t i = 0; i < pts.size(); ++i) {
        int px = (int)((pts[i].x - xmin) * xscale + 0.5);
        int py = height - 1 - (int)((pts[i].y - ymin) * yscale + 0.5);
        if (px < 0 || px >= width || py < 0 || py >= height) continue;

        unsigned char r, g, b;
        int id = labels[i];
        if (id == NOISE) {
            r = g = b = 60; // dark gray for noise
        } else {
            cluster_color(id, n_clusters, r, g, b);
        }

        // Draw a small circle (radius 2 for core-like points, 1 for others)
        int radius = 2;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx*dx + dy*dy > radius*radius) continue;
                int nx = px + dx, ny = py + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                int idx = (ny * width + nx) * 3;
                img[idx]     = r;
                img[idx + 1] = g;
                img[idx + 2] = b;
            }
        }
    }

    // Draw cluster centroids as white X markers
    auto res = dbscan(pts, (xmax-xmin)/50.0, 5); // rough centroids
    DBSCANResult tmp = dbscan(pts, (xmax - xmin) / 50.0, 5);
    for (int c = 0; c < tmp.n_clusters && c < (int)tmp.cluster_centroids.size(); ++c) {
        double cx = tmp.cluster_centroids[c][0];
        double cy = tmp.cluster_centroids[c][1];
        int cpx = (int)((cx - xmin) * xscale + 0.5);
        int cpy = height - 1 - (int)((cy - ymin) * yscale + 0.5);
        int mark_r = 5;
        for (int d = -mark_r; d <= mark_r; ++d) {
            for (int sign : {-1, 1}) {
                int nx = cpx + d, ny = cpy + sign*d;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int idx = (ny * width + nx) * 3;
                    img[idx] = img[idx+1] = img[idx+2] = (int)sqrt(d*d+(sign*d)*(sign*d)) < 3 ? 255 : 200;
                }
            }
        }
    }

    // Write PPM
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);
}

// Simpler centroid drawing
void write_ppm_final(const char* filename, const std::vector<Point>& pts,
                     const std::vector<int>& labels, int n_clusters,
                     const std::vector<std::vector<double>>& centroids,
                     int width, int height,
                     double xmin, double xmax, double ymin, double ymax) {
    std::vector<unsigned char> img(width * height * 3, 255);

    double xscale = (width - 1)  / (xmax - xmin);
    double yscale = (height - 1) / (ymax - ymin);

    for (size_t i = 0; i < pts.size(); ++i) {
        int px = (int)((pts[i].x - xmin) * xscale + 0.5);
        int py = height - 1 - (int)((pts[i].y - ymin) * yscale + 0.5);
        if (px < 0 || px >= width || py < 0 || py >= height) continue;

        unsigned char r, g, b;
        int id = labels[i];
        if (id == NOISE) {
            r = 100; g = 100; b = 100;
        } else {
            cluster_color(id, n_clusters, r, g, b);
        }
        int radius = 2;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx*dx + dy*dy > radius*radius) continue;
                int nx = px + dx, ny = py + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                int idx = (ny * width + nx) * 3;
                img[idx]     = r;
                img[idx + 1] = g;
                img[idx + 2] = b;
            }
        }
    }

    // Draw centroids
    for (int c = 0; c < (int)centroids.size(); ++c) {
        int cpx = (int)((centroids[c][0] - xmin) * xscale + 0.5);
        int cpy = height - 1 - (int)((centroids[c][1] - ymin) * yscale + 0.5);
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                if (abs(dx) == abs(dy) || dx == 0 || dy == 0) {
                    int nx = cpx + dx, ny = cpy + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int idx = (ny * width + nx) * 3;
                        img[idx] = img[idx+1] = 255;
                        img[idx+2] = 0;
                    }
                }
            }
        }
    }

    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);
}

// ─── Dataset Generators ─────────────────────────────────────────────────────

// Dataset 1: Blobs (well-separated Gaussian clusters)
std::vector<Point> generate_blobs(std::mt19937& rng) {
    std::vector<Point> pts;
    std::normal_distribution<double> gauss(0, 0.8);
    // 5 clusters
    double centers[5][2] = {{-5, 5}, {5, 5}, {-5, -5}, {5, -5}, {0, 0}};
    for (int c = 0; c < 5; ++c) {
        for (int i = 0; i < 80; ++i) {
            pts.emplace_back(centers[c][0] + gauss(rng), centers[c][1] + gauss(rng));
        }
    }
    // Add some uniform noise
    std::uniform_real_distribution<double> uni(-9, 9);
    for (int i = 0; i < 30; ++i) {
        pts.emplace_back(uni(rng), uni(rng));
    }
    return pts;
}

// Dataset 2: Moons (non-convex shapes — DBSCAN's strength)
std::vector<Point> generate_moons(std::mt19937& rng) {
    std::vector<Point> pts;
    std::normal_distribution<double> noise(0, 0.2);
    // Upper moon
    for (int i = 0; i < 200; ++i) {
        double t = (i / 199.0) * M_PI;
        pts.emplace_back(cos(t) * 4 + noise(rng), sin(t) * 4 + 3 + noise(rng));
    }
    // Lower moon
    for (int i = 0; i < 200; ++i) {
        double t = (i / 199.0) * M_PI;
        pts.emplace_back(cos(t) * 4 + 1 + noise(rng), -sin(t) * 4 - 3 + noise(rng));
    }
    // Noise
    std::uniform_real_distribution<double> uni(-6, 6);
    for (int i = 0; i < 20; ++i) {
        pts.emplace_back(uni(rng), uni(rng));
    }
    return pts;
}

// Dataset 3: Circles (nested rings)
std::vector<Point> generate_circles(std::mt19937& rng) {
    std::vector<Point> pts;
    std::normal_distribution<double> noise(0, 0.3);
    // Outer ring
    for (int i = 0; i < 250; ++i) {
        double a = (i / 249.0) * 2 * M_PI;
        pts.emplace_back(cos(a)*6 + noise(rng), sin(a)*6 + noise(rng));
    }
    // Inner ring
    for (int i = 0; i < 150; ++i) {
        double a = (i / 149.0) * 2 * M_PI;
        pts.emplace_back(cos(a)*3 + noise(rng), sin(a)*3 + noise(rng));
    }
    // Noise
    std::uniform_real_distribution<double> uni(-7, 7);
    for (int i = 0; i < 20; ++i) {
        pts.emplace_back(uni(rng), uni(rng));
    }
    return pts;
}

// Dataset 4: Anisotropic (elongated clusters in three corners)
std::vector<Point> generate_aniso(std::mt19937& rng) {
    std::vector<Point> pts;
    std::normal_distribution<double> gauss(0, 1);
    // Cluster 1: horizontal bar (top-left corner)
    for (int i = 0; i < 100; ++i) pts.emplace_back(gauss(rng)*4 - 5, gauss(rng)*0.3 + 5);
    // Cluster 2: vertical bar (bottom-left corner)
    for (int i = 0; i < 100; ++i) pts.emplace_back(gauss(rng)*0.3 - 5, gauss(rng)*4 - 5);
    // Cluster 3: diagonal bar (bottom-right corner)
    for (int i = 0; i < 100; ++i) {
        double a = gauss(rng);
        pts.emplace_back(a*2 + 5, a*2 - 5);
    }
    // Noise
    std::uniform_real_distribution<double> uni(-9, 9);
    for (int i = 0; i < 25; ++i) pts.emplace_back(uni(rng), uni(rng));
    return pts;
}

// ─── Validation ─────────────────────────────────────────────────────────────

// Silhouette-like quality metric: average within-cluster distance vs nearest-cluster distance
double compute_dbscan_quality(const std::vector<Point>& pts,
                               const DBSCANResult& res, double /* eps */) {
    int n = (int)pts.size();
    if (res.n_clusters <= 1) return 0.0;

    // Compute per-cluster mean distances
    std::vector<double> intra_mean(res.n_clusters, 0);
    std::vector<int> intra_cnt(res.n_clusters, 0);
    for (int i = 0; i < n; ++i) {
        if (res.labels[i] < 0) continue;
        int ci = res.labels[i];
        double sum = 0;
        int cnt = 0;
        for (int j = 0; j < n; ++j) {
            if (i == j || res.labels[j] != ci) continue;
            sum += sqrt(dist2(pts[i], pts[j]));
            ++cnt;
        }
        if (cnt > 0) { intra_mean[ci] += sum / cnt; intra_cnt[ci]++; }
    }
    for (int c = 0; c < res.n_clusters; ++c)
        if (intra_cnt[c] > 0) intra_mean[c] /= intra_cnt[c];

    // Compute inter-cluster distances (nearest cluster for each)
    double total_sil = 0;
    int valid = 0;
    for (int i = 0; i < n; ++i) {
        if (res.labels[i] < 0) continue;
        int ci = res.labels[i];
        double a = 0;
        int ca = 0;
        for (int j = 0; j < n; ++j) {
            if (i == j || res.labels[j] != ci) continue;
            a += sqrt(dist2(pts[i], pts[j]));
            ++ca;
        }
        if (ca == 0) continue;
        a /= ca;

        // Nearest other cluster
        double b_min = 1e18;
        for (int oc = 0; oc < res.n_clusters; ++oc) {
            if (oc == ci) continue;
            double b_sum = 0;
            int bc = 0;
            for (int j = 0; j < n; ++j) {
                if (res.labels[j] != oc) continue;
                b_sum += sqrt(dist2(pts[i], pts[j]));
                ++bc;
            }
            if (bc > 0) { b_sum /= bc; b_min = std::min(b_min, b_sum); }
        }
        if (b_min < 1e17 && std::max(a, b_min) > 1e-10) {
            total_sil += (b_min - a) / std::max(a, b_min);
            ++valid;
        }
    }
    return valid > 0 ? total_sil / valid : 0;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::mt19937 rng(42);

    struct Dataset {
        const char* name;
        std::vector<Point> points;
        double eps;
        int min_pts;
        int expected_clusters_min;
        int expected_clusters_max;
    };

    auto blobs   = generate_blobs(rng);
    auto moons   = generate_moons(rng);
    auto circles = generate_circles(rng);
    auto aniso   = generate_aniso(rng);

    Dataset datasets[] = {
        {"blobs",    blobs,   2.0,  5, 4, 6},
        {"moons",    moons,   1.0,  5, 2, 3},
        {"circles",  circles, 0.9,  5, 2, 3},
        {"aniso",    aniso,   1.2,  5, 3, 4},
    };

    int width = 800, height = 800;
    double margin = 1.0;

    printf("=== DBSCAN Clustering Results (2026-06-22) ===\n\n");

    for (auto& ds : datasets) {
        printf("Dataset: %s (%d points)\n", ds.name, (int)ds.points.size());

        auto res = dbscan(ds.points, ds.eps, ds.min_pts);

        double noise_rate = 100.0 * res.n_noise / ds.points.size();
        double quality = compute_dbscan_quality(ds.points, res, ds.eps);

        printf("  Clusters: %d  Noise: %d(%.1f%%)  Core: %d\n",
               res.n_clusters, res.n_noise, noise_rate, res.n_core);
        printf("  Silhouette score: %.4f\n", quality);

        // Cluster size distribution
        std::vector<int> sizes(res.n_clusters, 0);
        for (int l : res.labels) if (l >= 0) sizes[l]++;
        printf("  Sizes: ");
        for (int s : sizes) printf("%d ", s);
        printf("\n");

        // Validation: cluster count in expected range
        bool valid = res.n_clusters >= ds.expected_clusters_min &&
                     res.n_clusters <= ds.expected_clusters_max;
        printf("  Cluster count check: %s (expected %d-%d)\n",
               valid ? "✅ PASS" : "❌ FAIL",
               ds.expected_clusters_min, ds.expected_clusters_max);

        // Validation: noise rate sanity
        printf("  Noise rate check: %s (%.1f%%) [OK if <=25%%]\n",
               noise_rate <= 25.0 ? "✅ PASS" : "⚠️ HIGH", noise_rate);

        // Find bounding box
        double xmin =  1e18, xmax = -1e18, ymin =  1e18, ymax = -1e18;
        for (auto& p : ds.points) {
            xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        }
        xmin -= margin; xmax += margin;
        ymin -= margin; ymax += margin;

        // Write PPM
        char fname[128];
        snprintf(fname, sizeof(fname), "dbscan_%s.ppm", ds.name);
        write_ppm_final(fname, ds.points, res.labels, res.n_clusters,
                        res.cluster_centroids, width, height,
                        xmin, xmax, ymin, ymax);
        printf("  Output: %s\n", fname);
        printf("\n");
    }

    // ─── Summary validation ─────────────────────────────────────────────
    printf("=== Summary ===\n");

    // Global stats
    double total_quality = 0;
    int total_valid = 0;
    for (auto& ds : datasets) {
        auto res = dbscan(ds.points, ds.eps, ds.min_pts);
        double q = compute_dbscan_quality(ds.points, res, ds.eps);
        total_quality += q;
        if (res.n_clusters >= ds.expected_clusters_min &&
            res.n_clusters <= ds.expected_clusters_max) ++total_valid;
    }
    printf("Datasets with valid cluster count: %d/%zu\n", total_valid, sizeof(datasets)/sizeof(datasets[0]));
    printf("Mean Silhouette: %.4f\n", total_quality / (sizeof(datasets)/sizeof(datasets[0])));
    printf("All tests: %s\n", total_valid == 4 ? "✅ PASS" : "❌ FAIL");

    return total_valid == 4 ? 0 : 1;
}
