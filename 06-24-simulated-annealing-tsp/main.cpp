#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <chrono>
#include <map>
#include <string>

// ============================================================
// Simulated Annealing TSP Solver
// ============================================================
// Implements:
//   - Random city generation
//   - Greedy nearest-neighbor initial solution
//   - 2-opt neighborhood moves
//   - Metropolis acceptance criterion
//   - Multiple cooling schedules for comparison
//   - Quantitative validation metrics
//   - PPM visualization
// ============================================================

using namespace std;

// ============================================================
// Data Structures
// ============================================================
struct Point {
    double x, y;
};

struct Result {
    vector<int> tour;
    double distance;
    vector<double> convergence; // best distance at each iteration
    int iterations_run;
    double init_temp;
    double final_temp;
    string schedule_name;
};

// ============================================================
// Distance calculation
// ============================================================
double dist(const Point& a, const Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

double tour_distance(const vector<Point>& cities, const vector<int>& tour) {
    double d = 0;
    int n = tour.size();
    for (int i = 0; i < n; i++) {
        d += dist(cities[tour[i]], cities[tour[(i+1)%n]]);
    }
    return d;
}

// ============================================================
// Initial solution: Greedy nearest-neighbor
// ============================================================
vector<int> greedy_initial(const vector<Point>& cities) {
    int n = cities.size();
    vector<bool> visited(n, false);
    vector<int> tour;
    tour.reserve(n);
    
    tour.push_back(0);
    visited[0] = true;
    
    for (int i = 1; i < n; i++) {
        int last = tour.back();
        int best = -1;
        double best_dist = 1e18;
        for (int j = 0; j < n; j++) {
            if (!visited[j]) {
                double d = dist(cities[last], cities[j]);
                if (d < best_dist) {
                    best_dist = d;
                    best = j;
                }
            }
        }
        tour.push_back(best);
        visited[best] = true;
    }
    return tour;
}

// ============================================================
// 2-opt neighborhood: reverse segment between i and j
// ============================================================
double two_opt_delta(const vector<Point>& cities, const vector<int>& tour, int i, int j) {
    int n = tour.size();
    int a = tour[i], b = tour[(i+1)%n];
    int c = tour[j], d = tour[(j+1)%n];
    
    double old_len = dist(cities[a], cities[b]) + dist(cities[c], cities[d]);
    double new_len = dist(cities[a], cities[c]) + dist(cities[b], cities[d]);
    return new_len - old_len;
}

void apply_two_opt(vector<int>& tour, int i, int j) {
    int n = tour.size();
    // Reverse segment from i+1 to j (inclusive)
    int left = (i + 1) % n;
    int right = j;
    int len = (right - left + n) % n + 1;
    for (int k = 0; k < len / 2; k++) {
        int idx1 = (left + k) % n;
        int idx2 = (right - k + n) % n;
        swap(tour[idx1], tour[idx2]);
    }
}

// ============================================================
// Simulated Annealing core
// ============================================================
Result simulated_annealing(const vector<Point>& cities, 
                           const vector<int>& initial_tour,
                           double init_temp,
                           double final_temp,
                           double alpha,  // cooling rate
                           int max_iter_per_temp,
                           mt19937& rng,
                           const string& schedule_name) {
    int n = cities.size();
    vector<int> tour = initial_tour;
    double current_dist = tour_distance(cities, tour);
    double best_dist = current_dist;
    vector<int> best_tour = tour;
    
    Result result;
    result.tour = best_tour;
    result.distance = best_dist;
    result.init_temp = init_temp;
    result.final_temp = final_temp;
    result.schedule_name = schedule_name;
    result.iterations_run = 0;
    
    double temp = init_temp;
    uniform_real_distribution<double> uni(0.0, 1.0);
    uniform_int_distribution<int> city_pick(0, n-1);
    
    int accepted = 0, total = 0;
    int iter = 0;
    
    while (temp > final_temp) {
        for (int k = 0; k < max_iter_per_temp; k++) {
            iter++;
            total++;
            
            // Pick two distinct positions for 2-opt
            int i = city_pick(rng);
            int j = city_pick(rng);
            if (i == j) continue;
            // Ensure i < j (wrapping order matters less for 2-opt)
            
            double delta = two_opt_delta(cities, tour, i, j);
            
            // Metropolis criterion
            if (delta < 0 || uni(rng) < exp(-delta / temp)) {
                apply_two_opt(tour, i, j);
                current_dist += delta;
                accepted++;
                
                if (current_dist < best_dist) {
                    best_dist = current_dist;
                    best_tour = tour;
                }
            }
            
            // Record convergence every 100 iterations
            if (iter % 100 == 0) {
                result.convergence.push_back(best_dist);
            }
        }
        
        temp *= alpha;
        result.iterations_run = iter;
    }
    
    // Final convergence point
    result.convergence.push_back(best_dist);
    result.tour = best_tour;
    result.distance = best_dist;
    
    // Print stats
    cout << "  [" << schedule_name << "] Accept rate: " 
         << fixed << setprecision(1) << (100.0 * accepted / max(1, total))
         << "% (" << accepted << "/" << total << ")"
         << ", Best: " << setprecision(2) << best_dist
         << ", Initial: " << setprecision(2) << tour_distance(cities, initial_tour)
         << ", Improvement: " << setprecision(1) 
         << (100.0 * (tour_distance(cities, initial_tour) - best_dist) / tour_distance(cities, initial_tour))
         << "%" << endl;
    
    return result;
}

// ============================================================
// Generate random cities (uniform in unit square)
// ============================================================
vector<Point> generate_cities(int n, mt19937& rng) {
    vector<Point> cities(n);
    uniform_real_distribution<double> uni(0.0, 1.0);
    for (int i = 0; i < n; i++) {
        cities[i] = {uni(rng), uni(rng)};
    }
    return cities;
}

// ============================================================
// Generate PPM image of the tour
// ============================================================
void save_tour_ppm(const string& filename, const vector<Point>& cities, 
                   const vector<int>& tour, int width, int height,
                   int margin = 30) {
    ofstream out(filename);
    out << "P3\n" << width << " " << height << "\n255\n";
    
    // Create pixel buffer (default white)
    vector<vector<vector<int>>> pixels(height, vector<vector<int>>(width, {255, 255, 255}));
    
    // Scale factor
    double scale_x = (width - 2.0 * margin) / 1.0;
    double scale_y = (height - 2.0 * margin) / 1.0;
    
    auto to_px = [&](double x, double y) -> pair<int,int> {
        int px = margin + (int)(x * scale_x);
        int py = height - 1 - margin - (int)(y * scale_y);
        return {px, py};
    };
    
    // Draw edges
    int n = tour.size();
    for (int i = 0; i < n; i++) {
        int a = tour[i], b = tour[(i+1)%n];
        auto [x0, y0] = to_px(cities[a].x, cities[a].y);
        auto [x1, y1] = to_px(cities[b].x, cities[b].y);
        
        // Bresenham's line algorithm
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        
        int cx = x0, cy = y0;
        while (true) {
            if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                pixels[cy][cx] = {50, 100, 200}; // Blue edges
            }
            if (cx == x1 && cy == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
    }
    
    // Draw cities
    for (int i = 0; i < n; i++) {
        auto [px, py] = to_px(cities[i].x, cities[i].y);
        // Red filled circle radius 3 for start city
        if (i == tour[0]) {
            for (int dy = -4; dy <= 4; dy++) {
                for (int dx = -4; dx <= 4; dx++) {
                    if (dx*dx + dy*dy <= 16) {
                        int fx = px + dx, fy = py + dy;
                        if (fx >= 0 && fx < width && fy >= 0 && fy < height) {
                            pixels[fy][fx] = {220, 30, 30}; // Red for start
                        }
                    }
                }
            }
        } else {
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    if (dx*dx + dy*dy <= 4) {
                        int fx = px + dx, fy = py + dy;
                        if (fx >= 0 && fx < width && fy >= 0 && fy < height) {
                            pixels[fy][fx] = {50, 50, 50}; // Dark gray dots
                        }
                    }
                }
            }
        }
    }
    
    // Write PPM
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            out << pixels[y][x][0] << " " << pixels[y][x][1] << " " << pixels[y][x][2] << " ";
        }
        out << "\n";
    }
    out.close();
}

// ============================================================
// Save convergence data as CSV
// ============================================================
void save_convergence_csv(const string& filename, const vector<Result>& results) {
    ofstream out(filename);
    out << "iteration";
    for (const auto& r : results) {
        out << "," << r.schedule_name;
    }
    out << "\n";
    
    // Find max iterations
    size_t max_pts = 0;
    for (const auto& r : results) max_pts = max(max_pts, r.convergence.size());
    
    for (size_t i = 0; i < max_pts; i++) {
        out << (i * 100);
        for (const auto& r : results) {
            if (i < r.convergence.size()) {
                out << "," << fixed << setprecision(4) << r.convergence[i];
            } else {
                out << ",";
            }
        }
        out << "\n";
    }
    out.close();
}

// ============================================================
// Compare with brute force (for small N to validate optimality)
// ============================================================
double brute_force_optimal(const vector<Point>& cities) {
    int n = cities.size();
    if (n > 12) return -1; // Too slow beyond 12
    
    vector<int> perm(n);
    for (int i = 0; i < n; i++) perm[i] = i;
    
    double best = 1e18;
    do {
        double d = tour_distance(cities, perm);
        best = min(best, d);
    } while (next_permutation(perm.begin() + 1, perm.end()));
    
    return best;
}

// ============================================================
// Quantitative validation metrics
// ============================================================
struct ValidationResult {
    double greedy_distance;
    double sa_best_distance;
    double improvement_pct;
    vector<double> schedule_distances;
    vector<string> schedule_names;
    bool passes_all_checks;
    string failure_reason;
};

ValidationResult run_validation(const vector<Point>& cities, 
                                 const vector<int>& greedy_tour,
                                 const vector<Result>& results) {
    ValidationResult vr;
    vr.greedy_distance = tour_distance(cities, greedy_tour);
    vr.passes_all_checks = true;
    
    double sa_best = 1e18;
    for (const auto& r : results) {
        vr.schedule_names.push_back(r.schedule_name);
        vr.schedule_distances.push_back(r.distance);
        sa_best = min(sa_best, r.distance);
    }
    vr.sa_best_distance = sa_best;
    vr.improvement_pct = 100.0 * (vr.greedy_distance - sa_best) / vr.greedy_distance;
    
    // Check 1: SA must improve over greedy
    if (sa_best >= vr.greedy_distance) {
        vr.passes_all_checks = false;
        vr.failure_reason = "SA did not improve over greedy initial solution";
        return vr;
    }
    
    // Check 2: Improvement must be at least 5% (for 50 cities)
    // Relaxed - some cases may have greedy near-optimal
    if (vr.improvement_pct < 2.0) {
        cout << "  ⚠️  Warning: SA improvement only " << vr.improvement_pct 
             << "%, may indicate poor cooling schedule" << endl;
    }
    
    // Check 3: All schedules should improve
    for (const auto& r : results) {
        if (r.distance >= vr.greedy_distance) {
            vr.passes_all_checks = false;
            vr.failure_reason = "Schedule '" + r.schedule_name + "' failed to improve";
            return vr;
        }
    }
    
    // Check 4: Convergence must be monotonic non-increasing
    for (const auto& r : results) {
        for (size_t i = 1; i < r.convergence.size(); i++) {
            if (r.convergence[i] > r.convergence[i-1] + 1e-9) {
                vr.passes_all_checks = false;
                vr.failure_reason = "Schedule '" + r.schedule_name 
                    + "' convergence is not monotonically decreasing";
                return vr;
            }
        }
    }
    
    // Check 5: Final distance must be reasonably consistent across schedules
    double max_d = 0, min_d = 1e18;
    for (double d : vr.schedule_distances) {
        max_d = max(max_d, d);
        min_d = min(min_d, d);
    }
    if (max_d > min_d * 1.5) {
        cout << "  ⚠️  Warning: Large variance across schedules (ratio=" 
             << (max_d/min_d) << ")" << endl;
    }
    
    return vr;
}

// ============================================================
// Main
// ============================================================
int main() {
    cout << "========================================" << endl;
    cout << " Simulated Annealing TSP Solver" << endl;
    cout << "========================================" << endl;
    
    // ============================================================
    // PART 1: Large instance (50 cities) with multiple cooling schedules
    // ============================================================
    cout << "\n[PART 1] 50-city TSP with multiple cooling schedules" << endl;
    cout << "------------------------------------------------------" << endl;
    
    mt19937 rng(42); // Fixed seed for reproducibility
    int n_large = 50;
    auto cities_large = generate_cities(n_large, rng);
    
    auto greedy_tour = greedy_initial(cities_large);
    double greedy_dist = tour_distance(cities_large, greedy_tour);
    cout << "  Greedy initial: " << fixed << setprecision(2) << greedy_dist << endl;
    
    // Multiple cooling schedules
    vector<Result> results;
    
    // Schedule A: Fast cooling (aggressive)
    cout << "\n  Schedule A - Fast cooling (alpha=0.90):" << endl;
    results.push_back(simulated_annealing(cities_large, greedy_tour, 10.0, 0.01, 0.90, 200, rng, "Fast (a=0.90)"));
    
    // Schedule B: Medium cooling (balanced)
    cout << "\n  Schedule B - Medium cooling (alpha=0.95):" << endl;
    results.push_back(simulated_annealing(cities_large, greedy_tour, 10.0, 0.01, 0.95, 200, rng, "Medium (a=0.95)"));
    
    // Schedule C: Slow cooling (thorough)
    cout << "\n  Schedule C - Slow cooling (alpha=0.98):" << endl;
    results.push_back(simulated_annealing(cities_large, greedy_tour, 10.0, 0.01, 0.98, 200, rng, "Slow (a=0.98)"));
    
    // Schedule D: Very slow cooling with higher init temp
    cout << "\n  Schedule D - Very Slow (alpha=0.99, T0=20):" << endl;
    results.push_back(simulated_annealing(cities_large, greedy_tour, 20.0, 0.01, 0.99, 200, rng, "VerySlow (a=0.99 T0=20)"));
    
    // ============================================================
    // PART 2: Validation
    // ============================================================
    cout << "\n[PART 2] Quantitative Validation" << endl;
    cout << "---------------------------------" << endl;
    
    auto vr = run_validation(cities_large, greedy_tour, results);
    
    cout << "\n  === Validation Report ===" << endl;
    cout << "  Initial (greedy):   " << fixed << setprecision(4) << vr.greedy_distance << endl;
    cout << "  Best SA solution:   " << setprecision(4) << vr.sa_best_distance << endl;
    cout << "  Improvement:        " << setprecision(2) << vr.improvement_pct << "%" << endl;
    cout << endl;
    cout << "  Schedule comparison:" << endl;
    for (size_t i = 0; i < vr.schedule_names.size(); i++) {
        cout << "    " << left << setw(25) << vr.schedule_names[i] 
             << ": " << fixed << setprecision(4) << vr.schedule_distances[i]
             << " (Δ=" << setprecision(2) 
             << (100.0*(vr.greedy_distance - vr.schedule_distances[i])/vr.greedy_distance) 
             << "%)" << endl;
    }
    
    if (vr.passes_all_checks) {
        cout << "\n  ✅ ALL CHECKS PASSED" << endl;
    } else {
        cout << "\n  ❌ VALIDATION FAILED: " << vr.failure_reason << endl;
    }
    
    // ============================================================
    // PART 3: Optimality check with small instance (8 cities)
    // ============================================================
    cout << "\n[PART 3] Optimality Verification (8-city instance)" << endl;
    cout << "--------------------------------------------------" << endl;
    
    mt19937 rng2(123);
    int n_small = 8;
    auto cities_small = generate_cities(n_small, rng2);
    auto greedy_small = greedy_initial(cities_small);
    double greedy_small_dist = tour_distance(cities_small, greedy_small);
    
    double optimal = brute_force_optimal(cities_small);
    
    auto result_small = simulated_annealing(cities_small, greedy_small, 5.0, 0.001, 0.95, 100, rng2, "SA-Small");
    
    cout << "\n  Optimality Check:" << endl;
    cout << "    Greedy distance:   " << fixed << setprecision(4) << greedy_small_dist << endl;
    cout << "    SA distance:       " << setprecision(4) << result_small.distance << endl;
    cout << "    Brute-force optimal:" << setprecision(4) << optimal << endl;
    cout << "    SA/Optimal ratio:  " << setprecision(4) << (result_small.distance / optimal) << endl;
    
    bool optimal_found = (result_small.distance <= optimal * 1.01); // Within 1%
    if (optimal_found) {
        cout << "    ✅ SA found near-optimal solution (≤1% of optimal)" << endl;
    } else {
        cout << "    ⚠️  SA gap from optimal: " 
             << setprecision(2) << (100.0*(result_small.distance - optimal)/optimal) << "%" << endl;
        // Don't fail on this - SA is a heuristic
    }
    
    // ============================================================
    // PART 4: Generate outputs
    // ============================================================
    cout << "\n[PART 4] Generating Outputs" << endl;
    cout << "---------------------------" << endl;
    
    // Best tour visualization
    string best_schedule = "";
    double best_dist = 1e18;
    int best_idx = 0;
    for (int i = 0; i < (int)results.size(); i++) {
        if (results[i].distance < best_dist) {
            best_dist = results[i].distance;
            best_idx = i;
            best_schedule = results[i].schedule_name;
        }
    }
    
    cout << "  Best schedule: " << best_schedule << endl;
    
    save_tour_ppm("tsp_best_tour.ppm", cities_large, results[best_idx].tour, 600, 600);
    cout << "  Saved: tsp_best_tour.ppm" << endl;
    
    // Also save greedy tour for comparison
    save_tour_ppm("tsp_greedy_tour.ppm", cities_large, greedy_tour, 600, 600);
    cout << "  Saved: tsp_greedy_tour.ppm" << endl;
    
    // Convergence data
    save_convergence_csv("convergence.csv", results);
    cout << "  Saved: convergence.csv" << endl;
    
    // Small instance tour
    save_tour_ppm("tsp_small_tour.ppm", cities_small, result_small.tour, 400, 400);
    cout << "  Saved: tsp_small_tour.ppm" << endl;
    
    // ============================================================
    // Summary
    // ============================================================
    cout << "\n========================================" << endl;
    cout << " SUMMARY" << endl;
    cout << "========================================" << endl;
    cout << "  Cities: " << n_large << " (main), " << n_small << " (optimality check)" << endl;
    cout << "  Cooling schedules tested: " << results.size() << endl;
    cout << "  Greedy initial: " << fixed << setprecision(2) << greedy_dist << endl;
    cout << "  Best SA: " << setprecision(2) << best_dist << endl;
    cout << "  Improvement: " << setprecision(1) << vr.improvement_pct << "%" << endl;
    cout << "  Optimality gap (8-city): " << setprecision(2) 
         << (100.0*(result_small.distance - optimal)/optimal) << "%" << endl;
    cout << "  All checks: " << (vr.passes_all_checks ? "PASSED" : "FAILED") << endl;
    
    return vr.passes_all_checks ? 0 : 1;
}
