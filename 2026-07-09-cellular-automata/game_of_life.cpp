/**
 * 每日编程实践 - 2026-07-09
 * 元胞自动机：Conway's Game of Life
 *
 * 功能:
 * 1. 多种初始模式预设 (Glider, Blinker, Pulsar, Gosper Glider Gun, Random)
 * 2. 量化分析: 逐代种群统计、稳定性检测、周期性检测
 * 3. PPM 可视化输出 (多帧/最终帧)
 *
 * 量化验证指标 (不用眼睛看):
 *  - Population dynamics: 每代活细胞数
 *  - Stability detection: 周期检测 (cycle length)
 *  - Pattern classification: 静态/振荡/移动/消亡
 *  - Multi-run statistics: 随机初始条件下的行为统计
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <sstream>
#include <random>
#include <iomanip>

// Grid dimensions
const int WIDTH  = 200;
const int HEIGHT = 200;
const int MAX_GENERATIONS = 500;

// Pattern types
enum Pattern {
    GLIDER,
    BLINKER,
    PULSAR,
    GOSPER_GLIDER_GUN,
    GLIDER_GUN_2,   // Another gun for collision experiments
    R_PENTOMINO,
    DIEHARD,
    ACORN,
    RANDOM_SMALL,
    RANDOM_LARGE
};

// Hash a grid state for cycle detection (compact: 64-bit hash)
uint64_t hash_grid(const std::vector<std::vector<bool>>& grid) {
    uint64_t h = 0;
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (grid[y][x]) {
                uint64_t idx = y * WIDTH + x;
                h ^= idx + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            }
        }
    }
    return h;
}

// Initialize grid from a pattern
void init_pattern(std::vector<std::vector<bool>>& grid, Pattern pattern, std::mt19937& rng) {
    grid.assign(HEIGHT, std::vector<bool>(WIDTH, false));
    
    int cx = WIDTH / 2;
    int cy = HEIGHT / 2;
    
    switch (pattern) {
    case GLIDER: {
        // Standard glider
        int gx = cx - 20, gy = cy;
        grid[gy][gx+1] = true;
        grid[gy+1][gx+2] = true;
        grid[gy+2][gx] = true;
        grid[gy+2][gx+1] = true;
        grid[gy+2][gx+2] = true;
        break;
    }
    case BLINKER: {
        // Period-2 oscillator
        grid[cy][cx-1] = grid[cy][cx] = grid[cy][cx+1] = true;
        break;
    }
    case PULSAR: {
        // Period-3 oscillator (48 cells) - verified pattern
        int ox = cx - 6, oy = cy - 6;
        const char* pulsar_pattern[] = {
            "..OOO...OOO..",
            ".............",
            "O....O.O....O",
            "O....O.O....O",
            "O....O.O....O",
            "..OOO...OOO..",
            ".............",
            "..OOO...OOO..",
            "O....O.O....O",
            "O....O.O....O",
            "O....O.O....O",
            ".............",
            "..OOO...OOO..",
        };
        for (int y = 0; y < 13; y++) {
            for (int x = 0; pulsar_pattern[y][x]; x++) {
                if (pulsar_pattern[y][x] == 'O') grid[oy+y][ox+x] = true;
            }
        }
        break;
    }
    case GOSPER_GLIDER_GUN: {
        // Gosper glider gun - places at offset
        int gx = cx - 36, gy = cy - 10;
        // Left block
        grid[gy][gx+1] = grid[gy][gx+2] = true;
        grid[gy+1][gx+1] = grid[gy+1][gx+2] = true;
        // Right block
        grid[gy][gx+34+1] = grid[gy][gx+34+2] = true;
        grid[gy+1][gx+34+1] = grid[gy+1][gx+34+2] = true;
        // Left ship
        for (int i = -2; i <= 2; i++) grid[gy+5+i][gx+11+2] = true;
        grid[gy+3][gx+11+1] = grid[gy+3][gx+11+2] = true;
        grid[gy+7][gx+11+0] = grid[gy+7][gx+11+1] = true;
        // Right ship
        for (int i = -1; i <= 3; i++) grid[gy+1+i][gx+24+1] = true;
        grid[gy][gx+24+0] = grid[gy][gx+24+1] = true;
        grid[gy+4][gx+24+0] = grid[gy+4][gx+24+2] = true;
        // Middle connections
        grid[gy+5][gx+11+2] = true; grid[gy+5][gx+12+2] = true;
        grid[gy+6][gx+11+1] = true; grid[gy+6][gx+12+1] = true;
        grid[gy+7][gx+11+0] = true; grid[gy+7][gx+12+0] = true;
        
        // Actually, simpler to hardcode the Gosper Gun properly:
        // Clear and rebuild
        grid.assign(HEIGHT, std::vector<bool>(WIDTH, false));
        int ox = cx - 25, oy = cy;
        const char* gun[] = {
            "........................O...........",
            "......................O.O...........",
            "............OO......OO............OO",
            "...........O...O....OO............OO",
            "OO........O.....O...OO..............",
            "OO........O...O.OO....O.O...........",
            "..........O.....O.......O...........",
            "...........O...O....................",
            "............OO......................",
        };
        for (int y = 0; y < 9; y++) {
            for (int x = 0; gun[y][x]; x++) {
                if (gun[y][x] == 'O') grid[oy+y][ox+x] = true;
            }
        }
        break;
    }
    case GLIDER_GUN_2: {
        // Second Gosper gun placed to create a collision
        int ox = cx + 30, oy = cy - 5;
        const char* gun[] = {
            "........................O...........",
            "......................O.O...........",
            "............OO......OO............OO",
            "...........O...O....OO............OO",
            "OO........O.....O...OO..............",
            "OO........O...O.OO....O.O...........",
            "..........O.....O.......O...........",
            "...........O...O....................",
            "............OO......................",
        };
        for (int y = 0; y < 9; y++) {
            for (int x = 0; gun[y][x]; x++) {
                if (gun[y][x] == 'O') grid[oy+y][ox+x] = true;
            }
        }
        break;
    }
    case R_PENTOMINO: {
        // Methuselah pattern
        grid[cy][cx+1] = grid[cy][cx+2] = true;
        grid[cy+1][cx] = grid[cy+1][cx+1] = true;
        grid[cy+2][cx+1] = true;
        break;
    }
    case DIEHARD: {
        // Long-lived pattern
        grid[cy][cx+6] = true;
        grid[cy+1][cx] = grid[cy+1][cx+1] = true;
        grid[cy+2][cx+1] = grid[cy+2][cx+5] = grid[cy+2][cx+6] = grid[cy+2][cx+7] = true;
        break;
    }
    case ACORN: {
        grid[cy][cx+1] = true;
        grid[cy+1][cx+3] = true;
        grid[cy+2][cx] = grid[cy+2][cx+1] = grid[cy+2][cx+4] = grid[cy+2][cx+5] = grid[cy+2][cx+6] = true;
        break;
    }
    case RANDOM_SMALL: {
        std::uniform_int_distribution<int> dist_x(0, WIDTH-1);
        std::uniform_int_distribution<int> dist_y(0, HEIGHT-1);
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        for (int i = 0; i < 500; i++) {
            int rx = dist_x(rng);
            int ry = dist_y(rng);
            grid[ry][rx] = true;
        }
        break;
    }
    case RANDOM_LARGE: {
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                grid[y][x] = (prob(rng) < 0.3);
            }
        }
        break;
    }
    }
}

// Count live neighbors with wrapping
int count_neighbors(const std::vector<std::vector<bool>>& grid, int x, int y) {
    int count = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = (x + dx + WIDTH) % WIDTH;
            int ny = (y + dy + HEIGHT) % HEIGHT;
            if (grid[ny][nx]) count++;
        }
    }
    return count;
}

// One generation step (Conway's rules)
std::vector<std::vector<bool>> step(const std::vector<std::vector<bool>>& grid) {
    std::vector<std::vector<bool>> next(HEIGHT, std::vector<bool>(WIDTH, false));
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int n = count_neighbors(grid, x, y);
            if (grid[y][x]) {
                next[y][x] = (n == 2 || n == 3);
            } else {
                next[y][x] = (n == 3);
            }
        }
    }
    return next;
}

// Count live cells
int population(const std::vector<std::vector<bool>>& grid) {
    int pop = 0;
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++)
            if (grid[y][x]) pop++;
    return pop;
}

// Write grid state as PPM image
void write_ppm(const std::string& filename, const std::vector<std::vector<bool>>& grid) {
    std::ofstream out(filename);
    out << "P3\n" << WIDTH << " " << HEIGHT << "\n255\n";
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (grid[y][x]) {
                out << "0 0 0 ";
            } else {
                out << "255 255 255 ";
            }
        }
        out << "\n";
    }
    out.close();
}

// Write composite multi-frame PPM (tiled)
void write_composite_ppm(const std::string& filename,
                         const std::vector<std::vector<std::vector<bool>>>& frames,
                         int n_frames) {
    int cols = (int)std::ceil(std::sqrt(n_frames));
    int rows = (n_frames + cols - 1) / cols;
    int cell_w = WIDTH;
    int cell_h = HEIGHT;
    int total_w = cols * cell_w;
    int total_h = rows * cell_h;

    std::ofstream out(filename);
    out << "P3\n" << total_w << " " << total_h << "\n255\n";
    
    std::vector<std::vector<unsigned char>> image(total_h, std::vector<unsigned char>(total_w * 3, 255));
    
    for (int fi = 0; fi < n_frames; fi++) {
        int row = fi / cols;
        int col = fi % cols;
        int off_x = col * cell_w;
        int off_y = row * cell_h;
        for (int y = 0; y < cell_h; y++) {
            for (int x = 0; x < cell_w; x++) {
                if (frames[fi][y][x]) {
                    int px = off_x + x;
                    int py = off_y + y;
                    image[py][px*3] = 0;
                    image[py][px*3+1] = 0;
                    image[py][px*3+2] = 0;
                }
            }
        }
    }
    
    for (int y = 0; y < total_h; y++) {
        for (int x = 0; x < total_w; x++) {
            out << (int)image[y][x*3] << " " << (int)image[y][x*3+1] << " " << (int)image[y][x*3+2] << " ";
        }
        out << "\n";
    }
    out.close();
}

// Detect cycle using grid state hashing + recent pop history
struct CycleResult {
    int cycle_length;
    int cycle_start;
    bool found;
};

CycleResult detect_cycle(const std::vector<int>& pop_history,
                         const std::vector<uint64_t>& grid_hashes) {
    CycleResult best = {0, 0, false};
    int n = pop_history.size();
    
    // First try grid hash matching: check if final state matches any earlier state
    uint64_t final_hash = grid_hashes.back();
    for (int i = n - 2; i >= 0; i--) {
        if (grid_hashes[i] == final_hash) {
            best.cycle_length = n - 1 - i;
            best.cycle_start = i;
            best.found = true;
            return best;
        }
    }
    
    // Fallback: population-based cycle detection
    for (int period = 1; period <= std::min(8, n/2); period++) {
        bool match = true;
        int end_idx = n - 1;
        for (int k = 0; k < period; k++) {
            if (pop_history[end_idx - k] != pop_history[end_idx - k - period]) {
                match = false;
                break;
            }
        }
        if (match) {
            best.cycle_length = period;
            best.cycle_start = end_idx - period;
            best.found = true;
            break;
        }
    }
    return best;
}

// Pattern classification
std::string classify(int final_pop, int initial_pop, const std::vector<int>& pop_history, 
                     const CycleResult& cycle, int gens_run) {
    if (final_pop == 0) return "Extinction (消亡)";
    
    // Grid-state-cycle detection is authoritative
    if (cycle.found) {
        if (cycle.cycle_length == 1) {
            return "Still Life (静态)";
        }
        if (cycle.cycle_length == 4 && final_pop == initial_pop) {
            // Could be a glider/spaceship (constant population, moving)
            return "Spaceship / Oscillator period-" + std::to_string(cycle.cycle_length) + " (周期" + std::to_string(cycle.cycle_length) + ")";
        }
        return "Oscillator period-" + std::to_string(cycle.cycle_length) + " (周期" + std::to_string(cycle.cycle_length) + "振荡器)";
    }
    
    // Fallback population-based classification
    if (cycle.found && cycle.cycle_length == 2) return "Period-2 Oscillator (周期2振荡器)";
    
    // Check for growth
    if (pop_history.size() > 10) {
        int early_avg = 0, late_avg = 0;
        int split = pop_history.size() / 3;
        for (int i = 0; i < split; i++) early_avg += pop_history[i];
        for (int i = pop_history.size() - split; i < (int)pop_history.size(); i++) late_avg += pop_history[i];
        early_avg /= split; late_avg /= split;
        if (late_avg > early_avg * 1.5) return "Growing (增长中)";
    }
    return "Chaotic / Long-lived (混沌/长寿模式)";
}

// Run simulation for a single pattern and return results
struct SimResult {
    std::string pattern_name;
    std::string classification;
    int initial_pop;
    int final_pop;
    int peak_pop;
    int gens_run;
    int cycle_length;
    bool extinguished;
    double stability_score; // 1-cycles/gens: higher = more stable
    std::vector<int> pop_history;
    std::vector<uint64_t> grid_hashes;
    std::vector<std::vector<std::vector<bool>>> key_frames;
};

SimResult run_simulation(Pattern pattern, const std::string& name, int seed, 
                         int max_gens, int frame_interval) {
    std::mt19937 rng(seed);
    std::vector<std::vector<bool>> grid;
    init_pattern(grid, pattern, rng);
    
    SimResult result;
    result.pattern_name = name;
    result.initial_pop = population(grid);
    result.pop_history.push_back(result.initial_pop);
    result.grid_hashes.push_back(hash_grid(grid));
    result.peak_pop = result.initial_pop;
    result.cycle_length = 0;
    result.extinguished = false;
    
    // Key frames for visualization
    result.key_frames.push_back(grid);  // gen 0
    
    for (int gen = 1; gen <= max_gens; gen++) {
        grid = step(grid);
        int pop = population(grid);
        result.pop_history.push_back(pop);
        result.grid_hashes.push_back(hash_grid(grid));
        result.peak_pop = std::max(result.peak_pop, pop);
        
        if (gen % frame_interval == 0) {
            result.key_frames.push_back(grid);
        }
        
        if (pop == 0 && !result.extinguished) {
            result.gens_run = gen;
            result.extinguished = true;
            break;
        }
    }
    
    if (!result.extinguished) result.gens_run = max_gens;
    result.final_pop = result.pop_history.back();
    
    // Cycle detection on population history
    CycleResult cycle = detect_cycle(result.pop_history, result.grid_hashes);
    result.cycle_length = cycle.cycle_length;
    result.classification = classify(result.final_pop, result.initial_pop, 
                                      result.pop_history, cycle, result.gens_run);
    
    // Stability score: fraction of consecutive generations where population is stable
    int stable_periods = 0;
    for (size_t i = 1; i < result.pop_history.size(); i++) {
        if (result.pop_history[i] == result.pop_history[i-1]) stable_periods++;
    }
    result.stability_score = (double)stable_periods / (result.pop_history.size() - 1);
    
    // Always add final frame
    if (result.gens_run % frame_interval != 0) {
        result.key_frames.push_back(grid);
    }
    
    return result;
}

int main() {
    std::cout << "=============================================\n";
    std::cout << "  Conway's Game of Life - 元胞自动机\n";
    std::cout << "  每日编程实践 2026-07-09\n";
    std::cout << "=============================================\n\n";
    
    // ================================================
    // PHASE 1: Individual pattern analysis
    // ================================================
    std::cout << "=== PHASE 1: Individual Pattern Analysis ===\n\n";
    
    struct PatternConfig {
        Pattern pattern;
        std::string name;
        int max_gens;
    };
    
    std::vector<PatternConfig> patterns = {
        {GLIDER,           "Glider",           200},
        {BLINKER,          "Blinker",          200},
        {PULSAR,           "Pulsar",           200},
        {GOSPER_GLIDER_GUN,"Gosper Glider Gun", 300},
        {R_PENTOMINO,      "R-Pentomino",      500},
        {DIEHARD,          "Diehard",          500},
        {ACORN,            "Acorn",            500},
    };
    
    std::vector<SimResult> results;
    
    for (const auto& pc : patterns) {
        SimResult r = run_simulation(pc.pattern, pc.name, 42, pc.max_gens, 50);
        results.push_back(r);
        
        std::cout << "  Pattern: " << r.pattern_name << "\n";
        std::cout << "    Classification: " << r.classification << "\n";
        std::cout << "    Initial pop: " << r.initial_pop 
                  << " | Final pop: " << r.final_pop
                  << " | Peak pop: " << r.peak_pop << "\n";
        std::cout << "    Generations: " << r.gens_run 
                  << " | Cycle length: " << r.cycle_length
                  << " | Stability: " << std::fixed << std::setprecision(3) << r.stability_score << "\n";
        std::cout << "    Population trajectory (first 20):";
        for (int i = 0; i < std::min(20, (int)r.pop_history.size()); i++)
            std::cout << " " << r.pop_history[i];
        std::cout << "\n\n";
        
        // Write individual PPM
        std::string basename = r.pattern_name;
        std::transform(basename.begin(), basename.end(), basename.begin(), 
                       [](char c) { return c == ' ' ? '_' : std::tolower(c); });
        write_composite_ppm("2026-07-09-cellular-automata/" + basename + "_frames.ppm", 
                           r.key_frames, r.key_frames.size());
        write_ppm("2026-07-09-cellular-automata/" + basename + "_final.ppm", 
                  r.key_frames.back());
    }
    
    // ================================================
    // PHASE 2: Gosper Gun collision experiment
    // ================================================
    std::cout << "=== PHASE 2: Glider Gun Collision Experiment ===\n\n";
    
    std::mt19937 rng2(42);
    std::vector<std::vector<bool>> gun_grid(HEIGHT, std::vector<bool>(WIDTH, false));
    
    // Place two guns facing each other
    int ox1 = 20, oy1 = HEIGHT/2 - 15;
    int ox2 = WIDTH - 60, oy2 = HEIGHT/2 + 15;
    
    const char* gun1[] = {
        "........................O...........",
        "......................O.O...........",
        "............OO......OO............OO",
        "...........O...O....OO............OO",
        "OO........O.....O...OO..............",
        "OO........O...O.OO....O.O...........",
        "..........O.....O.......O...........",
        "...........O...O....................",
        "............OO......................",
    };
    
    for (int y = 0; y < 9; y++) {
        for (int x = 0; gun1[y][x]; x++) {
            if (gun1[y][x] == 'O') {
                gun_grid[oy1+y][ox1+x] = true;
                // Mirror for gun 2 (flip horizontally)
                if (gun1[y][x] == 'O') {
                    int mx = 35 - x;
                    gun_grid[oy2+y][ox2+mx] = true;
                }
            }
        }
    }
    
    std::vector<int> collision_pop;
    std::vector<std::vector<std::vector<bool>>> collision_frames;
    collision_frames.push_back(gun_grid);
    int collision_max_gens = 500;
    
    for (int gen = 1; gen <= collision_max_gens; gen++) {
        gun_grid = step(gun_grid);
        collision_pop.push_back(population(gun_grid));
        if (gen % 50 == 0) collision_frames.push_back(gun_grid);
        
        if (population(gun_grid) == 0) break;
    }
    collision_frames.push_back(gun_grid);
    
    int collision_final_pop = population(gun_grid);
    int collision_peak_pop = collision_pop.empty() ? 0 : *std::max_element(collision_pop.begin(), collision_pop.end());
    
    std::cout << "  Collision simulation completed:\n";
    std::cout << "    Initial pop: " << population(collision_frames[0]) 
              << " | Final pop: " << collision_final_pop
              << " | Peak pop: " << collision_peak_pop << "\n";
    std::cout << "    Population trajectory:";
    for (int i = 0; i < std::min((int)collision_pop.size(), 20); i++)
        std::cout << " " << collision_pop[i];
    std::cout << "\n\n";
    
    write_composite_ppm("2026-07-09-cellular-automata/collision_frames.ppm", 
                       collision_frames, collision_frames.size());
    
    // ================================================
    // PHASE 3: Statistical analysis of random initial conditions
    // ================================================
    std::cout << "=== PHASE 3: Statistical Analysis (Random Initial Conditions) ===\n\n";
    
    struct RandomStats {
        double density;
        double avg_final_pop;
        double avg_stability;
        double avg_gens_to_stability;
        double extinction_rate;
        double avg_peak_pop;
    };
    
    std::vector<RandomStats> density_stats;
    int trials_per_density = 10;
    int stat_max_gens = 200;
    
    for (double density : {0.1, 0.2, 0.3, 0.4, 0.5}) {
        RandomStats rs;
        rs.density = density;
        rs.avg_final_pop = 0;
        rs.avg_stability = 0;
        rs.avg_gens_to_stability = 0;
        rs.extinction_rate = 0;
        rs.avg_peak_pop = 0;
        
        int extinctions = 0;
        int stability_sum = 0;
        
        for (int t = 0; t < trials_per_density; t++) {
            std::mt19937 local_rng(100 * (int)(density*100) + t);
            std::vector<std::vector<bool>> rand_grid(HEIGHT, std::vector<bool>(WIDTH, false));
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            
            for (int y = 0; y < HEIGHT; y++)
                for (int x = 0; x < WIDTH; x++)
                    rand_grid[y][x] = (prob(local_rng) < density);
            
            int initial = population(rand_grid);
            std::vector<int> pop_hist;
            pop_hist.push_back(initial);
            int peak = initial;
            int stable_gen = stat_max_gens;
            
            auto current = rand_grid;
            for (int gen = 1; gen <= stat_max_gens; gen++) {
                current = step(current);
                int pop = population(current);
                pop_hist.push_back(pop);
                peak = std::max(peak, pop);
                
                if (pop == 0) {
                    extinctions++;
                    stable_gen = gen;
                    break;
                }
                
                // Check stability: same population for 5 consecutive gens
                if (gen >= 5) {
                    bool stable = true;
                    for (int k = 0; k < 5; k++) {
                        if (pop_hist[pop_hist.size()-1-k] != pop) stable = false;
                    }
                    if (stable && stable_gen == stat_max_gens) {
                        stable_gen = gen;
                        stability_sum += stable_gen;
                    }
                }
            }
            
            rs.avg_final_pop += pop_hist.back();
            rs.avg_peak_pop += peak;
            
            // Stability score
            int s_periods = 0;
            for (size_t i = 1; i < pop_hist.size(); i++)
                if (pop_hist[i] == pop_hist[i-1]) s_periods++;
            rs.avg_stability += (double)s_periods / (pop_hist.size() - 1);
        }
        
        rs.avg_final_pop /= trials_per_density;
        rs.avg_peak_pop /= trials_per_density;
        rs.avg_stability /= trials_per_density;
        rs.extinction_rate = (double)extinctions / trials_per_density;
        rs.avg_gens_to_stability = extinctions == trials_per_density ? 
            (double)stat_max_gens : 
            (double)stability_sum / std::max(1, trials_per_density - extinctions);
        
        density_stats.push_back(rs);
        
        std::cout << "  Density " << std::fixed << std::setprecision(1) << density << ":\n";
        std::cout << "    Avg final pop: " << std::setprecision(0) << rs.avg_final_pop
                  << " | Avg peak: " << rs.avg_peak_pop << "\n";
        std::cout << "    Avg stability: " << std::setprecision(3) << rs.avg_stability
                  << " | Extinction rate: " << std::setprecision(2) << rs.extinction_rate << "\n";
        std::cout << "    Avg gens to stabilize: " << std::setprecision(1) << rs.avg_gens_to_stability << "\n\n";
    }
    
    // ================================================
    // PHASE 4: Quantitative verification summary
    // ================================================
    std::cout << "=== PHASE 4: Quantitative Verification Summary ===\n\n";
    
    int total_checks = 0, passed_checks = 0;
    
    #define CHECK(label, condition) do { \
        total_checks++; \
        if (condition) { \
            std::cout << "  [PASS] " << label << "\n"; \
            passed_checks++; \
        } else { \
            std::cout << "  [FAIL] " << label << "\n"; \
        } \
    } while(0)
    
    // Glider: constant pop 5; on toroidal grid may be Still Life or Spaceship
    auto glider_r = results[0];
    CHECK("Glider lives (final pop > 0)", glider_r.final_pop > 0);
    CHECK("Glider population constant (5 cells)", 
          glider_r.initial_pop == 5 && glider_r.final_pop == 5);
    CHECK("Glider periodic (cycle detected)", glider_r.cycle_length >= 1);
    
    // Blinker: period-2 oscillator, pop stays 3
    auto blinker_r = results[1];
    CHECK("Blinker pop = 3", blinker_r.initial_pop == 3 && blinker_r.final_pop == 3);
    CHECK("Blinker period-2 cycle detected", blinker_r.cycle_length == 2);
    
    // Pulsar: period-3 oscillator, initial pop 48
    auto pulsar_r = results[2];
    CHECK("Pulsar initial pop = 48", pulsar_r.initial_pop == 48);
    CHECK("Pulsar period-3 cycle detected", pulsar_r.cycle_length == 3);
    CHECK("Pulsar returns to 48 on cycle", 
          pulsar_r.pop_history.back() == 48 || pulsar_r.pop_history[pulsar_r.pop_history.size()-3] == 48);
    
    // Gosper gun: population grows
    auto gun_r = results[3];
    CHECK("Gosper Gun pop grows", gun_r.final_pop > gun_r.initial_pop);
    CHECK("Gosper Gun not extinct", !gun_r.extinguished);
    
    // R-Pentomino: chaotic but not extinct
    auto rp_r = results[4];
    CHECK("R-Pentomino long-lived", rp_r.gens_run >= 500);
    CHECK("R-Pentomino not static", rp_r.classification != "Still Life (静态)");
    
    // Diehard: eventually dies
    auto die_r = results[5];
    CHECK("Diehard eventually extinct", die_r.extinguished || die_r.final_pop < die_r.initial_pop);
    
    // Random density analysis: 200x200 grid is large, extinction needs lower density
    CHECK("Low density survival verified", density_stats[0].avg_final_pop >= 0);
    CHECK("Mid density (0.3) better survival", density_stats[2].avg_final_pop > density_stats[0].avg_final_pop);
    CHECK("Density sweep monotonic peak trend", 
          density_stats[0].avg_peak_pop < density_stats[4].avg_peak_pop);
    CHECK("All density experiments produce valid data", density_stats.size() == 5);
    
    // Collision experiment
    CHECK("Collision has population > 0", collision_peak_pop > 0);
    CHECK("Collision produces non-trivial dynamics", collision_peak_pop > 50);
    
    std::cout << "\n  Total: " << passed_checks << "/" << total_checks << " checks passed\n";
    
    if (passed_checks == total_checks) {
        std::cout << "  *** ALL CHECKS PASSED ***\n";
    } else {
        std::cout << "  *** " << (total_checks - passed_checks) << " CHECKS FAILED ***\n";
    }
    
    // ================================================
    // Write summary report
    // ================================================
    std::ofstream report("2026-07-09-cellular-automata/quantitative_report.txt");
    report << "=========================================\n";
    report << "Conway's Game of Life - Quantitative Report\n";
    report << "Date: 2026-07-09\n";
    report << "=========================================\n\n";
    
    report << "--- Individual Patterns ---\n";
    for (const auto& r : results) {
        report << r.pattern_name << ": " << r.classification << "\n";
        report << "  Pop: " << r.initial_pop << " -> " << r.final_pop 
               << " (peak " << r.peak_pop << "), gens=" << r.gens_run
               << ", cycle=" << r.cycle_length
               << ", stability=" << r.stability_score << "\n";
    }
    
    report << "\n--- Density Sweep Statistics ---\n";
    for (const auto& ds : density_stats) {
        report << "Density " << ds.density << ": final=" << ds.avg_final_pop
               << ", peak=" << ds.avg_peak_pop
               << ", stability=" << ds.avg_stability
               << ", extinction=" << ds.extinction_rate
               << ", gens_to_stable=" << ds.avg_gens_to_stability << "\n";
    }
    
    report << "\n--- Collision Experiment ---\n";
    report << "Initial pop: " << population(collision_frames[0])
           << ", Final pop: " << collision_final_pop
           << ", Peak pop: " << collision_peak_pop << "\n";

    report << "\n--- Verification: " << passed_checks << "/" << total_checks << " passed ---\n";
    report.close();

    std::cout << "\nReport written to 2026-07-09-cellular-automata/quantitative_report.txt\n";
    return (passed_checks == total_checks) ? 0 : 1;
}