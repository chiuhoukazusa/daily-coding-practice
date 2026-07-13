/**
 * 每日编程实践 - 2026-06-26
 * Genetic Algorithm for Function Optimization
 * 
 * 核心技术: 遗传算法 (Genetic Algorithm)
 * - 锦标赛选择 (Tournament Selection)
 * - 模拟二进制交叉 (SBX - Simulated Binary Crossover)
 * - 多项式变异 (Polynomial Mutation)
 * - 精英保留 (Elitism)
 * 
 * 目标: 优化多维 Rastrigin 函数，寻找全局最小值
 *        f(x) = An + Σ[x_i² - A*cos(2π*x_i)], where A=10
 *        Global minimum: f(0,0,...,0) = 0
 * 
 * 量化验证:
 * - 每一代的最佳适应度和种群平均适应度
 * - 收敛速度（达到接近最优值的代数）
 * - 多次运行统计（均值、标准差）
 * - 与理论最优解的误差
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

// ============================================================
// Configuration
// ============================================================
#define POPULATION_SIZE 200
#define DIMENSIONS 10           // 10维优化问题
#define MAX_GENERATIONS 500
#define TOURNAMENT_SIZE 5
#define CROSSOVER_PROB 0.9
#define MUTATION_PROB (1.0 / DIMENSIONS)
#define MUTATION_ETA 20.0       // 多项式变异的分布指数
#define CROSSOVER_ETA 15.0      // SBX的分布指数
#define ELITE_COUNT 4
#define LOWER_BOUND -5.12
#define UPPER_BOUND 5.12
#define NUM_RUNS 10             // 独立运行次数

// ============================================================
// Rastrigin Function (minimization)
// ============================================================
double rastrigin(const double *x, int dim) {
    double A = 10.0;
    double sum = A * dim;
    for (int i = 0; i < dim; i++) {
        sum += x[i] * x[i] - A * cos(2.0 * M_PI * x[i]);
    }
    return sum;
}

// ============================================================
// Individual structure
// ============================================================
typedef struct {
    double genes[DIMENSIONS];
    double fitness;
    double constraints_violation;
} Individual;

// ============================================================
// Random number utilities
// ============================================================
double rand_double(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

// ============================================================
// Initialize population randomly
// ============================================================
void init_population(Individual *pop, int size) {
    for (int i = 0; i < size; i++) {
        for (int d = 0; d < DIMENSIONS; d++) {
            pop[i].genes[d] = rand_double(LOWER_BOUND, UPPER_BOUND);
        }
        pop[i].fitness = rastrigin(pop[i].genes, DIMENSIONS);
        pop[i].constraints_violation = 0.0;
    }
}

// ============================================================
// Tournament Selection
// ============================================================
int tournament_select(const Individual *pop, int size) {
    int best_idx = rand() % size;
    double best_fit = pop[best_idx].fitness;
    
    for (int i = 1; i < TOURNAMENT_SIZE; i++) {
        int idx = rand() % size;
        if (pop[idx].fitness < best_fit) {
            best_idx = idx;
            best_fit = pop[idx].fitness;
        }
    }
    return best_idx;
}

// ============================================================
// Simulated Binary Crossover (SBX)
// ============================================================
void sbx_crossover(const double *parent1, const double *parent2,
                   double *child1, double *child2) {
    for (int d = 0; d < DIMENSIONS; d++) {
        if (rand_double(0, 1) <= 0.5) {
            double u = rand_double(0, 1);
            double beta;
            if (u <= 0.5) {
                beta = pow(2.0 * u, 1.0 / (CROSSOVER_ETA + 1.0));
            } else {
                beta = pow(1.0 / (2.0 * (1.0 - u)), 1.0 / (CROSSOVER_ETA + 1.0));
            }
            
            child1[d] = 0.5 * ((1 + beta) * parent1[d] + (1 - beta) * parent2[d]);
            child2[d] = 0.5 * ((1 - beta) * parent1[d] + (1 + beta) * parent2[d]);
        } else {
            child1[d] = parent1[d];
            child2[d] = parent2[d];
        }
        
        // Boundary clamp
        if (child1[d] < LOWER_BOUND) child1[d] = LOWER_BOUND;
        if (child1[d] > UPPER_BOUND) child1[d] = UPPER_BOUND;
        if (child2[d] < LOWER_BOUND) child2[d] = LOWER_BOUND;
        if (child2[d] > UPPER_BOUND) child2[d] = UPPER_BOUND;
    }
}

// ============================================================
// Polynomial Mutation
// ============================================================
void polynomial_mutation(double *individual) {
    for (int d = 0; d < DIMENSIONS; d++) {
        if (rand_double(0, 1) < MUTATION_PROB) {
            double u = rand_double(0, 1);
            double delta;
            double y = individual[d];
            double delta_max = UPPER_BOUND - LOWER_BOUND;
            
            if (u < 0.5) {
                double delta_q = pow(2.0 * u, 1.0 / (MUTATION_ETA + 1.0)) - 1.0;
                delta = delta_q;
            } else {
                double delta_q = 1.0 - pow(2.0 * (1.0 - u), 1.0 / (MUTATION_ETA + 1.0));
                delta = delta_q;
            }
            
            individual[d] = y + delta * delta_max;
            
            // Boundary clamp
            if (individual[d] < LOWER_BOUND) individual[d] = LOWER_BOUND;
            if (individual[d] > UPPER_BOUND) individual[d] = UPPER_BOUND;
        }
    }
}

// ============================================================
// Comparison function for qsort (ascending fitness)
// ============================================================
int compare_individuals(const void *a, const void *b) {
    double fa = ((const Individual *)a)->fitness;
    double fb = ((const Individual *)b)->fitness;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

// ============================================================
// Copy individual
// ============================================================
void copy_individual(Individual *dest, const Individual *src) {
    memcpy(dest->genes, src->genes, DIMENSIONS * sizeof(double));
    dest->fitness = src->fitness;
    dest->constraints_violation = src->constraints_violation;
}

// ============================================================
// Single run of genetic algorithm
// Returns convergence data
// ============================================================
typedef struct {
    double best_fitness_history[MAX_GENERATIONS];
    double avg_fitness_history[MAX_GENERATIONS];
    double diversity_history[MAX_GENERATIONS];
    int generations_to_converge;  // Generations to reach 1e-4
    double final_best_fitness;
    double final_avg_fitness;
    double final_best_solution[DIMENSIONS];
} RunResult;

RunResult run_ga(unsigned int seed) {
    srand(seed);
    RunResult result;
    memset(&result, 0, sizeof(RunResult));
    result.generations_to_converge = -1;  // Not converged
    
    Individual population[POPULATION_SIZE];
    Individual offspring[POPULATION_SIZE * 2 + POPULATION_SIZE];  // offspring + parent pool
    Individual next_population[POPULATION_SIZE];
    
    init_population(population, POPULATION_SIZE);
    
    for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
        // Sort by fitness
        qsort(population, POPULATION_SIZE, sizeof(Individual), compare_individuals);
        
        // Record statistics
        double sum_fitness = 0.0, min_fitness = population[0].fitness;
        for (int i = 0; i < POPULATION_SIZE; i++) {
            sum_fitness += population[i].fitness;
        }
        
        // Diversity: average pairwise Euclidean distance (sampled)
        double diversity = 0.0;
        int sample_count = 50;
        for (int s = 0; s < sample_count; s++) {
            int i = rand() % POPULATION_SIZE;
            int j = rand() % POPULATION_SIZE;
            double dist = 0.0;
            for (int d = 0; d < DIMENSIONS; d++) {
                double diff = population[i].genes[d] - population[j].genes[d];
                dist += diff * diff;
            }
            diversity += sqrt(dist);
        }
        diversity /= sample_count;
        
        result.best_fitness_history[gen] = min_fitness;
        result.avg_fitness_history[gen] = sum_fitness / POPULATION_SIZE;
        result.diversity_history[gen] = diversity;
        
        // Check convergence
        if (result.generations_to_converge < 0 && min_fitness < 1e-4) {
            result.generations_to_converge = gen;
        }
        
        // ====================================================
        // Generate offspring
        // ====================================================
        int offspring_count = 0;
        
        // Elitism: keep best individuals
        for (int e = 0; e < ELITE_COUNT; e++) {
            copy_individual(&offspring[offspring_count++], &population[e]);
        }
        
        while (offspring_count < POPULATION_SIZE * 2 - 2) {
            int p1_idx = tournament_select(population, POPULATION_SIZE);
            int p2_idx = tournament_select(population, POPULATION_SIZE);
            
            // Ensure different parents
            while (p2_idx == p1_idx) {
                p2_idx = tournament_select(population, POPULATION_SIZE);
            }
            
            double child1[DIMENSIONS], child2[DIMENSIONS];
            
            if (rand_double(0, 1) < CROSSOVER_PROB) {
                sbx_crossover(population[p1_idx].genes, population[p2_idx].genes,
                             child1, child2);
            } else {
                memcpy(child1, population[p1_idx].genes, DIMENSIONS * sizeof(double));
                memcpy(child2, population[p2_idx].genes, DIMENSIONS * sizeof(double));
            }
            
            polynomial_mutation(child1);
            polynomial_mutation(child2);
            
            memcpy(offspring[offspring_count].genes, child1, DIMENSIONS * sizeof(double));
            offspring[offspring_count].fitness = rastrigin(child1, DIMENSIONS);
            offspring_count++;
            
            memcpy(offspring[offspring_count].genes, child2, DIMENSIONS * sizeof(double));
            offspring[offspring_count].fitness = rastrigin(child2, DIMENSIONS);
            offspring_count++;
        }
        
        // ====================================================
        // Selection: keep best POPULATION_SIZE from combined pool
        // ====================================================
        // Copy current population to the combined pool
        for (int i = 0; i < POPULATION_SIZE; i++) {
            copy_individual(&offspring[offspring_count + i], &population[i]);
        }
        int total_pool = offspring_count + POPULATION_SIZE;
        
        qsort(offspring, total_pool, sizeof(Individual), compare_individuals);
        
        // Select best POPULATION_SIZE
        for (int i = 0; i < POPULATION_SIZE; i++) {
            copy_individual(&next_population[i], &offspring[i]);
        }
        
        // Replace population
        memcpy(population, next_population, POPULATION_SIZE * sizeof(Individual));
    }
    
    // Final sort
    qsort(population, POPULATION_SIZE, sizeof(Individual), compare_individuals);
    
    result.final_best_fitness = population[0].fitness;
    for (int d = 0; d < DIMENSIONS; d++) {
        result.final_best_solution[d] = population[0].genes[d];
    }
    
    double final_sum = 0.0;
    for (int i = 0; i < POPULATION_SIZE; i++) {
        final_sum += population[i].fitness;
    }
    result.final_avg_fitness = final_sum / POPULATION_SIZE;
    
    return result;
}

// ============================================================
// Compute statistical summary
// ============================================================
typedef struct {
    double mean_best;
    double std_best;
    double mean_converged_gen;
    double std_converged_gen;
    int converged_count;
    double mean_diversity_final;
    double std_diversity_final;
} Statistics;

Statistics compute_statistics(RunResult *runs, int num_runs) {
    Statistics stats;
    memset(&stats, 0, sizeof(Statistics));
    
    double sum_best = 0.0, sum_sq_best = 0.0;
    double sum_gen = 0.0, sum_sq_gen = 0.0;
    double sum_div = 0.0, sum_sq_div = 0.0;
    int converged = 0;
    
    for (int i = 0; i < num_runs; i++) {
        sum_best += runs[i].final_best_fitness;
        sum_sq_best += runs[i].final_best_fitness * runs[i].final_best_fitness;
        sum_div += runs[i].diversity_history[MAX_GENERATIONS - 1];
        sum_sq_div += runs[i].diversity_history[MAX_GENERATIONS - 1] * 
                      runs[i].diversity_history[MAX_GENERATIONS - 1];
        
        if (runs[i].generations_to_converge >= 0) {
            converged++;
            sum_gen += runs[i].generations_to_converge;
            sum_sq_gen += (double)runs[i].generations_to_converge * 
                          runs[i].generations_to_converge;
        }
    }
    
    double n = (double)num_runs;
    stats.mean_best = sum_best / n;
    stats.std_best = sqrt((sum_sq_best / n) - (stats.mean_best * stats.mean_best));
    
    if (converged > 0) {
        stats.mean_converged_gen = sum_gen / converged;
        stats.std_converged_gen = sqrt((sum_sq_gen / converged) - 
                                        (stats.mean_converged_gen * stats.mean_converged_gen));
    }
    stats.converged_count = converged;
    
    stats.mean_diversity_final = sum_div / n;
    stats.std_diversity_final = sqrt((sum_sq_div / n) - 
                                      (stats.mean_diversity_final * stats.mean_diversity_final));
    
    return stats;
}

// ============================================================
// Main
// ============================================================
int main() {
    printf("========================================\n");
    printf(" Genetic Algorithm for Optimization\n");
    printf(" Daily Coding Practice - 2026-06-26\n");
    printf("========================================\n\n");
    
    printf("Problem: %d-dimensional Rastrigin function\n", DIMENSIONS);
    printf("Global minimum: f(0,0,...,0) = 0\n");
    printf("Population size: %d\n", POPULATION_SIZE);
    printf("Max generations: %d\n", MAX_GENERATIONS);
    printf("Runs: %d\n\n", NUM_RUNS);
    
    // Run multiple independent trials
    RunResult runs[NUM_RUNS];
    for (int r = 0; r < NUM_RUNS; r++) {
        printf("Run %2d/%2d ... ", r + 1, NUM_RUNS);
        fflush(stdout);
        runs[r] = run_ga((unsigned int)(time(NULL) + r * 9973));
        printf("Best fitness = %.6e", runs[r].final_best_fitness);
        if (runs[r].generations_to_converge >= 0) {
            printf(" (converged at gen %d)", runs[r].generations_to_converge);
        }
        printf("\n");
    }
    
    // Compute statistics
    Statistics stats = compute_statistics(runs, NUM_RUNS);
    
    printf("\n========================================\n");
    printf(" Statistical Summary\n");
    printf("========================================\n");
    printf("Best fitness:     mean=%.6e  std=%.6e\n", stats.mean_best, stats.std_best);
    printf("Converged runs:   %d / %d (%.1f%%)\n", 
           stats.converged_count, NUM_RUNS, 
           100.0 * stats.converged_count / NUM_RUNS);
    if (stats.converged_count > 0) {
        printf("Gen to converge:  mean=%.1f  std=%.1f\n", 
               stats.mean_converged_gen, stats.std_converged_gen);
    }
    printf("Final diversity:  mean=%.6f  std=%.6f\n", 
           stats.mean_diversity_final, stats.std_diversity_final);
    
    // Best run details
    int best_run_idx = 0;
    for (int i = 1; i < NUM_RUNS; i++) {
        if (runs[i].final_best_fitness < runs[best_run_idx].final_best_fitness) {
            best_run_idx = i;
        }
    }
    
    printf("\n========================================\n");
    printf(" Best Run (Run %d) Solution\n", best_run_idx + 1);
    printf("========================================\n");
    printf("Fitness: %.10e\n", runs[best_run_idx].final_best_fitness);
    printf("Variables:\n");
    for (int d = 0; d < DIMENSIONS; d++) {
        printf("  x[%2d] = %+.8e  (deviation: %.2e)\n", 
               d, runs[best_run_idx].final_best_solution[d],
               fabs(runs[best_run_idx].final_best_solution[d]));
    }
    
    // Print convergence data for plotting
    printf("\n========================================\n");
    printf(" Convergence Data (Best Run)\n");
    printf("========================================\n");
    printf("Gen,BestFitness,AvgFitness,Diversity\n");
    for (int gen = 0; gen < MAX_GENERATIONS; gen += 10) {
        printf("%d,%.6e,%.6e,%.6f\n", 
               gen,
               runs[best_run_idx].best_fitness_history[gen],
               runs[best_run_idx].avg_fitness_history[gen],
               runs[best_run_idx].diversity_history[gen]);
    }
    
    // Generate a simple PPM visualization of convergence
    printf("\n========================================\n");
    printf(" Generating Convergence Plot...\n");
    printf("========================================\n");
    
    int plot_width = 800, plot_height = 400;
    FILE *f = fopen("convergence_plot.ppm", "w");
    fprintf(f, "P3\n%d %d\n255\n", plot_width, plot_height);
    
    // Get log-scale max fitness for the plot
    double max_log_fitness = log10(runs[best_run_idx].best_fitness_history[0] + 1.0);
    double log_start = log10(runs[best_run_idx].best_fitness_history[0] + 1.0);
    
    unsigned char *pixels = malloc(plot_width * plot_height * 3);
    memset(pixels, 255, plot_width * plot_height * 3); // White background
    
    // Draw grid lines
    for (int x = 0; x < plot_width; x++) {
        int gen_idx = (x * MAX_GENERATIONS) / plot_width;
        if (gen_idx % 50 == 0) {
            for (int y = 0; y < plot_height; y++) {
                int idx = (y * plot_width + x) * 3;
                pixels[idx] = 230; pixels[idx+1] = 230; pixels[idx+2] = 230;
            }
        }
    }
    for (int y = 0; y < plot_height; y++) {
        double log_fit = log10(pow(10, max_log_fitness) * (1.0 - (double)y / plot_height) + 1.0);
        if (log_fit < 0.1) {
            for (int x = 0; x < plot_width; x++) {
                int idx = (y * plot_width + x) * 3;
                pixels[idx] = 230; pixels[idx+1] = 230; pixels[idx+2] = 230;
            }
        }
    }
    
    // Plot average fitness (blue)
    for (int x = 0; x < plot_width; x++) {
        int gen_idx = (x * MAX_GENERATIONS) / plot_width;
        if (gen_idx >= MAX_GENERATIONS) gen_idx = MAX_GENERATIONS - 1;
        
        double avg_fit = runs[best_run_idx].avg_fitness_history[gen_idx];
        double log_avg = log10(avg_fit + 1.0);
        double norm = log_avg / log_start;
        if (norm > 1.0) norm = 1.0;
        if (norm < 0.0) norm = 0.0;
        
        int y_plot = (int)((1.0 - norm) * plot_height);
        if (y_plot < 0) y_plot = 0;
        if (y_plot >= plot_height) y_plot = plot_height - 1;
        
        // Draw line with thickness
        for (int dy = -1; dy <= 1; dy++) {
            int y = y_plot + dy;
            if (y >= 0 && y < plot_height) {
                int idx = (y * plot_width + x) * 3;
                pixels[idx] = 50; pixels[idx+1] = 100; pixels[idx+2] = 200;
            }
        }
    }
    
    // Plot best fitness (red) on same chart
    for (int x = 0; x < plot_width; x++) {
        int gen_idx = (x * MAX_GENERATIONS) / plot_width;
        if (gen_idx >= MAX_GENERATIONS) gen_idx = MAX_GENERATIONS - 1;
        
        double best_fit = runs[best_run_idx].best_fitness_history[gen_idx];
        double log_best = best_fit > 0.0 ? log10(best_fit + 1.0) : 0.0;
        double norm = log_best / log_start;
        if (norm > 1.0) norm = 1.0;
        if (norm < 0.0) norm = 0.0;
        
        int y_plot = (int)((1.0 - norm) * plot_height);
        if (y_plot < 0) y_plot = 0;
        if (y_plot >= plot_height) y_plot = plot_height - 1;
        
        int idx = (y_plot * plot_width + x) * 3;
        pixels[idx] = 220; pixels[idx+1] = 30; pixels[idx+2] = 30;
    }
    
    // Write pixels
    for (int i = 0; i < plot_width * plot_height; i++) {
        fprintf(f, "%d %d %d\n", pixels[i*3], pixels[i*3+1], pixels[i*3+2]);
    }
    
    free(pixels);
    fclose(f);
    printf("Saved convergence_plot.ppm\n");
    
    // ========================================================
    // Validation: Random search baseline comparison
    // ========================================================
    printf("\n========================================\n");
    printf(" Random Search Baseline\n");
    printf("========================================\n");
    
    int random_evals = POPULATION_SIZE * MAX_GENERATIONS;
    double best_random = INFINITY;
    double random_points[DIMENSIONS];
    srand(time(NULL));
    
    for (int i = 0; i < random_evals; i++) {
        for (int d = 0; d < DIMENSIONS; d++) {
            random_points[d] = rand_double(LOWER_BOUND, UPPER_BOUND);
        }
        double fit = rastrigin(random_points, DIMENSIONS);
        if (fit < best_random) best_random = fit;
    }
    
    printf("Function evaluations: %d\n", random_evals);
    printf("Random search best:  %.6e\n", best_random);
    printf("GA best:             %.6e\n", stats.mean_best);
    printf("Improvement ratio:   %.2fx\n", best_random / stats.mean_best);
    
    // ========================================================
    // Quantitative verification summary
    // ========================================================
    printf("\n========================================\n");
    printf(" Quantitative Verification\n");
    printf("========================================\n");
    printf("✓ Distance to global optimum (f=0): %.6e\n", stats.mean_best);
    printf("✓ Convergence rate: %.1f%% runs converged\n", 
           100.0 * stats.converged_count / NUM_RUNS);
    printf("✓ Improvement over random: %.2fx\n", best_random / stats.mean_best);
    printf("✓ Stability (std/mean ratio): %.4f\n", 
           stats.mean_best > 0 ? stats.std_best / stats.mean_best : 0.0);
    
    if (stats.mean_best < 0.01) {
        printf("✅ VERIFIED: GA successfully optimizes Rastrigin function\n");
    } else {
        printf("⚠️  GA achieved suboptimal results (may need tuning)\n");
    }
    
    printf("\nDone.\n");
    return 0;
}
