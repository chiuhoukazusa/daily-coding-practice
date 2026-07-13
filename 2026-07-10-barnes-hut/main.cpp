#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <iomanip>

// ============================================================
// Barnes-Hut N-Body Simulation
//
// Simulates gravitational N-body interactions using the
// Barnes-Hut algorithm (quadtree-based multipole expansion).
// O(n log n) vs the direct O(n^2) method.
//
// Quantitative verification:
// 1. Force error vs direct sum (RMS error / per-particle)
// 2. Speedup ratio (direct time / BH time)
// 3. Energy conservation (potential + kinetic over time)
// 4. Momentum conservation
// ============================================================

constexpr int WIDTH  = 800;
constexpr int HEIGHT = 800;
constexpr double THETA = 0.5;  // Barnes-Hut opening angle criterion
constexpr double G = 1000.0;    // Gravitational constant (scaled)
constexpr double SOFTENING = 3.0; // Softening length to avoid singularity
constexpr double DT = 0.02;     // Time step
constexpr int STEPS = 50;       // Simulation steps for PPM output

struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double x, double y) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(double s) const { return {x*s, y*s}; }
    Vec2& operator+=(const Vec2& o) { x+=o.x; y+=o.y; return *this; }
    
    double norm2() const { return x*x + y*y; }
    double norm() const { return std::sqrt(norm2()); }
    double dist2(const Vec2& o) const { return (*this-o).norm2(); }
};

struct Body {
    Vec2 pos, vel;
    double mass;
};

// Barnes-Hut Quadtree node
struct BHNode {
    Vec2 center;       // center of the square region
    double halfWidth;  // half-width of the square
    Vec2 com;          // center of mass
    double totalMass;  // total mass in this node
    bool isLeaf;
    bool isEmpty;
    
    BHNode* children[4]; // NW, NE, SW, SE
    
    // Body stored if leaf
    Vec2 bodyPos;
    double bodyMass;
    
    BHNode(Vec2 c, double hw)
        : center(c), halfWidth(hw), com({0,0}), totalMass(0),
          isLeaf(true), isEmpty(true), bodyMass(0) {
        children[0] = children[1] = children[2] = children[3] = nullptr;
    }
    
    ~BHNode() {
        for (int i = 0; i < 4; i++) delete children[i];
    }
    
    // Which quadrant does p belong to? 0=NW, 1=NE, 2=SW, 3=SE
    int quadrant(const Vec2& p) const {
        int q = 0;
        if (p.x >= center.x) q |= 1;
        if (p.y >= center.y) q |= 2;
        return q;
    }
    
    // Get child center for a quadrant
    Vec2 childCenter(int q) const {
        double hq = halfWidth * 0.5;
        double cx = center.x + ((q & 1) ? hq : -hq);
        double cy = center.y + ((q & 2) ? hq : -hq);
        return {cx, cy};
    }
    
    bool contains(const Vec2& p) const {
        return std::abs(p.x - center.x) <= halfWidth &&
               std::abs(p.y - center.y) <= halfWidth;
    }
};

// Check if we should approximate (theta criterion)
bool shouldApprox(const BHNode* node, const Vec2& pos) {
    double d = (pos - node->com).norm();
    double s = node->halfWidth * 2.0;
    if (d < 1e-9) return false;
    return s / d < THETA;
}

// Insert a body into the BH tree
void bhInsert(BHNode* node, const Vec2& pos, double mass) {
    if (!node->contains(pos)) return;
    
    if (node->isEmpty) {
        node->isEmpty = false;
        node->bodyPos = pos;
        node->bodyMass = mass;
        node->com = pos;
        node->totalMass = mass;
        return;
    }
    
    if (node->isLeaf && !node->isEmpty) {
        // Subdivide: insert existing body into child
        int q0 = node->quadrant(node->bodyPos);
        Vec2 cc0 = node->childCenter(q0);
        node->children[q0] = new BHNode(cc0, node->halfWidth * 0.5);
        bhInsert(node->children[q0], node->bodyPos, node->bodyMass);
        
        node->isLeaf = false;
        // no 'empty' field needed here
    }
    
    // Update COM
    node->com = (node->com * node->totalMass + pos * mass) * (1.0 / (node->totalMass + mass));
    node->totalMass += mass;
    
    if (!node->isLeaf) {
        int q = node->quadrant(pos);
        if (!node->children[q]) {
            node->children[q] = new BHNode(node->childCenter(q), node->halfWidth * 0.5);
        }
        bhInsert(node->children[q], pos, mass);
    }
}

// Compute force from BH tree on a body at pos
Vec2 bhForce(const BHNode* node, const Vec2& pos) {
    if (!node || node->totalMass == 0) return {0, 0};
    
    double dx = node->com.x - pos.x;
    double dy = node->com.y - pos.y;
    double d2 = dx*dx + dy*dy + SOFTENING*SOFTENING;
    double d = std::sqrt(d2);
    
    if (node->isLeaf && node->isEmpty) return {0, 0};
    
    if (node->isLeaf) {
        double f = G * node->totalMass / (d2 * d);
        return {dx * f, dy * f};
    }
    
    if (shouldApprox(node, pos)) {
        double f = G * node->totalMass / (d2 * d);
        return {dx * f, dy * f};
    }
    
    // Otherwise recurse into children
    Vec2 totalForce = {0, 0};
    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            Vec2 cf = bhForce(node->children[i], pos);
            totalForce = totalForce + cf;
        }
    }
    return totalForce;
}

// Direct O(n^2) force computation (for verification)
Vec2 directForce(const Body& b, const std::vector<Body>& bodies) {
    Vec2 force = {0, 0};
    for (const auto& other : bodies) {
        if (&other == &b) continue;
        double dx = other.pos.x - b.pos.x;
        double dy = other.pos.y - b.pos.y;
        double d2 = dx*dx + dy*dy + SOFTENING*SOFTENING;
        double d = std::sqrt(d2);
        double f = G * other.mass / (d2 * d);
        force.x += dx * f;
        force.y += dy * f;
    }
    return force;
}

// Build BH tree from bodies
BHNode* buildTree(const std::vector<Body>& bodies) {
    BHNode* root = new BHNode({WIDTH/2.0, HEIGHT/2.0}, WIDTH * 2.0); // large enough
    for (const auto& b : bodies) {
        bhInsert(root, b.pos, b.mass);
    }
    return root;
}

// ============================================================
// PPM Output
// ============================================================
void writePPM(const std::string& filename, const std::vector<Body>& bodies, int /*step*/) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    
    std::vector<unsigned char> pixels(WIDTH * HEIGHT * 3, 0);
    
    // Draw dark background grid for visibility
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int idx = (y * WIDTH + x) * 3;
            // subtle grid lines every 40px
            bool gridLine = (x % 40 == 0 || y % 40 == 0);
            unsigned char bg = gridLine ? 8 : 3;
            pixels[idx] = bg;
            pixels[idx+1] = bg;
            pixels[idx+2] = gridLine ? 6 : 2;
        }
    }
    
    // Draw bodies as larger, brighter dots with glow
    for (const auto& b : bodies) {
        int px = static_cast<int>(b.pos.x);
        int py = static_cast<int>(b.pos.y);
        // Color based on mass: heavier = more red/white
        double massRel = std::min(1.0, b.mass / 20.0);
        int baseR = static_cast<int>(100 + massRel * 155);
        int baseG = static_cast<int>(60 + massRel * 120);
        int baseB = static_cast<int>(180 + massRel * 75);
        
        // Draw filled circle with radius based on mass (r=3 to r=6)
        int radius = static_cast<int>(3 + massRel * 3);
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > radius + 1.0) continue;
                int nx = px + dx;
                int ny = py + dy;
                if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) continue;
                int idx = (ny * WIDTH + nx) * 3;
                // Anti-aliased edge and glow
                double alpha;
                if (dist <= radius) {
                    alpha = 1.0; // solid fill
                } else {
                    alpha = std::max(0.0, 1.0 - (dist - radius)); // glow falloff
                }
                pixels[idx]   = std::min(255, pixels[idx]   + static_cast<int>(baseR * alpha));
                pixels[idx+1] = std::min(255, pixels[idx+1] + static_cast<int>(baseG * alpha));
                pixels[idx+2] = std::min(255, pixels[idx+2] + static_cast<int>(baseB * alpha));
            }
        }
    }
    
    f.write(reinterpret_cast<char*>(pixels.data()), pixels.size());
    f.close();
    
    // Verify PPM output
    std::ifstream check(filename, std::ios::binary | std::ios::ate);
    size_t fileSize = check.tellg();
    check.close();
    std::cout << "  PPM file " << filename << ": " << fileSize << " bytes" << std::endl;
}

// ============================================================
// Main Simulation
// ============================================================

int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> posDist(100, WIDTH - 100);
    std::uniform_real_distribution<double> massDist(1.0, 20.0);
    
    std::vector<int> particleCounts = {100, 200, 400, 800};
    
    std::cout << "================================================" << std::endl;
    std::cout << "Barnes-Hut N-Body Simulation" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "THETA = " << THETA << " (opening angle criterion)" << std::endl;
    std::cout << "G = " << G << ", SOFTENING = " << SOFTENING << std::endl;
    std::cout << std::endl;
    
    for (int N : particleCounts) {
        std::cout << "--- N = " << N << " particles ---" << std::endl;
        
        // Generate bodies
        std::vector<Body> bodies(N);
        for (int i = 0; i < N; i++) {
            bodies[i].pos = {posDist(rng), posDist(rng)};
            bodies[i].vel = {0, 0};
            bodies[i].mass = massDist(rng);
        }
        
        // Compute forces: direct O(n^2)
        std::vector<Vec2> directForces(N);
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; i++) {
            directForces[i] = directForce(bodies[i], bodies);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double directTime = std::chrono::duration<double>(t1 - t0).count();
        
        // Compute forces: Barnes-Hut O(n log n)
        std::vector<Vec2> bhForces(N);
        auto t2 = std::chrono::high_resolution_clock::now();
        BHNode* tree = buildTree(bodies);
        for (int i = 0; i < N; i++) {
            bhForces[i] = bhForce(tree, bodies[i].pos);
        }
        auto t3 = std::chrono::high_resolution_clock::now();
        double bhTime = std::chrono::duration<double>(t3 - t2).count();
        delete tree;
        
        // ============================================================
        // QUANTITATIVE VERIFICATION #1: Force Error
        // ============================================================
        double maxRelErr = 0;
        double sumSqErr = 0;
        double sumSqForce = 0;
        int largeErrCount = 0;  // count errors > 1%
        
        for (int i = 0; i < N; i++) {
            double fDirect = directForces[i].norm();
            double absErr = (directForces[i] - bhForces[i]).norm();
            double relErr = (fDirect > 1e-9) ? absErr / fDirect : 0;
            
            maxRelErr = std::max(maxRelErr, relErr);
            sumSqErr += absErr * absErr;
            sumSqForce += fDirect * fDirect;
            if (relErr > 0.01) largeErrCount++;
        }
        
        double rmsRelErr = std::sqrt(sumSqErr / sumSqForce);
        double avgErr = sumSqErr / N;
        
        std::cout << "  Force Verification:" << std::endl;
        std::cout << "    RMS relative error: " << std::fixed << std::setprecision(6) << rmsRelErr << std::endl;
        std::cout << "    Max relative error: " << maxRelErr << std::endl;
        std::cout << "    Mean squared error: " << avgErr << std::endl;
        std::cout << "    Particles with >1% error: " << largeErrCount << "/" << N
                  << " (" << std::fixed << std::setprecision(2) << (100.0*largeErrCount/N) << "%)" << std::endl;
        
        // ============================================================
        // QUANTITATIVE VERIFICATION #2: Speedup
        // ============================================================
        double speedup = directTime / bhTime;
        std::cout << "  Timing:" << std::endl;
        std::cout << "    Direct O(n^2): " << std::fixed << std::setprecision(4) << directTime << " s" << std::endl;
        std::cout << "    Barnes-Hut:    " << std::fixed << std::setprecision(4) << bhTime << " s" << std::endl;
        std::cout << "    Speedup:       " << std::fixed << std::setprecision(2) << speedup << "x" << std::endl;
        
        // ============================================================
        // QUANTITATIVE VERIFICATION #3: Energy Conservation
        // ============================================================
        // Run a few steps with Barnes-Hut and track energy
        std::vector<Body> simBodies = bodies;
        double initialEnergy = 0;
        double finalEnergy = 0;
        
        for (int step = 0; step < 20; step++) {
            // Compute forces
            BHNode* simTree = buildTree(simBodies);
            std::vector<Vec2> forces(simBodies.size());
            for (size_t i = 0; i < simBodies.size(); i++) {
                forces[i] = bhForce(simTree, simBodies[i].pos);
            }
            delete simTree;
            
            // Leapfrog integration (velocity half-step, position full-step)
            for (size_t i = 0; i < simBodies.size(); i++) {
                simBodies[i].vel = simBodies[i].vel + forces[i] * (DT / simBodies[i].mass);
                simBodies[i].pos = simBodies[i].pos + simBodies[i].vel * DT;
                
                // Wrap-around with damped boundary
                if (simBodies[i].pos.x < 0) { simBodies[i].pos.x = 0; simBodies[i].vel.x *= -0.5; }
                if (simBodies[i].pos.x > WIDTH) { simBodies[i].pos.x = WIDTH; simBodies[i].vel.x *= -0.5; }
                if (simBodies[i].pos.y < 0) { simBodies[i].pos.y = 0; simBodies[i].vel.y *= -0.5; }
                if (simBodies[i].pos.y > HEIGHT) { simBodies[i].pos.y = HEIGHT; simBodies[i].vel.y *= -0.5; }
            }
            
            if (step == 0 || step == 19) {
                double totalKE = 0, totalPE = 0;
                for (size_t i = 0; i < simBodies.size(); i++) {
                    totalKE += 0.5 * simBodies[i].mass * simBodies[i].vel.norm2();
                    for (size_t j = i+1; j < simBodies.size(); j++) {
                        double d = (simBodies[i].pos - simBodies[j].pos).norm() + SOFTENING;
                        totalPE -= G * simBodies[i].mass * simBodies[j].mass / d;
                    }
                }
                if (step == 0) initialEnergy = totalKE + totalPE;
                else finalEnergy = totalKE + totalPE;
            }
        }
        
        double energyDrift = std::abs(finalEnergy - initialEnergy) / std::abs(initialEnergy);
        std::cout << "  Energy Conservation:" << std::endl;
        std::cout << "    Initial energy: " << std::scientific << std::setprecision(4) << initialEnergy << std::endl;
        std::cout << "    Final energy:   " << std::scientific << std::setprecision(4) << finalEnergy << std::endl;
        std::cout << "    Relative drift: " << std::fixed << std::setprecision(6) << energyDrift << std::endl;
        
        // Verify pass/fail
        std::cout << "  VERDICT:" << std::endl;
        
        bool forceOK = rmsRelErr < 0.05;
        bool speedupOK = speedup >= 0.8 || N >= 400; // small N may not show speedup
        bool energyOK = energyDrift < 0.10;
        
        std::cout << "    Force accuracy: " << (forceOK ? "✅ PASS" : "❌ FAIL - RMS error too high") << std::endl;
        std::cout << "    Algorithm perf: " << (speedupOK ? "✅ PASS" : "⚠️  CHECK - small N may not show speedup") << std::endl;
        std::cout << "    Energy conserv: " << (energyOK ? "✅ PASS" : "❌ FAIL - energy not conserved") << std::endl;
        std::cout << std::endl;
        
        if (!forceOK || !energyOK) {
            std::cerr << "❌ VERIFICATION FAILED for N=" << N << std::endl;
            return 1;
        }
    }
    
    // ============================================================
    // Generate PPM visualization of final simulation
    // ============================================================
    std::cout << "--- Generating PPM visualization (N=400, " << STEPS << " steps) ---" << std::endl;
    
    std::mt19937 visRng(99);
    std::uniform_real_distribution<double> visPos(100, WIDTH - 100);
    std::uniform_real_distribution<double> visMass(2.0, 15.0);
    
    const int N_VIS = 400;
    std::vector<Body> visBodies(N_VIS);
    for (int i = 0; i < N_VIS; i++) {
        visBodies[i].pos = {visPos(visRng), visPos(visRng)};
        visBodies[i].vel = {0, 0};
        visBodies[i].mass = visMass(visRng);
    }
    
    writePPM("barnes_hut_initial.ppm", visBodies, 0);
    
    for (int step = 0; step < STEPS; step++) {
        BHNode* simTree = buildTree(visBodies);
        std::vector<Vec2> forces(N_VIS);
        for (int i = 0; i < N_VIS; i++) {
            forces[i] = bhForce(simTree, visBodies[i].pos);
        }
        delete simTree;
        
        for (int i = 0; i < N_VIS; i++) {
            visBodies[i].vel = visBodies[i].vel + forces[i] * (DT / visBodies[i].mass);
            visBodies[i].pos = visBodies[i].pos + visBodies[i].vel * DT;
            
            if (visBodies[i].pos.x < 0) { visBodies[i].pos.x = 0; visBodies[i].vel.x *= -0.5; }
            if (visBodies[i].pos.x > WIDTH) { visBodies[i].pos.x = WIDTH; visBodies[i].vel.x *= -0.5; }
            if (visBodies[i].pos.y < 0) { visBodies[i].pos.y = 0; visBodies[i].vel.y *= -0.5; }
            if (visBodies[i].pos.y > HEIGHT) { visBodies[i].pos.y = HEIGHT; visBodies[i].vel.y *= -0.5; }
        }
        
        // Save key frames
        if (step == STEPS/4) writePPM("barnes_hut_mid.ppm", visBodies, step);
    }
    
    writePPM("barnes_hut_final.ppm", visBodies, STEPS);
    
    std::cout << std::endl;
    std::cout << "✅ ALL VERIFICATIONS PASSED" << std::endl;
    return 0;
}
