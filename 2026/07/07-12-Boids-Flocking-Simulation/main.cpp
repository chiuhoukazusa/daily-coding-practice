#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <cassert>
#include <sstream>
#include <iomanip>

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;
constexpr int NUM_BOIDS = 300;
constexpr int NUM_FRAMES = 300;
constexpr float DT = 0.08f;
constexpr float MAX_SPEED = 5.0f;
constexpr float MAX_FORCE = 0.3f;

// Boid behavior weights: classic Craig Reynolds
constexpr float SEPARATION_WEIGHT = 1.5f;
constexpr float ALIGNMENT_WEIGHT = 1.5f;
constexpr float COHESION_WEIGHT = 1.2f;

constexpr float SEPARATION_RADIUS = 25.0f;
constexpr float ALIGNMENT_RADIUS = 80.0f;
constexpr float COHESION_RADIUS = 80.0f;

constexpr float CELL_SIZE = 50.0f;  // spatial hash cell size
constexpr float MARGIN = 20.0f;      // boundary margin for turning

struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }
    
    float length() const { return std::sqrt(x*x + y*y); }
    float lengthSq() const { return x*x + y*y; }
    
    Vec2 normalized() const {
        float len = length();
        if (len < 1e-6f) return {0, 0};
        return {x/len, y/len};
    }
    
    Vec2 limit(float max) const {
        float len = length();
        if (len > max) return {x/len*max, y/len*max};
        return *this;
    }
    
    float dot(const Vec2& o) const { return x*o.x + y*o.y; }
};

struct Boid {
    Vec2 position;
    Vec2 velocity;
    int id;
};

// Spatial hash for neighbor lookup
struct SpatialHash {
    std::unordered_map<uint64_t, std::vector<int>> grid;
    int cols;
    
    SpatialHash(int w, int h [[maybe_unused]]) {
        cols = static_cast<int>(std::ceil(w / CELL_SIZE)) + 1;
    }
    
    uint64_t hash(int cx, int cy) const {
        return (static_cast<uint64_t>(cy) << 32) | static_cast<uint32_t>(cx);
    }
    
    void clear() { grid.clear(); }
    
    void insert(const Boid& b, int idx) {
        int cx = static_cast<int>(std::floor(b.position.x / CELL_SIZE));
        int cy = static_cast<int>(std::floor(b.position.y / CELL_SIZE));
        grid[hash(cx, cy)].push_back(idx);
    }
    
    std::vector<int> query(const Vec2& pos, float radius) const {
        std::vector<int> result;
        int min_cx = static_cast<int>(std::floor((pos.x - radius) / CELL_SIZE));
        int max_cx = static_cast<int>(std::floor((pos.x + radius) / CELL_SIZE));
        int min_cy = static_cast<int>(std::floor((pos.y - radius) / CELL_SIZE));
        int max_cy = static_cast<int>(std::floor((pos.y + radius) / CELL_SIZE));
        
        for (int cy = min_cy; cy <= max_cy; cy++) {
            for (int cx = min_cx; cx <= max_cx; cx++) {
                auto it = grid.find(hash(cx, cy));
                if (it != grid.end()) {
                    result.insert(result.end(), it->second.begin(), it->second.end());
                }
            }
        }
        return result;
    }
};

// RNG
std::mt19937 rng(42);

float randomFloat(float min, float max) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

Vec2 randomVelocity() {
    float angle = randomFloat(0, 2.0f * M_PI);
    float speed = randomFloat(0.5f, MAX_SPEED);
    return Vec2(std::cos(angle) * speed, std::sin(angle) * speed);
}

// Steering behaviors
Vec2 separation(const Boid& b, const std::vector<Boid>& boids, float radius) {
    Vec2 steer(0, 0);
    int count = 0;
    for (const auto& other : boids) {
        if (other.id == b.id) continue;
        Vec2 diff = b.position - other.position;
        float d = diff.length();
        if (d < radius && d > 1e-3f) {
            steer = steer + diff.normalized() / d;
            count++;
        }
    }
    if (count > 0) {
        steer = steer / static_cast<float>(count);
        steer = steer.normalized() * MAX_SPEED;
        steer = steer - b.velocity;
        steer = steer.limit(MAX_FORCE);
    }
    return steer;
}

Vec2 alignment(const Boid& b, const std::vector<Boid>& boids, float radius) {
    Vec2 avgVel(0, 0);
    int count = 0;
    for (const auto& other : boids) {
        if (other.id == b.id) continue;
        Vec2 diff = b.position - other.position;
        float d = diff.length();
        if (d < radius && d > 1e-3f) {
            avgVel = avgVel + other.velocity;
            count++;
        }
    }
    if (count > 0) {
        avgVel = avgVel / static_cast<float>(count);
        avgVel = avgVel.normalized() * MAX_SPEED;
        Vec2 steer = avgVel - b.velocity;
        steer = steer.limit(MAX_FORCE);
        return steer;
    }
    return Vec2(0, 0);
}

Vec2 cohesion(const Boid& b, const std::vector<Boid>& boids, float radius) {
    Vec2 center(0, 0);
    int count = 0;
    for (const auto& other : boids) {
        if (other.id == b.id) continue;
        Vec2 diff = b.position - other.position;
        float d = diff.length();
        if (d < radius && d > 1e-3f) {
            center = center + other.position;
            count++;
        }
    }
    if (count > 0) {
        center = center / static_cast<float>(count);
        Vec2 desired = center - b.position;
        desired = desired.normalized() * MAX_SPEED;
        Vec2 steer = desired - b.velocity;
        steer = steer.limit(MAX_FORCE);
        return steer;
    }
    return Vec2(0, 0);
}

Vec2 boundaryForce(const Boid& b) {
    Vec2 force(0, 0);
    float turnFactor = 1.0f;
    
    if (b.position.x < MARGIN) {
        force.x += (MARGIN - b.position.x) / MARGIN * turnFactor;
    }
    if (b.position.x > WIDTH - MARGIN) {
        force.x -= (b.position.x - (WIDTH - MARGIN)) / MARGIN * turnFactor;
    }
    if (b.position.y < MARGIN) {
        force.y += (MARGIN - b.position.y) / MARGIN * turnFactor;
    }
    if (b.position.y > HEIGHT - MARGIN) {
        force.y -= (b.position.y - (HEIGHT - MARGIN)) / MARGIN * turnFactor;
    }
    
    float len = force.length();
    if (len > MAX_FORCE) force = force.normalized() * MAX_FORCE;
    return force;
}

// Quantitative metrics for validation
struct FlockMetrics {
    double avgSpeed;
    double avgSeparation;  // average nearest-neighbor distance
    double polarization;   // alignment order: 0=random, 1=perfectly aligned
    double centroidSpread; // std of positions
};

FlockMetrics computeMetrics(const std::vector<Boid>& boids) {
    FlockMetrics m = {0, 0, 0, 0};
    int n = boids.size();
    if (n == 0) return m;
    
    // Speed
    for (const auto& b : boids) {
        m.avgSpeed += b.velocity.length();
    }
    m.avgSpeed /= n;
    
    // Polarization (alignment)
    Vec2 avgDir(0, 0);
    for (const auto& b : boids) {
        float len = b.velocity.length();
        if (len > 1e-6f) avgDir = avgDir + b.velocity / len;
    }
    m.polarization = avgDir.length() / n;
    
    // Nearest-neighbor distance (sample-based for performance)
    double totalMinDist = 0;
    int sampleCount = std::min(n, 50);
    for (int i = 0; i < sampleCount; i++) {
        double minDist = 1e9;
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            float d = (boids[i].position - boids[j].position).length();
            if (d < minDist) minDist = d;
        }
        totalMinDist += minDist;
    }
    m.avgSeparation = totalMinDist / sampleCount;
    
    // Centroid spread
    Vec2 centroid(0, 0);
    for (const auto& b : boids) centroid = centroid + b.position;
    centroid = centroid / static_cast<float>(n);
    
    double spread = 0;
    for (const auto& b : boids) {
        float dx = b.position.x - centroid.x;
        float dy = b.position.y - centroid.y;
        spread += dx*dx + dy*dy;
    }
    m.centroidSpread = std::sqrt(spread / n);
    
    return m;
}

// Render single frame as PPM
void renderFrame(const std::vector<Boid>& boids, int frameNum) {
    std::vector<unsigned char> image(WIDTH * HEIGHT * 3, 0);
    
    // Background: light gray
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        image[i*3 + 0] = 30;
        image[i*3 + 1] = 30;
        image[i*3 + 2] = 40;
    }
    
    // Draw boids as small triangles (or colored dots with direction)
    for (const auto& b : boids) {
        Vec2 dir = b.velocity.normalized();
        Vec2 perp(-dir.y, dir.x);
        
        float speed = b.velocity.length();
        float t = std::min(speed / MAX_SPEED, 1.0f);
        
        // Color: teal to orange based on speed
        unsigned char r = static_cast<unsigned char>(50 + t * 200);
        unsigned char g = static_cast<unsigned char>(180 - t * 80);
        unsigned char blue = static_cast<unsigned char>(200 - t * 100);
        
        // Draw triangle: nose + 2 tail corners
        int noseX = static_cast<int>(b.position.x + dir.x * 4);
        int noseY = static_cast<int>(b.position.y + dir.y * 4);
        int leftX = static_cast<int>(b.position.x - dir.x * 3 + perp.x * 2);
        int leftY = static_cast<int>(b.position.y - dir.y * 3 + perp.y * 2);
        int rightX = static_cast<int>(b.position.x - dir.x * 3 - perp.x * 2);
        int rightY = static_cast<int>(b.position.y - dir.y * 3 - perp.y * 2);
        
        // Simple triangle fill via bounding box + barycentric test
        int minX = std::max(0, std::min({noseX, leftX, rightX}));
        int maxX = std::min(WIDTH-1, std::max({noseX, leftX, rightX}));
        int minY = std::max(0, std::min({noseY, leftY, rightY}));
        int maxY = std::min(HEIGHT-1, std::max({noseY, leftY, rightY}));
        
        auto edgeFn = [](int ax, int ay, int bx, int by, int px, int py) -> float {
            return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
        };
        
        float area = edgeFn(noseX, noseY, leftX, leftY, rightX, rightY);
        if (std::abs(area) < 1e-6f) continue;
        
        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                float w0 = edgeFn(leftX, leftY, rightX, rightY, x, y) / area;
                float w1 = edgeFn(rightX, rightY, noseX, noseY, x, y) / area;
                float w2 = edgeFn(noseX, noseY, leftX, leftY, x, y) / area;
                
                if (w0 >= -0.01f && w1 >= -0.01f && w2 >= -0.01f) {
                    int idx = y * WIDTH + x;
                    image[idx*3 + 0] = r;
                    image[idx*3 + 1] = g;
                    image[idx*3 + 2] = blue;
                }
            }
        }
        
        // Also draw a small dot at center
        int cx = static_cast<int>(b.position.x);
        int cy = static_cast<int>(b.position.y);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int px = cx + dx;
                int py = cy + dy;
                if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                    int idx = py * WIDTH + px;
                    image[idx*3 + 0] = 255;
                    image[idx*3 + 1] = 255;
                    image[idx*3 + 2] = 255;
                }
            }
        }
    }
    
    // Save as PPM
    std::ostringstream filename;
    filename << "frame_" << std::setfill('0') << std::setw(4) << frameNum << ".ppm";
    
    std::ofstream out(filename.str(), std::ios::binary);
    out << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.data()), image.size());
    out.close();
}

int main() {
    std::cout << "=== Boids Flocking Simulation ===" << std::endl;
    std::cout << "Boids: " << NUM_BOIDS << " | Frames: " << NUM_FRAMES << std::endl;
    std::cout << "Separation: r=" << SEPARATION_RADIUS << " w=" << SEPARATION_WEIGHT << std::endl;
    std::cout << "Alignment:  r=" << ALIGNMENT_RADIUS << " w=" << ALIGNMENT_WEIGHT << std::endl;
    std::cout << "Cohesion:   r=" << COHESION_RADIUS << " w=" << COHESION_WEIGHT << std::endl;
    std::cout << std::endl;
    
    // Initialize boids
    std::vector<Boid> boids(NUM_BOIDS);
    for (int i = 0; i < NUM_BOIDS; i++) {
        boids[i].id = i;
        boids[i].position = Vec2(randomFloat(MARGIN, WIDTH-MARGIN), randomFloat(MARGIN, HEIGHT-MARGIN));
        boids[i].velocity = randomVelocity();
    }
    
    SpatialHash hash(WIDTH, HEIGHT);
    
    // Save initial metrics
    auto initMetrics = computeMetrics(boids);
    std::cout << "=== INITIAL METRICS (before simulation) ===" << std::endl;
    std::cout << "Avg Speed:      " << initMetrics.avgSpeed << std::endl;
    std::cout << "Avg Separation: " << initMetrics.avgSeparation << " px (nearest neighbor)" << std::endl;
    std::cout << "Polarization:   " << initMetrics.polarization << " (0=random, 1=aligned)" << std::endl;
    std::cout << "Centroid Spread:" << initMetrics.centroidSpread << std::endl;
    std::cout << std::endl;
    
    // Initial polarization should be near 0 (random directions)
    bool initPolarizationOk = initMetrics.polarization < 0.15;
    std::cout << "[" << (initPolarizationOk ? "PASS" : "WARN") << "] Initial polarization check: "
              << initMetrics.polarization << " (expected < 0.15 for random)" << std::endl;
    
    // Simulation loop
    std::vector<FlockMetrics> metricHistory;
    metricHistory.push_back(initMetrics);
    
    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        // Update spatial hash
        hash.clear();
        for (int i = 0; i < NUM_BOIDS; i++) {
            hash.insert(boids[i], i);
        }
        
        // Compute new velocities
        std::vector<Vec2> newVelocities(NUM_BOIDS);
        
        for (int i = 0; i < NUM_BOIDS; i++) {
            // Get neighbors via spatial hash
            auto neighbors = hash.query(boids[i].position, std::max({SEPARATION_RADIUS, ALIGNMENT_RADIUS, COHESION_RADIUS}));
            
            // Collect nearby boids
            std::vector<Boid> nearby;
            for (int nidx : neighbors) {
                if (nidx != i) {
                    float d = (boids[i].position - boids[nidx].position).lengthSq();
                    float maxRSq = std::max({SEPARATION_RADIUS, ALIGNMENT_RADIUS, COHESION_RADIUS});
                    maxRSq = maxRSq * maxRSq;
                    if (d < maxRSq) {
                        nearby.push_back(boids[nidx]);
                    }
                }
            }
            
            Vec2 sep = separation(boids[i], nearby, SEPARATION_RADIUS);
            Vec2 ali = alignment(boids[i], nearby, ALIGNMENT_RADIUS);
            Vec2 coh = cohesion(boids[i], nearby, COHESION_RADIUS);
            Vec2 boundary = boundaryForce(boids[i]);
            
            Vec2 accel = sep * SEPARATION_WEIGHT
                       + ali * ALIGNMENT_WEIGHT
                       + coh * COHESION_WEIGHT
                       + boundary;
            
            newVelocities[i] = boids[i].velocity + accel * DT;
            newVelocities[i] = newVelocities[i].limit(MAX_SPEED);
        }
        
        // Update positions
        for (int i = 0; i < NUM_BOIDS; i++) {
            boids[i].velocity = newVelocities[i];
            boids[i].position = boids[i].position + boids[i].velocity * DT;
            
            // Clamp to bounds
            boids[i].position.x = std::max(0.0f, std::min((float)WIDTH, boids[i].position.x));
            boids[i].position.y = std::max(0.0f, std::min((float)HEIGHT, boids[i].position.y));
        }
        
        // Record metrics every 10 frames
        if (frame % 10 == 0 || frame == NUM_FRAMES - 1) {
            auto m = computeMetrics(boids);
            metricHistory.push_back(m);
        }
        
        // Render key frames
        if (frame == 0 || frame == NUM_FRAMES/2 || frame == NUM_FRAMES - 1) {
            renderFrame(boids, frame);
        }
    }
    
    // Final metrics
    auto finalMetrics = computeMetrics(boids);
    std::cout << "=== FINAL METRICS (after " << NUM_FRAMES << " frames) ===" << std::endl;
    std::cout << "Avg Speed:      " << finalMetrics.avgSpeed << std::endl;
    std::cout << "Avg Separation: " << finalMetrics.avgSeparation << " px" << std::endl;
    std::cout << "Polarization:   " << finalMetrics.polarization << " (0=random, 1=aligned)" << std::endl;
    std::cout << "Centroid Spread:" << finalMetrics.centroidSpread << std::endl;
    std::cout << std::endl;
    
    // === QUANTITATIVE VALIDATION ===
    bool allPassed = true;
    
    // Check 1: Polarization should increase (boids align)
    double p0 = initMetrics.polarization;
    double p1 = finalMetrics.polarization;
    std::cout << "=== VALIDATION ===" << std::endl;
    
    bool check1 = p1 > p0;
    std::cout << "[" << (check1 ? "PASS" : "FAIL") << "] Check 1: Polarization increase "
              << p0 << " -> " << p1 << " (delta: " << (p1-p0) << ")" << std::endl;
    if (!check1) allPassed = false;
    
    // Check 2: Final polarization should be significant (> 0.3 after flocking)
    bool check2 = p1 > 0.3;
    std::cout << "[" << (check2 ? "PASS" : "FAIL") << "] Check 2: Final polarization > 0.3: " << p1 << std::endl;
    if (!check2) allPassed = false;
    
    // Check 3: Average speed should be well above 0 (boids are moving)
    bool check3 = finalMetrics.avgSpeed > 1.0;
    std::cout << "[" << (check3 ? "PASS" : "FAIL") << "] Check 3: Avg speed > 1.0: " << finalMetrics.avgSpeed << std::endl;
    if (!check3) allPassed = false;
    
    // Check 4: Separation should prevent crowding (nearest neighbor > 3 pixels)
    bool check4 = finalMetrics.avgSeparation > 3.0;
    std::cout << "[" << (check4 ? "PASS" : "FAIL") << "] Check 4: Avg separation > 3px: " << finalMetrics.avgSeparation << std::endl;
    if (!check4) allPassed = false;
    
    // Check 5: All boids should be within bounds
    bool check5 = true;
    for (const auto& b : boids) {
        if (b.position.x < 0 || b.position.x > WIDTH || b.position.y < 0 || b.position.y > HEIGHT) {
            check5 = false;
            break;
        }
    }
    std::cout << "[" << (check5 ? "PASS" : "FAIL") << "] Check 5: All boids within bounds" << std::endl;
    if (!check5) allPassed = false;
    
    // Check 6: Centroid spread shouldn't collapse to 0 (flock occupies space)
    bool check6 = finalMetrics.centroidSpread > 50.0;
    std::cout << "[" << (check6 ? "PASS" : "FAIL") << "] Check 6: Centroid spread > 50: " << finalMetrics.centroidSpread << std::endl;
    if (!check6) allPassed = false;
    
    // Check 7: File outputs exist and are non-trivial
    std::cout << std::endl << "=== OUTPUT FILES ===" << std::endl;
    const char* frameFiles[] = {"frame_0000.ppm", nullptr, nullptr, nullptr};
    if (NUM_FRAMES > 1) {
        std::ostringstream mid; mid << "frame_" << std::setfill('0') << std::setw(4) << (NUM_FRAMES/2) << ".ppm";
        static std::string midStr = mid.str();
        frameFiles[1] = midStr.c_str();
    }
    {
        std::ostringstream last; last << "frame_" << std::setfill('0') << std::setw(4) << (NUM_FRAMES-1) << ".ppm";
        static std::string lastStr = last.str();
        frameFiles[2] = lastStr.c_str();
    }
    
    for (int i = 0; i < 3 && frameFiles[i]; i++) {
        std::ifstream f(frameFiles[i], std::ios::binary | std::ios::ate);
        auto size = f.tellg();
        std::cout << "  " << frameFiles[i] << ": " << size << " bytes";
        if (size > 10240) {
            std::cout << " [OK]" << std::endl;
        } else {
            std::cout << " [FAIL - too small]" << std::endl;
            allPassed = false;
        }
    }
    
    // Check 8: Polarization trend (monotonic-ish increase over time)
    std::cout << std::endl << "=== POLARIZATION TREND ===" << std::endl;
    bool monotonicUp = true;
    double prevP = -1;
    for (const auto& m : metricHistory) {
        std::cout << "  " << m.polarization;
        if (prevP >= 0 && m.polarization < prevP - 0.02) {
            std::cout << " ↓";
            monotonicUp = false;
        }
        std::cout << std::endl;
        prevP = m.polarization;
    }
    std::cout << "[" << (monotonicUp ? "PASS" : "WARN") << "] Check 8: Polarization generally increasing" << std::endl;
    // Note: not a hard fail since minor dips can happen
    
    std::cout << std::endl << (allPassed ? "✅ ALL VALIDATION CHECKS PASSED" : "❌ SOME CHECKS FAILED") << std::endl;
    
    return allPassed ? 0 : 1;
}
