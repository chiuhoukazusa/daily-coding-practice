#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <cstdio>
#include <random>
#include <algorithm>

static const float H = 16.0f;
static const float H2 = H*H;
static const float MASS = 1.0f;
static const float REST_DENS = 300.0f;
static const float PI = 3.14159265358979f;
static const float POLY6 = 315.0f / (65.0f * PI * std::pow(H, 9));

struct Vec2 { float x, y; };

inline float kernel_poly6(float r2) {
    if (r2 >= H2) return 0.0f;
    float diff = H2 - r2;
    return POLY6 * diff * diff * diff;
}

int main() {
    // Test: two particles at distance 0 (same position)
    float r2 = 0;
    printf("poly6(0) = %e\n", kernel_poly6(r2));
    printf("POLY6 const = %e\n", POLY6);
    
    // Test: density for a grid of 25x32=800 particles with spacing 0.8*H=12.8
    float spacing = H * 0.8f;
    
    // How many neighbors within H radius for grid spacing?
    float count_neighbors = 0;
    for (float dx = -3*H; dx <= 3*H; dx += spacing) {
        for (float dy = -3*H; dy <= 3*H; dy += spacing) {
            float r2_test = dx*dx + dy*dy;
            count_neighbors += kernel_poly6(r2_test);
        }
    }
    printf("Sum of poly6 for grid neighbors: %e\n", count_neighbors);
    printf("Density estimate (MASS * sum): %e\n", MASS * count_neighbors);
    printf("REST_DENS = %e\n", REST_DENS);
    return 0;
}
