#include <iostream>
#include <random>
#include <cmath>

struct RNG {
    std::mt19937 rng{42};
    float operator()() { return std::uniform_real_distribution<float>(0,1)(rng); }
};

int main() {
    RNG rng1, rng2;
    for (int i = 0; i < 5; i++)
        std::cout << rng1() << " " << rng2() << std::endl;
    return 0;
}
