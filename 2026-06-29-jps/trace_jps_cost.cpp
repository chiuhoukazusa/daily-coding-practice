// Trace JPS path cost manually
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
using namespace std;

struct Pt {int x,y;};

// Given JPS path points, compute actual step-by-step cost
int compute_actual_cost(const vector<Pt>& path) {
    int total = 0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        Pt a = path[i], b = path[i+1];
        int dx = abs(b.x - a.x), dy = abs(b.y - a.y);
        // Number of diagonal and straight steps to go from a to b
        // We go in a straight line (same dx/dy direction as the jump)
        int sx = (b.x > a.x) ? 1 : ((b.x < a.x) ? -1 : 0);
        int sy = (b.y > a.y) ? 1 : ((b.y < a.y) ? -1 : 0);
        Pt c = a;
        while (!(c.x == b.x && c.y == b.y)) {
            if (sx != 0 && sy != 0) {
                total += 14;
                c.x += sx; c.y += sy;
            } else if (sx != 0) {
                total += 10;
                c.x += sx;
            } else {
                total += 10;
                c.y += sy;
            }
        }
    }
    return total;
}

int main() {
    // Example: 3 diagonal jumps from (0,0) to (3,3)
    vector<Pt> diag_path = {{0,0}, {1,1}, {2,2}, {3,3}};
    printf("3 diagonal steps: step-by-step=%d, expected=%d\n", 
           compute_actual_cost(diag_path), 42);
    
    // Single 3-diagonal jump
    vector<Pt> jump_path = {{0,0}, {3,3}};
    printf("1 jump of 3 diag: step-by-step=%d, expected=%d\n", 
           compute_actual_cost(jump_path), 42);
    
    // Mixed: diagonal then horizontal
    vector<Pt> mixed_path = {{0,0}, {3,3}, {5,3}};
    printf("Mixed: step-by-step=%d, expected=%d\n",
           compute_actual_cost(mixed_path), 42+20);
    
    return 0;
}
