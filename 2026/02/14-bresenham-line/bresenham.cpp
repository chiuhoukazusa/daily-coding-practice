#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>

// Simple PPM image format writer
void write_ppm(const std::string& filename, const std::vector<std::vector<int>>& pixels, int width, int height) {
    std::ofstream file(filename);
    file << "P3\n";
    file << width << " " << height << "\n";
    file << "255\n";
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (pixels[y][x]) {
                file << "255 255 255 "; // White for drawn pixels
            } else {
                file << "0 0 0 ";       // Black for background
            }
        }
        file << "\n";
    }
    file.close();
}

// Bresenham line algorithm
void draw_line(std::vector<std::vector<int>>& pixels, int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // Draw current point
        if (x1 >= 0 && y1 >= 0 && 
            x1 < static_cast<int>(pixels[0].size()) && 
            y1 < static_cast<int>(pixels.size())) {
            pixels[y1][x1] = 1;
        }
        
        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

int main() {
    const int width = 800;
    const int height = 600;
    
    // Create canvas
    std::vector<std::vector<int>> pixels(height, std::vector<int>(width, 0));
    
    // Draw multiple lines to demonstrate the algorithm
    draw_line(pixels, 50, 50, 750, 50);     // Horizontal line
    draw_line(pixels, 50, 550, 750, 550);   // Another horizontal
    draw_line(pixels, 50, 50, 50, 550);     // Vertical line
    draw_line(pixels, 750, 50, 750, 550);   // Another vertical
    
    // Draw diagonal lines
    draw_line(pixels, 50, 50, 750, 550);    // Main diagonal
    draw_line(pixels, 750, 50, 50, 550);    // Anti-diagonal
    
    // Draw some random lines
    draw_line(pixels, 200, 150, 600, 250);
    draw_line(pixels, 300, 300, 500, 400);
    draw_line(pixels, 400, 200, 400, 500);
    draw_line(pixels, 150, 450, 650, 150);
    
    // Write to PPM file
    write_ppm("bresenham_output.ppm", pixels, width, height);
    
    std::cout << "Bresenham line drawing completed!" << std::endl;
    std::cout << "Output saved as bresenham_output.ppm" << std::endl;
    std::cout << "Image dimensions: " << width << "x" << height << std::endl;
    std::cout << "Lines drawn: 10 lines with various orientations" << std::endl;
    
    return 0;
}