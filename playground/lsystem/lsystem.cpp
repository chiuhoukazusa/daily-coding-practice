#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <string>
#include <vector>
#include <cmath>
#include <stack>
#include <map>

const double PI = 3.14159265358979323846;

struct Color {
    unsigned char r, g, b;
    Color(int r = 0, int g = 0, int b = 0) : r(r), g(g), b(b) {}
};

struct Canvas {
    int width, height;
    std::vector<unsigned char> pixels;
    
    Canvas(int w, int h, Color bg = Color(255, 255, 255)) 
        : width(w), height(h), pixels(w * h * 3) {
        for (int i = 0; i < w * h; i++) {
            pixels[i * 3] = bg.r;
            pixels[i * 3 + 1] = bg.g;
            pixels[i * 3 + 2] = bg.b;
        }
    }
    
    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        pixels[idx] = c.r;
        pixels[idx + 1] = c.g;
        pixels[idx + 2] = c.b;
    }
    
    void drawLine(double x0, double y0, double x1, double y1, Color c, int thickness = 1) {
        int dx = abs((int)x1 - (int)x0);
        int dy = abs((int)y1 - (int)y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        
        int ix0 = (int)x0, iy0 = (int)y0;
        int ix1 = (int)x1, iy1 = (int)y1;
        
        while (true) {
            for (int dy = -thickness/2; dy <= thickness/2; dy++) {
                for (int dx = -thickness/2; dx <= thickness/2; dx++) {
                    setPixel(ix0 + dx, iy0 + dy, c);
                }
            }
            
            if (ix0 == ix1 && iy0 == iy1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; ix0 += sx; }
            if (e2 < dx) { err += dx; iy0 += sy; }
        }
    }
    
    void save(const char* filename) {
        stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
    }
};

// L-System 生成器
class LSystem {
public:
    std::string axiom;
    std::map<char, std::string> rules;
    
    LSystem(const std::string& ax) : axiom(ax) {}
    
    void addRule(char from, const std::string& to) {
        rules[from] = to;
    }
    
    std::string generate(int iterations) {
        std::string current = axiom;
        
        for (int iter = 0; iter < iterations; iter++) {
            std::string next = "";
            for (char c : current) {
                if (rules.find(c) != rules.end()) {
                    next += rules[c];
                } else {
                    next += c;
                }
            }
            current = next;
        }
        
        return current;
    }
};

// 状态结构
struct TurtleState {
    double x, y, angle;
};

// 渲染 L-System
void renderLSystem(Canvas& canvas, const std::string& commands, 
                   double startX, double startY, double startAngle,
                   double stepLength, double angleStep, Color color, int thickness = 1) {
    
    std::stack<TurtleState> stateStack;
    TurtleState state = {startX, startY, startAngle};
    
    for (char cmd : commands) {
        if (cmd == 'F' || cmd == 'G') {
            // 画线
            double newX = state.x + stepLength * cos(state.angle * PI / 180.0);
            double newY = state.y - stepLength * sin(state.angle * PI / 180.0);
            canvas.drawLine(state.x, state.y, newX, newY, color, thickness);
            state.x = newX;
            state.y = newY;
            
        } else if (cmd == 'f') {
            // 移动不画线
            state.x += stepLength * cos(state.angle * PI / 180.0);
            state.y -= stepLength * sin(state.angle * PI / 180.0);
            
        } else if (cmd == '+') {
            state.angle += angleStep;
            
        } else if (cmd == '-') {
            state.angle -= angleStep;
            
        } else if (cmd == '[') {
            stateStack.push(state);
            
        } else if (cmd == ']') {
            if (!stateStack.empty()) {
                state = stateStack.top();
                stateStack.pop();
            }
        }
    }
}

int main() {
    const int W = 1000, H = 1000;
    
    // 1. 科赫雪花 (Koch Snowflake)
    {
        LSystem koch("F--F--F");
        koch.addRule('F', "F+F--F+F");
        std::string commands = koch.generate(4);
        
        Canvas canvas(W, H, Color(240, 248, 255));
        renderLSystem(canvas, commands, W/2 - 300, H/2 + 200, 0, 2, 60, Color(0, 0, 139), 1);
        canvas.save("lsystem_koch_snowflake.png");
    }
    
    // 2. 龙形曲线 (Dragon Curve)
    {
        LSystem dragon("FX");
        dragon.addRule('X', "X+YF+");
        dragon.addRule('Y', "-FX-Y");
        std::string commands = dragon.generate(12);
        
        Canvas canvas(W, H, Color(255, 250, 240));
        renderLSystem(canvas, commands, W/2 - 200, H/2, 0, 3, 90, Color(220, 20, 60), 1);
        canvas.save("lsystem_dragon_curve.png");
    }
    
    // 3. 分形植物 (Fractal Plant)
    {
        LSystem plant("X");
        plant.addRule('X', "F+[[X]-X]-F[-FX]+X");
        plant.addRule('F', "FF");
        std::string commands = plant.generate(6);
        
        Canvas canvas(W, H, Color(240, 255, 240));
        renderLSystem(canvas, commands, W/2, H - 50, 90, 3, 25, Color(34, 139, 34), 1);
        canvas.save("lsystem_fractal_plant.png");
    }
    
    // 4. 灌木 (Bush)
    {
        LSystem bush("F");
        bush.addRule('F', "FF+[+F-F-F]-[-F+F+F]");
        std::string commands = bush.generate(4);
        
        Canvas canvas(W, H, Color(245, 245, 220));
        renderLSystem(canvas, commands, W/2, H - 50, 90, 4, 22.5, Color(107, 142, 35), 2);
        canvas.save("lsystem_bush.png");
    }
    
    // 5. Sierpiński 三角形
    {
        LSystem sierpinski("F-G-G");
        sierpinski.addRule('F', "F-G+F+G-F");
        sierpinski.addRule('G', "GG");
        std::string commands = sierpinski.generate(6);
        
        Canvas canvas(W, H, Color(255, 245, 238));
        renderLSystem(canvas, commands, W/2 - 400, H/2 + 300, 0, 2, 120, Color(255, 140, 0), 1);
        canvas.save("lsystem_sierpinski.png");
    }
    
    // 6. 希尔伯特曲线 (Hilbert Curve)
    {
        LSystem hilbert("L");
        hilbert.addRule('L', "+RF-LFL-FR+");
        hilbert.addRule('R', "-LF+RFR+FL-");
        std::string commands = hilbert.generate(6);
        
        Canvas canvas(W, H, Color(250, 250, 250));
        renderLSystem(canvas, commands, 50, H - 50, 0, 4, 90, Color(138, 43, 226), 2);
        canvas.save("lsystem_hilbert_curve.png");
    }
    
    return 0;
}
