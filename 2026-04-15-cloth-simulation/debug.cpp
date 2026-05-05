#include <cmath>
#include <cstdio>
#include <vector>

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 normalized() const { float l = len(); return l>1e-8f ? *this/l : Vec3(0,0,0); }
};

int main() {
    // Test cloth init positions
    int rows=20, cols=20;
    float spacing=0.2f;
    float startX=-1.9f, startY=1.5f, startZ=-0.9f;
    
    // Camera
    Vec3 eye(2.5f, 1.0f, 4.0f);
    Vec3 center(0.0f, -0.5f, 0.0f);
    Vec3 up(0, 1, 0);
    float fov = 45.0f;
    int W=480, H=360;
    float aspect = (float)W/H;
    float znear = 0.1f;
    
    Vec3 forward = (center - eye).normalized();
    Vec3 right = forward.cross(up).normalized();
    Vec3 actualUp = right.cross(forward);
    
    printf("Forward: %.3f %.3f %.3f\n", forward.x, forward.y, forward.z);
    printf("Right: %.3f %.3f %.3f\n", right.x, right.y, right.z);
    printf("ActualUp: %.3f %.3f %.3f\n", actualUp.x, actualUp.y, actualUp.z);
    
    // Test a few cloth positions
    for (int r = 0; r < rows; r += 5) {
        for (int c = 0; c < cols; c += 5) {
            Vec3 world(startX + c*spacing, startY, startZ + r*spacing);
            Vec3 d = world - eye;
            float x = d.dot(right);
            float y = d.dot(actualUp);
            float z = d.dot(forward);
            if (z < znear) { printf("[%d,%d] BEHIND CAM\n", r, c); continue; }
            float tanHalf = tanf(fov * 0.5f * 3.14159265f / 180.0f);
            float px = x / (z * tanHalf * aspect);
            float py = y / (z * tanHalf);
            float sx = (px + 1.0f) * 0.5f * W;
            float sy = (1.0f - (py + 1.0f) * 0.5f) * H;
            printf("[%d,%d] world(%.2f,%.2f,%.2f) -> z=%.2f sx=%.1f sy=%.1f\n",
                   r, c, world.x, world.y, world.z, z, sx, sy);
        }
    }
    return 0;
}
