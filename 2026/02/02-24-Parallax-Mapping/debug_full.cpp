#include <iostream>
#include <cmath>
#include <iomanip>

const double PI = 3.14159265358979323846;

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double t) const { return Vec3(x*t, y*t, z*t); }
    Vec3 operator/(double t) const { return Vec3(x/t, y/t, z/t); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { return *this / length(); }
};

struct Vec2 {
    double x, y;
    Vec2(double x=0, double y=0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
    Vec2 operator*(double t) const { return Vec2(x*t, y*t); }
    Vec2 operator/(double t) const { return Vec2(x/t, y/t); }
};

// 球体类
class Sphere {
public:
    Vec3 center;
    double radius;
    
    Sphere(const Vec3& c, double r) : center(c), radius(r) {}
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
    
    void getUV(const Vec3& point, double& u, double& v) const {
        Vec3 p = (point - center).normalize();
        u = 0.5 + std::atan2(p.z, p.x) / (2.0 * PI);
        v = 0.5 - std::asin(p.y) / PI;
    }
    
    void getTBN(const Vec3& point, Vec3& T, Vec3& B, Vec3& N) const {
        N = getNormal(point);
        
        Vec3 up = std::abs(N.y) < 0.999 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        T = up.cross(N).normalize();
        B = N.cross(T).normalize();
    }
};

double brick_depth(double u, double v) {
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    return is_mortar ? 0.2 : 1.0;  // 灰浆0.2（浅），砖块1.0（深）
}

int main() {
    // 测试点：球面上从相机看向球心的光线击中的点
    Sphere sphere(Vec3(0, 0, -3), 1.0);
    
    // 选择一个侧面的点（不是正中心）进行测试
    Vec3 hit_point(0.5, 0.3, -2.2);  // 球面上偏左上的点
    Vec3 view_origin(0, 0, 0);  // 相机位置
    Vec3 view_dir = (view_origin - hit_point).normalize();
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "========== 完整 Parallax Mapping 调试 ==========" << std::endl;
    std::cout << "\n【1】世界空间信息" << std::endl;
    std::cout << "  击中点: (" << hit_point.x << ", " << hit_point.y << ", " << hit_point.z << ")" << std::endl;
    std::cout << "  视线方向: (" << view_dir.x << ", " << view_dir.y << ", " << view_dir.z << ")" << std::endl;
    
    // 计算UV和TBN
    double u, v;
    sphere.getUV(hit_point, u, v);
    std::cout << "\n【2】UV坐标" << std::endl;
    std::cout << "  原始UV: (" << u << ", " << v << ")" << std::endl;
    
    Vec3 T, B, N;
    sphere.getTBN(hit_point, T, B, N);
    std::cout << "\n【3】TBN坐标系" << std::endl;
    std::cout << "  T (切线): (" << T.x << ", " << T.y << ", " << T.z << ")" << std::endl;
    std::cout << "  B (副切线): (" << B.x << ", " << B.y << ", " << B.z << ")" << std::endl;
    std::cout << "  N (法线): (" << N.x << ", " << N.y << ", " << N.z << ")" << std::endl;
    
    // 转换到切线空间
    Vec3 view_tangent(view_dir.dot(T), view_dir.dot(B), view_dir.dot(N));
    std::cout << "\n【4】切线空间视线方向" << std::endl;
    std::cout << "  view_tangent: (" << view_tangent.x << ", " << view_tangent.y << ", " << view_tangent.z << ")" << std::endl;
    std::cout << "  长度: " << view_tangent.length() << std::endl;
    
    // Parallax计算
    const double height_scale = 0.3;
    Vec2 P = Vec2(view_tangent.x, view_tangent.y) / view_tangent.z * height_scale;
    std::cout << "\n【5】P向量计算" << std::endl;
    std::cout << "  P = (view_tangent.xy / view_tangent.z) * height_scale" << std::endl;
    std::cout << "  P = (" << P.x << ", " << P.y << ")" << std::endl;
    std::cout << "  P长度: " << std::sqrt(P.x*P.x + P.y*P.y) << std::endl;
    
    // 步进
    const int num_layers = 16;
    double layer_depth = 1.0 / num_layers;
    Vec2 delta = P / num_layers;
    
    std::cout << "\n【6】步进参数" << std::endl;
    std::cout << "  层数: " << num_layers << std::endl;
    std::cout << "  每层深度: " << layer_depth << std::endl;
    std::cout << "  每层UV偏移: (" << delta.x << ", " << delta.y << ")" << std::endl;
    
    Vec2 current_uv(u, v);
    double current_layer = 0.0;
    
    std::cout << "\n【7】步进过程（前5步 + 碰撞层）" << std::endl;
    for (int i = 0; i < num_layers; i++) {
        double depth = brick_depth(current_uv.x, current_uv.y);
        
        if (i < 5) {
            std::cout << "  Step " << i << ": UV(" << current_uv.x << ", " << current_uv.y 
                      << ") depth=" << depth << " layer=" << current_layer;
            if (current_layer < depth) {
                std::cout << " [继续]" << std::endl;
            } else {
                std::cout << " [碰撞!]" << std::endl;
            }
        }
        
        if (current_layer >= depth) {
            std::cout << "\n  >>> 碰撞发生在第 " << i << " 层 <<<" << std::endl;
            std::cout << "  最终UV: (" << current_uv.x << ", " << current_uv.y << ")" << std::endl;
            std::cout << "  UV偏移量: Δu=" << (current_uv.x - u) << ", Δv=" << (current_uv.y - v) << std::endl;
            
            double offset_magnitude = std::sqrt((current_uv.x-u)*(current_uv.x-u) + (current_uv.y-v)*(current_uv.y-v));
            std::cout << "  偏移幅度: " << offset_magnitude << std::endl;
            
            if (offset_magnitude < 0.001) {
                std::cout << "\n❌ 警告：偏移量过小！几乎没有视差效果！" << std::endl;
            } else if (offset_magnitude < 0.01) {
                std::cout << "\n⚠️  偏移量较小，效果可能不明显" << std::endl;
            } else {
                std::cout << "\n✅ 偏移量正常" << std::endl;
            }
            
            return 0;
        }
        
        current_uv = current_uv - delta;
        current_layer += layer_depth;
    }
    
    std::cout << "\n未碰撞！最终UV: (" << current_uv.x << ", " << current_uv.y << ")" << std::endl;
    return 0;
}
