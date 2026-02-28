/**
 * BVH Accelerated Ray Tracer - 2026-03-01
 * 
 * 使用层次包围盒（Bounding Volume Hierarchy）加速光线追踪
 * 
 * 核心技术：
 * - AABB (Axis-Aligned Bounding Box) 包围盒
 * - SAH (Surface Area Heuristic) 构建策略
 * - 递归 BVH 树结构
 * - 对比：暴力遍历 vs BVH 加速
 * 
 * 场景：大量随机球体的渲染，展示 BVH 的性能优势
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <limits>
#include <random>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

// ============================================================
// 基础数学工具
// ============================================================

struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { return *this / length(); }
    
    double& operator[](int i) { return (&x)[i]; }
    const double& operator[](int i) const { return (&x)[i]; }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
// 光线
// ============================================================

struct Ray {
    Vec3 origin, direction;
    Vec3 at(double t) const { return origin + direction * t; }
};

// ============================================================
// AABB 轴对齐包围盒
// ============================================================

struct AABB {
    Vec3 min_pt, max_pt;
    
    AABB() : min_pt(1e30, 1e30, 1e30), max_pt(-1e30, -1e30, -1e30) {}
    AABB(const Vec3& a, const Vec3& b) : min_pt(a), max_pt(b) {}
    
    // 合并两个 AABB
    static AABB merge(const AABB& a, const AABB& b) {
        Vec3 small(std::min(a.min_pt.x, b.min_pt.x),
                   std::min(a.min_pt.y, b.min_pt.y),
                   std::min(a.min_pt.z, b.min_pt.z));
        Vec3 large(std::max(a.max_pt.x, b.max_pt.x),
                   std::max(a.max_pt.y, b.max_pt.y),
                   std::max(a.max_pt.z, b.max_pt.z));
        return AABB(small, large);
    }
    
    // 计算表面积（用于 SAH）
    double surface_area() const {
        Vec3 d = max_pt - min_pt;
        return 2.0 * (d.x*d.y + d.y*d.z + d.z*d.x);
    }
    
    // 包围盒质心
    Vec3 centroid() const {
        return (min_pt + max_pt) * 0.5;
    }
    
    // 光线与 AABB 相交测试（slab method）
    bool intersect(const Ray& ray, double t_min, double t_max) const {
        for (int i = 0; i < 3; i++) {
            double inv_d = 1.0 / ray.direction[i];
            double t0 = (min_pt[i] - ray.origin[i]) * inv_d;
            double t1 = (max_pt[i] - ray.origin[i]) * inv_d;
            if (inv_d < 0) std::swap(t0, t1);
            t_min = std::max(t_min, t0);
            t_max = std::min(t_max, t1);
            if (t_max <= t_min) return false;
        }
        return true;
    }
};

// ============================================================
// 材质
// ============================================================

struct Material {
    enum Type { DIFFUSE, METAL, GLASS } type;
    Vec3 albedo;
    double roughness; // 金属粗糙度
    double ior;       // 折射率（玻璃）
    
    static Material diffuse(Vec3 color) { return {DIFFUSE, color, 0, 0}; }
    static Material metal(Vec3 color, double rough) { return {METAL, color, rough, 0}; }
    static Material glass(double ior) { return {GLASS, {1,1,1}, 0, ior}; }
};

// ============================================================
// 碰撞记录
// ============================================================

struct HitRecord {
    Vec3 point;
    Vec3 normal;
    double t;
    bool front_face;
    Material mat;
    
    void set_face_normal(const Ray& ray, const Vec3& outward_normal) {
        front_face = ray.direction.dot(outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

// ============================================================
// 球体
// ============================================================

struct Sphere {
    Vec3 center;
    double radius;
    Material mat;
    
    AABB bounding_box() const {
        Vec3 r_vec(radius, radius, radius);
        return AABB(center - r_vec, center + r_vec);
    }
    
    bool intersect(const Ray& ray, double t_min, double t_max, HitRecord& rec) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double half_b = oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = half_b * half_b - a * c;
        
        if (discriminant < 0) return false;
        double sqrt_d = std::sqrt(discriminant);
        
        double t = (-half_b - sqrt_d) / a;
        if (t < t_min || t > t_max) {
            t = (-half_b + sqrt_d) / a;
            if (t < t_min || t > t_max) return false;
        }
        
        rec.t = t;
        rec.point = ray.at(t);
        Vec3 outward_normal = (rec.point - center) / radius;
        rec.set_face_normal(ray, outward_normal);
        rec.mat = mat;
        return true;
    }
};

// ============================================================
// BVH 节点
// ============================================================

struct BVHNode {
    AABB bbox;
    int left, right;   // 子节点索引（-1 表示叶子节点）
    int sphere_idx;    // 球体索引（叶子节点有效）
    bool is_leaf;
    
    BVHNode() : left(-1), right(-1), sphere_idx(-1), is_leaf(false) {}
};

// ============================================================
// BVH 树
// ============================================================

class BVH {
public:
    std::vector<BVHNode> nodes;
    const std::vector<Sphere>& spheres;
    int bvh_intersection_tests = 0; // 统计 BVH 测试次数
    
    BVH(const std::vector<Sphere>& spheres) : spheres(spheres) {
        if (spheres.empty()) return;
        
        std::vector<int> indices(spheres.size());
        for (int i = 0; i < (int)spheres.size(); i++) indices[i] = i;
        
        build(indices, 0, (int)indices.size());
    }
    
    // 递归构建 BVH
    int build(std::vector<int>& indices, int start, int end) {
        int node_idx = (int)nodes.size();
        nodes.emplace_back();
        
        int count = end - start;
        
        if (count == 1) {
            // 叶子节点
            nodes[node_idx].is_leaf = true;
            nodes[node_idx].sphere_idx = indices[start];
            nodes[node_idx].bbox = spheres[indices[start]].bounding_box();
            return node_idx;
        }
        
        // 计算所有球心的包围盒
        AABB centroid_bbox;
        for (int i = start; i < end; i++) {
            Vec3 c = spheres[indices[i]].center;
            centroid_bbox.min_pt.x = std::min(centroid_bbox.min_pt.x, c.x);
            centroid_bbox.min_pt.y = std::min(centroid_bbox.min_pt.y, c.y);
            centroid_bbox.min_pt.z = std::min(centroid_bbox.min_pt.z, c.z);
            centroid_bbox.max_pt.x = std::max(centroid_bbox.max_pt.x, c.x);
            centroid_bbox.max_pt.y = std::max(centroid_bbox.max_pt.y, c.y);
            centroid_bbox.max_pt.z = std::max(centroid_bbox.max_pt.z, c.z);
        }
        
        // 选择最长轴分裂
        Vec3 extent = centroid_bbox.max_pt - centroid_bbox.min_pt;
        int axis = 0;
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent[axis]) axis = 2;
        
        // SAH（Surface Area Heuristic）分割
        int mid = sah_split(indices, start, end, axis, centroid_bbox);
        
        int left_child = build(indices, start, mid);
        int right_child = build(indices, mid, end);
        
        nodes[node_idx].left = left_child;
        nodes[node_idx].right = right_child;
        nodes[node_idx].is_leaf = false;
        nodes[node_idx].bbox = AABB::merge(nodes[left_child].bbox, nodes[right_child].bbox);
        
        return node_idx;
    }
    
    // SAH 分割策略
    int sah_split(std::vector<int>& indices, int start, int end, int axis, const AABB& centroid_bbox) {
        int count = end - start;
        
        if (count <= 4) {
            // 小数量直接中值分割
            std::sort(indices.begin() + start, indices.begin() + end,
                [&](int a, int b) {
                    return spheres[a].center[axis] < spheres[b].center[axis];
                });
            return start + count / 2;
        }
        
        // SAH 桶数量
        const int NUM_BUCKETS = 12;
        struct Bucket { AABB bbox; int count = 0; };
        Bucket buckets[NUM_BUCKETS];
        
        double extent = centroid_bbox.max_pt[axis] - centroid_bbox.min_pt[axis];
        if (extent < 1e-10) {
            // 退化情况：所有质心在同一位置
            return start + count / 2;
        }
        
        // 将球体分配到桶中
        for (int i = start; i < end; i++) {
            double c = spheres[indices[i]].center[axis];
            int b = (int)(NUM_BUCKETS * (c - centroid_bbox.min_pt[axis]) / extent);
            if (b >= NUM_BUCKETS) b = NUM_BUCKETS - 1;
            buckets[b].count++;
            buckets[b].bbox = AABB::merge(buckets[b].bbox, spheres[indices[i]].bounding_box());
        }
        
        // 计算每个分割点的 SAH 代价
        double costs[NUM_BUCKETS - 1];
        for (int i = 0; i < NUM_BUCKETS - 1; i++) {
            AABB b0, b1;
            int cnt0 = 0, cnt1 = 0;
            for (int j = 0; j <= i; j++) {
                b0 = AABB::merge(b0, buckets[j].bbox);
                cnt0 += buckets[j].count;
            }
            for (int j = i + 1; j < NUM_BUCKETS; j++) {
                b1 = AABB::merge(b1, buckets[j].bbox);
                cnt1 += buckets[j].count;
            }
            costs[i] = 0.125 + (cnt0 * b0.surface_area() + cnt1 * b1.surface_area()) /
                        AABB::merge(b0, b1).surface_area();
        }
        
        // 找最小代价分割
        int min_bucket = 0;
        double min_cost = costs[0];
        for (int i = 1; i < NUM_BUCKETS - 1; i++) {
            if (costs[i] < min_cost) {
                min_cost = costs[i];
                min_bucket = i;
            }
        }
        
        // 按分割点分区
        double split_val = centroid_bbox.min_pt[axis] + (min_bucket + 1) * extent / NUM_BUCKETS;
        auto it = std::partition(indices.begin() + start, indices.begin() + end,
            [&](int idx) {
                return spheres[idx].center[axis] < split_val;
            });
        
        int mid = (int)(it - indices.begin());
        if (mid == start || mid == end) mid = start + count / 2; // 防止退化
        return mid;
    }
    
    // BVH 遍历 - 找最近交点
    bool intersect(const Ray& ray, double t_min, double t_max, HitRecord& rec, int& tests) const {
        if (nodes.empty()) return false;
        return intersect_node(0, ray, t_min, t_max, rec, tests);
    }
    
    bool intersect_node(int node_idx, const Ray& ray, double t_min, double t_max,
                         HitRecord& rec, int& tests) const {
        const BVHNode& node = nodes[node_idx];
        tests++;
        
        if (!node.bbox.intersect(ray, t_min, t_max)) return false;
        
        if (node.is_leaf) {
            return spheres[node.sphere_idx].intersect(ray, t_min, t_max, rec);
        }
        
        bool hit_left = false, hit_right = false;
        HitRecord rec_left, rec_right;
        double t_closest = t_max;
        
        if (node.left >= 0) {
            hit_left = intersect_node(node.left, ray, t_min, t_closest, rec_left, tests);
            if (hit_left) t_closest = rec_left.t;
        }
        if (node.right >= 0) {
            hit_right = intersect_node(node.right, ray, t_min, t_closest, rec_right, tests);
        }
        
        if (hit_right) { rec = rec_right; return true; }
        if (hit_left)  { rec = rec_left;  return true; }
        return false;
    }
};

// ============================================================
// 随机工具
// ============================================================

static std::mt19937 rng(42);
static std::uniform_real_distribution<double> dist(0.0, 1.0);

double rand01() { return dist(rng); }
double rand_range(double a, double b) { return a + (b - a) * rand01(); }

Vec3 rand_in_unit_sphere() {
    while (true) {
        Vec3 p(rand_range(-1,1), rand_range(-1,1), rand_range(-1,1));
        if (p.dot(p) < 1.0) return p;
    }
}

Vec3 rand_unit_vec() { return rand_in_unit_sphere().normalize(); }

Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2 * v.dot(n) * n;
}

Vec3 refract(const Vec3& uv, const Vec3& n, double etai_over_etat) {
    double cos_theta = std::min((-uv).dot(n), 1.0);
    Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    Vec3 r_out_parallel = -std::sqrt(std::abs(1.0 - r_out_perp.dot(r_out_perp))) * n;
    return r_out_perp + r_out_parallel;
}

double schlick(double cosine, double ior) {
    double r0 = (1 - ior) / (1 + ior);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow(1 - cosine, 5);
}

// ============================================================
// 场景管理
// ============================================================

struct Scene {
    std::vector<Sphere> spheres;
    std::unique_ptr<BVH> bvh;
    
    void build_bvh() {
        bvh = std::make_unique<BVH>(spheres);
    }
    
    // 暴力遍历（对比用）
    bool intersect_brute(const Ray& ray, double t_min, double t_max, HitRecord& rec) const {
        HitRecord tmp;
        bool hit = false;
        double closest = t_max;
        for (const auto& sphere : spheres) {
            if (sphere.intersect(ray, t_min, closest, tmp)) {
                hit = true;
                closest = tmp.t;
                rec = tmp;
            }
        }
        return hit;
    }
    
    // BVH 加速遍历
    bool intersect_bvh(const Ray& ray, double t_min, double t_max, HitRecord& rec, int& tests) const {
        return bvh->intersect(ray, t_min, t_max, rec, tests);
    }
};

// ============================================================
// 路径追踪
// ============================================================

Vec3 ray_color(const Ray& ray, const Scene& scene, int depth, bool use_bvh, int& total_tests) {
    if (depth <= 0) return {0, 0, 0};
    
    HitRecord rec;
    int tests = 0;
    bool hit = use_bvh ? scene.intersect_bvh(ray, 0.001, 1e10, rec, tests)
                        : scene.intersect_brute(ray, 0.001, 1e10, rec);
    total_tests += tests;
    
    if (!hit) {
        // 天空渐变
        Vec3 unit_dir = ray.direction.normalize();
        double t = 0.5 * (unit_dir.y + 1.0);
        return (1 - t) * Vec3(1.0, 1.0, 1.0) + t * Vec3(0.5, 0.7, 1.0);
    }
    
    Ray scattered;
    Vec3 attenuation;
    
    const Material& mat = rec.mat;
    
    if (mat.type == Material::DIFFUSE) {
        Vec3 scatter_dir = rec.normal + rand_unit_vec();
        // 防止退化
        if (scatter_dir.dot(scatter_dir) < 1e-8) scatter_dir = rec.normal;
        scattered = {rec.point, scatter_dir.normalize()};
        attenuation = mat.albedo;
    }
    else if (mat.type == Material::METAL) {
        Vec3 reflected = reflect(ray.direction.normalize(), rec.normal);
        scattered = {rec.point, reflected + mat.roughness * rand_in_unit_sphere()};
        attenuation = mat.albedo;
        if (scattered.direction.dot(rec.normal) <= 0) return {0, 0, 0};
    }
    else { // GLASS
        attenuation = {1, 1, 1};
        double ior = rec.front_face ? (1.0 / mat.ior) : mat.ior;
        Vec3 unit_dir = ray.direction.normalize();
        double cos_theta = std::min((-unit_dir).dot(rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
        bool cannot_refract = ior * sin_theta > 1.0;
        Vec3 direction;
        if (cannot_refract || schlick(cos_theta, ior) > rand01()) {
            direction = reflect(unit_dir, rec.normal);
        } else {
            direction = refract(unit_dir, rec.normal, ior);
        }
        scattered = {rec.point, direction};
    }
    
    return attenuation * ray_color(scattered, scene, depth - 1, use_bvh, total_tests);
}

// ============================================================
// 生成随机场景
// ============================================================

Scene generate_scene(int num_spheres) {
    Scene scene;
    
    // 地面
    scene.spheres.push_back({
        {0, -1000, 0}, 1000,
        Material::diffuse({0.5, 0.5, 0.5})
    });
    
    // 三个大球
    scene.spheres.push_back({{0, 1, 0}, 1.0, Material::glass(1.5)});
    scene.spheres.push_back({{-4, 1, 0}, 1.0, Material::diffuse({0.4, 0.2, 0.1})});
    scene.spheres.push_back({{4, 1, 0}, 1.0, Material::metal({0.7, 0.6, 0.5}, 0.0)});
    
    // 随机小球
    std::mt19937 local_rng(12345);
    std::uniform_real_distribution<double> d(0.0, 1.0);
    
    int placed = 0;
    int attempts = 0;
    while (placed < num_spheres && attempts < num_spheres * 10) {
        attempts++;
        double cx = d(local_rng) * 22 - 11;
        double cz = d(local_rng) * 22 - 11;
        Vec3 center(cx, 0.2, cz);
        
        // 避免与大球重叠
        bool ok = true;
        Vec3 big_centers[] = {{0,1,0},{-4,1,0},{4,1,0}};
        for (auto& bc : big_centers) {
            if ((center - bc).length() < 1.2) { ok = false; break; }
        }
        if (!ok) continue;
        
        double mat_choice = d(local_rng);
        Material mat;
        if (mat_choice < 0.7) {
            Vec3 albedo(d(local_rng)*d(local_rng), d(local_rng)*d(local_rng), d(local_rng)*d(local_rng));
            mat = Material::diffuse(albedo);
        } else if (mat_choice < 0.9) {
            Vec3 albedo(0.5 + 0.5*d(local_rng), 0.5 + 0.5*d(local_rng), 0.5 + 0.5*d(local_rng));
            mat = Material::metal(albedo, 0.5*d(local_rng));
        } else {
            mat = Material::glass(1.5);
        }
        
        scene.spheres.push_back({center, 0.2, mat});
        placed++;
    }
    
    return scene;
}

// ============================================================
// PPM 图像写入
// ============================================================

struct Image {
    int width, height;
    std::vector<Vec3> pixels;
    
    Image(int w, int h) : width(w), height(h), pixels(w * h) {}
    
    Vec3& at(int x, int y) { return pixels[y * width + x]; }
    
    void save_png(const std::string& filename) const {
        // 使用 PPM 格式，然后用 ImageMagick 转换
        std::string ppm_file = filename + ".ppm";
        std::ofstream out(ppm_file, std::ios::binary);
        out << "P6\n" << width << " " << height << "\n255\n";
        for (const auto& p : pixels) {
            // Gamma 校正
            double r = std::sqrt(std::max(0.0, std::min(1.0, p.x)));
            double g = std::sqrt(std::max(0.0, std::min(1.0, p.y)));
            double b = std::sqrt(std::max(0.0, std::min(1.0, p.z)));
            out << (uint8_t)(255.999 * r)
                << (uint8_t)(255.999 * g)
                << (uint8_t)(255.999 * b);
        }
        out.close();
        
        // 转换为 PNG
        std::string cmd = "convert " + ppm_file + " " + filename + " 2>/dev/null";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            std::remove(ppm_file.c_str());
        } else {
            // 如果 convert 失败，保留 PPM 并重命名
            std::rename(ppm_file.c_str(), filename.c_str());
            std::cout << "  [注意: ImageMagick 未安装，保存为 PPM 格式]\n";
        }
    }
};

// ============================================================
// 相机
// ============================================================

struct Camera {
    Vec3 origin, lower_left, horizontal, vertical;
    Vec3 u, v, w;
    double lens_radius;
    
    Camera(Vec3 look_from, Vec3 look_at, Vec3 vup,
           double vfov, double aspect, double aperture, double focus_dist) {
        double theta = vfov * M_PI / 180.0;
        double h = std::tan(theta / 2);
        double viewport_height = 2.0 * h;
        double viewport_width = aspect * viewport_height;
        
        w = (look_from - look_at).normalize();
        u = vup.cross(w).normalize();
        v = w.cross(u);
        
        origin = look_from;
        horizontal = focus_dist * viewport_width * u;
        vertical = focus_dist * viewport_height * v;
        lower_left = origin - horizontal*0.5 - vertical*0.5 - focus_dist*w;
        lens_radius = aperture / 2;
    }
    
    Ray get_ray(double s, double t) const {
        Vec3 rd = lens_radius * rand_in_unit_sphere();
        Vec3 offset = u * rd.x + v * rd.y;
        return {origin + offset,
                (lower_left + s*horizontal + t*vertical - origin - offset).normalize()};
    }
};

// ============================================================
// 渲染统计
// ============================================================

struct RenderStats {
    double render_time_ms;
    long long total_tests;
    long long total_rays;
    double tests_per_ray;
};

// ============================================================
// 渲染函数
// ============================================================

RenderStats render(Image& img, const Scene& scene, const Camera& cam,
                   int samples, int max_depth, bool use_bvh) {
    auto start = std::chrono::high_resolution_clock::now();
    long long total_tests = 0;
    long long total_rays = 0;
    
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vec3 color(0, 0, 0);
            for (int s = 0; s < samples; s++) {
                double u = (x + rand01()) / (img.width - 1);
                double v = (y + rand01()) / (img.height - 1);
                Ray ray = cam.get_ray(u, v);
                int tests = 0;
                color = color + ray_color(ray, scene, max_depth, use_bvh, tests);
                total_tests += tests;
                total_rays++;
            }
            img.at(x, img.height - 1 - y) = color / (double)samples;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    return {ms, total_tests, total_rays, (double)total_tests / total_rays};
}

// ============================================================
// 生成对比图（左：BVH，右：暴力）
// ============================================================

void generate_comparison_image(const std::string& output_path, int num_spheres) {
    std::cout << "\n=== BVH 加速光线追踪对比 ===\n";
    std::cout << "场景球体数量: " << num_spheres << "\n";
    
    // 生成场景
    Scene scene = generate_scene(num_spheres);
    scene.build_bvh();
    std::cout << "BVH 节点数量: " << scene.bvh->nodes.size() << "\n";
    
    // 相机设置
    Vec3 look_from(13, 2, 3);
    Vec3 look_at(0, 0, 0);
    double dist_to_focus = 10.0;
    double aperture = 0.1;
    
    Camera cam(look_from, look_at, {0,1,0}, 20, 2.0, aperture, dist_to_focus);
    
    int W = 400, H = 200;
    int samples = 4;
    int max_depth = 8;
    
    Image img_bvh(W, H);
    Image img_brute(W, H);
    
    std::cout << "\n渲染 BVH 版本...\n";
    auto stats_bvh = render(img_bvh, scene, cam, samples, max_depth, true);
    
    std::cout << "渲染暴力版本...\n";
    auto stats_brute = render(img_brute, scene, cam, samples, max_depth, false);
    
    // 合并对比图（左BVH，右暴力）
    Image combined(W * 2, H);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            combined.at(x, y) = img_bvh.at(x, y);
            combined.at(x + W, y) = img_brute.at(x, y);
        }
    }
    
    combined.save_png(output_path);
    
    // 性能报告
    std::cout << "\n=== 性能对比 ===\n";
    std::cout << "BVH 版本:\n";
    std::cout << "  渲染时间: " << stats_bvh.render_time_ms << " ms\n";
    std::cout << "  AABB 测试/光线: " << stats_bvh.tests_per_ray << "\n";
    std::cout << "  总测试次数: " << stats_bvh.total_tests << "\n";
    
    std::cout << "\n暴力版本:\n";
    std::cout << "  渲染时间: " << stats_brute.render_time_ms << " ms\n";
    std::cout << "  球体测试/光线: " << num_spheres << " (固定)\n";
    std::cout << "  总测试次数: " << stats_brute.total_tests << "\n";
    
    double speedup = stats_brute.render_time_ms / stats_bvh.render_time_ms;
    std::cout << "\n加速比: " << speedup << "x\n";
    
    if (speedup > 1.0) {
        std::cout << "✅ BVH 比暴力遍历快 " << speedup << " 倍\n";
    } else {
        std::cout << "⚠️  小场景下 BVH 开销可能大于收益\n";
    }
}

// ============================================================
// 生成 BVH 可视化（调试用）
// ============================================================

void visualize_bvh(const std::string& output_path, int num_spheres) {
    std::cout << "\n生成 BVH 可视化...\n";
    
    Scene scene = generate_scene(num_spheres);
    scene.build_bvh();
    
    // 从上方俯视渲染 BVH 层级
    int W = 600, H = 600;
    Image img(W, H);
    
    // 绘制球体位置（2D 俯视图）
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            img.at(x, y) = {0.1, 0.1, 0.15}; // 深蓝背景
        }
    }
    
    // 坐标映射：世界坐标 [-13, 13] -> 图像坐标 [0, W]
    auto world_to_img = [&](double wx, double wz, int& px, int& py) {
        px = (int)((wx + 13) / 26.0 * W);
        py = (int)((wz + 13) / 26.0 * H);
    };
    
    // 绘制 AABB（选取前几层）
    std::vector<std::pair<int, int>> layer_nodes; // (depth, node_idx)
    std::vector<int> stack_nodes;
    std::vector<int> stack_depths;
    
    if (!scene.bvh->nodes.empty()) {
        stack_nodes.push_back(0);
        stack_depths.push_back(0);
    }
    
    while (!stack_nodes.empty()) {
        int ni = stack_nodes.back(); stack_nodes.pop_back();
        int depth = stack_depths.back(); stack_depths.pop_back();
        
        if (depth > 3) continue; // 只画前4层
        
        const BVHNode& node = scene.bvh->nodes[ni];
        
        // 绘制 AABB 边界（只用 XZ 平面）
        Vec3 colors[] = {
            {1.0, 0.3, 0.3}, // 红 - 第0层
            {0.3, 1.0, 0.3}, // 绿 - 第1层
            {0.3, 0.3, 1.0}, // 蓝 - 第2层
            {1.0, 1.0, 0.3}, // 黄 - 第3层
        };
        Vec3 color = colors[std::min(depth, 3)];
        double alpha = 1.0 / (depth + 1.0);
        
        int x0, y0, x1, y1;
        world_to_img(node.bbox.min_pt.x, node.bbox.min_pt.z, x0, y0);
        world_to_img(node.bbox.max_pt.x, node.bbox.max_pt.z, x1, y1);
        
        // 确保范围有效
        x0 = std::max(0, std::min(W-1, x0));
        x1 = std::max(0, std::min(W-1, x1));
        y0 = std::max(0, std::min(H-1, y0));
        y1 = std::max(0, std::min(H-1, y1));
        
        // 画矩形边框
        for (int px = x0; px <= x1; px++) {
            if (y0 >= 0 && y0 < H) {
                Vec3& p0 = img.at(px, y0);
                p0 = p0 * (1-alpha) + color * alpha;
            }
            if (y1 >= 0 && y1 < H) {
                Vec3& p1 = img.at(px, y1);
                p1 = p1 * (1-alpha) + color * alpha;
            }
        }
        for (int py = y0; py <= y1; py++) {
            if (x0 >= 0 && x0 < W) {
                Vec3& p0 = img.at(x0, py);
                p0 = p0 * (1-alpha) + color * alpha;
            }
            if (x1 >= 0 && x1 < W) {
                Vec3& p1 = img.at(x1, py);
                p1 = p1 * (1-alpha) + color * alpha;
            }
        }
        
        if (!node.is_leaf) {
            if (node.left >= 0)  { stack_nodes.push_back(node.left);  stack_depths.push_back(depth+1); }
            if (node.right >= 0) { stack_nodes.push_back(node.right); stack_depths.push_back(depth+1); }
        }
    }
    
    // 绘制球体位置（白色圆点）
    for (size_t i = 1; i < scene.spheres.size(); i++) { // 跳过地面
        const auto& s = scene.spheres[i];
        int px, py;
        world_to_img(s.center.x, s.center.z, px, py);
        
        int r = std::max(1, (int)(s.radius * W / 26.0));
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                if (dx*dx + dy*dy <= r*r) {
                    int nx = px + dx, ny = py + dy;
                    if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
                        // 根据材质颜色
                        Vec3 c = s.mat.albedo;
                        if (s.mat.type == Material::GLASS) c = {0.8, 0.9, 1.0};
                        img.at(nx, ny) = c;
                    }
                }
            }
        }
    }
    
    img.save_png(output_path);
    std::cout << "✅ BVH 可视化已保存\n";
}

// ============================================================
// 主函数
// ============================================================

int main() {
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║  BVH Accelerated Ray Tracer - 2026-03-01  ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n\n";
    
    // 生成主渲染对比图（50个球体，展示性能）
    generate_comparison_image("bvh_comparison.png", 50);
    
    // 生成 BVH 结构可视化
    visualize_bvh("bvh_visualization.png", 50);
    
    // 额外：生成高质量单张（BVH加速）
    std::cout << "\n生成高质量最终渲染...\n";
    Scene scene = generate_scene(80);
    scene.build_bvh();
    
    Vec3 look_from(13, 2, 3);
    Vec3 look_at(0, 0, 0);
    Camera cam(look_from, look_at, {0,1,0}, 20, 16.0/9.0, 0.1, 10.0);
    
    int W = 800, H = 450;
    int samples = 8;
    int max_depth = 10;
    
    Image img(W, H);
    auto stats = render(img, scene, cam, samples, max_depth, true);
    img.save_png("bvh_output.png");
    
    std::cout << "\n最终渲染统计:\n";
    std::cout << "  分辨率: " << W << "x" << H << "\n";
    std::cout << "  采样数: " << samples << "\n";
    std::cout << "  场景球体: " << scene.spheres.size() << "\n";
    std::cout << "  BVH节点: " << scene.bvh->nodes.size() << "\n";
    std::cout << "  渲染时间: " << stats.render_time_ms << " ms\n";
    std::cout << "  平均 AABB 测试/光线: " << stats.tests_per_ray << "\n";
    
    std::cout << "\n✅ 所有输出文件已生成:\n";
    std::cout << "  - bvh_comparison.png   (左:BVH, 右:暴力 对比图)\n";
    std::cout << "  - bvh_visualization.png (BVH包围盒结构可视化)\n";
    std::cout << "  - bvh_output.png        (高质量最终渲染)\n";
    
    return 0;
}
