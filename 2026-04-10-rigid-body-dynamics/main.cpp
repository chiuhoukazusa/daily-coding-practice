/*
 * Rigid Body Dynamics 2D
 * 
 * 功能：
 * - 2D 刚体动力学模拟（矩形、圆形刚体）
 * - SAT（分离轴定理）碰撞检测
 * - 脉冲解算（Impulse-based collision resolution）
 * - 摩擦力模拟
 * - 重力 + 积分（半隐式 Euler）
 * - 软光栅化输出序列帧 + 合成图
 *
 * 输出：
 * - rigid_body_output.png     (时序合成图 - 多帧对比)
 * - rigid_body_sequence_*.png (序列帧)
 */

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <algorithm>
#include <limits>
#include <cstdio>
#include <cassert>
#include <random>

// ─── STB Image Write ──────────────────────────────────────────────────────────
#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ─── Math Types ──────────────────────────────────────────────────────────────
struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s) const { return {x*s, y*s}; }
    Vec2 operator/(float s) const { return {x/s, y/s}; }
    Vec2& operator+=(const Vec2& o) { x+=o.x; y+=o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x-=o.x; y-=o.y; return *this; }
    float dot(const Vec2& o) const { return x*o.x + y*o.y; }
    float cross(const Vec2& o) const { return x*o.y - y*o.x; } // 2D cross product (scalar)
    float length() const { return std::sqrt(x*x + y*y); }
    Vec2 normalized() const { float l = length(); return l > 1e-8f ? (*this / l) : Vec2(0,0); }
    Vec2 perp() const { return {-y, x}; } // perpendicular (rotate 90°)
};
Vec2 operator*(float s, const Vec2& v) { return {s*v.x, s*v.y}; }

// ─── Color ───────────────────────────────────────────────────────────────────
struct Color {
    uint8_t r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
};

// ─── Canvas ──────────────────────────────────────────────────────────────────
struct Canvas {
    int width, height;
    std::vector<Color> pixels;

    Canvas(int w, int h, Color bg = {30, 30, 40}) 
        : width(w), height(h), pixels(w * h, bg) {}

    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        pixels[y * width + x] = c;
    }
    Color getPixel(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return {0,0,0};
        return pixels[y * width + x];
    }

    // Blend pixel with alpha
    void blendPixel(int x, int y, Color c, float alpha) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        Color& p = pixels[y * width + x];
        p.r = uint8_t(p.r * (1-alpha) + c.r * alpha);
        p.g = uint8_t(p.g * (1-alpha) + c.g * alpha);
        p.b = uint8_t(p.b * (1-alpha) + c.b * alpha);
    }

    // Draw line (Bresenham)
    void drawLine(int x0, int y0, int x1, int y1, Color c, float alpha = 1.0f) {
        int dx = std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = -std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
        int err = dx+dy;
        while (true) {
            blendPixel(x0, y0, c, alpha);
            if (x0==x1 && y0==y1) break;
            int e2 = 2*err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // Draw thick line
    void drawThickLine(Vec2 a, Vec2 b, Color c, int thickness = 2) {
        Vec2 dir = (b - a).normalized();
        Vec2 perp = dir.perp();
        for (int t = -thickness/2; t <= thickness/2; t++) {
            Vec2 offset = perp * float(t);
            drawLine(int(a.x+offset.x), int(a.y+offset.y),
                     int(b.x+offset.x), int(b.y+offset.y), c);
        }
    }

    // Fill circle
    void fillCircle(int cx, int cy, int r, Color c) {
        for (int y = cy-r; y <= cy+r; y++) {
            for (int x = cx-r; x <= cx+r; x++) {
                int dx = x-cx, dy = y-cy;
                if (dx*dx + dy*dy <= r*r)
                    setPixel(x, y, c);
            }
        }
    }

    // Draw circle outline (anti-aliased)
    void drawCircleOutline(int cx, int cy, int r, Color c, int thickness = 2) {
        for (float angle = 0; angle < 2*M_PI; angle += 0.005f) {
            for (int t = -thickness/2; t <= thickness/2; t++) {
                int x = int(cx + (r+t)*std::cos(angle));
                int y = int(cy + (r+t)*std::sin(angle));
                setPixel(x, y, c);
            }
        }
    }

    // Fill convex polygon (given vertices in world space, already in pixel coords)
    void fillPolygon(const std::vector<Vec2>& verts, Color c) {
        if (verts.size() < 3) return;
        int minY = height, maxY = 0;
        for (auto& v : verts) {
            minY = std::min(minY, (int)v.y);
            maxY = std::max(maxY, (int)v.y);
        }
        minY = std::max(0, minY);
        maxY = std::min(height-1, maxY);

        for (int y = minY; y <= maxY; y++) {
            std::vector<float> xs;
            int n = (int)verts.size();
            for (int i = 0; i < n; i++) {
                Vec2 a = verts[i], b = verts[(i+1)%n];
                if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                    float t = (y - a.y) / (b.y - a.y);
                    xs.push_back(a.x + t * (b.x - a.x));
                }
            }
            std::sort(xs.begin(), xs.end());
            for (int i = 0; i+1 < (int)xs.size(); i += 2) {
                int x0 = std::max(0, (int)xs[i]);
                int x1 = std::min(width-1, (int)xs[i+1]);
                for (int x = x0; x <= x1; x++)
                    setPixel(x, y, c);
            }
        }
    }

    // Draw polygon outline
    void drawPolygonOutline(const std::vector<Vec2>& verts, Color c, int thickness=2) {
        int n = (int)verts.size();
        for (int i = 0; i < n; i++) {
            drawThickLine(verts[i], verts[(i+1)%n], c, thickness);
        }
    }

    bool save(const std::string& filename) {
        return stbi_write_png(filename.c_str(), width, height, 3, pixels.data(), width * 3) != 0;
    }
};

// ─── Physics World Size ──────────────────────────────────────────────────────
const int WORLD_W = 600;
const int WORLD_H = 600;

// Physics coords: (0,0) bottom-left, (WORLD_W, WORLD_H) top-right
// Screen coords: y is flipped

Vec2 worldToScreen(Vec2 p, int /*w*/ = WORLD_W, int h = WORLD_H) {
    return {p.x, float(h) - p.y};
}

// ─── Rigid Body Types ────────────────────────────────────────────────────────
enum class ShapeType { RECT, CIRCLE };

struct RigidBody {
    ShapeType shape;
    
    // Position (center), velocity
    Vec2 pos, vel;
    float angle, angularVel;  // radians, rad/s
    
    // Mass properties
    float mass, invMass;
    float inertia, invInertia;  // moment of inertia
    
    // Shape params
    float hw, hh;   // half-width, half-height (for rect)
    float radius;   // for circle
    
    // Material
    float restitution;  // bounciness 0..1
    float friction;     // friction 0..1
    
    // For rendering
    Color color;
    
    bool isStatic;  // infinite mass (ground, walls)

    // Construct rect
    static RigidBody makeRect(Vec2 pos, float hw, float hh, float mass_val, Color col,
                               float e = 0.3f, float f = 0.4f) {
        RigidBody b;
        b.shape = ShapeType::RECT;
        b.pos = pos; b.vel = {0,0};
        b.angle = 0; b.angularVel = 0;
        b.hw = hw; b.hh = hh; b.radius = 0;
        b.mass = mass_val;
        b.invMass = (mass_val > 0) ? 1.0f/mass_val : 0;
        // I = (1/12) * m * (w^2 + h^2)
        b.inertia = (mass_val > 0) ? (mass_val * (4*hw*hw + 4*hh*hh) / 12.0f) : 0;
        b.invInertia = (b.inertia > 0) ? 1.0f/b.inertia : 0;
        b.restitution = e;
        b.friction = f;
        b.color = col;
        b.isStatic = (mass_val <= 0);
        return b;
    }

    // Construct circle
    static RigidBody makeCircle(Vec2 pos, float r, float mass_val, Color col,
                                 float e = 0.4f, float f = 0.3f) {
        RigidBody b;
        b.shape = ShapeType::CIRCLE;
        b.pos = pos; b.vel = {0,0};
        b.angle = 0; b.angularVel = 0;
        b.hw = 0; b.hh = 0; b.radius = r;
        b.mass = mass_val;
        b.invMass = (mass_val > 0) ? 1.0f/mass_val : 0;
        // I = 0.5 * m * r^2
        b.inertia = (mass_val > 0) ? (0.5f * mass_val * r * r) : 0;
        b.invInertia = (b.inertia > 0) ? 1.0f/b.inertia : 0;
        b.restitution = e;
        b.friction = f;
        b.color = col;
        b.isStatic = (mass_val <= 0);
        return b;
    }

    // Get rect vertices in world space
    std::vector<Vec2> getVertices() const {
        float c = std::cos(angle), s = std::sin(angle);
        std::vector<Vec2> verts(4);
        Vec2 corners[4] = {{-hw,-hh},{hw,-hh},{hw,hh},{-hw,hh}};
        for (int i = 0; i < 4; i++) {
            verts[i] = pos + Vec2(c*corners[i].x - s*corners[i].y,
                                  s*corners[i].x + c*corners[i].y);
        }
        return verts;
    }

    // Apply impulse at contact point (in world space)
    void applyImpulse(Vec2 impulse, Vec2 contactPoint) {
        if (isStatic) return;
        vel += impulse * invMass;
        Vec2 r = contactPoint - pos;
        angularVel += invInertia * r.cross(impulse);
    }
};

// ─── Collision Detection ─────────────────────────────────────────────────────
struct ContactInfo {
    bool hasContact;
    Vec2 normal;      // from B to A
    float depth;      // penetration depth
    Vec2 contactPoint;
};

// Project vertices onto axis, return [min, max]
std::pair<float,float> projectOnAxis(const std::vector<Vec2>& verts, Vec2 axis) {
    float mn = std::numeric_limits<float>::max();
    float mx = -std::numeric_limits<float>::max();
    for (auto& v : verts) {
        float p = v.dot(axis);
        mn = std::min(mn, p);
        mx = std::max(mx, p);
    }
    return {mn, mx};
}

// SAT test between two convex polygons
ContactInfo satRectRect(const RigidBody& a, const RigidBody& b) {
    auto vertsA = a.getVertices();
    auto vertsB = b.getVertices();

    // Collect axes (normals of each edge)
    std::vector<Vec2> axes;
    auto addAxes = [&](const std::vector<Vec2>& verts) {
        int n = (int)verts.size();
        for (int i = 0; i < n; i++) {
            Vec2 edge = verts[(i+1)%n] - verts[i];
            axes.push_back(edge.perp().normalized());
        }
    };
    addAxes(vertsA);
    addAxes(vertsB);

    float minOverlap = std::numeric_limits<float>::max();
    Vec2 bestAxis;

    for (auto& axis : axes) {
        auto [minA, maxA] = projectOnAxis(vertsA, axis);
        auto [minB, maxB] = projectOnAxis(vertsB, axis);
        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0) return {false, {}, 0, {}};
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    // Ensure normal points from B to A
    Vec2 d = a.pos - b.pos;
    if (d.dot(bestAxis) < 0) bestAxis = bestAxis * -1;

    // Approximate contact point as midpoint of overlap region
    auto [minA, maxA] = projectOnAxis(vertsA, bestAxis);
    auto [minB, maxB] = projectOnAxis(vertsB, bestAxis);
    float overlapStart = std::max(minA, minB);
    float overlapEnd = std::min(maxA, maxB);
    float cpProj = (overlapStart + overlapEnd) * 0.5f;

    // Find perp axis to get contact point (center of overlap in 2D)
    Vec2 perp = bestAxis.perp();
    auto [minAp, maxAp] = projectOnAxis(vertsA, perp);
    auto [minBp, maxBp] = projectOnAxis(vertsB, perp);
    float perpOverlapStart = std::max(minAp, minBp);
    float perpOverlapEnd = std::min(maxAp, maxBp);
    float cpPerp = (perpOverlapStart + perpOverlapEnd) * 0.5f;

    Vec2 cp = bestAxis * cpProj + perp * cpPerp;

    return {true, bestAxis, minOverlap, cp};
}

// Circle vs Circle
ContactInfo circleCircle(const RigidBody& a, const RigidBody& b) {
    Vec2 d = a.pos - b.pos;
    float dist = d.length();
    float sumR = a.radius + b.radius;
    if (dist >= sumR) return {false, {}, 0, {}};
    Vec2 normal = dist > 1e-8f ? d.normalized() : Vec2(0, 1);
    float depth = sumR - dist;
    Vec2 cp = b.pos + normal * b.radius;
    return {true, normal, depth, cp};
}

// Circle vs Rect (SAT-based)
ContactInfo circleRect(const RigidBody& circle, const RigidBody& rect) {
    // Transform circle center to rect local space
    float c = std::cos(-rect.angle), s = std::sin(-rect.angle);
    Vec2 d = circle.pos - rect.pos;
    Vec2 local = {c*d.x - s*d.y, s*d.x + c*d.y};

    // Clamp to rect extents
    Vec2 closest = {
        std::max(-rect.hw, std::min(rect.hw, local.x)),
        std::max(-rect.hh, std::min(rect.hh, local.y))
    };

    Vec2 diff = local - closest;
    float dist = diff.length();

    if (dist >= circle.radius) return {false, {}, 0, {}};

    // Circle center inside rect
    if (dist < 1e-8f) {
        // Push out along smallest axis
        float ox = rect.hw - std::abs(local.x);
        float oy = rect.hh - std::abs(local.y);
        Vec2 normal_local;
        float depth;
        if (ox < oy) {
            normal_local = {(local.x > 0) ? 1.0f : -1.0f, 0};
            depth = ox + circle.radius;
        } else {
            normal_local = {0, (local.y > 0) ? 1.0f : -1.0f};
            depth = oy + circle.radius;
        }
        // Rotate normal back to world space
        float ca = std::cos(rect.angle), sa = std::sin(rect.angle);
        Vec2 normal = {ca*normal_local.x - sa*normal_local.y,
                       sa*normal_local.x + ca*normal_local.y};
        Vec2 cp = circle.pos - normal * circle.radius;
        return {true, normal, depth, cp};
    }

    // Normal in local space = diff direction
    Vec2 normal_local = diff.normalized();
    float depth = circle.radius - dist;

    // Rotate back to world space
    float ca = std::cos(rect.angle), sa = std::sin(rect.angle);
    Vec2 normal = {ca*normal_local.x - sa*normal_local.y,
                   sa*normal_local.x + ca*normal_local.y};
    Vec2 cp = circle.pos - normal * circle.radius;
    return {true, normal, depth, cp};
}

// Dispatch collision detection
ContactInfo detectCollision(const RigidBody& a, const RigidBody& b) {
    if (a.shape == ShapeType::RECT && b.shape == ShapeType::RECT)
        return satRectRect(a, b);
    if (a.shape == ShapeType::CIRCLE && b.shape == ShapeType::CIRCLE)
        return circleCircle(a, b);
    if (a.shape == ShapeType::CIRCLE && b.shape == ShapeType::RECT)
        return circleRect(a, b);
    if (a.shape == ShapeType::RECT && b.shape == ShapeType::CIRCLE) {
        // Swap, then negate normal
        auto info = circleRect(b, a);
        if (info.hasContact) info.normal = info.normal * -1;
        return info;
    }
    return {false, {}, 0, {}};
}

// ─── Impulse Resolution ──────────────────────────────────────────────────────
void resolveCollision(RigidBody& a, RigidBody& b, const ContactInfo& info) {
    if (!info.hasContact) return;

    Vec2 n = info.normal;  // from B to A
    Vec2 rA = info.contactPoint - a.pos;
    Vec2 rB = info.contactPoint - b.pos;

    // Relative velocity at contact point
    Vec2 vA = a.vel + Vec2(-a.angularVel * rA.y, a.angularVel * rA.x);
    Vec2 vB = b.vel + Vec2(-b.angularVel * rB.y, b.angularVel * rB.x);
    Vec2 relVel = vA - vB;

    float velAlongNormal = relVel.dot(n);
    if (velAlongNormal > 0) return;  // Separating, no impulse needed

    float e = std::min(a.restitution, b.restitution);

    // Denominator for impulse magnitude
    float rACrossN = rA.cross(n);
    float rBCrossN = rB.cross(n);
    float invMassSum = a.invMass + b.invMass 
                     + rACrossN * rACrossN * a.invInertia
                     + rBCrossN * rBCrossN * b.invInertia;

    if (invMassSum < 1e-10f) return;

    float j = -(1 + e) * velAlongNormal / invMassSum;

    // Normal impulse
    Vec2 impulse = n * j;
    a.applyImpulse(impulse, info.contactPoint);
    b.applyImpulse(impulse * -1, info.contactPoint);

    // Friction impulse
    Vec2 tangent = (relVel - n * relVel.dot(n)).normalized();
    float velAlongTangent = relVel.dot(tangent);

    float rACrossT = rA.cross(tangent);
    float rBCrossT = rB.cross(tangent);
    float invMassSumT = a.invMass + b.invMass 
                      + rACrossT * rACrossT * a.invInertia
                      + rBCrossT * rBCrossT * b.invInertia;

    if (invMassSumT < 1e-10f) return;

    float jt = -velAlongTangent / invMassSumT;
    float mu = std::sqrt(a.friction * b.friction);

    Vec2 frictionImpulse;
    if (std::abs(jt) < j * mu) {
        frictionImpulse = tangent * jt;  // static friction
    } else {
        frictionImpulse = tangent * (-j * mu);  // kinetic friction
    }

    a.applyImpulse(frictionImpulse, info.contactPoint);
    b.applyImpulse(frictionImpulse * -1, info.contactPoint);

    // Positional correction (Baumgarte)
    const float slop = 0.5f;
    const float percent = 0.4f;
    float correction = std::max(info.depth - slop, 0.0f) / 
                       (a.invMass + b.invMass) * percent;
    Vec2 correctionVec = n * correction;
    if (!a.isStatic) a.pos += correctionVec * a.invMass;
    if (!b.isStatic) b.pos -= correctionVec * b.invMass;
}

// ─── Drawing ─────────────────────────────────────────────────────────────────
void drawBody(Canvas& canvas, const RigidBody& body) {
    if (body.shape == ShapeType::RECT) {
        auto verts = body.getVertices();
        std::vector<Vec2> screenVerts(verts.size());
        for (size_t i = 0; i < verts.size(); i++)
            screenVerts[i] = worldToScreen(verts[i]);

        // Slightly darker fill
        Color fill = {
            uint8_t(body.color.r * 0.6f),
            uint8_t(body.color.g * 0.6f),
            uint8_t(body.color.b * 0.6f)
        };
        canvas.fillPolygon(screenVerts, fill);
        canvas.drawPolygonOutline(screenVerts, body.color, 2);

        // Draw orientation indicator
        Vec2 center = worldToScreen(body.pos);
        float c = std::cos(body.angle), s = std::sin(body.angle);
        Vec2 dir = {c * body.hw, s * body.hw};
        Vec2 tipWorld = body.pos + dir;
        Vec2 tip = worldToScreen(tipWorld);
        canvas.drawThickLine(center, tip, {255,255,255}, 2);
    } else {
        Vec2 sc = worldToScreen(body.pos);
        int r = int(body.radius);
        Color fill = {
            uint8_t(body.color.r * 0.6f),
            uint8_t(body.color.g * 0.6f),
            uint8_t(body.color.b * 0.6f)
        };
        canvas.fillCircle(int(sc.x), int(sc.y), r, fill);
        canvas.drawCircleOutline(int(sc.x), int(sc.y), r, body.color, 2);
        // Orientation indicator
        float c = std::cos(body.angle), s = std::sin(body.angle);
        Vec2 tip = worldToScreen(body.pos + Vec2(c*body.radius*0.7f, s*body.radius*0.7f));
        canvas.drawThickLine(sc, tip, {255,255,255}, 2);
    }
}

void drawScene(Canvas& canvas, const std::vector<RigidBody>& bodies, int frame, float t) {
    // Background grid
    for (int x = 0; x < WORLD_W; x += 50) {
        for (int y = 0; y < WORLD_H; y++) {
            canvas.blendPixel(x, y, {60,60,70}, 0.4f);
        }
    }
    for (int y = 0; y < WORLD_H; y += 50) {
        for (int x = 0; x < WORLD_W; x++) {
            canvas.blendPixel(x, y, {60,60,70}, 0.4f);
        }
    }

    // Draw all bodies
    for (auto& b : bodies) {
        drawBody(canvas, b);
    }

    // Draw frame info
    char buf[64];
    std::snprintf(buf, sizeof(buf), "t=%.2fs", t);
    // Simple text rendering using setPixel for digits
    // (minimal, just dots)
    (void)buf; (void)frame;
}

// ─── Physics Step ────────────────────────────────────────────────────────────
const float GRAVITY = -600.0f; // pixels/s^2

void physicsStep(std::vector<RigidBody>& bodies, float dt) {
    // Apply gravity and integrate
    for (auto& b : bodies) {
        if (b.isStatic) continue;
        b.vel.y += GRAVITY * dt;
        b.pos += b.vel * dt;
        b.angle += b.angularVel * dt;

        // Angular damping
        b.angularVel *= std::pow(0.98f, dt * 60.0f);
    }

    // Detect and resolve collisions (multiple iterations for stability)
    for (int iter = 0; iter < 10; iter++) {
        for (int i = 0; i < (int)bodies.size(); i++) {
            for (int j = i+1; j < (int)bodies.size(); j++) {
                auto info = detectCollision(bodies[i], bodies[j]);
                if (info.hasContact) {
                    resolveCollision(bodies[i], bodies[j], info);
                }
            }
        }
    }

    // World boundary clamp (keep bodies in world)
    for (auto& b : bodies) {
        if (b.isStatic) continue;
        // Simple bounds check for circles
        if (b.shape == ShapeType::CIRCLE) {
            if (b.pos.x - b.radius < 0) { b.pos.x = b.radius; if (b.vel.x < 0) b.vel.x *= -0.5f; }
            if (b.pos.x + b.radius > WORLD_W) { b.pos.x = WORLD_W - b.radius; if (b.vel.x > 0) b.vel.x *= -0.5f; }
            if (b.pos.y - b.radius < 0) { b.pos.y = b.radius; if (b.vel.y < 0) b.vel.y *= -0.5f; }
            if (b.pos.y + b.radius > WORLD_H) { b.pos.y = WORLD_H - b.radius; if (b.vel.y > 0) b.vel.y *= -0.5f; }
        }
    }
}

// ─── Scene Setup ─────────────────────────────────────────────────────────────
std::vector<RigidBody> createScene() {
    std::vector<RigidBody> bodies;

    // ── Static objects (ground, walls, ramps) ──

    // Ground
    auto ground = RigidBody::makeRect({300, 10}, 295, 10, 0, {80, 120, 80});
    ground.isStatic = true; ground.invMass = 0; ground.invInertia = 0;
    bodies.push_back(ground);

    // Left wall
    auto wallL = RigidBody::makeRect({10, 300}, 10, 290, 0, {80, 80, 120});
    wallL.isStatic = true; wallL.invMass = 0; wallL.invInertia = 0;
    bodies.push_back(wallL);

    // Right wall
    auto wallR = RigidBody::makeRect({590, 300}, 10, 290, 0, {80, 80, 120});
    wallR.isStatic = true; wallR.invMass = 0; wallR.invInertia = 0;
    bodies.push_back(wallR);

    // Ramp (angled platform)
    auto ramp = RigidBody::makeRect({200, 150}, 90, 10, 0, {100, 80, 60});
    ramp.angle = 0.4f; // ~23°
    ramp.isStatic = true; ramp.invMass = 0; ramp.invInertia = 0;
    bodies.push_back(ramp);

    // Another platform
    auto platform = RigidBody::makeRect({420, 220}, 70, 8, 0, {100, 80, 60});
    platform.isStatic = true; platform.invMass = 0; platform.invInertia = 0;
    bodies.push_back(platform);

    // ── Dynamic objects ──

    // Stack of boxes
    for (int i = 0; i < 4; i++) {
        Color col = {uint8_t(80 + i*40), uint8_t(120 + i*20), uint8_t(200)};
        auto box = RigidBody::makeRect({120.0f + i*2.0f, 40.0f + i*60.0f}, 25, 25, 2.0f, col);
        box.vel = {float(i)*10, 0};
        bodies.push_back(box);
    }

    // Circles
    auto c1 = RigidBody::makeCircle({350, 450}, 28, 1.5f, {230, 100, 100});
    c1.vel = {-80, 50};
    bodies.push_back(c1);

    auto c2 = RigidBody::makeCircle({450, 380}, 20, 1.0f, {100, 230, 100});
    c2.vel = {-60, -30};
    bodies.push_back(c2);

    auto c3 = RigidBody::makeCircle({490, 480}, 18, 0.8f, {230, 230, 100});
    c3.vel = {-100, 20};
    bodies.push_back(c3);

    // A heavy rect that will knock things around
    auto bigBox = RigidBody::makeRect({300, 500}, 40, 35, 5.0f, {200, 150, 80});
    bigBox.vel = {30, -20};
    bigBox.angularVel = 0.5f;
    bodies.push_back(bigBox);

    // Small fast circle
    auto fastCirc = RigidBody::makeCircle({500, 520}, 15, 0.5f, {180, 100, 220});
    fastCirc.vel = {-150, 100};
    bodies.push_back(fastCirc);

    return bodies;
}

// ─── Main ────────────────────────────────────────────────────────────────────
int main() {
    // Simulation parameters
    const float DT = 1.0f / 60.0f;
    const int TOTAL_FRAMES = 360;  // 6 seconds at 60fps

    std::vector<RigidBody> bodies = createScene();

    // Select frames to save: t=0, 1, 2, 3, 4, 5 seconds
    std::vector<int> saveAtFrames = {0, 60, 120, 180, 240, 300};
    std::vector<Canvas> savedFrames;

    printf("Simulating Rigid Body Dynamics 2D...\n");
    printf("Bodies: %zu\n", bodies.size());

    int savedCount = 0;

    for (int frame = 0; frame <= TOTAL_FRAMES; frame++) {
        float t = frame * DT;

        // Check if we should save this frame
        bool shouldSave = false;
        for (int sf : saveAtFrames) {
            if (frame == sf) { shouldSave = true; break; }
        }

        if (shouldSave) {
            Canvas canvas(WORLD_W, WORLD_H);
            drawScene(canvas, bodies, frame, t);

            // Save individual frame
            char filename[64];
            std::snprintf(filename, sizeof(filename), "rigid_body_seq_%03d.png", savedCount);
            canvas.save(filename);
            printf("  Saved frame %d (t=%.1fs) -> %s\n", frame, t, filename);
            savedFrames.push_back(canvas);
            savedCount++;
        }

        if (frame < TOTAL_FRAMES)
            physicsStep(bodies, DT);
    }

    printf("Saved %d sequence frames\n", savedCount);

    // Create comparison output: 2x3 grid of 6 frames
    // Each frame is WORLD_W x WORLD_H = 600x600
    // Grid: 3 columns x 2 rows = 1800 x 1200
    const int COLS = 3, ROWS = 2;
    const int MARGIN = 4;
    int outW = COLS * WORLD_W + (COLS + 1) * MARGIN;
    int outH = ROWS * WORLD_H + (ROWS + 1) * MARGIN;
    Canvas output(outW, outH, {20, 20, 28});

    for (int i = 0; i < (int)savedFrames.size() && i < COLS * ROWS; i++) {
        int col = i % COLS;
        int row = i / COLS;
        int offX = MARGIN + col * (WORLD_W + MARGIN);
        int offY = MARGIN + row * (WORLD_H + MARGIN);

        // Copy frame pixels
        auto& src = savedFrames[i];
        for (int y = 0; y < WORLD_H; y++) {
            for (int x = 0; x < WORLD_W; x++) {
                output.setPixel(offX + x, offY + y, src.getPixel(x, y));
            }
        }

        // Draw time label (simple box with frame time)
        float t = saveAtFrames[i] * DT;
        (void)t;
        // Draw border
        for (int x = offX-1; x < offX+WORLD_W+1; x++) {
            output.setPixel(x, offY-1, {100, 100, 120});
            output.setPixel(x, offY+WORLD_H, {100, 100, 120});
        }
        for (int y = offY-1; y < offY+WORLD_H+1; y++) {
            output.setPixel(offX-1, y, {100, 100, 120});
            output.setPixel(offX+WORLD_W, y, {100, 100, 120});
        }
    }

    output.save("rigid_body_output.png");
    printf("Saved: rigid_body_output.png (%dx%d)\n", outW, outH);

    // Print some stats for validation
    printf("\n=== Simulation Stats ===\n");
    for (size_t i = 0; i < bodies.size(); i++) {
        auto& b = bodies[i];
        if (!b.isStatic) {
            printf("Body %zu: pos=(%.1f,%.1f) vel=(%.1f,%.1f) angle=%.2f\n",
                   i, b.pos.x, b.pos.y, b.vel.x, b.vel.y, b.angle);
        }
    }

    return 0;
}
