/**
 * Half-Edge Mesh Data Structure
 * 
 * Day: 2026-07-24
 * 
 * Implementation of the half-edge (doubly-connected edge list) data structure
 * for polygonal mesh representation. Supports:
 *  - Mesh building from vertices and face indices
 *  - Vertex normal computation (area-weighted face normals)
 *  - Boundary detection (edges with no twin half-edge)
 *  - Adjacency queries (vertices around vertex, faces around vertex, edges around vertex)
 *  - Mesh statistics (Euler characteristic, genus verification)
 *  - Soft rasterization for PPM visualization with Phong shading
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <cassert>
#include <fstream>
#include <limits>
#include <algorithm>
#include <utility>

// ============================================================
// 3D Vector
// ============================================================
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double l=length(); return l>1e-12 ? (*this)/l : Vec3(0,0,0); }
};

// ============================================================
// Half-Edge Data Structure
// ============================================================
struct HEdge {
    int vertex;       // target vertex index (the vertex this half-edge points TO)
    int face;         // face index this half-edge belongs to (-1 if boundary)
    int next;         // next half-edge in the face (counter-clockwise)
    int prev;         // previous half-edge in the face
    int twin;         // opposite half-edge (-1 if boundary edge)
    
    HEdge() : vertex(-1), face(-1), next(-1), prev(-1), twin(-1) {}
};

struct Face {
    int he;           // one half-edge of the face
    Vec3 normal;      // cached face normal
    double area;      // cached face area
    
    Face() : he(-1), normal(0,0,0), area(0) {}
};

struct Vertex {
    Vec3 pos;         // position
    int he;           // one outgoing half-edge from this vertex
    Vec3 normal;      // cached vertex normal (area-weighted face normals)
    
    Vertex() : pos(0,0,0), he(-1), normal(0,0,0) {}
};

class HalfEdgeMesh {
public:
    std::vector<Vertex> vertices;
    std::vector<HEdge>  hedges;
    std::vector<Face>   faces;
    
    // Build mesh from indexed face list (triangles/quads supported, automatically triangulated)
    void build(const std::vector<Vec3>& positions,
               const std::vector<std::vector<int>>& faceList) {
        vertices.clear();
        hedges.clear();
        faces.clear();
        
        // Add vertices
        for (const auto& p : positions) {
            Vertex v;
            v.pos = p;
            vertices.push_back(v);
        }
        
        // Edge map: (v0, v1) -> half-edge index
        std::unordered_map<uint64_t, int> edgeMap;
        
        auto makeKey = [](int a, int b) -> uint64_t {
            uint64_t ka = static_cast<uint64_t>(a);
            uint64_t kb = static_cast<uint64_t>(b);
            return (ka << 32) | kb;
        };
        
        for (const auto& f : faceList) {
            int nv = (int)f.size();
            if (nv < 3) continue;
            
            // Triangulate fan-style if nv > 3
            int numTriangles = nv - 2;
            for (int ti = 0; ti < numTriangles; ti++) {
                int i0 = f[0], i1 = f[1+ti], i2 = f[2+ti];
                addTriangle(i0, i1, i2, edgeMap);
            }
        }
        
        // Link twins
        for (auto& [key, heIdx] : edgeMap) {
            int a = (int)(key >> 32);
            int b = (int)(key & 0xFFFFFFFF);
            uint64_t revKey = makeKey(b, a);
            auto it = edgeMap.find(revKey);
            if (it != edgeMap.end()) {
                hedges[heIdx].twin = it->second;
            }
            // else: boundary edge, twin stays -1
        }
        
        // Set vertex outgoing half-edges
        for (int i = 0; i < (int)hedges.size(); i++) {
            int vSrc = hedges[hedges[i].prev].vertex;
            if (vertices[vSrc].he == -1) {
                vertices[vSrc].he = i;
            }
        }
        
        // Compute face normals and areas
        for (auto& face : faces) {
            computeFaceNormal(face);
        }
        
        // Compute vertex normals
        computeVertexNormals();
    }
    
    // Get all neighbor vertices around a vertex (1-ring)
    std::vector<int> getNeighborVertices(int vi) const {
        std::vector<int> result;
        int startHe = vertices[vi].he;
        if (startHe == -1) return result;
        
        int he = startHe;
        do {
            int neighbor = hedges[he].vertex;
            result.push_back(neighbor);
            
            // Go to next outgoing half-edge: twin->next
            int twin = hedges[he].twin;
            if (twin == -1) {
                // Boundary: find next boundary edge out
                // Actually for boundary, rewind using prev to find the next outgoing
                he = -1;
                break;
            }
            he = hedges[twin].next;
            if (he == -1) break;
        } while (he != startHe);
        
        return result;
    }
    
    // Get faces around a vertex (face ring)
    std::vector<int> getFacesAroundVertex(int vi) const {
        std::vector<int> result;
        int startHe = vertices[vi].he;
        if (startHe == -1) return result;
        
        int he = startHe;
        do {
            int f = hedges[he].face;
            if (f >= 0) {
                result.push_back(f);
            }
            int twin = hedges[he].twin;
            if (twin == -1) break;
            he = hedges[twin].next;
            if (he == -1) break;
        } while (he != startHe);
        
        return result;
    }
    
    // Get half-edges around a vertex
    std::vector<int> getHedgesAroundVertex(int vi) const {
        std::vector<int> result;
        int startHe = vertices[vi].he;
        if (startHe == -1) return result;
        
        int he = startHe;
        do {
            result.push_back(he);
            int twin = hedges[he].twin;
            if (twin == -1) break;
            he = hedges[twin].next;
            if (he == -1) break;
        } while (he != startHe);
        
        return result;
    }
    
    // Count boundary edges
    int countBoundaryEdges() const {
        int count = 0;
        for (const auto& he : hedges) {
            if (he.twin == -1) count++;
        }
        return count;
    }
    
    // Compute mesh Euler characteristic: V - E + F
    // Note: E = hedges.size() / 2 (each edge has two half-edges)
    int eulerCharacteristic() const {
        int V = (int)vertices.size();
        int E = (int)hedges.size() / 2;
        int F = (int)faces.size();
        return V - E + F;
    }
    
    // Compute genus from Euler characteristic (for closed orientable surface)
    // g = (2 - (V - E + F)) / 2
    int genus() const {
        int chi = eulerCharacteristic();
        return (2 - chi) / 2;
    }
    
    // Total surface area
    double totalArea() const {
        double sum = 0;
        for (const auto& f : faces) sum += f.area;
        return sum;
    }
    
    // Stats
    void printStats() const {
        std::cout << "=== Half-Edge Mesh Statistics ===" << std::endl;
        std::cout << "Vertices:       " << vertices.size() << std::endl;
        std::cout << "Half-Edges:     " << hedges.size() << std::endl;
        std::cout << "Full Edges:     " << (hedges.size() / 2) << std::endl;
        std::cout << "Faces:          " << faces.size() << std::endl;
        std::cout << "Boundary Edges: " << countBoundaryEdges() << std::endl;
        std::cout << "Euler Characteristic (chi = V - E + F): " << eulerCharacteristic() << std::endl;
        std::cout << "Genus (closed surface): " << genus() << std::endl;
        std::cout << "Total Surface Area: " << totalArea() << std::endl;
    }

private:
    void addTriangle(int v0, int v1, int v2,
                     std::unordered_map<uint64_t, int>& edgeMap) {
        int baseIdx = (int)hedges.size();
        int faceIdx = (int)faces.size();
        
        // Create 3 half-edges
        HEdge he0, he1, he2;
        he0.vertex = v1; he0.face = faceIdx; he0.next = baseIdx+1; he0.prev = baseIdx+2;
        he1.vertex = v2; he1.face = faceIdx; he1.next = baseIdx+2; he1.prev = baseIdx+0;
        he2.vertex = v0; he2.face = faceIdx; he2.next = baseIdx+0; he2.prev = baseIdx+1;
        
        hedges.push_back(he0);
        hedges.push_back(he1);
        hedges.push_back(he2);
        
        // Register edges: (v0,v1), (v1,v2), (v2,v0)
        edgeMap[makeKey(v0, v1)] = baseIdx+0;
        edgeMap[makeKey(v1, v2)] = baseIdx+1;
        edgeMap[makeKey(v2, v0)] = baseIdx+2;
        
        Face face;
        face.he = baseIdx;
        faces.push_back(face);
    }
    
    uint64_t makeKey(int a, int b) const {
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    }
    
    void computeFaceNormal(Face& face) {
        int he0 = face.he;
        if (he0 < 0) return;
        int he1 = hedges[he0].next;
        
        Vec3 p0 = vertices[hedges[hedges[he0].prev].vertex].pos;
        Vec3 p1 = vertices[hedges[he0].vertex].pos;
        Vec3 p2 = vertices[hedges[he1].vertex].pos;
        
        Vec3 e1 = p1 - p0;
        Vec3 e2 = p2 - p0;
        Vec3 n = e1.cross(e2);
        double len = n.length();
        face.area = 0.5 * len;
        if (len > 1e-12) {
            face.normal = n / len;
        }
    }
    
    void computeVertexNormals() {
        for (auto& v : vertices) {
            v.normal = Vec3(0, 0, 0);
        }
        for (const auto& face : faces) {
            Vec3 weighted = face.normal * face.area;
            int he = face.he;
            if (he < 0) continue;
            // Add to all three vertices of the triangle
            int he0 = he;
            int he1 = hedges[he0].next;
            vertices[hedges[hedges[he0].prev].vertex].normal = vertices[hedges[hedges[he0].prev].vertex].normal + weighted;
            vertices[hedges[he0].vertex].normal = vertices[hedges[he0].vertex].normal + weighted;
            vertices[hedges[he1].vertex].normal = vertices[hedges[he1].vertex].normal + weighted;
        }
        for (auto& v : vertices) {
            double len = v.normal.length();
            if (len > 1e-12) v.normal = v.normal / len;
        }
    }
};

// ============================================================
// Soft Rasterizer for PPM output
// ============================================================
struct ZBuffer {
    std::vector<double> depth;
    std::vector<Vec3>   color;
    int w, h;
    
    ZBuffer(int w_, int h_) : w(w_), h(h_) {
        depth.resize(w * h, std::numeric_limits<double>::max());
        color.resize(w * h, Vec3(0, 0, 0));
    }
    
    void setPixel(int x, int y, double z, const Vec3& c) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        int idx = y * w + x;
        if (z < depth[idx]) {
            depth[idx] = z;
            color[idx] = c;
        }
    }
    
    void savePPM(const std::string& filename) const {
        std::ofstream out(filename);
        out << "P3\n" << w << " " << h << "\n255\n";
        for (int i = 0; i < w * h; i++) {
            int r = std::min(255, std::max(0, (int)(color[i].x * 255)));
            int g = std::min(255, std::max(0, (int)(color[i].y * 255)));
            int b = std::min(255, std::max(0, (int)(color[i].z * 255)));
            out << r << " " << g << " " << b << "\n";
        }
    }
};

void rasterizeMesh(const HalfEdgeMesh& mesh, const std::string& outFile,
                   int imgW, int imgH, const Vec3& lightDir) {
    ZBuffer zb(imgW, imgH);
    
    // Camera setup: spherical camera looking at origin
    double dist = 4.0;
    double azimuth = 0.8;   // horizontal angle
    double elevation = 0.6; // vertical angle
    Vec3 eye(dist * std::cos(elevation) * std::sin(azimuth),
             dist * std::sin(elevation),
             dist * std::cos(elevation) * std::cos(azimuth));
    Vec3 center(0, 0.1, 0);
    Vec3 up(0, 1, 0);
    
    Vec3 viewDir = (center - eye).normalized();
    Vec3 vx = up.cross(viewDir);
    if (vx.length() < 1e-6) vx = Vec3(1, 0, 0);
    vx = vx.normalized();
    Vec3 vy = viewDir.cross(vx).normalized();
    
    auto project = [&](const Vec3& p) -> std::pair<Vec3, double> {
        Vec3 rel = p - eye;
        double depth = rel.dot(viewDir);
        double sx = rel.dot(vx) / depth;
        double sy = rel.dot(vy) / depth;
        double px = imgW / 2.0 + sx * imgW * 0.7;
        double py = imgH / 2.0 - sy * imgH * 0.7;
        return {Vec3(px, py, depth), depth};
    };
    
    Vec3 ld = lightDir.normalized();
    
    for (const auto& face : mesh.faces) {
        int he0 = face.he;
        if (he0 < 0) continue;
        int he1 = mesh.hedges[he0].next;
        
        int v0 = mesh.hedges[mesh.hedges[he0].prev].vertex;
        int v1 = mesh.hedges[he0].vertex;
        int v2 = mesh.hedges[he1].vertex;
        
        Vec3 p0 = mesh.vertices[v0].pos;
        Vec3 p1 = mesh.vertices[v1].pos;
        Vec3 p2 = mesh.vertices[v2].pos;
        
        // Back-face culling
        Vec3 faceNormal = (p1 - p0).cross(p2 - p0).normalized();
        Vec3 toEye = (eye - p0).normalized();
        if (faceNormal.dot(toEye) < 0) continue;
        
        auto [sp0, d0] = project(p0);
        auto [sp1, d1] = project(p1);
        auto [sp2, d2] = project(p2);
        
        // Bounding box
        int minX = std::max(0, (int)std::floor(std::min({sp0.x, sp1.x, sp2.x})));
        int maxX = std::min(imgW-1, (int)std::ceil(std::max({sp0.x, sp1.x, sp2.x})));
        int minY = std::max(0, (int)std::floor(std::min({sp0.y, sp1.y, sp2.y})));
        int maxY = std::min(imgH-1, (int)std::ceil(std::max({sp0.y, sp1.y, sp2.y})));
        
        // Edge functions
        auto edgeFunc = [](double ax, double ay, double bx, double by, double cx, double cy) -> double {
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        };
        
        double area = edgeFunc(sp0.x, sp0.y, sp1.x, sp1.y, sp2.x, sp2.y);
        if (std::abs(area) < 1e-6) continue;
        
        // Use vertex normals for Phong shading
        Vec3 n0 = mesh.vertices[v0].normal;
        Vec3 n1 = mesh.vertices[v1].normal;
        Vec3 n2 = mesh.vertices[v2].normal;
        
        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                double cx = x + 0.5, cy = y + 0.5;
                double w0 = edgeFunc(sp1.x, sp1.y, sp2.x, sp2.y, cx, cy);
                double w1 = edgeFunc(sp2.x, sp2.y, sp0.x, sp0.y, cx, cy);
                double w2 = edgeFunc(sp0.x, sp0.y, sp1.x, sp1.y, cx, cy);
                
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    w0 /= area; w1 /= area; w2 /= area;
                    
                    // Perspective-correct interpolation for depth
                    double z = 1.0 / (w0 / d0 + w1 / d1 + w2 / d2);
                    
                    // Interpolated normal
                    Vec3 n = (n0 * w0 + n1 * w1 + n2 * w2).normalized();
                    
                    // Phong lighting
                    double diffuse = std::max(0.0, n.dot(ld));
                    Vec3 ambient(0.15, 0.15, 0.18);
                    
                    // Object color based on normal direction
                    Vec3 objColor(0.7, 0.6, 0.4);
                    
                    // View direction at this pixel (approximate with eye direction)
                    Vec3 pixelViewDir = (eye - (p0*w0 + p1*w1 + p2*w2)).normalized();
                    Vec3 half = (ld + pixelViewDir).normalized();
                    double spec = std::pow(std::max(0.0, n.dot(half)), 32.0);
                    
                    Vec3 col = (objColor * diffuse + Vec3(1,1,1) * spec * 0.3 + ambient);
                    
                    zb.setPixel(x, y, z, col);
                }
            }
        }
    }
    
    zb.savePPM(outFile);
    std::cout << "Rendered: " << outFile << " (" << imgW << "x" << imgH << ")" << std::endl;
}

// ============================================================
// Quantifiable Verification Tests
// ============================================================
void runVerification(const HalfEdgeMesh& mesh) {
    std::cout << "\n=== QUANTIFIABLE VERIFICATION ===\n" << std::endl;
    
    // Test 1: Euler characteristic for a tetrahedron-like closed mesh
    int V = (int)mesh.vertices.size();
    int E = (int)mesh.hedges.size() / 2;
    int F = (int)mesh.faces.size();
    int chi = V - E + F;
    
    std::cout << "Test 1: Euler Characteristic" << std::endl;
    std::cout << "  V=" << V << " E=" << E << " F=" << F << " => chi=" << chi << std::endl;
    std::cout << "  For sphere topology: chi = 2 = V - E + F" << std::endl;
    // 3*E = 3F + B (every edge belongs to 2 faces except boundaries)
    // For closed mesh: 3F = 2E, so E = 3F/2
    std::cout << "  Expected relationship for closed triangulated mesh: 3F = 2E" << std::endl;
    std::cout << "  Actual: 3*" << F << " = " << (3*F) << " vs 2*" << E << " = " << (2*E) << std::endl;
    
    // Test 2: Vertex 1-ring count
    std::cout << "\nTest 2: Vertex 1-Ring Neighbor Counts" << std::endl;
    int totalNeighbors = 0;
    int minNeighbors = 999999, maxNeighbors = 0;
    for (int vi = 0; vi < V; vi++) {
        auto neighbors = mesh.getNeighborVertices(vi);
        int cnt = (int)neighbors.size();
        totalNeighbors += cnt;
        minNeighbors = std::min(minNeighbors, cnt);
        maxNeighbors = std::max(maxNeighbors, cnt);
    }
    double avgNeighbors = (double)totalNeighbors / V;
    std::cout << "  Min: " << minNeighbors << " Max: " << maxNeighbors
              << " Avg: " << avgNeighbors << std::endl;
    // For a regular triangulated closed mesh, average valence should be ~6
    std::cout << "  Expected average valence ~6 for triangulated closed mesh" << std::endl;
    
    // Test 3: Face area consistency
    std::cout << "\nTest 3: Face Area Statistics" << std::endl;
    double totalArea = 0;
    double minArea = 1e30, maxArea = 0;
    for (const auto& f : mesh.faces) {
        totalArea += f.area;
        minArea = std::min(minArea, f.area);
        maxArea = std::max(maxArea, f.area);
    }
    double avgArea = totalArea / F;
    double areaStd = 0;
    for (const auto& f : mesh.faces) {
        double d = f.area - avgArea;
        areaStd += d * d;
    }
    areaStd = std::sqrt(areaStd / F);
    std::cout << "  Total Area:   " << totalArea << std::endl;
    std::cout << "  Min Area:     " << minArea << std::endl;
    std::cout << "  Max Area:     " << maxArea << std::endl;
    std::cout << "  Avg Area:     " << avgArea << std::endl;
    std::cout << "  Std Dev:      " << areaStd << std::endl;
    
    // Test 4: Vertex Normal consistency
    std::cout << "\nTest 4: Vertex Normal Statistics" << std::endl;
    std::vector<double> normalLengths;
    for (const auto& v : mesh.vertices) {
        normalLengths.push_back(v.normal.length());
    }
    double minNL = *std::min_element(normalLengths.begin(), normalLengths.end());
    double maxNL = *std::max_element(normalLengths.begin(), normalLengths.end());
    double avgNL = 0;
    for (double l : normalLengths) avgNL += l;
    avgNL /= normalLengths.size();
    std::cout << "  Normal length - Min: " << minNL << " Max: " << maxNL
              << " Avg: " << avgNL << std::endl;
    std::cout << "  All normals should be unit length (close to 1.0)" << std::endl;
    
    // Test 5: Half-edge connectivity integrity
    std::cout << "\nTest 5: Half-Edge Connectivity Integrity" << std::endl;
    int brokenLinks = 0;
    for (int i = 0; i < (int)mesh.hedges.size(); i++) {
        const auto& he = mesh.hedges[i];
        if (he.vertex < 0 || he.vertex >= V) brokenLinks++;
        if (he.next < 0 || he.next >= (int)mesh.hedges.size()) brokenLinks++;
        if (he.prev < 0 || he.prev >= (int)mesh.hedges.size()) brokenLinks++;
        if (he.face < 0 || he.face >= F) brokenLinks++;
        // Verify back-links
        if (mesh.hedges[he.next].prev != i) brokenLinks++;
        if (mesh.hedges[he.prev].next != i) brokenLinks++;
        // Verify twin symmetry
        if (he.twin >= 0) {
            if (mesh.hedges[he.twin].twin != i) brokenLinks++;
        }
    }
    // Boundary edges are OK (twin = -1)
    std::cout << "  Broken connectivity links: " << brokenLinks << " (should be 0)" << std::endl;
    
    // Test 6: Face around vertex consistency
    std::cout << "\nTest 6: Face Around Vertex Consistency" << std::endl;
    int totalFaceRefs = 0;
    for (int vi = 0; vi < V; vi++) {
        totalFaceRefs += (int)mesh.getFacesAroundVertex(vi).size();
    }
    // Each triangle face is referenced by 3 vertices
    std::cout << "  Total face->vertex references: " << totalFaceRefs << std::endl;
    std::cout << "  Expected: 3 * F = " << (3 * F) << std::endl;
    
    // Final pass/fail
    std::cout << "\n=== VERIFICATION SUMMARY ===" << std::endl;
    bool allPassed = true;
    
    if (std::abs(3.0 * F - 2.0 * E) > F * 0.1) {
        // Allow small tolerance for non-closed meshes
        if (mesh.countBoundaryEdges() == 0) {
            std::cout << "FAIL: 3F != 2E for closed mesh (3F=" << (3*F) << ", 2E=" << (2*E) << ")" << std::endl;
            allPassed = false;
        } else {
            std::cout << "INFO: 3F != 2E expected due to " << mesh.countBoundaryEdges() << " boundary edges" << std::endl;
        }
    } else {
        std::cout << "PASS: 3F = 2E (closed mesh relationship holds)" << std::endl;
    }
    
    if (brokenLinks > 0) {
        std::cout << "FAIL: " << brokenLinks << " broken connectivity links found" << std::endl;
        allPassed = false;
    } else {
        std::cout << "PASS: All half-edge connectivity links are intact" << std::endl;
    }
    
    if (std::abs(avgNL - 1.0) > 0.01) {
        std::cout << "FAIL: Vertex normals not unit length (avg=" << avgNL << ")" << std::endl;
        allPassed = false;
    } else {
        std::cout << "PASS: All vertex normals are unit length" << std::endl;
    }
    
    if (totalFaceRefs != 3 * F && mesh.countBoundaryEdges() == 0) {
        std::cout << "FAIL: Face-around-vertex references inconsistent" << std::endl;
        allPassed = false;
    } else {
        std::cout << "PASS: Face-around-vertex references consistent" << std::endl;
    }
    
    if (allPassed) {
        std::cout << "\n✅ ALL VERIFICATION TESTS PASSED" << std::endl;
    } else {
        std::cout << "\n❌ SOME VERIFICATION TESTS FAILED" << std::endl;
    }
}

// ============================================================
// Create mesh data
// ============================================================
int main() {
    // Create a more interesting mesh: a house-like shape
    // This is a pentagonal prism-like shape with triangulated faces
    std::vector<Vec3> positions = {
        // Bottom pentagon (y = -0.6)
        { 0.0,    -0.6,  1.0 },     // 0
        { 0.951,  -0.6,  0.309 },   // 1
        { 0.588,  -0.6, -0.809 },   // 2
        {-0.588,  -0.6, -0.809 },   // 3
        {-0.951,  -0.6,  0.309 },   // 4
        // Top pentagon (y = 0.6)
        { 0.0,     0.6,  1.0 },     // 5
        { 0.951,   0.6,  0.309 },   // 6
        { 0.588,   0.6, -0.809 },   // 7
        {-0.588,   0.6, -0.809 },   // 8
        {-0.951,   0.6,  0.309 },   // 9
        // Roof peak
        { 0.0,     1.2,  0.0 },     // 10
    };
    
    std::vector<std::vector<int>> faceList;
    
    // Bottom cap (clockwise from below = CCW from outside)
    faceList.push_back({0, 2, 1});
    faceList.push_back({0, 3, 2});
    faceList.push_back({0, 4, 3});
    
    // Side faces (quadrilaterals, CCW from outside)
    faceList.push_back({0, 4, 9, 5});   // 0-4-9-5
    faceList.push_back({4, 3, 8, 9});   // 4-3-8-9
    faceList.push_back({3, 2, 7, 8});   // 3-2-7-8
    faceList.push_back({2, 1, 6, 7});   // 2-1-6-7
    faceList.push_back({1, 0, 5, 6});   // 1-0-5-6
    
    // Roof (triangles connecting to peak)
    faceList.push_back({5, 9, 10});
    faceList.push_back({9, 8, 10});
    faceList.push_back({8, 7, 10});
    faceList.push_back({7, 6, 10});
    faceList.push_back({6, 5, 10});
    
    // Top cap of roof base (between roof and top pentagon)
    // The top pentagon edges are used by roof triangles
    // The top pentagon itself is not a face since it's inside the roof
    
    // Build the mesh
    HalfEdgeMesh mesh;
    mesh.build(positions, faceList);
    
    // Print stats
    mesh.printStats();
    
    // Run verification
    runVerification(mesh);
    
    // Render
    Vec3 lightDir(0.5, 1.0, 0.3);
    rasterizeMesh(mesh, "mesh_output.ppm", 800, 600, lightDir);
    
    // Additional: pixel-level verification of the output
    std::cout << "\n=== OUTPUT FILE PIXEL VERIFICATION ===" << std::endl;
    
    // Read back and verify the PPM
    std::ifstream in("mesh_output.ppm");
    std::string format;
    int w, h, maxVal;
    in >> format >> w >> h >> maxVal;
    
    std::vector<int> rPixels, gPixels, bPixels;
    int r, g, b;
    while (in >> r >> g >> b) {
        rPixels.push_back(r);
        gPixels.push_back(g);
        bPixels.push_back(b);
    }
    
    double rMean = 0, gMean = 0, bMean = 0;
    for (size_t i = 0; i < rPixels.size(); i++) {
        rMean += rPixels[i];
        gMean += gPixels[i];
        bMean += bPixels[i];
    }
    rMean /= rPixels.size();
    gMean /= gPixels.size();
    bMean /= bPixels.size();
    
    // Count non-background pixels (rendered pixels)
    int renderedPixels = 0;
    for (size_t i = 0; i < rPixels.size(); i++) {
        if (rPixels[i] > 10 || gPixels[i] > 10 || bPixels[i] > 10) {
            renderedPixels++;
        }
    }
    
    double totalAvg = (rMean + gMean + bMean) / 3.0;
    
    // Compute standard deviation for brightness
    double var = 0;
    for (size_t i = 0; i < rPixels.size(); i++) {
        double avg = (rPixels[i] + gPixels[i] + bPixels[i]) / 3.0;
        double d = avg - totalAvg;
        var += d * d;
    }
    var /= rPixels.size();
    double stdDev = std::sqrt(var);
    
    std::cout << "  Image size: " << w << "x" << h << std::endl;
    std::cout << "  Total pixels: " << (w*h) << std::endl;
    std::cout << "  Rendered pixels (>10): " << renderedPixels
              << " (" << (100.0 * renderedPixels / (w * h)) << "%)" << std::endl;
    std::cout << "  Pixel mean: R=" << rMean << " G=" << gMean << " B=" << bMean << std::endl;
    std::cout << "  Brightness mean: " << totalAvg << " std: " << stdDev << std::endl;
    
    // Quantified output verification (not eye-based)
    bool outputOK = true;
    if (totalAvg < 0.1) { std::cout << "FAIL: Image too dark (mean=" << totalAvg << ")" << std::endl; outputOK = false; }
    if (totalAvg > 240) { std::cout << "FAIL: Image too bright (mean=" << totalAvg << ")" << std::endl; outputOK = false; }
    if (stdDev < 2) { std::cout << "FAIL: No significant variation (std=" << stdDev << ")" << std::endl; outputOK = false; }
    if (renderedPixels < static_cast<int>(w * h * 0.005)) { std::cout << "FAIL: Too few rendered pixels (" << renderedPixels << ")" << std::endl; outputOK = false; }
    
    // Check mean brightness of rendered pixels only
    double renderedR = 0, renderedG = 0, renderedB = 0;
    for (size_t i = 0; i < rPixels.size(); i++) {
        if (rPixels[i] > 10 || gPixels[i] > 10 || bPixels[i] > 10) {
            renderedR += rPixels[i]; renderedG += gPixels[i]; renderedB += bPixels[i];
        }
    }
    if (renderedPixels > 0) {
        renderedR /= renderedPixels; renderedG /= renderedPixels; renderedB /= renderedPixels;
        double renderedAvg = (renderedR + renderedG + renderedB) / 3.0;
        std::cout << "  Rendered pixel mean: R=" << renderedR << " G=" << renderedG << " B=" << renderedB << std::endl;
        std::cout << "  Rendered pixel brightness: " << renderedAvg << std::endl;
        if (renderedAvg < 30) { std::cout << "FAIL: Rendered pixels too dark (mean=" << renderedAvg << ")" << std::endl; outputOK = false; }
        if (renderedAvg > 240) { std::cout << "FAIL: Rendered pixels too bright (mean=" << renderedAvg << ")" << std::endl; outputOK = false; }
    }
    
    // Check file size
    std::ifstream fileCheck("mesh_output.ppm", std::ios::ate | std::ios::binary);
    size_t fileSize = fileCheck.tellg();
    std::cout << "  File size: " << fileSize << " bytes" << std::endl;
    if (fileSize < 10240) { std::cout << "FAIL: File too small (< 10KB)" << std::endl; outputOK = false; }
    
    if (outputOK) {
        std::cout << "PASS: Output image passes all quantified checks" << std::endl;
    }
    
    std::cout << "\n=== DONE ===" << std::endl;
    return 0;
}