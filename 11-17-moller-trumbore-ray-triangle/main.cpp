// Möller-Trumbore Ray-Triangle Intersection
// Fast, direct barycentric intersection without precomputing plane, plus backface culling.
// Quantified validation: analytic ground-truth rays, barycentric reconstruction, and
// correctness vs. brute-force plane intersection on a random mesh.

#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <algorithm>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x };
    }
    double len() const { return std::sqrt(x*x+y*y+z*z); }
};

struct Ray { Vec3 org, dir; };
struct Tri { Vec3 v0, v1, v2; };

// Möller-Trumbore. Returns true if hit in [0, tmax], outputs t, u, v (barycentric w.r.t. v1, v2).
// u,v >=0, u+v <=1. Backface culling optional.
bool mollerTrumbore(const Ray& r, const Tri& tri, double* tout, double* uout, double* vout,
                    double tmax = 1e18, bool cull = false) {
    const double EPS = 1e-12;
    Vec3 e1 = tri.v1 - tri.v0;
    Vec3 e2 = tri.v2 - tri.v0;
    Vec3 pvec = r.dir.cross(e2);
    double det = e1.dot(pvec);
    if (cull) {
        if (det < EPS) return false;
    } else {
        if (std::fabs(det) < EPS) return false;
    }
    double invDet = 1.0 / det;
    Vec3 tvec = r.org - tri.v0;
    double u = tvec.dot(pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;
    Vec3 qvec = tvec.cross(e1);
    double v = r.dir.dot(qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;
    double t = e2.dot(qvec) * invDet;
    if (t < 0.0 || t > tmax) return false;
    *tout = t; *uout = u; *vout = v;
    return true;
}

// Brute-force plane-intersection reference (geometric, independent formulation).
bool planeRef(const Ray& r, const Tri& tri, double* tout, double* uout, double* vout, double tmax = 1e18) {
    const double EPS = 1e-12;
    Vec3 e1 = tri.v1 - tri.v0;
    Vec3 e2 = tri.v2 - tri.v0;
    Vec3 n = e1.cross(e2);
    double denom = n.dot(r.dir);
    if (std::fabs(denom) < EPS) return false;
    double d = -n.dot(tri.v0);
    double t = -(n.dot(r.org) + d) / denom;
    if (t < 0.0 || t > tmax) return false;
    Vec3 P = r.org + r.dir * t;
    // Barycentric via area ratios (independent of MT formulation).
    Vec3 v0v1 = tri.v1 - tri.v0, v0v2 = tri.v2 - tri.v0, v0p = P - tri.v0;
    double d00 = v0v1.dot(v0v1);
    double d01 = v0v1.dot(v0v2);
    double d11 = v0v2.dot(v0v2);
    double d20 = v0p.dot(v0v1);
    double d21 = v0p.dot(v0v2);
    double den = d00*d11 - d01*d01;
    if (std::fabs(den) < EPS) return false;
    double v = (d11*d20 - d01*d21) / den;
    double w = (d00*d21 - d01*d20) / den;
    double u = 1.0 - v - w;
    if (u < -1e-9 || v < -1e-9 || w < -1e-9) return false;
    *tout = t; *uout = v; *vout = w;
    return true;
}

int main() {
    setvbuf(stdout, nullptr, _IOLBF, 1024);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);

    printf("=== Moller-Trumbore Ray-Triangle Intersection ===\n\n");

    // ---- Test 1: Analytic ground truth ----
    // Ray aimed straight down the z-axis at a triangle in z=5 plane, known hit point = center.
    printf("[Test 1] Analytic ground-truth (t, barycentrics)\n");
    {
        Tri tri{ {0,0,5}, {4,0,5}, {0,4,5} };
        Ray ray{ {1,1,0}, {0,0,1} };
        double t,u,v;
        bool hit = mollerTrumbore(ray, tri, &t,&u,&v);
        // Expected: hit point (1,1,5). Barycentric: P = v0 + u*e1 + v*e2, e1=(4,0,0), e2=(0,4,0)
        // 1 = u*4, 1 = v*4 -> u=v=0.25. t=5.
        Vec3 P = ray.org + ray.dir*t;
        double eu = std::fabs(u-0.25), ev = std::fabs(v-0.25), et = std::fabs(t-5.0);
        printf("  hit=%d t=%f (exp 5.000)  u=%f v=%f (exp 0.25 each)  P=(%f,%f,%f)\n",
               hit?1:0, t, u, v, P.x, P.y, P.z);
        printf("  errors: |t|=%.3e |u|=%.3e |v|=%.3e -> %s\n\n",
               et, eu, ev, (hit && et<1e-9 && eu<1e-9 && ev<1e-9) ? "PASS" : "FAIL");
    }

    // ---- Test 2: Backface culling ----
    printf("[Test 2] Backface culling\n");
    {
        // Front-facing triangle: CCW winding seen from origin (normal +z).
        // e1=(4,0,0), e2=(0,4,0), normal=e1×e2=(0,0,16)=+z. Ray dir=+z, so det=e1·(dir×e2)=
        // (4,0,0)·((0,0,1)×(0,4,0)) = (4,0,0)·(-4,0,0) = -16 < 0 => backface for det>0 convention.
        double t,u,v;
        // Case A: ray dir +z, triangle normal +z (det<0) -> culled.
        bool cA = mollerTrumbore(Ray{{1,1,0},{0,0,1}}, Tri{{0,0,5},{4,0,5},{0,4,5}}, &t,&u,&v, 1e18, true);
        // Case B: ray dir -z, triangle normal +z (det>0) -> kept.
        bool cB = mollerTrumbore(Ray{{1,1,10},{0,0,-1}}, Tri{{0,0,5},{4,0,5},{0,4,5}}, &t,&u,&v, 1e18, true);
        // No-cull always hits both.
        bool nA = mollerTrumbore(Ray{{1,1,0},{0,0,1}}, Tri{{0,0,5},{4,0,5},{0,4,5}}, &t,&u,&v, 1e18, false);
        bool nB = mollerTrumbore(Ray{{1,1,10},{0,0,-1}}, Tri{{0,0,5},{4,0,5},{0,4,5}}, &t,&u,&v, 1e18, false);
        printf("  cull: backfacing(+z dir vs +z n)=%d (exp 0), frontfacing(-z dir vs +z n)=%d (exp 1)\n", cA?1:0, cB?1:0);
        printf("  no-cull: +z dir=%d (exp 1), -z dir=%d (exp 1)\n", nA?1:0, nB?1:0);
        printf("  -> %s\n\n", (!cA && cB && nA && nB) ? "PASS" : "FAIL");
    }

    // ---- Test 3: Missing cases (parallel ray, miss, behind origin) ----
    printf("[Test 3] Edge cases (parallel / miss / behind)\n");
    {
        Tri tri{ {0,0,5}, {4,0,5}, {0,4,5} };
        double t,u,v;
        Ray parallel{ {1,1,0}, {1,0,0} };       // parallel to plane
        Ray miss{ {10,10,0}, {0,0,1} };          // misses triangle (x+y=20 > 4)
        Ray behind{ {1,1,10}, {0,0,1} };         // origin beyond triangle, dir away
        bool h1 = mollerTrumbore(parallel, tri, &t,&u,&v);
        bool h2 = mollerTrumbore(miss, tri, &t,&u,&v);
        bool h3 = mollerTrumbore(behind, tri, &t,&u,&v);
        int expect = (h1||h2||h3) ? 0 : 1;
        printf("  parallel=%d miss=%d behind=%d (all should be 0) -> %s\n\n",
               h1?1:0, h2?1:0, h3?1:0, expect ? "PASS" : "FAIL");
    }

    // ---- Test 4: Random mesh consistency vs plane reference ----
    printf("[Test 4] Random mesh: M-T vs plane-reference consistency\n");
    {
        int N = 5000;
        std::vector<Tri> tris(N);
        for (auto& t : tris) {
            Vec3 c{ uni(rng)*200, uni(rng)*200, uni(rng)*200 + 200 };
            t.v0 = c + Vec3{uni(rng)*10, uni(rng)*10, uni(rng)*3};
            t.v1 = c + Vec3{uni(rng)*10, uni(rng)*10, uni(rng)*3};
            t.v2 = c + Vec3{uni(rng)*10, uni(rng)*10, uni(rng)*3};
        }
        int agree = 0, disagree = 0;
        double maxTerr = 0, maxUerr = 0, maxVerr = 0;
        for (int i = 0; i < 200000; i++) {
            Ray r{ Vec3{uni(rng)*400, uni(rng)*400, 0}, Vec3{uni(rng), uni(rng), 1} };
            // nearest hit via MT
            double tBest=1e18, uBest=0, vBest=0; int idxMT=-1;
            for (int j=0;j<N;j++) { double t,u,v;
                if (mollerTrumbore(r, tris[j], &t,&u,&v, tBest)) { tBest=t; uBest=u; vBest=v; idxMT=j; } }
            // nearest hit via plane ref
            double tR=1e18, uR=0, vR=0; int idxR=-1;
            for (int j=0;j<N;j++) { double t,u,v;
                if (planeRef(r, tris[j], &t,&u,&v, tR)) { tR=t; uR=u; vR=v; idxR=j; } }
            if ((idxMT<0) != (idxR<0)) { disagree++; continue; }
            if (idxMT<0) { agree++; continue; }
            if (idxMT != idxR) { disagree++; continue; }
            agree++;
            maxTerr = std::max(maxTerr, std::fabs(tBest-tR));
            maxUerr = std::max(maxUerr, std::fabs(uBest-uR));
            maxVerr = std::max(maxVerr, std::fabs(vBest-vR));
        }
        printf("  rays=%d  agree=%d (%.4f)  disagree=%d\n",
               200000, agree, agree/200000.0, disagree);
        printf("  max |dt|=%.3e  max |du|=%.3e  max |dv|=%.3e\n\n",
               maxTerr, maxUerr, maxVerr);
        if (disagree==0 && maxTerr<1e-6) printf("  -> PASS\n\n"); else printf("  -> FAIL\n\n");
    }

    // ---- Test 5: Barycentric point reconstruction ----
    printf("[Test 5] Hit point reconstruction from barycentrics\n");
    {
        double maxErr = 0; long hits = 0, total = 0;
        for (int i=0;i<100000;i++) {
            // Build a well-conditioned, non-degenerate triangle.
            Vec3 v0{ uni(rng)*10, uni(rng)*10, uni(rng)*10 + 50 };
            Vec3 v1 = v0 + Vec3{ 8, 0, 0 };
            Vec3 v2 = v0 + Vec3{ 0, 8, 0 };
            Tri tri{ v0, v1, v2 };
            Vec3 n = (v1-v0).cross(v2-v0); n = n * (1.0/n.len());
            // Uniform-in-triangle sampling (square-root method) scaled to 0.9 => strictly interior,
            // never on the boundary and never sum > 1.0.
            double a = (uni(rng)+1.0)/2.0;   // [0,1]
            double b = (uni(rng)+1.0)/2.0;   // [0,1]
            double uc = 0.9 * std::sqrt(a) * b;
            double vc = 0.9 * std::sqrt(a) * (1.0 - b);
            // uc,vc in [0, 0.9], uc+vc = 0.9*sqrt(a) <= 0.9 < 1.0 always.
            Vec3 P = v0 + (v1-v0)*uc + (v2-v0)*vc;
            Ray r{ P - n*5.0, n };   // origin 5 units above plane, pointing down along normal
            double t,u,v;
            total++;
            if (mollerTrumbore(r, tri, &t,&u,&v)) {
                Vec3 R = r.org + r.dir*t;
                maxErr = std::max(maxErr, (R-P).len());
                hits++;
            }
        }
        printf("  total=%ld hits=%ld  max reconstruction error=%.3e\n", total, hits, maxErr);
        if (hits==total && maxErr<1e-9) printf("  -> PASS\n\n"); else printf("  -> FAIL\n\n");
    }

    // ---- Test 6: Performance benchmark ----
    printf("[Test 6] Performance (MT vs plane reference)\n");
    {
        int N = 20000;
        std::vector<Tri> tris(N);
        std::vector<Ray> rays(2000);
        for (auto& t : tris) {
            Vec3 c{ uni(rng)*1000, uni(rng)*1000, uni(rng)*1000+100 };
            t.v0 = c + Vec3{uni(rng),uni(rng),uni(rng)};
            t.v1 = c + Vec3{uni(rng),uni(rng),uni(rng)};
            t.v2 = c + Vec3{uni(rng),uni(rng),uni(rng)};
        }
        for (auto& r : rays) r = Ray{ Vec3{uni(rng)*1000, uni(rng)*1000, 0}, Vec3{uni(rng),uni(rng),1} };

        long cntMT=0, cntRef=0;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (auto& r : rays) { double t,u,v;
            for (auto& tr : tris) if (mollerTrumbore(r,tr,&t,&u,&v)) cntMT++; }
        auto t1 = std::chrono::high_resolution_clock::now();
        for (auto& r : rays) { double t,u,v;
            for (auto& tr : tris) if (planeRef(r,tr,&t,&u,&v)) cntRef++; }
        auto t2 = std::chrono::high_resolution_clock::now();

        double msMT = std::chrono::duration<double,std::milli>(t1-t0).count();
        double msRef = std::chrono::duration<double,std::milli>(t2-t1).count();
        printf("  ray-triangle tests=%ld (MT hits=%ld, ref hits=%ld)\n",
               (long)rays.size()*N, cntMT, cntRef);
        printf("  MT time=%.1fms  ref time=%.1fms  speedup=%.2fx\n", msMT, msRef, msRef/msMT);
        double rate = rays.size()*N / (msMT/1000.0) / 1e6;
        printf("  MT throughput=%.2f M tests/sec\n\n", rate);
    }

    printf("=== DONE ===\n");
    return 0;
}
