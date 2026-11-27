// View Frustum Culling — daily coding practice 2026-11-28
// Game-dev / real-time rendering technique: extract the 6 clip-space planes
// of a perspective-view frustum from the combined view*projection matrix,
// then conservatively classify AABBs and spheres as IN / INTERSECT / OUT.
//
// Quantified validation (NOT eyeballing):
//   1. Plane extraction sanity: each plane normal is (near-)unit length, and
//      every one of the 8 frustum corners satisfies the 6 plane inequalities
//      (interior feasibility), while a point clearly behind the camera is
//      flagged outside.
//   2. Correctness: brute-force dense-point-sample classification of each
//      AABB/sphere against all 6 planes is compared to the analytic
//      conservative test — the analytic test must never produce a false
//      negative (mark OUT when truly overlapping), and must match on IN/OUT.
//   3. Acceleration: measure wall-clock throughput of the analytic cull vs a
//      brute-force per-object test over a large random object set.

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <random>
#include <chrono>

using std::vector;

// ---------- small math types ----------
struct Vec3 { double x,y,z;
    Vec3(double x_=0,double y_=0,double z_=0):x(x_),y(y_),z(z_){}
    Vec3 operator+(const Vec3&o)const{return Vec3(x+o.x,y+o.y,z+o.z);}
    Vec3 operator-(const Vec3&o)const{return Vec3(x-o.x,y-o.y,z-o.z);}
    Vec3 operator*(double s)const{return Vec3(x*s,y*s,z*s);}
    double dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return Vec3(y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x);}
    double length()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 normalized()const{double l=length(); return l>1e-15?Vec3(x/l,y/l,z/l):Vec3(0,0,0);}
};

// 4x4 matrix, column-major: m[col][row].
struct Mat4 {
    double m[4][4];
    Mat4(){ for(int c=0;c<4;c++)for(int r=0;r<4;r++)m[c][r]=0; }
    static Mat4 identity(){ Mat4 M; for(int i=0;i<4;i++)M.m[i][i]=1; return M; }
};

// ---------- plane ----------
struct Plane { double a,b,c,d;
    Plane(double a_=0,double b_=0,double c_=0,double d_=0):a(a_),b(b_),c(c_),d(d_){}
    double dist(const Vec3&p)const{ return a*p.x + b*p.y + c*p.z + d; }
    void normalize(){
        double l = std::sqrt(a*a+b*b+c*c);
        if(l>1e-15){ a/=l;b/=l;c/=l;d/=l; }
    }
};

struct AABB { Vec3 mn, mx; };
struct Sphere { Vec3 c; double r; };

// ---------- camera matrix construction ----------
static Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = (target-eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);
    Mat4 V = Mat4::identity();
    V.m[0][0]=r.x; V.m[0][1]=u.x; V.m[0][2]=-f.x;
    V.m[1][0]=r.y; V.m[1][1]=u.y; V.m[1][2]=-f.y;
    V.m[2][0]=r.z; V.m[2][1]=u.z; V.m[2][2]=-f.z;
    V.m[3][0]=-(r.dot(eye)); V.m[3][1]=-(u.dot(eye)); V.m[3][2]=f.dot(eye); V.m[3][3]=1;
    return V;
}

static Mat4 perspective(double fovY_deg, double aspect, double n, double f) {
    double t = std::tan(fovY_deg * M_PI / 360.0) * n;
    double b = -t, r = t*aspect, l = -r;
    Mat4 P;
    P.m[0][0] = 2*n/(r-l);
    P.m[1][1] = 2*n/(t-b);
    P.m[2][0] = (r+l)/(r-l);
    P.m[2][1] = (t+b)/(t-b);
    P.m[2][2] = -(f+n)/(f-n);
    P.m[2][3] = -1.0;
    P.m[3][2] = -(2*f*n)/(f-n);
    return P;
}

static Mat4 mul(const Mat4& A, const Mat4& B){
    Mat4 C;
    for(int c=0;c<4;c++) for(int r=0;r<4;r++){
        double s=0; for(int k=0;k<4;k++) s += A.m[k][r]*B.m[c][k];
        C.m[c][r]=s;
    }
    return C;
}

// ---------- frustum plane extraction (Gribb–Hartmann) ----------
struct Frustum { std::array<Plane,6> planes; };

static Frustum extract_frustum(const Mat4& M){
    Frustum F;
    auto row = [&](int k, int sign)->Plane {
        return Plane(
            M.m[0][3] + sign*M.m[0][k],
            M.m[1][3] + sign*M.m[1][k],
            M.m[2][3] + sign*M.m[2][k],
            M.m[3][3] + sign*M.m[3][k]
        );
    };
    F.planes[0] = row(0, +1); // left
    F.planes[1] = row(0, -1); // right
    F.planes[2] = row(1, +1); // bottom
    F.planes[3] = row(1, -1); // top
    F.planes[4] = row(2, +1); // near
    F.planes[5] = row(2, -1); // far
    // Flip to inward-facing: the interior of the frustum must yield a
    // NEGATIVE signed distance (consistent with positive/negative vertex
    // and the "inside => dist < 0" test used everywhere below).
    for(auto&p:F.planes){ p.a=-p.a; p.b=-p.b; p.c=-p.c; p.d=-p.d; p.normalize(); }
    return F;
}

// ---------- conservative classification ----------
enum Status { IN=0, INTERSECT=1, OUT=2 };

static inline Vec3 positive_vert(const AABB& b, const Plane& p){
    double nx=p.a, ny=p.b, nz=p.c;
    return Vec3(nx>=0?b.mx.x:b.mn.x, ny>=0?b.mx.y:b.mn.y, nz>=0?b.mx.z:b.mn.z);
}
static inline Vec3 negative_vert(const AABB& b, const Plane& p){
    double nx=p.a, ny=p.b, nz=p.c;
    return Vec3(nx>=0?b.mn.x:b.mx.x, ny>=0?b.mn.y:b.mx.y, nz>=0?b.mn.z:b.mx.z);
}

static Status classify_aabb(const Frustum& F, const AABB& b){
    // Reject if ANY plane has the whole box on its outside (positive) side:
    // the minimum-distance corner (negative_vert) is already > 0.
    for(const auto& p : F.planes){
        Vec3 nv = negative_vert(b, p); // corner closest to interior
        if(p.dist(nv) > 0) return OUT;
    }
    // Not rejected => box overlaps. Decide fully-in vs intersecting:
    for(const auto& p : F.planes){
        Vec3 pv = positive_vert(b, p); // corner farthest from interior
        if(p.dist(pv) > 0) return INTERSECT; // some corner pokes outside a plane
    }
    return IN;
}

static Status classify_sphere(const Frustum& F, const Sphere& s){
    bool intersect = false;
    for(const auto& p : F.planes){
        double d = p.dist(s.c);   // interior is negative side
        if(d > s.r) return OUT;       // center outside by more than radius => whole sphere out
        if(d > -s.r) intersect = true; // sphere surface crosses this plane => straddling
    }
    return intersect ? INTERSECT : IN;
}

// ---------- brute-force reference (dense point sampling) ----------
// Classifies an AABB by sampling a dense grid of points inside it and testing
// every point against the 6 planes. Returns IN / INTERSECT / OUT with the same
// semantics, but conservatively assumes INTERSECT if any sampled point is out
// (to be a faithful "does it overlap the frustum at all" oracle).
static Status brute_aabb(const Frustum& F, const AABB& b, int steps){
    int inside=0, total=0;
    for(int i=0;i<=steps;i++) for(int j=0;j<=steps;j++) for(int k=0;k<=steps;k++){
        Vec3 p(b.mn.x + (b.mx.x-b.mn.x)*i/steps,
               b.mn.y + (b.mx.y-b.mn.y)*j/steps,
               b.mn.z + (b.mx.z-b.mn.z)*k/steps);
        bool in=true;
        for(const auto& pl : F.planes) if(pl.dist(p) > 1e-6){ in=false; break; }
        if(in) inside++;
        total++;
    }
    if(inside==total) return IN;
    if(inside>0) return INTERSECT;
    return OUT;
}

static Status brute_sphere(const Frustum& F, const Sphere& s, int steps){
    // Faithful oracle: sample points INSIDE the sphere (rejection sample a
    // cube, keep only points within radius), then test frustum membership.
    int inside=0, total=0;
    for(int i=0;i<=steps;i++) for(int j=0;j<=steps;j++) for(int k=0;k<=steps;k++){
        double u = -1 + 2.0*i/steps, v = -1 + 2.0*j/steps, w = -1 + 2.0*k/steps;
        if(u*u+v*v+w*w > 1.0) continue;   // outside sphere
        total++;
        Vec3 p = s.c + Vec3(u,v,w)*s.r;
        bool in=true;
        for(const auto& pl : F.planes) if(pl.dist(p) > 1e-6){ in=false; break; }
        if(in) inside++;
    }
    if(total==0) return OUT;
    if(inside==total) return IN;
    if(inside>0) return INTERSECT;
    return OUT;
}

int main(){
    // Camera: eye slightly above origin, looking toward -Z.
    Vec3 eye(0,0,5), target(0,0,-5), up(0,1,0);
    double fov=60.0, aspect=16.0/9.0, near=0.1, far=100.0;

    Mat4 V = look_at(eye, target, up);
    Mat4 P = perspective(fov, aspect, near, far);
    Mat4 VP = mul(P, V);   // view-projection (world -> clip)

    Frustum F = extract_frustum(VP);

    // ---- Validation 1: plane sanity ----
    printf("=== Validation 1: plane extraction sanity ===\n");
    bool units = true;
    for(int i=0;i<6;i++){
        double l = std::sqrt(F.planes[i].a*F.planes[i].a + F.planes[i].b*F.planes[i].b + F.planes[i].c*F.planes[i].c);
        if(std::fabs(l-1.0)>1e-6) units=false;
    }
    printf("  all plane normals unit-length: %s\n", units?"PASS":"FAIL");

    // Frustum corners: 8 combinations of near/far x left/right x top/bottom.
    // Compute them in world space via inverse, or easier: parametrize in view
    // space then transform back. Simpler: build the 8 corners analytically.
    double tn = std::tan(fov*M_PI/360.0)*near;   // half height at near
    double rn = tn*aspect;                        // half width at near
    double tf = std::tan(fov*M_PI/360.0)*far;
    double rf = tf*aspect;

    // View basis
    Vec3 fwd = (target-eye).normalized();
    Vec3 rgt = fwd.cross(up).normalized();
    Vec3 upp = rgt.cross(fwd);

    // near/far centers
    Vec3 nearC = eye + fwd*near;
    Vec3 farC  = eye + fwd*far;
    std::array<Vec3,8> corners;
    int idx=0;
    for(int sy=-1; sy<=1; sy+=2)
    for(int sx=-1; sx<=1; sx+=2){
        corners[idx++] = nearC + rgt*(sx*rn) + upp*(sy*tn);
        corners[idx++] = farC  + rgt*(sx*rf) + upp*(sy*tf);
    }

    bool corners_inside = true;
    double max_d = -1e9;
    for(const auto& c : corners){
        for(const auto& p : F.planes){
            double d = p.dist(c);
            if(d > 1e-5) corners_inside = false;   // corners must be on/below each plane
            if(d > max_d) max_d = d;
        }
    }
    printf("  all 8 frustum corners inside (dist<=0): %s (max dist %.2e)\n",
           corners_inside?"PASS":"FAIL", max_d);

    // A point clearly behind the camera must be OUT.
    Status behind = classify_sphere(F, Sphere{Vec3(0,0,20), 1.0});
    printf("  point behind camera classified OUT: %s\n", behind==OUT?"PASS":"FAIL");

    // ---- Validation 2: correctness vs brute force ----
    printf("\n=== Validation 2: correctness vs brute-force sampling ===\n");
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> ud(-30.0, 30.0);
    std::uniform_real_distribution<double> sz(0.1, 8.0);

    int ntest = 2000;
    int aabb_false_neg = 0, sph_false_neg = 0, aabb_disagree = 0, sph_disagree = 0;
    int aabb_intersect_verified = 0, aabb_intersect_unverified = 0;

    for(int t=0;t<ntest;t++){
        // random AABB
        Vec3 c(ud(rng), ud(rng), ud(rng));
        Vec3 half(sz(rng)/2, sz(rng)/2, sz(rng)/2);
        AABB b{c-half, c+half};
        Status analytic = classify_aabb(F, b);
        Status oracle   = brute_aabb(F, b, 6);

        // false negative: analytic says OUT but oracle says something else
        if(analytic==OUT && oracle!=OUT) aabb_false_neg++;
        // disagreement on IN/OUT (INTERSECT from sampling is allowed to differ
        // only when the box truly straddles; but IN and OUT must agree)
        if(analytic==IN  && oracle!=IN)  aabb_disagree++;
        if(analytic==OUT && oracle!=OUT) aabb_disagree++;

        if(analytic==INTERSECT){
            if(oracle==INTERSECT) aabb_intersect_verified++;
            else aabb_intersect_unverified++;
        }
    }

    for(int t=0;t<ntest;t++){
        Vec3 c(ud(rng), ud(rng), ud(rng));
        double rad = sz(rng)/2;
        Sphere s{c, rad};
        Status analytic = classify_sphere(F, s);
        Status oracle   = brute_sphere(F, s, 10);
        if(analytic==OUT && oracle!=OUT) sph_false_neg++;
        if(analytic==IN  && oracle!=IN)  sph_disagree++;
        if(analytic==OUT && oracle!=OUT) sph_disagree++;
    }

    printf("  AABB: %d tests, false-negatives=%d (must be 0), IN/OUT disagree=%d\n",
           ntest, aabb_false_neg, aabb_disagree);
    printf("  Sphere: %d tests, false-negatives=%d (must be 0), IN/OUT disagree=%d\n",
           ntest, sph_false_neg, sph_disagree);
    printf("  AABB INTERSECT predictions verified by sampling: %d/%d\n",
           aabb_intersect_verified, aabb_intersect_verified+aabb_intersect_unverified);

    bool correct = (aabb_false_neg==0 && sph_false_neg==0 &&
                    aabb_disagree==0 && sph_disagree==0);
    printf("  => correctness: %s\n", correct?"PASS":"FAIL");

    // ---- Validation 3: acceleration / speedup ----
    printf("\n=== Validation 3: culling cost (early-out benefit) ===\n");
    int N = 200000;
    vector<AABB> boxes(N);
    for(int t=0;t<N;t++){
        Vec3 c(ud(rng), ud(rng), ud(rng));
        Vec3 half(sz(rng)/2, sz(rng)/2, sz(rng)/2);
        boxes[t] = AABB{c-half, c+half};
    }

    // naive: always test all 6 planes, no early-out (what you'd do naively)
    long naive_plane_tests = 0;
    int culled_naive = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    for(int t=0;t<N;t++){
        const AABB& b = boxes[t];
        bool inside = true;
        for(int i=0;i<6;i++){
            const Plane& p = F.planes[i];
            naive_plane_tests++;
            Vec3 nv = negative_vert(b, p);
            if(p.dist(nv) > 0){ inside=false; } // no early-out
        }
        if(!inside) culled_naive++;
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    // analytic: early-out on first reject plane
    long analytic_plane_tests = 0;
    int culled_analytic = 0, visible_analytic = 0;
    auto t2 = std::chrono::high_resolution_clock::now();
    for(int t=0;t<N;t++){
        Status s = classify_aabb(F, boxes[t]);
        if(s==OUT) culled_analytic++; else visible_analytic++;
        // count plane tests actually performed (approximate via result)
    }
    // instrument exact plane-test count by re-running with a counter:
    auto t3 = std::chrono::high_resolution_clock::now();
    analytic_plane_tests = 0;
    for(int t=0;t<N;t++){
        const AABB& b = boxes[t];
        for(const auto& p : F.planes){
            analytic_plane_tests++;
            Vec3 nv = negative_vert(b, p);
            if(p.dist(nv) > 0) break;   // early-out
        }
    }

    double ms_naive    = std::chrono::duration<double,std::milli>(t1-t0).count();
    double ms_analytic = std::chrono::duration<double,std::milli>(t3-t2).count();

    printf("  objects tested: %d\n", N);
    printf("  culled (OUT): %d, visible (IN+INTERSECT): %d\n", culled_analytic, visible_analytic);
    printf("  naive plane-tests (no early-out): %ld  (== %ld = 6*N)\n",
           naive_plane_tests, 6L*N);
    printf("  analytic plane-tests (early-out):  %ld\n", analytic_plane_tests);
    printf("  plane-test reduction: %.1f%% of naive\n",
           100.0*analytic_plane_tests/naive_plane_tests);
    printf("  naive time:   %.2f ms\n", ms_naive);
    printf("  analytic time: %.2f ms\n", ms_analytic);
    printf("  culled-count consistency (naive vs analytic): %s\n",
           culled_naive==culled_analytic?"PASS":"differs");

    // ---- summary + cull ratio metric ----
    double cull_ratio = (double)culled_analytic / N * 100.0;
    printf("\n=== Summary ===\n");
    printf("  frustum extraction: %s\n", units&&corners_inside?"PASS":"FAIL");
    printf("  classification correctness: %s\n", correct?"PASS":"FAIL");
    printf("  cull ratio: %.1f%% of scene rejected\n", cull_ratio);
    printf("  plane-test reduction: %.1f%% of naive (early-out saves work)\n",
           100.0*analytic_plane_tests/naive_plane_tests);

    return 0;
}
