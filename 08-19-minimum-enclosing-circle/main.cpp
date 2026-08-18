// Minimum Enclosing Circle — Welzl's randomized incremental algorithm
//
// Purpose:
//   1. Implement Welzl's algorithm to find the smallest circle enclosing a set of 2D points.
//   2. Quantitatively verify correctness against a brute-force O(n^4) reference:
//      - every generated circle must contain *all* points (max violation <= eps)
//      - radius must match the brute-force optimum within floating-point tolerance
//   3. Measure the speedup of Welzl vs brute force.
//
// Core pieces:
//   - circle_from_2(a,b): circle with diameter ab
//   - circle_from_3(a,b,c): circle through three non-collinear points (solve linear system)
//   - circle_from_1/2/3 helpers that early-exit when a point already lies inside
//   - Welzl(P, R), R = up to 3 "boundary" points
//
// Output: an ASCII / PPM visualization of points + enclosing circle (for a human eyeball,
// but correctness is established by the numeric assertions, NOT by the image).

#include <bits/stdc++.h>
using namespace std;

struct Point {
    double x, y;
    Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

struct Circle {
    Point c;
    double r;
    Circle() : c(0,0), r(0) {}
    Circle(Point c_, double r_) : c(c_), r(r_) {}
};

const double EPS = 1e-9;

double dist2(const Point& a, const Point& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx*dx + dy*dy;
}
double dist(const Point& a, const Point& b) { return sqrt(dist2(a,b)); }

bool inside(const Circle& C, const Point& p) {
    return dist2(C.c, p) <= C.r*C.r + EPS;
}

// Circle with diameter endpoints a,b
Circle circle_from_2(const Point& a, const Point& b) {
    Point c((a.x+b.x)/2.0, (a.y+b.y)/2.0);
    return Circle(c, dist(a,b)/2.0);
}

// Circle through three points (assumes non-collinear)
Circle circle_from_3(const Point& a, const Point& b, const Point& c) {
    // Solve center (cx, cy) via perpendicular bisectors:
    // Using the standard formula with 2x2 linear system.
    double A = b.x - a.x, B = b.y - a.y;
    double C = c.x - a.x, D = c.y - a.y;
    double E = A*(a.x+b.x) + B*(a.y+b.y);
    double F = C*(a.x+c.x) + D*(a.y+c.y);
    double G = 2.0 * (A*(c.y-b.y) - B*(c.x-b.x));
    if (fabs(G) < EPS) {
        // collinear — fall back to largest diameter
        Circle c1 = circle_from_2(a,b), c2 = circle_from_2(a,c), c3 = circle_from_2(b,c);
        Circle best = c1;
        if (c2.r > best.r) best = c2;
        if (c3.r > best.r) best = c3;
        return best;
    }
    double cx = (D*E - B*F) / G;
    double cy = (A*F - C*E) / G;
    Point center(cx, cy);
    return Circle(center, dist(center, a));
}

// Minimal circle through 0..3 boundary points and containing all of P
Circle welzl_inner(vector<Point>& P, vector<Point>& R, int n) {
    if (n == 0 || R.size() == 3) {
        if (R.empty())      return Circle(Point(0,0), 0);
        if (R.size() == 1)  return Circle(R[0], 0);
        if (R.size() == 2)  return circle_from_2(R[0], R[1]);
        return circle_from_3(R[0], R[1], R[2]);
    }
    // pick a random index among remaining points
    int idx = rand() % n;
    Point p = P[idx];
    swap(P[idx], P[n-1]);

    Circle C = welzl_inner(P, R, n-1);
    if (inside(C, p)) return C;

    R.push_back(p);
    C = welzl_inner(P, R, n-1);
    R.pop_back();
    return C;
}

Circle minimum_enclosing_circle(vector<Point> P) {
    vector<Point> R;
    R.reserve(3);
    unsigned seed = 12345;
    // We use deterministic seed for reproducibility in verification
    // but randomized order is still beneficial vs adversarial input.
    for (int i = 0; i < 10; i++) { /* warm */ }
    // Fisher-Yates shuffle with fixed seed for reproducible randomness
    mt19937 rng(seed);
    shuffle(P.begin(), P.end(), rng);
    return welzl_inner(P, R, (int)P.size());
}

// Brute-force reference: returns the exact minimum enclosing circle.
Circle brute_force_mec(const vector<Point>& pts) {
    int n = pts.size();
    Circle best(Point(0,0), 0);

    auto circum = [&](int i,int j,int k)->Circle{
        return circle_from_3(pts[i], pts[j], pts[k]);
    };

    // Special cases n=0,1,2
    if (n == 0) return Circle(Point(0,0),0);
    if (n == 1) return Circle(pts[0],0);
    if (n == 2) return circle_from_2(pts[0], pts[1]);

    double best_r = 1e18;
    for (int i = 0; i < n; i++) {
        for (int j = i+1; j < n; j++) {
            Circle c2 = circle_from_2(pts[i], pts[j]);
            bool ok = true;
            for (int t = 0; t < n; t++) if (!inside(c2, pts[t])) { ok=false; break; }
            if (ok && c2.r < best_r) { best_r = c2.r; best = c2; }
            for (int k = j+1; k < n; k++) {
                Circle c3 = circum(i,j,k);
                ok = true;
                for (int t = 0; t < n; t++) if (!inside(c3, pts[t])) { ok=false; break; }
                if (ok && c3.r < best_r) { best_r = c3.r; best = c3; }
            }
        }
    }
    if (best_r > 1e17) { // fallback (all collinear handled by pairs already)
        for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++) {
            Circle c2 = circle_from_2(pts[i], pts[j]);
            if (c2.r < best_r) { best_r = c2.r; best = c2; }
        }
    }
    return best;
}

// ---- PPM visualization ----
void write_ppm(const vector<Point>& pts, const Circle& C, const string& path) {
    const int W = 800, H = 800;
    vector<unsigned char> img(W*H*3, 255); // white background

    auto put = [&](int px, int py, unsigned char r, unsigned char g, unsigned char b){
        if (px<0||px>=W||py<0||py>=H) return;
        img[(py*W+px)*3+0]=r; img[(py*W+px)*3+1]=g; img[(py*W+px)*3+2]=b;
    };

    double minx=1e18,maxx=-1e18,miny=1e18,maxy=-1e18;
    for (auto&p:pts){ minx=min(minx,p.x);maxx=max(maxx,p.x);miny=min(miny,p.y);maxy=max(maxy,p.y);}
    // include circle extent
    minx=min(minx,C.c.x-C.r); maxx=max(maxx,C.c.x+C.r);
    miny=min(miny,C.c.y-C.r); maxy=max(maxy,C.c.y+C.r);
    double pad = 0.05*(max(maxx-minx,maxy-miny)+1e-9);
    minx-=pad; maxx+=pad; miny-=pad; maxy+=pad;

    auto to_px = [&](double x,double y)->pair<int,int>{
        int px = (int)((x-minx)/(maxx-minx)*W);
        int py = (int)((y-miny)/(maxy-miny)*H);
        px = max(0,min(W-1,px)); py=max(0,min(H-1,py));
        return {px,py};
    };

    // draw circle (thick)
    auto [cx,cy] = to_px(C.c.x, C.c.y);
    double rx = C.r/(maxx-minx)*W;
    double ry = C.r/(maxy-miny)*H;
    for (int a=0;a<3600;a++){
        double th = a/3600.0*2*M_PI;
        int px = (int)(cx + rx*cos(th));
        int py = (int)(cy + ry*sin(th));
        put(px,py,220,20,20);
        put(px+1,py,220,20,20); put(px-1,py,220,20,20);
        put(px,py+1,220,20,20); put(px,py-1,220,20,20);
    }
    // center
    put(cx,cy,255,0,0); put(cx+1,cy,255,0,0);put(cx-1,cy,255,0,0);put(cx,cy+1,255,0,0);put(cx,cy-1,255,0,0);

    // draw points
    for (auto&p:pts){ auto [px,py]=to_px(p.x,p.y); put(px,py,0,80,200); put(px+1,py,0,80,200);put(px-1,py,0,80,200);put(px,py+1,0,80,200);put(px,py-1,0,80,200);}

    FILE* f = fopen(path.c_str(),"wb");
    fprintf(f,"P6\n%d %d\n255\n",W,H);
    fwrite(img.data(),1,img.size(),f);
    fclose(f);
}

int main() {
    srand(12345);

    // ============ VERIFICATION ============
    const int TRIALS = 200;
    int all_inside_ok = 0;     // count trials where every point is inside Welzl circle
    double max_radius_diff = 0; // max |welzl.r - brute.r| over valid trials
    int radius_match = 0;      // trials where radius matches within 1e-6
    double max_violation = 0;  // max distance a point sticks out of the circle

    for (int t = 0; t < TRIALS; t++) {
        int n = 2 + rand() % 18; // 2..19 points
        vector<Point> pts;
        // random points, sometimes cluster, sometimes near-circle, sometimes extreme
        int mode = t % 3;
        for (int i = 0; i < n; i++) {
            double x = (rand()%20001)/100.0 - 100.0; // [-100,100]
            double y = (rand()%20001)/100.0 - 100.0;
            if (mode == 0) { x*=0.1; y*=0.1; }            // tight cluster
            else if (mode == 1) { /* spread */ }
            else { double ang = (rand()%6283)/1000.0;    // near circle ring
                   double rad = 80 + (rand()%500)/100.0;
                   x = rad*cos(ang); y = rad*sin(ang); }
            pts.push_back(Point(x,y));
        }
        Circle welzl = minimum_enclosing_circle(pts);
        Circle brute = brute_force_mec(pts);

        // 1. all points inside welzl circle
        double worst = 0;
        for (auto&p:pts) {
            double d = dist(p, welzl.c);
            worst = max(worst, d - welzl.r);
        }
        max_violation = max(max_violation, worst);
        if (worst <= 1e-6) all_inside_ok++;

        // 2. radius matches brute force
        double diff = fabs(welzl.r - brute.r);
        max_radius_diff = max(max_radius_diff, diff);
        if (diff <= 1e-6) radius_match++;
    }

    printf("================ 量化验证结果 (Welzl 最小包围圆) ================\n");
    printf("试验次数            : %d\n", TRIALS);
    printf("所有点在圆内        : %d / %d  (%.1f%%)\n", all_inside_ok, TRIALS, 100.0*all_inside_ok/TRIALS);
    printf("最大越界距离        : %.6e (应 ~0)\n", max_violation);
    printf("半径与暴力基准一致  : %d / %d  (%.1f%%)\n", radius_match, TRIALS, 100.0*radius_match/TRIALS);
    printf("最大半径误差        : %.6e (应 ~0)\n", max_radius_diff);
    printf("----------------------------------------------------------------\n");

    bool pass = (all_inside_ok == TRIALS) && (max_violation <= 1e-6)
                && (radius_match == TRIALS) && (max_radius_diff <= 1e-6);
    printf("%s\n", pass ? "✅ 全部量化验证通过" : "❌ 存在验证失败");

    // ============ PERFORMANCE ============
    // Build a moderately large point set and time Welzl vs brute (small enough for brute)
    int nBig = 60;
    vector<Point> big;
    for (int i=0;i<nBig;i++) big.push_back(Point((rand()%20001)/100.0-100.0,(rand()%20001)/100.0-100.0));

    auto t0 = chrono::high_resolution_clock::now();
    Circle w = minimum_enclosing_circle(big);
    auto t1 = chrono::high_resolution_clock::now();
    double tw = chrono::duration<double,milli>(t1-t0).count();

    t0 = chrono::high_resolution_clock::now();
    volatile Circle b = brute_force_mec(big);
    t1 = chrono::high_resolution_clock::now();
    double tb = chrono::duration<double,milli>(t1-t0).count();
    (void)b;

    printf("----------------------------------------------------------------\n");
    printf("点数 n=%d : Welzl=%.3f ms, 暴力=%.3f ms, 加速比=%.2fx\n", nBig, tw, tb, tb/tw);

    // ============ VISUALIZATION ============
    vector<Point> vis = big;
    write_ppm(vis, w, "welzl_mec_output.ppm");
    printf("已生成可视化: welzl_mec_output.ppm (中心=(%.2f,%.2f), 半径=%.3f)\n", w.c.x, w.c.y, w.r);

    return pass ? 0 : 1;
}
