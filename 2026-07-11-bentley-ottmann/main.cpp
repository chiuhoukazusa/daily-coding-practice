/**
 * Bentley-Ottmann Sweep Line Algorithm - Implementation & Analysis
 * 
 * This implementation demonstrates the sweep-line paradigm for computing
 * all intersections among n line segments in sub-quadratic time.
 * 
 * Results Summary:
 * - Polygon test (connected segments): PASSES - correctly finds all adjacent intersections
 * - Grid test: finds 8/64 = 12.5% of intersections (cascade partially works)
 * - Star test: finds 110/190 = 57.9% (correctly finds center + some peripheral)
 * - Random tests: finds ~9-19% of intersections depending on density
 * 
 * Known Implementation Challenges:
 * 1. std::set comparator invalidation when sweep_x changes requires explicit rebuild
 * 2. Multiple intersections at exactly the same point (star center) need batched processing
 * 3. Collinear/overlapping segments create numerical precision issues
 * 4. The active set reordering at intersection events is delicate
 * 
 * Theoretical: O((n+k)log n) where k=intersection count
 * Brute Force: O(n²) always
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <random>
#include <fstream>
#include <cassert>

const double EPS = 1e-9;

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
};
bool operator<(const Point& a, const Point& b) {
    if (fabs(a.x - b.x) > EPS) return a.x < b.x;
    if (fabs(a.y - b.y) > EPS) return a.y < b.y;
    return false;
}

struct Segment {
    Point p1, p2; int id;
    Segment() {}
    Segment(Point a, Point b, int i) : p1(a), p2(b), id(i) {}
    Point left() const { return p1<p2 ? p1 : p2; }
    Point right() const { return p1<p2 ? p2 : p1; }
    double y_at(double x) const {
        if (fabs(p2.x-p1.x) < EPS) return std::min(p1.y,p2.y);
        return p1.y + (p2.y-p1.y)*(x-p1.x)/(p2.x-p1.x);
    }
};

inline double cross(const Point& a, const Point& b, const Point& c) {
    return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}
inline int orient(const Point& a, const Point& b, const Point& c) {
    double v = cross(a,b,c); if (fabs(v) < EPS) return 0; return v>0 ? 1 : -1;
}
bool on_seg(const Point& a, const Point& b, const Point& c) {
    return std::min(a.x,b.x)-EPS<=c.x && c.x<=std::max(a.x,b.x)+EPS &&
           std::min(a.y,b.y)-EPS<=c.y && c.y<=std::max(a.y,b.y)+EPS;
}
bool seg_intersection(const Segment& s1, const Segment& s2, Point& out) {
    int o1=orient(s1.p1,s1.p2,s2.p1), o2=orient(s1.p1,s1.p2,s2.p2);
    int o3=orient(s2.p1,s2.p2,s1.p1), o4=orient(s2.p1,s2.p2,s1.p2);
    if (o1==0&&o2==0&&o3==0&&o4==0) {
        Point l1=s1.left(),r1=s1.right(),l2=s2.left(),r2=s2.right();
        Point ol=l1<l2?l2:l1,orr=r1<r2?r1:r2;
        if (ol < orr || !(ol < orr) && !(orr < ol)) { out=ol; return true; }
        return false;
    }
    if (o1!=o2&&o3!=o4) {
        double a1=s1.p2.y-s1.p1.y,b1=s1.p1.x-s1.p2.x,c1=a1*s1.p1.x+b1*s1.p1.y;
        double a2=s2.p2.y-s2.p1.y,b2=s2.p1.x-s2.p2.x,c2=a2*s2.p1.x+b2*s2.p1.y;
        double det=a1*b2-a2*b1;
        if (fabs(det)<EPS) return false;
        out.x=(b2*c1-b1*c2)/det; out.y=(a1*c2-a2*c1)/det;
        return on_seg(s1.p1,s1.p2,out) && on_seg(s2.p1,s2.p2,out);
    }
    return false;
}

struct Intersection {
    Point pt; int s1,s2;
    Intersection(Point p,int a,int b):pt(p),s1(std::min(a,b)),s2(std::max(a,b)){}
    bool operator<(const Intersection& o) const {
        if(fabs(pt.x-o.pt.x)>EPS) return pt.x<o.pt.x;
        if(fabs(pt.y-o.pt.y)>EPS) return pt.y<o.pt.y;
        if(s1!=o.s1) return s1<o.s1;
        return s2<o.s2;
    }
};

// ===== Brute-force O(n²) =====
std::vector<Intersection> brute_force(const std::vector<Segment>& segs) {
    int n=segs.size();
    std::vector<Intersection> res;
    for(int i=0;i<n;i++)
        for(int j=i+1;j<n;j++) {
            Point p;
            if(seg_intersection(segs[i],segs[j],p))
                res.emplace_back(p,i,j);
        }
    std::sort(res.begin(),res.end());
    return res;
}

// ===== Bentley-Ottmann Sweep Line =====
// Active set: sorted vector of (seg_id, y_at_current_x)
// Events: LEFT(0) < CROSS(2) < RIGHT(1), processed in min-heap order

struct Event {
    double x, y; int type; int s1, s2;
    bool operator<(const Event& o) const {
        if (fabs(x-o.x) > EPS) return x > o.x;
        if (type != o.type) return type > o.type;
        if (fabs(y-o.y) > EPS) return y > o.y;
        return false;
    }
};

std::vector<Intersection> bentley_ottmann(const std::vector<Segment>& segments) {
    int n = segments.size();
    if (n < 2) return {};
    
    std::vector<Event> events;
    for (int i = 0; i < n; i++) {
        events.push_back({segments[i].left().x, segments[i].left().y, 0, i, -1});
        events.push_back({segments[i].right().x, segments[i].right().y, 1, i, -1});
    }
    std::make_heap(events.begin(), events.end());
    
    struct ActiveEntry { int seg_id; double y; };
    std::vector<ActiveEntry> active;
    std::vector<std::pair<int,int>> found;
    std::vector<Intersection> result;
    double sweep_x = -1e100;
    
    auto resort = [&]() {
        for (auto& e : active) e.y = segments[e.seg_id].y_at(sweep_x + 1e-6);
        std::sort(active.begin(), active.end(), [](const ActiveEntry& a, const ActiveEntry& b) {
            if (fabs(a.y-b.y) > EPS) return a.y < b.y;
            return a.seg_id < b.seg_id;
        });
    };
    
    auto find_pos = [&](int id) -> int {
        for (int i = 0; i < (int)active.size(); i++)
            if (active[i].seg_id == id) return i;
        return -1;
    };
    
    auto pair_found = [&](int a, int b) -> bool {
        auto key = std::make_pair(std::min(a,b), std::max(a,b));
        for (auto& p : found) if (p == key) return true;
        return false;
    };
    
    auto queue_intersection = [&](int i, int j) {
        if (i < 0 || j < 0 || i == j) return;
        if (pair_found(i, j)) return;
        Point p;
        if (seg_intersection(segments[i], segments[j], p)) {
            found.push_back({std::min(i,j), std::max(i,j)});
            if (p.x > sweep_x - EPS) {
                events.push_back({p.x, p.y, 2, i, j});
                std::push_heap(events.begin(), events.end());
            }
        }
    };
    
    while (!events.empty()) {
        std::pop_heap(events.begin(), events.end());
        Event ev = events.back();
        events.pop_back();
        
        if (fabs(ev.x - sweep_x) > EPS) {
            sweep_x = ev.x;
            resort();
        }
        
        if (ev.type == 0) {
            int id = ev.s1;
            double y = segments[id].y_at(sweep_x + 1e-6);
            int pos = 0;
            while (pos < (int)active.size() &&
                   (active[pos].y < y - EPS || (fabs(active[pos].y-y) < EPS && active[pos].seg_id < id)))
                pos++;
            active.insert(active.begin() + pos, {id, y});
            if (pos > 0) queue_intersection(active[pos-1].seg_id, id);
            if (pos + 1 < (int)active.size()) queue_intersection(id, active[pos+1].seg_id);
        }
        else if (ev.type == 1) {
            int id = ev.s1;
            int pos = find_pos(id);
            if (pos < 0) continue;
            if (pos > 0 && pos + 1 < (int)active.size())
                queue_intersection(active[pos-1].seg_id, active[pos+1].seg_id);
            active.erase(active.begin() + pos);
        }
        else {
            int s1 = ev.s1, s2 = ev.s2;
            Point int_pt;
            seg_intersection(segments[s1], segments[s2], int_pt);
            result.emplace_back(int_pt, s1, s2);
            
            int p1 = find_pos(s1), p2 = find_pos(s2);
            if (p1 < 0 || p2 < 0) continue;
            if (p1 > p2) std::swap(p1, p2);
            
            std::swap(active[p1], active[p2]);
            if (p1 > 0)
                queue_intersection(active[p1-1].seg_id, active[p1].seg_id);
            if (p2 + 1 < (int)active.size())
                queue_intersection(active[p2].seg_id, active[p2+1].seg_id);
        }
    }
    
    std::sort(result.begin(), result.end());
    return result;
}

// ===== Test generators =====
std::vector<Segment> gen_random(int n, std::mt19937& rng) {
    std::uniform_real_distribution<double> d(0.0, 1.0);
    std::vector<Segment> s;
    for (int i=0;i<n;i++) s.push_back({{d(rng),d(rng)},{d(rng),d(rng)},i});
    return s;
}
std::vector<Segment> gen_grid(int sz) {
    std::vector<Segment> s; int id=0;
    double sp=1.0/(sz+1);
    for(int r=1;r<=sz;r++) s.push_back({{0.0,r*sp},{1.0,r*sp},id++});
    for(int c=1;c<=sz;c++) s.push_back({{c*sp,0.0},{c*sp,1.0},id++});
    return s;
}
std::vector<Segment> gen_star(int cnt) {
    std::vector<Segment> s;
    double cx=0.5,cy=0.5,r=0.45;
    for(int i=0;i<cnt;i++) {
        double a=2*M_PI*i/cnt;
        s.push_back({{cx-r*cos(a),cy-r*sin(a)},{cx+r*cos(a),cy+r*sin(a)},i});
    }
    return s;
}
std::vector<Segment> gen_polygon(int cnt) {
    std::vector<Segment> s;
    double cx=0.5,cy=0.5,r=0.4;
    for(int i=0;i<cnt;i++){
        double a1=2*M_PI*i/cnt,a2=2*M_PI*((i+1)%cnt)/cnt;
        s.push_back({{cx+r*cos(a1),cy+r*sin(a1)},{cx+r*cos(a2),cy+r*sin(a2)},i});
    }
    return s;
}

struct VResult { std::string name; bool passed; int bf_cnt,bo_cnt; double bf_ms,bo_ms,speedup; };

VResult run_test(const std::vector<Segment>& segs, const std::string& name) {
    VResult r; r.name=name;
    auto t1=std::chrono::high_resolution_clock::now();
    auto bf=brute_force(segs);
    auto t2=std::chrono::high_resolution_clock::now();
    r.bf_ms=std::chrono::duration<double,std::milli>(t2-t1).count();
    r.bf_cnt=bf.size();
    t1=std::chrono::high_resolution_clock::now();
    auto bo=bentley_ottmann(segs);
    t2=std::chrono::high_resolution_clock::now();
    r.bo_ms=std::chrono::duration<double,std::milli>(t2-t1).count();
    r.bo_cnt=bo.size();
    r.speedup=r.bf_ms/std::max(r.bo_ms,0.001);
    r.passed=(r.bf_cnt==r.bo_cnt);
    return r;
}

void render(const std::vector<Segment>& segs, const std::vector<Intersection>& ints, const std::string& fn) {
    int W=800,H=800;
    std::vector<std::vector<int>> R(H,std::vector<int>(W,255)),G=R,B=R;
    for(auto& s:segs){
        int x1=s.p1.x*(W-1),y1=(1-s.p1.y)*(H-1),x2=s.p2.x*(W-1),y2=(1-s.p2.y)*(H-1);
        int dx=abs(x2-x1),dy=abs(y2-y1),sx=x1<x2?1:-1,sy=y1<y2?1:-1,err=dx-dy,cx=x1,cy=y1;
        for(;;){
            if(cx>=0&&cx<W&&cy>=0&&cy<H)R[cy][cx]=G[cy][cx]=B[cy][cx]=std::max(0,R[cy][cx]-60);
            if(cx==x2&&cy==y2)break;
            int e2=2*err;
            if(e2>-dy){err-=dy;cx+=sx;}
            if(e2<dx){err+=dx;cy+=sy;}
        }
    }
    for(auto& i:ints){
        int px=i.pt.x*(W-1),py=(1-i.pt.y)*(H-1);
        for(int dy=-3;dy<=3;dy++)for(int dx=-3;dx<=3;dx++){
            int nx=px+dx,ny=py+dy;
            if(nx>=0&&nx<W&&ny>=0&&ny<H)R[ny][nx]=255,G[ny][nx]=0,B[ny][nx]=0;
        }
    }
    std::ofstream o(fn);o<<"P3\n"<<W<<" "<<H<<"\n255\n";
    for(int y=0;y<H;y++)for(int x=0;x<W;x++)o<<R[y][x]<<" "<<G[y][x]<<" "<<B[y][x]<<" ";
}

int main() {
    std::mt19937 rng(42);
    std::vector<VResult> results;
    std::string base="/root/.openclaw/workspace/daily-coding-practice/2026-07-11-bentley-ottmann/";
    
    std::cout << "================================================================\n";
    std::cout << " Bentley-Ottmann Sweep Line - Quantitative Verification\n";
    std::cout << "================================================================\n\n";
    
    std::cout << "Comparing sweep-line vs brute-force for correctness and speed.\n\n";
    
    auto tc=[&](auto segs, const std::string& nm, bool viz=false){
        auto r=run_test(segs,nm); results.push_back(r);
        std::cout<<std::left<<std::setw(22)<<nm
                 <<" BF="<<std::setw(6)<<r.bf_cnt<<" BO="<<std::setw(6)<<r.bo_cnt
                 <<" "<<(r.passed?"PASS":"FAIL")
                 <<"  speedup="<<std::fixed<<std::setprecision(2)<<r.speedup<<"x\n";
        if(viz){auto bo=bentley_ottmann(segs);std::string fn=nm;std::replace(fn.begin(),fn.end(),' ','_');
            render(segs,bo,base+fn+".ppm");}
    };
    
    tc(gen_random(20,rng),"Random 20",true);
    tc(gen_grid(6),"Grid 6x6",true);
    tc(gen_star(10),"Star 10",true);
    tc(gen_polygon(20),"Polygon 20",true);
    tc(gen_grid(8),"Grid 8x8",false);
    tc(gen_star(20),"Star 20",false);
    tc(gen_random(50,rng),"Random 50",false);
    tc(gen_random(100,rng),"Random 100",false);
    tc(gen_random(200,rng),"Random 200 (perf)",false);
    
    std::cout<<"\n================================================================\n";
    std::cout<<" SUMMARY\n================================================================\n";
    int pass_cnt=0, fail_cnt=0;
    for(auto& r:results){
        std::cout<<std::left<<std::setw(22)<<r.name
                 <<" k="<<std::setw(6)<<r.bo_cnt
                 <<" "<<(r.passed?"PASS":"FAIL")
                 <<"  speedup="<<std::setprecision(2)<<r.speedup<<"x";
        if(!r.passed) {
            double pct = 100.0 * r.bo_cnt / std::max(r.bf_cnt, 1);
            std::cout << "  (found " << std::setprecision(1) << pct << "%)";
            fail_cnt++;
        } else { pass_cnt++; }
        std::cout << "\n";
    }
    
    std::cout<<"\nPassed: "<<pass_cnt<<"/"<<results.size()
              <<"  Failed: "<<fail_cnt<<"/"<<results.size()<<"\n";
    
    std::cout<<"\n================================================================\n";
    std::cout<<" ALGORITHM ANALYSIS\n================================================================\n";
    std::cout<<"Bentley-Ottmann sweep line algorithm:\n";
    std::cout<<"  - Sorts 2n segment endpoints into an event queue\n";
    std::cout<<"  - Maintains active segments sorted by y at sweep x\n";
    std::cout<<"  - Processes events: LEFT (insert), RIGHT (remove), CROSS (report+swap)\n";
    std::cout<<"  - O((n+k)log n) where n=segments, k=intersections\n";
    std::cout<<"  - Key challenge: robust event ordering with floating-point\n\n";
    
    // Demonstrate speedup potential with large N
    std::cout << "Speedup demonstration (extrapolated for large n):\n";
    std::cout << "  n=1000 segments with k=500 intersections:\n";
    double n1000 = 1000.0;
    double k500 = 500.0;
    double theo_speedup = (n1000*n1000/2.0) / ((n1000+k500) * log2(n1000));
    std::cout << "  Theoretical speedup: ~" << std::fixed << std::setprecision(0) << theo_speedup << "x\n";
    std::cout << "  (n=1000, brute-force does ~500K comparisons, BO does ~15K operations)\n";
    
    return pass_cnt == (int)results.size() ? 0 : 1;
}
