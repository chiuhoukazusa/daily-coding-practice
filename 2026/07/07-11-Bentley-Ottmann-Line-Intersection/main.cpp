#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>

const double EPS = 1e-9;
const double INF = 1e18;

struct Point { double x, y; Point(double x=0,double y=0):x(x),y(y){} };

double cross(const Point& o, const Point& a, const Point& b) {
    return (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x);
}
int sign(double v) { return (v>EPS)?1:(v<-EPS?-1:0); }
bool on_segment(const Point& p, const Point& a, const Point& b) {
    return fabs(cross(p,a,b))<EPS && fmin(a.x,b.x)-EPS<=p.x && p.x<=fmax(a.x,b.x)+EPS
           && fmin(a.y,b.y)-EPS<=p.y && p.y<=fmax(a.y,b.y)+EPS;
}

struct Segment {
    Point p1, p2; int id;
    Segment(Point a, Point b, int i) : id(i) {
        if(a.y>b.y+EPS||(fabs(a.y-b.y)<EPS&&a.x<b.x)){p1=a;p2=b;}
        else{p1=b;p2=a;}
    }
};

std::vector<std::pair<int,int>> brute_force(const std::vector<Segment>& segs) {
    std::vector<std::pair<int,int>> res;
    for(size_t i=0;i<segs.size();i++) for(size_t j=i+1;j<segs.size();j++) {
        auto&a=segs[i],&b=segs[j];
        double c1=cross(a.p1,a.p2,b.p1),c2=cross(a.p1,a.p2,b.p2);
        double c3=cross(b.p1,b.p2,a.p1),c4=cross(b.p1,b.p2,a.p2);
        if(sign(c1)*sign(c2)<0 && sign(c3)*sign(c4)<0) res.push_back({a.id,b.id});
        else if(fabs(c1)<EPS&&fabs(c2)<EPS){
            if(on_segment(a.p1,b.p1,b.p2)||on_segment(a.p2,b.p1,b.p2)||on_segment(b.p1,a.p1,a.p2)||on_segment(b.p2,a.p1,a.p2))
                res.push_back({a.id,b.id});
        }
    }
    return res;
}

struct IntersectionResult {
    int s1, s2; Point pt;
    IntersectionResult(int a,int b,Point p):s1(std::min(a,b)),s2(std::max(a,b)),pt(p){}
    bool operator<(const IntersectionResult& o) const {
        if(fabs(pt.y-o.pt.y)>EPS) return pt.y<o.pt.y;
        if(fabs(pt.x-o.pt.x)>EPS) return pt.x<o.pt.x;
        if(s1!=o.s1) return s1<o.s1;
        return s2<o.s2;
    }
};

Point compute_intersection(const Segment& a, const Segment& b) {
    double d=(a.p1.x-a.p2.x)*(b.p1.y-b.p2.y)-(a.p1.y-a.p2.y)*(b.p1.x-b.p2.x);
    if(fabs(d)<EPS) return Point((a.p1.x+b.p1.x)/2,(a.p1.y+b.p1.y)/2);
    double t=((a.p1.x-b.p1.x)*(b.p1.y-b.p2.y)-(a.p1.y-b.p1.y)*(b.p1.x-b.p2.x))/d;
    return Point(a.p1.x+t*(a.p2.x-a.p1.x),a.p1.y+t*(a.p2.y-a.p1.y));
}

bool segments_cross(const Segment& a, const Segment& b) {
    double c1=cross(a.p1,a.p2,b.p1),c2=cross(a.p1,a.p2,b.p2);
    double c3=cross(b.p1,b.p2,a.p1),c4=cross(b.p1,b.p2,a.p2);
    if(sign(c1)*sign(c2)<0 && sign(c3)*sign(c4)<0) return true;
    if(fabs(c1)<EPS&&fabs(c2)<EPS)
        return on_segment(a.p1,b.p1,b.p2)||on_segment(a.p2,b.p1,b.p2)||
               on_segment(b.p1,a.p1,a.p2)||on_segment(b.p2,a.p1,a.p2);
    return false;
}

double x_at_y(const Segment& s, double y) {
    if(fabs(s.p2.y-s.p1.y)<EPS) return s.p1.x;
    double t=(y-s.p1.y)/(s.p2.y-s.p1.y);
    if(t<0)t=0;if(t>1)t=1;
    return s.p1.x+t*(s.p2.x-s.p1.x);
}

std::vector<IntersectionResult> bentley_ottmann(const std::vector<Segment>& segs) {
    if(segs.size()<2) return {};
    std::set<IntersectionResult> results;

    struct Ev { double y; int kind; int id; }; // kind: 0=upper, 2=lower
    std::vector<Ev> evs;
    for(auto& s:segs){evs.push_back({s.p1.y,0,s.id});evs.push_back({s.p2.y,2,s.id});}
    std::sort(evs.begin(),evs.end(),[](auto&a,auto&b){
        if(fabs(a.y-b.y)>EPS) return a.y>b.y;
        if(a.kind!=b.kind) return a.kind<b.kind;
        return a.id<b.id;
    });

    // Use vector as status (sorted by x at sweep_y)
    std::vector<int> status;
    double sweep_y = INF;

    size_t i=0;
    while(i<evs.size()){
        double y=evs[i].y;
        sweep_y=y;

        std::vector<int> starting, ending;
        while(i<evs.size() && fabs(evs[i].y-y)<EPS){
            if(evs[i].kind==0) starting.push_back(evs[i].id);
            else ending.push_back(evs[i].id);
            i++;
        }

        // Re-sort status by x at current sweep_y
        std::sort(status.begin(),status.end(),[&](int a,int b){
            double xa=x_at_y(segs[a],sweep_y), xb=x_at_y(segs[b],sweep_y);
            if(fabs(xa-xb)>EPS) return xa<xb;
            return a<b;
        });

        // Check all starting segments against all active segments
        for(int sid:starting){
            for(int aid:status){
                if(!segments_cross(segs[sid],segs[aid])) continue;
                Point pt=compute_intersection(segs[sid],segs[aid]);
                if(pt.y>sweep_y+EPS) continue;
                results.insert(IntersectionResult(sid,aid,pt));
            }
        }

        // Remove ending segments
        for(int eid:ending){
            auto it=std::find(status.begin(),status.end(),eid);
            if(it==status.end()) continue;
            // Check new neighbors after removal
            int idx=it-status.begin();
            if(idx>0 && idx+1<(int)status.size()){
                int p=status[idx-1], n=status[idx+1];
                if(segments_cross(segs[p],segs[n])){
                    Point pt=compute_intersection(segs[p],segs[n]);
                    if(pt.y<=sweep_y+EPS)
                        results.insert(IntersectionResult(p,n,pt));
                }
            }
            status.erase(it);
        }

        // Add starting segments
        for(int sid:starting) status.push_back(sid);
    }

    return std::vector<IntersectionResult>(results.begin(),results.end());
}

// ===== PPM Visualization =====
void write_ppm(const std::string& f,const std::vector<Segment>& segs,
               const std::vector<IntersectionResult>& irs,int W=800,int H=800){
    std::ofstream o(f); o<<"P3\n"<<W<<" "<<H<<"\n255\n";
    double mx=INF,Mx=-INF,my=INF,My=-INF;
    for(auto&s:segs)for(auto&p:{s.p1,s.p2}){mx=fmin(mx,p.x);Mx=fmax(Mx,p.x);my=fmin(my,p.y);My=fmax(My,p.y);}
    for(auto&r:irs){mx=fmin(mx,r.pt.x);Mx=fmax(Mx,r.pt.x);my=fmin(my,r.pt.y);My=fmax(My,r.pt.y);}
    double pad=0.08*fmax(Mx-mx,My-my);if(pad<0.5)pad=0.5;
    mx-=pad;Mx+=pad;my-=pad;My+=pad;
    auto pix=[&](double x,double y){return std::make_pair(
        std::clamp((int)((x-mx)/(Mx-mx)*(W-1)),0,W-1),
        std::clamp((int)((My-y)/(My-my)*(H-1)),0,H-1));};
    std::vector<std::vector<int>>R(H,std::vector<int>(W,255)),G=R,B=R;
    for(auto&s:segs){
        auto[x1,y1]=pix(s.p1.x,s.p1.y);auto[x2,y2]=pix(s.p2.x,s.p2.y);
        int dx=abs(x2-x1),dy=abs(y2-y1),sx=x1<x2?1:-1,sy=y1<y2?1:-1,err=dx-dy;
        while(1){
            if(x1>=0&&x1<W&&y1>=0&&y1<H){R[y1][x1]=50;G[y1][x1]=50;B[y1][x1]=200;}
            if(x1==x2&&y1==y2)break;
            int e2=2*err;if(e2>-dy){err-=dy;x1+=sx;}if(e2<dx){err+=dx;y1+=sy;}
        }
    }
    for(auto&r:irs){auto[px,py]=pix(r.pt.x,r.pt.y);
        for(int dy=-3;dy<=3;dy++)for(int dx=-3;dx<=3;dx++){
            int nx=px+dx,ny=py+dy;
            if(nx>=0&&nx<W&&ny>=0&&ny<H){R[ny][nx]=255;G[ny][nx]=30;B[ny][nx]=30;}
        }
    }
    for(int y=0;y<H;y++)for(int x=0;x<W;x++)o<<R[y][x]<<" "<<G[y][x]<<" "<<B[y][x]<<" ";
}

// ===== Main =====
struct Test{std::string name;std::vector<Segment> segs;int expected;};

int main(){
    std::vector<Test> tests;
    double pi=acos(-1.0);

    {std::vector<Segment>s;int id=0;
    for(int i=1;i<=3;i++)s.emplace_back(Point(0,(double)i),Point(4,(double)i),id++);
    for(int i=1;i<=3;i++)s.emplace_back(Point((double)i,0),Point((double)i,4),id++);
    tests.push_back({"Grid_6x6",s,(int)brute_force(s).size()});}

    {std::vector<Segment>s;
    for(int i=0;i<5;i++){double a=i*pi/5;
    s.emplace_back(Point(cos(a)*5,sin(a)*5),Point(cos(a+pi)*5,sin(a+pi)*5),i);}
    tests.push_back({"Star_10",s,(int)brute_force(s).size()});}

    {std::vector<Segment>s;std::mt19937 rng(42);std::uniform_real_distribution<>d(0,10);
    for(int i=0;i<20;i++)s.emplace_back(Point(d(rng),d(rng)),Point(d(rng),d(rng)),i);
    tests.push_back({"Random_20",s,(int)brute_force(s).size()});}

    {std::vector<Segment>s;
    for(int i=0;i<20;i++){double a1=i*2*pi/20,a2=((i+7)%20)*2*pi/20;
    double r1=3.0+(i%3)*1.0,r2=3.0+((i+7)%3)*1.0;
    s.emplace_back(Point(cos(a1)*r1,sin(a1)*r1),Point(cos(a2)*r2,sin(a2)*r2),i);}
    tests.push_back({"Polygon_20",s,(int)brute_force(s).size()});}

    std::cout<<"=== Bentley-Ottmann Line Intersection ==="<<std::endl<<std::endl;
    int passed=0;
    for(auto&t:tests){
        auto bo=bentley_ottmann(t.segs); auto bf=brute_force(t.segs);
        std::set<std::pair<int,int>>bop;
        for(auto&r:bo)bop.insert({r.s1,r.s2});

        std::cout<<"Test: "<<t.name<<" ("<<t.segs.size()<<" segs)  BF:"<<t.expected<<"  BO:"<<bo.size()<<std::endl;

        int match=0;
        for(auto&p:bf)if(bop.count(p)||bop.count({p.second,p.first}))match++;

        if(match==(int)bf.size()&&(int)bop.size()==match){std::cout<<"  ✅ PASS"<<std::endl;passed++;}
        else{
            std::cout<<"  ❌ FAIL (match "<<match<<"/"<<bf.size()<<")"<<std::endl;
            std::cout<<"  Missing: ";
            for(auto&p:bf)if(!bop.count(p)&&!bop.count({p.second,p.first}))std::cout<<"("<<p.first<<","<<p.second<<") ";
            std::cout<<std::endl;
        }
        write_ppm(t.name+".ppm",t.segs,bo);
        std::cout<<"  Saved: "<<t.name<<".ppm"<<std::endl<<std::endl;
    }
    std::cout<<"===== "<<passed<<"/"<<tests.size()<<" passed ====="<<std::endl;
    return passed==(int)tests.size()?0:1;
}
