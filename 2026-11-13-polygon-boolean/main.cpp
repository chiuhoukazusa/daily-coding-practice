// Polygon Boolean Operations: Union / Intersection / Difference
// Method: sweep-line over x-slabs; within each slab compute exact interval
//         coverage of each polygon, then combine for boolean result.
// This yields exact results (no sampling error) and is verified via Shoelace
// area against analytic ground truth.

#include <bits/stdc++.h>
using namespace std;
typedef long double ld;
const ld EPS = 1e-12;

struct Pt { ld x,y; };
ld cross(const Pt&a,const Pt&b){return a.x*b.y-a.y*b.x;}
ld signedArea(const vector<Pt>&p){ ld a=0; for(size_t i=0;i<p.size();i++){size_t j=(i+1)%p.size(); a+=cross(p[i],p[j]);} return a*0.5L; }

// orientation
int ori(const Pt&a,const Pt&b,const Pt&c){ ld v=cross({b.x-a.x,b.y-a.y},{c.x-a.x,c.y-a.y}); if(fabs(v)<EPS)return 0; return v>0?1:-1; }
bool onSeg(const Pt&p,const Pt&a,const Pt&b){ return ori(a,b,p)==0 && min(a.x,b.x)-EPS<=p.x&&p.x<=max(a.x,b.x)+EPS && min(a.y,b.y)-EPS<=p.y&&p.y<=max(a.y,b.y)+EPS; }
bool segInt(const Pt&a,const Pt&b,const Pt&c,const Pt&d,Pt&o){
    int o1=ori(a,b,c),o2=ori(a,b,d),o3=ori(c,d,a),o4=ori(c,d,b);
    if(o1!=o2 && o3!=o4){
        Pt r={b.x-a.x,b.y-a.y}, s={d.x-c.x,d.y-c.y};
        ld den=cross(r,s);
        if(fabs(den)>EPS){ ld t=cross({c.x-a.x,c.y-a.y},s)/den; o={a.x+r.x*t,a.y+r.y*t}; return true; }
    }
    return false;
}

// x of intersection of horizontal line y with segment (a,b); returns pair valid
// returns true and set x if intersects (exclusive of horizontal degeneracy)
bool xsectAtY(const Pt&a,const Pt&b,ld y,ld&x){
    ld y0=a.y,y1=b.y;
    if((y-y0)*(y-y1)>0) return false; // both above or below
    // ignore exactly horizontal edges (contribute zero measure)
    if(fabs(y1-y0)<EPS) return false;
    x = a.x + (y-a.y)*(b.x-a.x)/(y1-y0);
    return true;
}

// coverage intervals of a polygon at horizontal line y (as union of [l,r] segments)
vector<pair<ld,ld>> coverAt(const vector<Pt>&p, ld y){
    vector<ld> xs;
    int n=p.size();
    for(int i=0;i<n;i++){
        int j=(i+1)%n;
        ld x;
        if(xsectAtY(p[i],p[j],y,x)) xs.push_back(x);
    }
    sort(xs.begin(),xs.end());
    vector<pair<ld,ld>> iv;
    for(size_t i=0;i+1<xs.size();i+=2){
        iv.push_back({xs[i],xs[i+1]});
    }
    return iv;
}

// area of union of intervals [l,r] over given limits
ld overlapLen(const vector<pair<ld,ld>>&A,const vector<pair<ld,ld>>&B){
    // union of intervals from A and B; but for intersect we merge differently.
    // Here compute total covered length of A ∪ B
    vector<pair<ld,ld>> all=A; all.insert(all.end(),B.begin(),B.end());
    sort(all.begin(),all.end());
    ld tot=0, curL=0, curR=-1e30; bool has=false;
    for(auto&iv:all){
        ld l=iv.first, r=iv.second;
        if(fabs(r-l)<EPS) continue;
        if(!has){curL=l;curR=r;has=true;}
        else if(l<=curR+EPS){curR=max(curR,r);}
        else {tot+=curR-curL; curL=l;curR=r;}
    }
    if(has) tot+=curR-curL;
    return tot;
}

ld lengthUnion(const vector<pair<ld,ld>>&A){
    vector<pair<ld,ld>> all=A; sort(all.begin(),all.end());
    ld tot=0,curL=0,curR=-1e30;bool has=false;
    for(auto&iv:all){ld l=iv.first,r=iv.second;if(fabs(r-l)<EPS)continue;
        if(!has){curL=l;curR=r;has=true;} else if(l<=curR+EPS){curR=max(curR,r);} else{tot+=curR-curL;curL=l;curR=r;}}
    if(has) tot+=curR-curL;
    return tot;
}

ld lengthInter(const vector<pair<ld,ld>>&A,const vector<pair<ld,ld>>&B){
    // sum of min(r)-max(l) for overlapping pairs
    ld tot=0;
    for(auto&a:A){for(auto&b:B){
        ld l=max(a.first,b.first), r=min(a.second,b.second);
        if(r>l+EPS) tot+=r-l;
    }}
    return tot;
}

int main(){
    // ===== Unified solver: sweep over y, integrate coverage lengths exactly =====
    // Critical y = vertex y's ∪ segment-intersection y's. Within each slab the
    // coverage length L(y) is linear; trapezoidal rule with interior-limit sampling
    // (y0+eps, y1-eps) is exact for piecewise-linear L.
    auto solve=[&](const vector<Pt>&P, const vector<Pt>&Q, ld&U, ld&I, ld&D){
        set<ld> ys;
        auto addPoly=[&](const vector<Pt>&g){ for(auto&p:g) ys.insert(p.y); };
        addPoly(P); addPoly(Q);
        // collect segment intersections between P and Q
        int np=P.size(), nq=Q.size();
        for(int i=0;i<np;i++){ int i2=(i+1)%np;
            for(int j=0;j<nq;j++){ int j2=(j+1)%nq;
                Pt o; if(segInt(P[i],P[i2],Q[j],Q[j2],o)){
                    if(onSeg(o,P[i],P[i2]) && onSeg(o,Q[j],Q[j2])) ys.insert(o.y);
                }
            }
        }
        vector<ld> v(ys.begin(),ys.end());
        auto cov=[&](const vector<Pt>&g,ld yy){ vector<ld> xs; int n=g.size();
            for(int i=0;i<n;i++){int j=(i+1)%n;ld x; if(xsectAtY(g[i],g[j],yy,x)) xs.push_back(x);}
            sort(xs.begin(),xs.end()); vector<pair<ld,ld>> iv; for(size_t i=0;i+1<xs.size();i+=2) iv.push_back({xs[i],xs[i+1]}); return iv; };
        U=I=D=0;
        const ld EP=1e-9;
        for(size_t k=0;k+1<v.size();k++){
            ld y0=v[k], y1=v[k+1];
            if(fabs(y1-y0)<EP) continue;
            // trapezoidal over [y0,y1], sampling interior limits
            ld lo=y0+EP, hi=y1-EP;
            auto cu=[&](ld y){ auto cA=cov(P,y), cB=cov(Q,y); return overlapLen(cA,cB); };
            auto ci=[&](ld y){ auto cA=cov(P,y), cB=cov(Q,y); return lengthInter(cA,cB); };
            auto cd=[&](ld y){ auto cA=cov(P,y), cB=cov(Q,y); return lengthUnion(cA)-lengthInter(cA,cB); };
            ld h=y1-y0;
            U += (cu(lo)+cu(hi))*0.5L*h;
            I += (ci(lo)+ci(hi))*0.5L*h;
            D += (cd(lo)+cd(hi))*0.5L*h;
        }
    };

    auto check=[&](const string&name, ld got, ld exp, int&fails){
        bool ok = fabs(got-exp)<1e-6;
        cout<<setw(22)<<left<<name<<" = "<<setprecision(6)<<got<<"  (expected "<<exp<<")  "<<(ok?"PASS":"FAIL")<<endl;
        if(!ok) fails++;
    };

    int fails=0;
    cout<<fixed<<setprecision(6);

    // ----- Primary case: two overlapping squares -----
    {
        vector<Pt> A={{0,0},{4,0},{4,4},{0,4}};
        vector<Pt> B={{2,2},{6,2},{6,6},{2,6}};
        ld aA=fabs(signedArea(A)), aB=fabs(signedArea(B));
        ld U,I,D; solve(A,B,U,I,D);
        cout<<"areaA="<<aA<<"  areaB="<<aB<<endl;
        check("A_union_B",U,28.0,fails);
        check("A_intersect_B",I,4.0,fails);
        check("A_minus_B",D,12.0,fails);
    }

    cout<<endl<<"===== Extended Test Suite ====="<<endl;
    // 1. Disjoint squares
    {
        vector<Pt> P={{0,0},{2,0},{2,2},{0,2}};
        vector<Pt> Q={{3,0},{5,0},{5,2},{3,2}};
        ld U,I,D; solve(P,Q,U,I,D);
        check("disjoint.union",U,8.0,fails); check("disjoint.inter",I,0.0,fails); check("disjoint.diff",D,4.0,fails);
    }
    // 2. Full containment
    {
        vector<Pt> P={{0,0},{4,0},{4,4},{0,4}};
        vector<Pt> Q={{1,1},{3,1},{3,3},{1,3}};
        ld U,I,D; solve(P,Q,U,I,D);
        check("contain.union",U,16.0,fails); check("contain.inter",I,4.0,fails); check("contain.diff",D,12.0,fails);
    }
    // 3. Rotated diamond inside square
    {
        vector<Pt> P={{0,0},{4,0},{4,4},{0,4}};
        vector<Pt> Q={{2,0},{4,2},{2,4},{0,2}};
        ld U,I,D; solve(P,Q,U,I,D);
        check("diamond.union",U,16.0,fails); check("diamond.inter",I,8.0,fails); check("diamond.diff",D,8.0,fails);
    }
    // 4. Two overlapping right triangles
    {
        vector<Pt> P={{0,0},{4,0},{4,4}};
        vector<Pt> Q={{2,0},{6,0},{6,4}};
        ld U,I,D; solve(P,Q,U,I,D);
        check("tri.union",U,14.0,fails); check("tri.inter",I,2.0,fails); check("tri.diff",D,6.0,fails);
    }
    // 5. Concave L-shapes (consistency identities)
    {
        vector<Pt> P={{0,0},{4,0},{4,2},{2,2},{2,4},{0,4}}; // area 12
        vector<Pt> Q={{2,2},{6,2},{6,4},{4,4},{4,6},{2,6}}; // area 12
        ld aP=fabs(signedArea(P)), aQ=fabs(signedArea(Q));
        ld U,I,D; solve(P,Q,U,I,D);
        check("concave.U+I=P+Q", U+I, aP+aQ, fails);
        check("concave.D+I=P", D+I, aP, fails);
        check("concave.inter+diff.diff=I+D", I+D, aP, fails); // D+I=P implies I+D=P
    }
    // 6. Rotated (non-axis-aligned) squares overlap
    {
        // Square A: [0,2]x[0,2]; Square B: rotated 45deg diamond centered (2,1), radius 1
        // diamond B: vertices (2,0),(3,1),(2,2),(1,1); area 2
        vector<Pt> P={{0,0},{2,0},{2,2},{0,2}}; // area 4
        vector<Pt> Q={{2,0},{3,1},{2,2},{1,1}}; // area 2
        ld U,I,D; solve(P,Q,U,I,D);
        // diamond B spans x[1,3],y[0,2]; square A spans x[0,2],y[0,2].
        // B is fully within y[0,2]; B's left half (x in [1,2]) overlaps A (x<2), right half x>2 outside A.
        // overlap area = half of B = 1 (by symmetry, B left of x=2 is triangle (1,1)(2,0)(2,2) area 1)
        check("rot.union",U, 4.0+2.0-1.0, fails);
        check("rot.inter",I, 1.0, fails);
        check("rot.diff",D, 4.0-1.0, fails);
    }

    cout<<endl<<(fails==0?"=== ALL TEST SUITES PASSED ===":"=== FAILURES: "+to_string(fails)+" ===")<<endl;
    return fails==0?0:1;
}
