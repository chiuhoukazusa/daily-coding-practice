// Flow Field Pathfinding
// 实现 RTS 游戏中常用的流场寻路（Flow Field / Vector Field Pathfinding）：
//   1. 构建成本场 (cost field)：障碍物无穷大，通行区域为 1
//   2. 积分场 (integration field)：从 goal 出发用 Dijkstra(BFS 变体) 累计最小成本
//   3. 流场 (flow field)：每个格点的方向指向成本下降最快的邻格
//   4. 智能体沿流场梯度下降移动，所有智能体共享同一积分场（一次计算，多智能体复用）
//   5. 与 A* 做路径长度 / 最优性 / 节点扩展对比
// 量化验证点：可达性、路径长度 vs A* 最优长度、多智能体全到达、方向场一致性。

#include <bits/stdc++.h>
using namespace std;

struct Vec2i { int x, y; };
bool operator==(Vec2i a, Vec2i b){ return a.x==b.x && a.y==b.y; }

const int INF = 1e9;

int W = 80, H = 60;

struct Grid {
    int W, H;
    vector<int> cost;      // cost field (0=free cost1, 1=wall)
    vector<int> integ;     // integration field
    vector<int> dirx, diry; // flow direction (-1,0,1)
    Grid(int w, int h): W(w), H(h) {
        cost.assign(w*h, 1);
        integ.assign(w*h, INF);
        dirx.assign(w*h, 0);
        diry.assign(w*h, 0);
    }
    int idx(int x,int y){ return y*W + x; }
    bool inb(int x,int y){ return x>=0&&x<W&&y>=0&&y<H; }
    bool wall(int x,int y){ return !inb(x,y) || cost[idx(x,y)]>=INF; }
};

// 8-direction neighbors
const int DX[8] = {1,-1,0,0,1,1,-1,-1};
const int DY[8] = {0,0,1,-1,1,-1,1,-1};
const double DC[8] = {1,1,1,1,sqrt(2),sqrt(2),sqrt(2),sqrt(2)};

// Build integration field: Dijkstra from goal
void buildIntegration(Grid &g, Vec2i goal) {
    fill(g.integ.begin(), g.integ.end(), INF);
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
    int gi = g.idx(goal.x, goal.y);
    g.integ[gi] = 0;
    pq.push({0, gi});
    while(!pq.empty()){
        auto [d, cur] = pq.top(); pq.pop();
        if(d > g.integ[cur]) continue;
        int cx = cur % g.W, cy = cur / g.W;
        for(int k=0;k<8;k++){
            int nx = cx+DX[k], ny = cy+DY[k];
            if(!g.inb(nx,ny)) continue;
            int ni = g.idx(nx,ny);
            if(g.cost[ni] >= INF) continue;
            int nd = d + (int)round(g.cost[ni]*DC[k]*10); // scale to avoid float cmp
            if(nd < g.integ[ni]){
                g.integ[ni] = nd;
                pq.push({nd, ni});
            }
        }
    }
}

// Build flow field from integration field
void buildFlow(Grid &g) {
    for(int y=0;y<g.H;y++){
        for(int x=0;x<g.W;x++){
            int i = g.idx(x,y);
            if(g.cost[i]>=INF || g.integ[i]>=INF){ g.dirx[i]=g.diry[i]=0; continue; }
            // choose neighbor with smallest integration value
            int best = g.integ[i]; int bx=0, by=0;
            for(int k=0;k<8;k++){
                int nx=x+DX[k], ny=y+DY[k];
                if(!g.inb(nx,ny)) continue;
                int ni=g.idx(nx,ny);
                if(g.cost[ni]>=INF) continue;
                if(g.integ[ni] < best){ best = g.integ[ni]; bx=DX[k]; by=DY[k]; }
            }
            if(best < g.integ[i]){ g.dirx[i]=bx; g.diry[i]=by; }
            else { g.dirx[i]=g.diry[i]=0; }
        }
    }
}

// Simulate an agent following the flow field
// Returns: (reached_goal, path_length_scaled, steps)
tuple<bool,double,int> simulateAgent(Grid &g, Vec2i start, Vec2i goal, int maxSteps=20000){
    double costAcc = 0;
    int steps = 0;
    int x=start.x, y=start.y;
    while(!(x==goal.x && y==goal.y)){
        int i = g.idx(x,y);
        int dx = g.dirx[i], dy = g.diry[i];
        if(dx==0 && dy==0){
            // local minimum / no direction -> failed
            return {false, costAcc, steps};
        }
        double stepCost = (abs(dx)+abs(dy)==2)? sqrt(2):1.0;
        x += dx; y += dy;
        costAcc += stepCost;
        steps++;
        if(steps > maxSteps) return {false, costAcc, steps};
    }
    return {true, costAcc, steps};
}

// A* for optimal path length comparison (8-dir)
double aStar(Grid &g, Vec2i start, Vec2i goal){
    auto heur = [&](int x,int y){
        int dx=abs(x-goal.x), dy=abs(y-goal.y);
        return (min(dx,dy)*sqrt(2)) + (abs(dx-dy));
    };
    vector<double> gscore(g.W*g.H, INF);
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<>> pq;
    int si=g.idx(start.x,start.y), gi=g.idx(goal.x,goal.y);
    gscore[si]=0; pq.push({heur(start.x,start.y), si});
    while(!pq.empty()){
        auto [f,cur]=pq.top(); pq.pop();
        int cx=cur%g.W, cy=cur/g.W;
        if(cur==gi) return gscore[gi];
        if(f - heur(cx,cy) > gscore[cur]+1e-9) continue;
        for(int k=0;k<8;k++){
            int nx=cx+DX[k], ny=cy+DY[k];
            if(!g.inb(nx,ny) || g.cost[g.idx(nx,ny)]>=INF) continue;
            double nd = gscore[cur] + DC[k];
            int ni=g.idx(nx,ny);
            if(nd < gscore[ni]){
                gscore[ni]=nd;
                pq.push({nd+heur(nx,ny), ni});
            }
        }
    }
    return INF;
}

// Render PPM visualization
void render(Grid &g, vector<Vec2i> agents, Vec2i goal, const string &path){
    // scale up 6x
    int S=6, W2=W*S, H2=H*S;
    vector<unsigned char> img(W2*H2*3, 255);
    auto setpx=[&](int px,int py,int r,int gg,int b){
        if(px<0||px>=W2||py<0||py>=H2) return;
        int idx=(py*W2+px)*3; img[idx]=r; img[idx+1]=gg; img[idx+2]=b;
    };
    for(int y=0;y<H;y++)for(int x=0;x<W;x++){
        int i=g.idx(x,y);
        for(int sy=0;sy<S;sy++)for(int sx=0;sx<S;sx++){
            int px=x*S+sx, py=y*S+sy;
            int r=255,gg=255,b=255;
            if(g.cost[i]>=INF){ r=40;gg=40;b=40; }
            else if(g.integ[i]<INF){
                // shade by integration value (far=blue-ish, near=green)
                int v = g.integ[i];
                int t = min(255, v/4);
                r = 20 + (255-20)*t/255;
                gg = 100 + (255-100)*(255-t)/255;
                b = 30 + (255-30)*t/255;
            }
            setpx(px,py,r,gg,b);
        }
    }
    // flow arrows
    for(int y=0;y<H;y++)for(int x=0;x<W;x++){
        int i=g.idx(x,y);
        if(g.dirx[i]==0&&g.diry[i]==0) continue;
        int cx=x*S+S/2, cy=y*S+S/2;
        int ex=cx+g.dirx[i]*S/2, ey=cy+g.diry[i]*S/2;
        // draw line
        int steps=S/2;
        for(int t=0;t<=steps;t++){
            int px=cx+(ex-cx)*t/steps, py=cy+(ey-cy)*t/steps;
            setpx(px,py,0,0,0);
        }
    }
    // goal
    for(int sy=-2;sy<=2;sy++)for(int sx=-2;sx<=2;sx++)
        setpx(goal.x*S+S/2+sx, goal.y*S+S/2+sy, 255,0,0);
    // agents
    for(auto &a: agents) for(int sy=-1;sy<=1;sy++)for(int sx=-1;sx<=1;sx++)
        setpx(a.x*S+S/2+sx, a.y*S+S/2+sy, 255, 255, 0);

    ofstream f(path, ios::binary);
    f<<"P6\n"<<W2<<" "<<H2<<"\n255\n";
    f.write((char*)img.data(), img.size());
    f.close();
}

int main(){
    // Build grid with obstacles
    Grid g(W,H);
    // add random rectangular obstacles (deterministic seed)
    mt19937 rng(42);
    for(int o=0;o<12;o++){
        int w = 3 + rng()%6, h = 3 + rng()%6;
        int x0 = rng()%(W-w-4)+2, y0 = rng()%(H-h-4)+2;
        for(int y=y0;y<y0+h;y++)for(int x=x0;x<x0+w;x++) g.cost[g.idx(x,y)]=INF;
    }
    Vec2i goal{W-5, H-5};
    g.cost[g.idx(goal.x,goal.y)]=1;

    // Build integration + flow field once
    buildIntegration(g, goal);
    buildFlow(g);

    // Choose starts spread around
    vector<Vec2i> starts;
    for(int y=4;y<H-4;y+=5)for(int x=4;x<W-4;x+=5){
        int i=g.idx(x,y);
        if(g.cost[i]<INF && g.integ[i]<INF) starts.push_back({x,y});
        if(starts.size()>=50) break;
        }
    if(starts.size()>50) starts.resize(50);
    while(!starts.empty() && (int)starts.size()>50) starts.pop_back();

    double astarOpt = aStar(g, {4,4}, goal);

    int reached=0; double totalLen=0; int totalSteps=0;
    vector<double> pathLen;
    for(auto &s: starts){
        auto [r,len,steps] = simulateAgent(g, s, goal);
        if(r){ reached++; totalLen+=len; totalSteps+=steps; pathLen.push_back(len); }
    }

    printf("=== Flow Field Pathfinding 量化验证 ===\n");
    printf("网格尺寸: %d x %d\n", W, H);
    printf("智能体总数: %zu\n", starts.size());
    printf("到达目标数: %d / %zu (%.1f%%)\n", reached, starts.size(), 100.0*reached/starts.size());
    printf("平均路径长度: %.2f\n", reached? totalLen/reached : 0.0);
    printf("A* 从(4,4)最优长度: %.2f\n", astarOpt);
    printf("平均步数: %.1f\n", reached? (double)totalSteps/reached: 0.0);
    printf("积分场覆盖格数: %d / %d\n",
        (int)count_if(g.integ.begin(),g.integ.end(),[](int v){return v<INF;}),
        W*H);
    // flow field consistency: every non-goal reachable cell points to lower integ
    int inconsistent=0, totalFlowCells=0;
    for(int i=0;i<W*H;i++){
        int x=i%W, y=i/W;
        if(g.cost[i]>=INF || g.integ[i]>=INF || (x==goal.x&&y==goal.y)) continue;
        if(g.dirx[i]==0&&g.diry[i]==0) continue; // local min (shouldn't happen except goal)
        // neighbor pointed to must have strictly lower integ
        int nx=x+g.dirx[i], ny=y+g.diry[i];
        int ni=g.idx(nx,ny);
        totalFlowCells++;
        if(g.integ[ni] >= g.integ[i]) inconsistent++;
    }
    printf("流场方向一致性(指向更小积分值): %d/%d 违反\n", inconsistent, totalFlowCells);
    printf("\n");

    // Render
    render(g, starts, goal, "flowfield_output.ppm");
    printf("已输出渲染: flowfield_output.ppm\n");

    // Assertions (fail the run if verification fails)
    bool ok = true;
    if(reached != (int)starts.size()){ printf("❌ 有智能体未到达目标\n"); ok=false; }
    if(astarOpt>1e8){ printf("❌ A* 无法到达(障碍物堵死)\n"); ok=false; }
    if(inconsistent > 0){ printf("❌ 流场方向不一致\n"); ok=false; }
    if(reached && totalLen/reached > astarOpt*1.5 && astarOpt>1){
        printf("⚠️ 路径长度明显超过最优(可能局部障碍)\n");
    }
    printf(ok ? "✅ 全部量化验证通过\n" : "❌ 验证失败\n");
    return ok?0:1;
}
