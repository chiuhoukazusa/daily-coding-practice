#include <bits/stdc++.h>
using namespace std;

static int naiveDP(int k, int n) {
    vector<vector<int>> dp(k+1, vector<int>(n+1, 0));
    for (int f=1; f<=n; ++f) dp[1][f]=f;
    for (int e=2; e<=k; ++e)
        for (int f=1; f<=n; ++f){
            dp[e][f]=INT_MAX;
            for (int x=1; x<=f; ++x)
                dp[e][f]=min(dp[e][f], 1+max(dp[e-1][x-1], dp[e][f-x]));
        }
    return dp[k][n];
}
static int monotoneDP(int k, int n) {
    vector<vector<int>> dp(k+1, vector<int>(n+1,0));
    for (int f=1; f<=n; ++f) dp[1][f]=f;
    for (int e=2; e<=k; ++e){
        int x=1;
        for (int f=1; f<=n; ++f){
            while (x<f && dp[e-1][x-1] < dp[e][f-x]) ++x;
            dp[e][f]=1+dp[e-1][x-1];
        }
    }
    return dp[k][n];
}
static int logDP(int k, int n) {
    vector<vector<long long>> dp(n+1, vector<long long>(k+1,0));
    for (int m=1; m<=n; ++m){
        for (int e=1; e<=k; ++e) dp[m][e]=dp[m-1][e-1]+dp[m-1][e]+1;
        if (dp[m][k]>=n) return m;
    }
    return n;
}

int main(){
    // Correctness cross-check
    bool ok=true;
    for(int k=1;k<=20;++k) for(int n=0;n<=300;++n)
        if(naiveDP(k,n)!=monotoneDP(k,n) || monotoneDP(k,n)!=logDP(k,n)){ok=false;}
    cout<<"交叉正确性(20x300 全组合三算法一致): "<<(ok?"✅":"❌")<<"\n\n";

    // Timing benchmark (naive is O(n*k^2), limit its size)
    vector<pair<int,int>> cases = {{10,300},{20,300},{20,600},{50,600},{10,2000},{50,2000},{100,5000}};
    cout<<"算法性能基准 (毫秒):\n";
    cout<<setw(14)<<"case"<<setw(12)<<"naive O(nk2)"<<setw(14)<<"monotone O(nk)"<<setw(14)<<"binary O(klog n)"<<"\n";
    for(auto &c: cases){
        int k=c.first, n=c.second;
        double tn=0, tm=0, tb=0;
        if(n<=600){ // naive only for tractable sizes
            auto a=chrono::high_resolution_clock::now(); volatile int r=naiveDP(k,n); auto b=chrono::high_resolution_clock::now();
            tn=chrono::duration<double,milli>(b-a).count(); (void)r;
        }
        auto a=chrono::high_resolution_clock::now(); volatile int rm=monotoneDP(k,n); auto b=chrono::high_resolution_clock::now(); (void)rm;
        tm=chrono::duration<double,milli>(b-a).count();
        a=chrono::high_resolution_clock::now(); volatile int rb=logDP(k,n); b=chrono::high_resolution_clock::now(); (void)rb;
        tb=chrono::duration<double,milli>(b-a).count();
        cout<<setw(10)<<("k="+to_string(k)+",n="+to_string(n));
        if(n<=600) cout<<setw(12)<<tn; else cout<<setw(12)<<"(skip)";
        cout<<setw(14)<<tm<<setw(14)<<tb<<"\n";
    }
    return 0;
}
