#include <bits/stdc++.h>
using namespace std;
// Demonstrate space: naive table O(n*m) vs Hirschberg O(min(n,m))
static vector<int> hr(const string&a,const string&b,int n,int m){
    vector<int> p(m+1),c(m+1);
    for(int i=1;i<=n;++i){for(int j=1;j<=m;++j)c[j]=(a[i-1]==b[j-1])?p[j-1]+1:max(p[j],c[j-1]);swap(p,c);}
    return p;
}
static void rec(const string&a,int al,int ah,const string&b,int bl,int bh,string&o){
    int n=ah-al,m=bh-bl;
    if(n==0)return;
    if(n==1){for(int j=bl;j<bh;++j)if(b[j]==a[al]){o+=a[al];return;}return;}
    int am=al+n/2;
    auto f=hr(a.substr(al,am-al),b.substr(bl,bh-bl),am-al,bh-bl);
    string ar=a.substr(am,ah-am),br=b.substr(bl,bh-bl);reverse(ar.begin(),ar.end());reverse(br.begin(),br.end());
    auto bw=hr(ar,br,ah-am,bh-bl);
    int best=-1,bk=0;for(int k=0;k<=m;++k){int v=f[k]+bw[m-k];if(v>best){best=v;bk=k;}}
    rec(a,al,am,b,bl,bl+bk,o);rec(a,am,ah,b,bl+bk,bh,o);
}
int main(){
    int n=20000,m=20000;
    string a(n,'a'),b(m,'a');
    // memory estimate for naive full table: n*m ints * 4 bytes
    double naive_bytes = (double)(n+1)*(m+1)*4;
    // Hirschberg peak is a few vectors of size m
    // actual process RSS measured via /proc/self/status after running Hirschberg
    string o; rec(a,0,n,b,0,m,o);
    ifstream st("/proc/self/status"); string line; long rss=0;
    while(getline(st,line)){ if(line.rfind("VmRSS:",0)==0){ sscanf(line.c_str(),"VmRSS: %ld kB",&rss);} }
    cout << "input size n=m="<<n<<endl;
    cout << "naive full table would need ~" << naive_bytes/ (1024.0*1024.0) << " MB (int table)"<<endl;
    cout << "Hirschberg actual process RSS ~" << rss/1024.0 << " MB"<<endl;
    cout << "Hirschberg LCS length (all identical) = " << o.size() << " (expect "<<n<<")"<<endl;
    return 0;
}
