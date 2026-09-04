// Levenshtein Edit Distance — Wagner-Fischer Dynamic Programming
// 每日编程实践 2026-09-05
// 技术点:
//   1. 完整 DP 矩阵 (O(m*n) 空间) 计算编辑距离
//   2. 回溯 (traceback) 得到一条最优对齐 (插入/删除/替换序列)
//   3. 滚动数组优化 O(min(m,n)) 空间
//   4. 量化验证: 参考值对比、距离上下界、自反性/对称性、三角形不等式、优化版==完整版一致性
#include <bits/stdc++.h>
using namespace std;

// ---------- 1. 完整 DP 矩阵 + 回溯 ----------
struct Alignment {
    int distance;
    // 对齐操作序列: 'M'=匹配/替换, 'I'=插入(对 b), 'D'=删除(对 a)
    string ops;
    // 对齐后的两行字符串
    string alignedA, alignedB;
};

Alignment wagner_fischer_full(const string& a, const string& b) {
    int n = a.size(), m = b.size();
    vector<vector<int>> dp(n + 1, vector<int>(m + 1, 0));
    // 初始化
    for (int i = 0; i <= n; i++) dp[i][0] = i; // 全部删除
    for (int j = 0; j <= m; j++) dp[0][j] = j; // 全部插入

    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= m; j++) {
            int cost = (a[i-1] == b[j-1]) ? 0 : 1;
            dp[i][j] = min({dp[i-1][j] + 1,      // 删除
                            dp[i][j-1] + 1,      // 插入
                            dp[i-1][j-1] + cost}); // 替换/匹配
        }
    }

    // 回溯
    Alignment res;
    res.distance = dp[n][m];
    int i = n, j = m;
    string ops, alignedA, alignedB;
    while (i > 0 || j > 0) {
        if (i > 0 && j > 0 && dp[i][j] == dp[i-1][j-1] + (a[i-1] == b[j-1] ? 0 : 1)) {
            // 对角: 匹配或替换
            alignedA.push_back(a[i-1]);
            alignedB.push_back(b[j-1]);
            ops.push_back(a[i-1] == b[j-1] ? 'M' : 'S');
            i--; j--;
        } else if (i > 0 && dp[i][j] == dp[i-1][j] + 1) {
            // 删除 a 的字符
            alignedA.push_back(a[i-1]);
            alignedB.push_back('-');
            ops.push_back('D');
            i--;
        } else {
            // 插入 b 的字符
            alignedA.push_back('-');
            alignedB.push_back(b[j-1]);
            ops.push_back('I');
            j--;
        }
    }
    reverse(ops.begin(), ops.end());
    reverse(alignedA.begin(), alignedA.end());
    reverse(alignedB.begin(), alignedB.end());
    res.ops = ops;
    res.alignedA = alignedA;
    res.alignedB = alignedB;
    return res;
}

// ---------- 2. 滚动数组优化 O(min(m,n)) 空间 ----------
int wagner_fischer_optimized(const string& a, const string& b) {
    // 保证 a 是较短的字符串, 空间 O(min)
    const string* s1 = &a;
    const string* s2 = &b;
    if (a.size() > b.size()) swap(s1, s2);
    int n = s1->size(), m = s2->size();

    vector<int> prev(m + 1), cur(m + 1);
    for (int j = 0; j <= m; j++) prev[j] = j;
    for (int i = 1; i <= n; i++) {
        cur[0] = i;
        for (int j = 1; j <= m; j++) {
            int cost = ((*s1)[i-1] == (*s2)[j-1]) ? 0 : 1;
            cur[j] = min({prev[j] + 1, cur[j-1] + 1, prev[j-1] + cost});
        }
        swap(prev, cur);
    }
    return prev[m];
}

// ---------- 3. 暴力递归 (仅用于极小字符串的交叉验证) ----------
// 带备忘录的自顶向下, 与矩阵版独立实现, 用于验证正确性
int memo_dp(const string& a, const string& b, int i, int j,
            vector<vector<int>>& memo) {
    if (i == (int)a.size()) return (int)b.size() - j;
    if (j == (int)b.size()) return (int)a.size() - i;
    if (memo[i][j] != -1) return memo[i][j];
    int cost = (a[i] == b[j]) ? 0 : 1;
    int r = min({memo_dp(a,b,i+1,j,memo)+1,
                 memo_dp(a,b,i,j+1,memo)+1,
                 memo_dp(a,b,i+1,j+1,memo)+cost});
    return memo[i][j] = r;
}

int edit_distance_memo(const string& a, const string& b) {
    vector<vector<int>> memo(a.size(), vector<int>(b.size(), -1));
    return memo_dp(a, b, 0, 0, memo);
}

int main() {
    vector<pair<string,string>> tests = {
        {"kitten", "sitting"},       // 经典: 3
        {"flaw", "lawn"},            // 经典: 2
        {"", ""},                    // 0
        {"abc", ""},                 // 3 (全部删除)
        {"", "xyz"},                 // 3 (全部插入)
        {"abc", "abc"},              // 0 (相同)
        {"sunday", "saturday"},      // 经典: 3
        {"intention", "execution"},  // 经典: 5
        {"algorithm", "altruistic"}, // 长串
        {"gumbo", "gambol"},         // 2
        {"a", "b"},                  // 1
        {"ab", "ba"},                // 2 (交换, 两次替换)
    };

    int failures = 0;
    int total = tests.size();
    cout << "===== Levenshtein Edit Distance 验证 =====\n";
    cout << "测试对数: " << total << "\n\n";

    for (auto& [a, b] : tests) {
        Alignment full = wagner_fischer_full(a, b);
        int opt = wagner_fischer_optimized(a, b);
        int memo = edit_distance_memo(a, b);

        bool ok = (full.distance == opt) && (opt == memo);
        cout << "[" << (ok ? "PASS" : "FAIL") << "] \"" << a << "\" -> \"" << b
             << "\"  dist=" << full.distance;
        if (!ok) {
            cout << "  (mismatch! full=" << full.distance
                 << " opt=" << opt << " memo=" << memo << ")";
        }
        cout << "\n";
        // 打印对齐 (仅小串)
        if (a.size() + b.size() <= 40) {
            cout << "      A: " << full.alignedA << "\n";
            cout << "      B: " << full.alignedB << "\n";
            cout << "    ops: " << full.ops << "\n";
        }
        if (!ok) failures++;
    }

    // ---- 经典参考值精确断言 ----
    cout << "\n===== 经典参考值断言 =====\n";
    struct Ref { string a, b; int expected; };
    vector<Ref> refs = {
        {"kitten","sitting",3}, {"flaw","lawn",2}, {"sunday","saturday",3},
        {"intention","execution",5}, {"gumbo","gambol",2}, {"","",0},
        {"abc","",3}, {"","xyz",3}, {"a","b",1}, {"abc","abc",0},
    };
    int ref_fail = 0;
    for (auto& r : refs) {
        int d = wagner_fischer_optimized(r.a, r.b);
        bool ok = (d == r.expected);
        if (!ok) ref_fail++;
        cout << "  [" << (ok?"PASS":"FAIL") << "] dist(\"" << r.a
             << "\",\"" << r.b << "\")=" << d << " (期望 " << r.expected << ")\n";
    }

    // ---- 数学性质验证 ----
    cout << "\n===== 数学性质验证 =====\n";
    // 1. 自反性: dist(x,x)==0
    int self_fail = 0;
    vector<string> words = {"hello","editing","distance","xyzzy","abc123",""};
    for (auto& w : words) {
        if (wagner_fischer_optimized(w, w) != 0) self_fail++;
    }
    cout << "  自反性 dist(x,x)==0: " << (self_fail==0?"PASS":"FAIL")
         << " (" << (words.size()-self_fail) << "/" << words.size() << ")\n";

    // 2. 对称性: dist(x,y)==dist(y,x)
    int sym_fail = 0;
    vector<pair<string,string>> sym_tests = {
        {"abc","abd"}, {"kitten","sitting"}, {"algorithm","logarithm"},
        {"","abcd"}, {"xyz","abc"}
    };
    for (auto& [x,y] : sym_tests) {
        if (wagner_fischer_optimized(x,y) != wagner_fischer_optimized(y,x)) sym_fail++;
    }
    cout << "  对称性 dist(x,y)==dist(y,x): " << (sym_fail==0?"PASS":"FAIL")
         << " (" << (sym_tests.size()-sym_fail) << "/" << sym_tests.size() << ")\n";

    // 3. 上下界: max(|x|-|y|) <= dist <= max(|x|,|y|)
    mt19937 rng(42);
    string alpha = "abcdefgh";
    int bound_fail = 0, bound_total = 200;
    for (int t = 0; t < bound_total; t++) {
        int L1 = rng() % 12, L2 = rng() % 12;
        string s1, s2;
        for (int i = 0; i < L1; i++) s1.push_back(alpha[rng()%alpha.size()]);
        for (int i = 0; i < L2; i++) s2.push_back(alpha[rng()%alpha.size()]);
        int d = wagner_fischer_optimized(s1, s2);
        int lower = abs((int)s1.size() - (int)s2.size());
        int upper = max((int)s1.size(), (int)s2.size());
        if (d < lower || d > upper) bound_fail++;
    }
    cout << "  上下界 max(|a|-|b|)<=dist<=max(|a|,|b|): "
         << (bound_fail==0?"PASS":"FAIL")
         << " (" << (bound_total-bound_fail) << "/" << bound_total << " 随机串)\n";

    // 4. 三角形不等式: dist(a,c) <= dist(a,b)+dist(b,c)  (随机 + b 由 a 变异得到, 紧致)
    int tri_fail = 0, tri_total = 200;
    for (int t = 0; t < tri_total; t++) {
        int L = rng() % 12;
        string a, b, c;
        for (int i = 0; i < L; i++) a.push_back(alpha[rng()%alpha.size()]);
        // b: 对 a 做随机编辑
        b = a;
        int edits = rng() % 5;
        for (int e = 0; e < edits; e++) {
            int op = rng() % 3, pos = rng() % max(1,(int)b.size());
            if (op == 0 && !b.empty()) b[pos] = alpha[rng()%alpha.size()]; // 替换
            else if (op == 1) b.insert(b.begin()+pos, alpha[rng()%alpha.size()]); // 插入
            else if (!b.empty()) b.erase(b.begin()+pos); // 删除
        }
        // c: 对 b 再做随机编辑
        c = b;
        edits = rng() % 5;
        for (int e = 0; e < edits; e++) {
            int op = rng() % 3, pos = rng() % max(1,(int)c.size());
            if (op == 0 && !c.empty()) c[pos] = alpha[rng()%alpha.size()];
            else if (op == 1) c.insert(c.begin()+pos, alpha[rng()%alpha.size()]);
            else if (!c.empty()) c.erase(c.begin()+pos);
        }
        int dab = wagner_fischer_optimized(a,b);
        int dbc = wagner_fischer_optimized(b,c);
        int dac = wagner_fischer_optimized(a,c);
        if (dac > dab + dbc) tri_fail++;
    }
    cout << "  三角形不等式 dist(a,c)<=dist(a,b)+dist(b,c): "
         << (tri_fail==0?"PASS":"FAIL")
         << " (" << (tri_total-tri_fail) << "/" << tri_total << " 随机三元组)\n";

    // ---- 性能对比 ----
    cout << "\n===== 性能对比 (滚动数组 vs 完整矩阵) =====\n";
    {
        int L = 2000;
        string big1(L,'a'), big2(L,'a');
        // 让它们有差异
        for (int i = 0; i < L; i+=100) { big1[i]='b'; big2[i]='c'; }

        auto t0 = chrono::high_resolution_clock::now();
        int d1 = wagner_fischer_optimized(big1, big2);
        auto t1 = chrono::high_resolution_clock::now();
        Alignment f = wagner_fischer_full(big1, big2);
        auto t2 = chrono::high_resolution_clock::now();

        double opt_ms = chrono::duration<double,milli>(t1-t0).count();
        double full_ms = chrono::duration<double,milli>(t2-t1).count();
        cout << "  字符串长度: " << L << " x " << L << "\n";
        cout << "  滚动数组: dist=" << d1 << " 用时 " << opt_ms << " ms\n";
        cout << "  完整矩阵: dist=" << f.distance << " 用时 " << full_ms << " ms\n";
        cout << "  一致性(opt==full): " << (d1==f.distance?"PASS":"FAIL") << "\n";
        cout << "  空间: 滚动数组 O(min)=" << L+1 << " 个 int vs 完整矩阵 O(n*m)="
             << (long long)(L+1)*(L+1) << " 个 int\n";
    }

    // ---- 汇总 ----
    cout << "\n===== 汇总 =====\n";
    cout << "  功能用例: " << (failures==0?"全部通过":"存在失败") << "\n";
    cout << "  经典参考值: " << (ref_fail==0?"全部通过":"存在失败") << "\n";
    bool all_ok = (failures==0 && ref_fail==0 && self_fail==0 && sym_fail==0
                   && bound_fail==0 && tri_fail==0);
    cout << (all_ok ? "✅ 全部量化验证通过" : "❌ 存在失败项") << "\n";
    return all_ok ? 0 : 1;
}
