// Z-Algorithm (Z-Function) — 线性时间字符串匹配
// 实现：
//  1. Z-Function：对字符串 S，Z[i] = S[0..] 与 S[i..] 的最长公共前缀长度
//  2. 线性时间模式匹配：构造 P + '#' + T，在 T 中找出所有 P 出现位置
//  3. 朴素 O(n*m) 基准实现用于正确性 + 加速比对比
// 量化验证：
//  - Z 函数与朴素 O(n) 逐字符对比（大量随机字符串 + 边界情况）
//  - 模式匹配位置与朴素实现完全一致
//  - 时间对比（朴素 vs Z 算法），验证线性复杂度的加速比
#include <bits/stdc++.h>
using namespace std;

// ---- Z-Function（线性时间） ----
vector<int> z_function(const string& s) {
    int n = (int)s.size();
    vector<int> z(n, 0);
    // [l, r) 是当前最靠右的 Z-box
    int l = 0, r = 0;
    for (int i = 1; i < n; i++) {
        if (i < r) {
            z[i] = min(r - i, z[i - l]);
        }
        while (i + z[i] < n && s[z[i]] == s[i + z[i]]) {
            z[i]++;
        }
        if (i + z[i] > r) {
            l = i;
            r = i + z[i];
        }
    }
    return z;
}

// ---- 朴素 Z 函数 O(n^2)（用于正确性基准） ----
// 约定：z[0] = 0（标准 Z 函数定义）
vector<int> z_naive(const string& s) {
    int n = (int)s.size();
    vector<int> z(n, 0);
    for (int i = 1; i < n; i++) {
        while (i + z[i] < n && s[z[i]] == s[i + z[i]]) z[i]++;
    }
    return z;
}

// ---- 朴素模式匹配 O(n*m) ----
vector<int> match_naive(const string& text, const string& pattern) {
    vector<int> positions;
    int n = text.size(), m = pattern.size();
    if (m == 0) return positions;
    for (int i = 0; i + m <= n; i++) {
        bool ok = true;
        for (int j = 0; j < m; j++) {
            if (text[i + j] != pattern[j]) { ok = false; break; }
        }
        if (ok) positions.push_back(i);
    }
    return positions;
}

// ---- 用 Z 算法做模式匹配（线性时间） ----
vector<int> match_z(const string& text, const string& pattern) {
    string combined = pattern + '#' + text;
    vector<int> z = z_function(combined);
    int m = pattern.size();
    int offset = m + 1;
    vector<int> positions;
    for (int i = 0; i < (int)text.size(); i++) {
        if (z[offset + i] >= m) positions.push_back(i);
    }
    return positions;
}

// ---- 随机字符串生成 ----
mt19937 rng(12345);
string random_string(int len, int alphabet) {
    string s;
    for (int i = 0; i < len; i++) {
        s.push_back(char('a' + rng() % alphabet));
    }
    return s;
}

int main() {
    // ---- 1. Z 函数正确性验证 ----
    int z_tests = 0, z_ok = 0;
    // 边界情况 + 随机情况
    vector<string> special = {
        "", "a", "aaaa", "ab", "abcab", "aabxaab", "abacabadabacaba"
    };
    for (const string& s : special) {
        auto z1 = z_function(s);
        auto z2 = z_naive(s);
        z_tests++;
        if (z1 == z2) z_ok++;
    }
    // 随机情况
    for (int t = 0; t < 200; t++) {
        int len = 1 + rng() % 200;
        int alpha = 1 + rng() % 4;   // 小字母表更易触发大量匹配，压测边界
        string s = random_string(len, alpha);
        auto z1 = z_function(s);
        auto z2 = z_naive(s);
        z_tests++;
        if (z1 == z2) z_ok++;
    }
    printf("== Z 函数正确性（线性 vs 朴素 O(n^2)） ==\n");
    printf("测试用例: %d, 通过: %d, 失败: %d\n", z_tests, z_ok, z_tests - z_ok);

    // ---- 2. 模式匹配正确性验证 ----
    int match_tests = 0, match_ok = 0;
    for (int t = 0; t < 300; t++) {
        int n = 1 + rng() % 500;
        int m = 1 + rng() % 50;
        int alpha = 1 + rng() % 3;   // 小字母表 -> 大量重叠匹配
        string text = random_string(n, alpha);
        string pattern = random_string(m, alpha);
        auto p1 = match_z(text, pattern);
        auto p2 = match_naive(text, pattern);
        match_tests++;
        if (p1 == p2) match_ok++;
        else {
            printf("FAIL: text len=%d pattern len=%d alpha=%d\n", n, m, alpha);
        }
    }
    printf("== 模式匹配位置正确性（Z vs 朴素 O(nm)） ==\n");
    printf("测试用例: %d, 通过: %d, 失败: %d\n", match_tests, match_ok, match_tests - match_ok);

    // ---- 3. 性能对比（加速比验证） ----
    // 构造大文本 + 重复模式，增大朴素算法的工作量
    printf("\n== 性能对比 ==\n");
    vector<pair<int,int>> configs = {
        {20000, 1000},     // 文本 2 万，模式 1 千   （朴素 ~2e7 次）
        {100000, 2000},    // 文本 10 万，模式 2 千   （朴素 ~2e8 次）
        {500000, 5000}     // 文本 50 万，模式 5 千   （朴素 ~2.5e9 次，最坏情况）
    };
    for (auto& cfg : configs) {
        int n = cfg.first, m = cfg.second;
        // 用全 'a' 制造最坏情况（朴素 O(n*m) 反复前缀匹配不回退）
        string pattern(m, 'a');
        string text(n, 'a');

        auto t0 = chrono::high_resolution_clock::now();
        auto rz = match_z(text, pattern);
        auto t1 = chrono::high_resolution_clock::now();
        auto rn = match_naive(text, pattern);
        auto t2 = chrono::high_resolution_clock::now();

        double time_z = chrono::duration<double, milli>(t1 - t0).count();
        double time_n = chrono::duration<double, milli>(t2 - t1).count();
        double speedup = time_n / time_z;

        printf("n=%d m=%d | Z: %.2f ms | naive: %.2f ms | 加速比 %.2fx | 匹配数一致: %s\n",
               n, m, time_z, time_n, speedup,
               (rz == rn ? "YES" : "NO"));
    }

    // ---- 4. 输出验证总结 ----
    printf("\n== 总结 ==\n");
    printf("Z函数正确性: %s\n", (z_ok == z_tests ? "PASS" : "FAIL"));
    printf("模式匹配正确性: %s\n", (match_ok == match_tests ? "PASS" : "FAIL"));
    return 0;
}
