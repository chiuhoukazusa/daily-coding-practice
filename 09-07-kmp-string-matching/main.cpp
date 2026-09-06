// KMP String Matching (Knuth-Morris-Pratt)
// 每日编程实践 09-07
// 前缀函数 (prefix function / failure function) 实现 + 全程序量化验证

#include <bits/stdc++.h>
using namespace std;

// ---------- 1. 前缀函数 pi[i] = 最长真前缀 == 后缀 的长度 ----------
vector<int> prefix_function(const string& s) {
    int n = (int)s.size();
    vector<int> pi(n, 0);
    for (int i = 1; i < n; i++) {
        int j = pi[i - 1];
        while (j > 0 && s[i] != s[j]) j = pi[j - 1];
        if (s[i] == s[j]) j++;
        pi[i] = j;
    }
    return pi;
}

// ---------- 2. KMP 模式匹配：返回所有匹配起始下标 ----------
vector<int> kmp_search(const string& text, const string& pattern) {
    if (pattern.empty()) return {}; // 空模式：无匹配（约定）
    vector<int> pi = prefix_function(pattern);
    vector<int> matches;
    int j = 0;
    for (int i = 0; i < (int)text.size(); i++) {
        while (j > 0 && text[i] != pattern[j]) j = pi[j - 1];
        if (text[i] == pattern[j]) j++;
        if (j == (int)pattern.size()) {
            matches.push_back(i - j + 1);
            j = pi[j - 1]; // 继续找重叠匹配
        }
    }
    return matches;
}

// ---------- 3. 朴素搜索基准 ----------
vector<int> naive_search(const string& text, const string& pattern) {
    vector<int> matches;
    int n = (int)text.size(), m = (int)pattern.size();
    for (int i = 0; i + m <= n; i++) {
        bool ok = true;
        for (int k = 0; k < m; k++) {
            if (text[i + k] != pattern[k]) { ok = false; break; }
        }
        if (ok) matches.push_back(i);
    }
    return matches;
}

// ---------- 测试辅助 ----------
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s\n", msg); } \
} while (0)

// 随机字符串生成
string random_string(int len, int alphabet, mt19937& rng) {
    string s(len, 'a');
    uniform_int_distribution<int> d(0, alphabet - 1);
    for (int i = 0; i < len; i++) s[i] = char('a' + d(rng));
    return s;
}

int main() {
    printf("=== KMP String Matching — 量化验证 ===\n\n");

    // ---- Test 1: 前缀函数基础性质 ----
    printf("[Test 1] 前缀函数性质验证 (pi[0]=0, pi[i]<=i)\n");
    {
        vector<string> tests = {"a", "aaaa", "ababab", "abcabcd", "aabaaab"};
        for (auto& s : tests) {
            auto pi = prefix_function(s);
            for (int i = 0; i < (int)s.size(); i++) {
                CHECK(pi[i] >= 0 && pi[i] <= i, "pi[i] 越界");
            }
            CHECK(pi[0] == 0, "pi[0] != 0");
        }
        // 已知手算/标准值
        CHECK(prefix_function("aaaa")[3] == 3, "aaaa pi[3] 应为 3");
        auto pi_ababab = prefix_function("ababab");
        CHECK(pi_ababab[5] == 4, "ababab pi[5] 应为 4 (最长真前缀后缀 abab)");
        auto pi_abc = prefix_function("abcabcd");
        CHECK(pi_abc[6] == 0, "abcabcd pi[6] 应为 0");
        CHECK(pi_abc[3] == 1, "abcabcd pi[3] 应为 1");
        printf("  通过\n");
    }

    // ---- Test 2: 已知 case 正确性 ----
    printf("[Test 2] 手算已知匹配结果\n");
    {
        string t = "ababcabcabababd";
        string p = "ababd";
        auto r = kmp_search(t, p);
        CHECK(r.size() == 1 && r[0] == 10, "ababd 应在位置 10");
        p = "ababa";
        r = kmp_search(t, p);
        // "ababa" 在 t 中存在？ t=ababcabcabababd，检查
        // 手工：naive 为准
        auto nr = naive_search(t, p);
        CHECK(r == nr, "kmp 与 naive 结果不一致 (ababa)");
    }

    // ---- Test 3: 大规模随机正确性 (KMP vs naive) ----
    printf("[Test 3] 随机文本 KMP vs 朴素 正确性 (200 组)\n");
    {
        mt19937 rng(12345);
        int correct = 0, total = 0;
        for (int iter = 0; iter < 200; iter++) {
            int alphabet = 2 + (iter % 4); // 2~5 字母表(增加重复触发回退)
            int n = 50 + (int)(rng() % 300);
            int m = 1 + (int)(rng() % 12);
            string text = random_string(n, alphabet, rng);
            string pat  = random_string(m, alphabet, rng);
            auto a = kmp_search(text, pat);
            auto b = naive_search(text, pat);
            total += (int)b.size();
            if (a == b) correct++;
            else {
                printf("  MISMATCH text len=%d pat=%s\n", n, pat.c_str());
            }
        }
        CHECK(correct == 200, "随机正确性存在不一致");
        printf("  200 组全部一致 (总匹配数 %d)\n", total);
    }

    // ---- Test 4: 重叠匹配 ----
    printf("[Test 4] 重叠匹配正确性\n");
    {
        string t = "aaaaa";
        string p = "aaa";
        auto r = kmp_search(t, p);
        CHECK(r.size() == 3, "aaaaa 中 aaa 应有 3 个重叠匹配");
        CHECK(r[0] == 0 && r[1] == 1 && r[2] == 2, "重叠匹配位置应 0,1,2");
    }

    // ---- Test 5: 边界情况 ----
    printf("[Test 5] 边界情况\n");
    {
        CHECK(kmp_search("abc", "abcd").empty(), "模式长于文本");
        CHECK(kmp_search("", "").empty(), "空文本空模式");
        CHECK(kmp_search("xyz", "").empty(), "空模式");
        auto r = kmp_search("hello", "hello");
        CHECK(r.size() == 1 && r[0] == 0, "模式等于文本");
        CHECK(kmp_search("abc", "x").empty(), "不存在模式");
    }

    // ---- Test 6: 时间复杂度 / 性能基准 ----
    printf("[Test 6] 性能基准 (最坏情况: 全 a 文本 + 带 b 尾的模式)\n");
    {
        // 构造朴素算法最坏 case：text 全 'a'，pattern "aaaa...b"
        // 朴素 O(n*m)；KMP O(n+m)
        int n = 2000000;
        string text(n, 'a');
        int m = 1000;
        string pat(m - 1, 'a');
        pat += 'b';

        auto t0 = chrono::high_resolution_clock::now();
        auto rKmp = kmp_search(text, pat);
        auto t1 = chrono::high_resolution_clock::now();
        auto rNaive = naive_search(text, pat);
        auto t2 = chrono::high_resolution_clock::now();

        double kmpMs = chrono::duration<double, milli>(t1 - t0).count();
        double naiveMs = chrono::duration<double, milli>(t2 - t1).count();

        CHECK(rKmp == rNaive, "最坏 case 结果不一致");
        CHECK(rKmp.empty(), "最坏 case 应无匹配");

        printf("  KMP    时间: %.2f ms (n=%d, m=%d)\n", kmpMs, n, m);
        printf("  朴素   时间: %.2f ms\n", naiveMs);
        double speedup = naiveMs / max(kmpMs, 1e-6);
        printf("  加速比: %.1fx\n", speedup);
        CHECK(speedup > 50.0, "最坏 case 下 KMP 应显著快于朴素 (>50x)");
    }

    // ---- 汇总 ----
    printf("\n======================================\n");
    printf("  通过: %d  失败: %d\n", g_pass, g_fail);
    if (g_fail == 0) printf("  ✅ 全部量化验证通过\n");
    else printf("  ❌ 存在失败用例\n");
    printf("======================================\n");
    return g_fail == 0 ? 0 : 1;
}
