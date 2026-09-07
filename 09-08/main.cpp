// Manacher's Algorithm — Longest Palindromic Substring
// 量化验证: 与 O(n^2) 中心扩展基准对比正确性 + 加速比
#include <bits/stdc++.h>
using namespace std;

// O(n^2) 中心扩展求最长回文子串 (回文长度, 供正确性参考)
string bruteLPS(const string &s) {
    if (s.empty()) return "";
    int n = s.size();
    int bestL = 0, bestStart = 0;
    for (int c = 0; c < n; c++) {
        // 奇数长度
        for (int l = c, r = c; l >= 0 && r < n && s[l] == s[r]; l--, r++) {
            int len = r - l + 1;
            if (len > bestL) { bestL = len; bestStart = l; }
        }
        // 偶数长度
        for (int l = c, r = c + 1; l >= 0 && r < n && s[l] == s[r]; l--, r++) {
            int len = r - l + 1;
            if (len > bestL) { bestL = len; bestStart = l; }
        }
    }
    return s.substr(bestStart, bestL);
}

// Manacher O(n) 求最长回文子串
string manacherLPS(const string &s) {
    if (s.empty()) return "";
    // 插入分隔符, 统一奇偶
    string t = "#";
    for (char ch : s) { t.push_back(ch); t.push_back('#'); }
    int n = t.size();
    vector<int> p(n, 0);
    int center = 0, right = 0;
    int maxLen = 0, maxCenter = 0;
    for (int i = 0; i < n; i++) {
        int mirror = 2 * center - i;
        if (i < right) p[i] = min(right - i, p[mirror]);
        // 扩展
        while (i - p[i] - 1 >= 0 && i + p[i] + 1 < n &&
               t[i - p[i] - 1] == t[i + p[i] + 1]) {
            p[i]++;
        }
        if (i + p[i] > right) { center = i; right = i + p[i]; }
        if (p[i] > maxLen) { maxLen = p[i]; maxCenter = i; }
    }
    int start = (maxCenter - maxLen) / 2; // 映射回原串
    return s.substr(start, maxLen);
}

// 生成随机字符串
string randomString(int len, mt19937 &rng, int alphabet) {
    string s;
    uniform_int_distribution<int> dist(0, alphabet - 1);
    for (int i = 0; i < len; i++) s.push_back('a' + dist(rng));
    return s;
}

// 生成含长回文的字符串 (增大回文长度, 更考验正确性)
string randomWithPalindromes(int len, mt19937 &rng) {
    string s(len, 'a');
    uniform_int_distribution<int> cdist(0, 3); // 小字母表 -> 更多回文
    for (int i = 0; i < len; i++) s[i] = 'a' + cdist(rng);
    return s;
}

int main() {
    mt19937 rng(12345);

    cout << "=== Manacher 最长回文子串 — 量化验证 ===" << endl;

    // 1. 正确性验证: 与 O(n^2) 中心扩展对比, 多种字符串
    int correct = 0, total = 0;
    vector<string> testCases = {
        "babad", "cbbd", "a", "aa", "", "abcba", "abacdfgdcaba",
        "aaaa", "racecar", "abccba", "forgeeksskeegfor", "abcddcba",
    };
    for (const string &s : testCases) {
        string b = bruteLPS(s);
        string m = manacherLPS(s);
        total++;
        bool ok = (b.size() == m.size());
        correct += ok;
        cout << "  case \"" << (s.empty() ? "(empty)" : s) << "\" -> brute "
             << b.size() << " / manacher " << m.size()
             << (ok ? "  ✅" : "  ❌ MISMATCH") << endl;
    }

    // 随机字符串正确性 (小字母表, 高回文密度)
    for (int len : {1, 2, 5, 17, 50, 100}) {
        for (int t = 0; t < 20; t++) {
            string s = randomWithPalindromes(len, rng);
            total++;
            if (bruteLPS(s).size() == manacherLPS(s).size()) correct++;
            else {
                cout << "  ❌ 随机失配: len=" << len << " s=" << s
                     << " brute=" << bruteLPS(s).size()
                     << " manacher=" << manacherLPS(s).size() << endl;
            }
        }
    }
    cout << "正确性: " << correct << "/" << total << " 通过" << endl;
    if (correct != total) { cout << "❌ 存在失配" << endl; return 1; }

    // 2. 性能对比: 加速比 (Manacher O(n) vs 中心扩展 O(n^2))
    cout << "\n=== 性能基准 (求最长回文子串长度) ===" << endl;
    struct Row { int n; double bruteMs; double manMs; double speedup; };
    vector<Row> rows;
    vector<int> sizes = {100, 500, 1000, 2000, 5000, 10000, 20000};

    for (int n : sizes) {
        // 性能测试用 alphabet=1 (全同字符): 中心扩展每次扩展到底 -> 真正 O(n^2)
        // 这才是 Manacher 相对 O(n^2) 的最坏情况基准
        string s(n, 'a');
        // 末尾混入少量不同字符, 保持回文结构有趣且非平凡
        for (int i = n / 2; i < n; i += 97) s[i] = 'b';

        auto t0 = chrono::high_resolution_clock::now();
        volatile size_t bl = bruteLPS(s).size();
        auto t1 = chrono::high_resolution_clock::now();
        volatile size_t ml = manacherLPS(s).size();
        auto t2 = chrono::high_resolution_clock::now();
        (void)bl; (void)ml;

        double bruteMs = chrono::duration<double, milli>(t1 - t0).count();
        double manMs = chrono::duration<double, milli>(t2 - t1).count();
        double speedup = bruteMs / max(manMs, 1e-6);
        rows.push_back({n, bruteMs, manMs, speedup});
        printf("  n=%6d  brute=%8.2fms  manacher=%7.3fms  speedup=%7.1fx\n",
               n, bruteMs, manMs, speedup);
    }

    // 断言: 大输入时加速比显著 (Manacher 必须远快)
    double lastSpeedup = rows.back().speedup;
    cout << "\n最大输入 (n=" << sizes.back() << ") 加速比: "
         << lastSpeedup << "x" << endl;
    if (lastSpeedup < 10.0) {
        cout << "❌ 加速比不足 10x, Manacher 优化未体现" << endl;
        return 1;
    }
    cout << "✅ 加速比验证通过 (Manacher O(n) 显著优于 O(n^2))" << endl;

    // 3. 大回文场景验证 (构造一个已知长回文, 验证能找到)
    string base = randomString(500, rng, 2);
    string rev = base; reverse(rev.begin(), rev.end());
    string big = base + rev; // 长度 1000 的完整回文
    string m = manacherLPS(big);
    size_t expected = big.size();
    cout << "\n构造回文长度=" << expected << ", Manacher 找到=" << m.size()
         << (m.size() == expected ? "  ✅" : "  ❌") << endl;
    if (m.size() != expected) return 1;

    cout << "\n=== 全部验证通过 ✅ ===" << endl;
    return 0;
}
