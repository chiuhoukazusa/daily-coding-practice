// Boyer-Moore String Search (坏字符规则 + 好后缀规则)
// 对比朴素搜索(Naive)，验证正确性 + 量化加速比
#include <bits/stdc++.h>
using namespace std;

// ---------- 朴素搜索 ----------
vector<int> naive_search(const string& text, const string& pat) {
    vector<int> res;
    int n = text.size(), m = pat.size();
    if (m == 0) return res;
    for (int i = 0; i + m <= n; ++i) {
        bool ok = true;
        for (int j = 0; j < m; ++j) {
            if (text[i + j] != pat[j]) { ok = false; break; }
        }
        if (ok) res.push_back(i);
    }
    return res;
}

// ---------- Boyer-Moore (坏字符 + 好后缀) ----------
// 返回所有匹配位置；cmp 统计字符比较次数
vector<int> boyer_moore_search(const string& text, const string& pat, long long* cmp = nullptr) {
    vector<int> res;
    int n = text.size(), m = pat.size();
    if (m == 0) return res;

    // --- 坏字符表: bad_char[c] = 模式中字符 c 最靠右位置到末尾的距离 ---
    vector<int> bad_char(256, m);
    for (int i = 0; i < m; ++i) bad_char[(unsigned char)pat[i]] = m - 1 - i;

    // --- 好后缀表 (经典算法) ---
    // suffix[i] = 以 i 结尾的模式前缀 与 模式后缀 的最长匹配长度
    vector<int> suffix(m, 0);
    suffix[m - 1] = m;
    int g = m - 1, f = 0;
    for (int i = m - 2; i >= 0; --i) {
        if (i > g && suffix[i + m - 1 - f] < i - g) {
            suffix[i] = suffix[i + m - 1 - f];
        } else {
            if (i < g) g = i;
            f = i;
            while (g >= 0 && pat[g] == pat[g + m - 1 - f]) --g;
            suffix[i] = f - g;
        }
    }

    // good_suffix[j] = 当失配发生在位置 j (0-indexed, j 已匹配后缀长度为 m-1-j) 时的位移
    vector<int> good_suffix(m, m);
    // Case 2: 若后缀匹配某个子串，移动使其对齐
    for (int i = 0; i < m; ++i) {
        good_suffix[i] = m; // 默认整个模式长度
    }
    // 用 suffix 表填充：suffix[i] 处匹配，对齐到位置
    for (int i = m - 1; i >= 0; --i) {
        if (suffix[i] == i + 1) { // 前缀 == 后缀，Case 1
            for (int j = 0; j < m - 1 - i; ++j) {
                if (good_suffix[j] == m) good_suffix[j] = m - 1 - i;
            }
        }
    }
    // Case 3: 好后缀是模式的某个子串
    for (int i = 0; i <= m - 2; ++i) {
        good_suffix[m - 1 - suffix[i]] = m - 1 - i;
    }

    // --- 主搜索循环 ---
    int i = 0;
    while (i + m <= n) {
        int j = m - 1;
        while (j >= 0 && text[i + j] == pat[j]) {
            if (cmp) (*cmp)++;
            --j;
        }
        if (j < 0) {
            // 完整匹配
            res.push_back(i);
            if (cmp) (*cmp)++; // 最后一次比较（j 从 -1 出来，需补一次）
            i += (i + m < n) ? good_suffix[0] : 1;
        } else {
            if (cmp) (*cmp)++; // 失配的那次比较
            int bc_shift = bad_char[(unsigned char)text[i + j]] - (m - 1 - j);
            if (bc_shift <= 0) bc_shift = 1;
            int gs_shift = good_suffix[j];
            i += max(bc_shift, gs_shift);
        }
    }
    return res;
}

int main() {
    mt19937 rng(20260904);
    auto rand_int = [&](int lo, int hi) { return uniform_int_distribution<int>(lo, hi)(rng); };

    int total_tests = 0, failed = 0;

    // 大量随机正确性测试
    for (int r = 0; r < 50000; ++r) {
        int n = rand_int(50, 300);
        int mp = min(30, max(1, n / rand_int(3, 15)));
        string text(n, ' '), pat(mp, ' ');
        int alpha = rand_int(2, 26);
        for (char &c : text) c = 'a' + rand_int(0, alpha - 1);
        for (char &c : pat) c = 'a' + rand_int(0, alpha - 1);

        vector<int> nv = naive_search(text, pat);
        vector<int> bv = boyer_moore_search(text, pat);
        total_tests++;
        if (nv != bv) {
            failed++;
            if (failed <= 3) {
                cerr << "MISMATCH text=\"" << text << "\" pat=\"" << pat << "\"\n";
                cerr << "  naive=" << nv.size() << " bm=" << bv.size() << "\n";
            }
        }
    }

    // 性能基准：长文本 + 短模式 (BM 优势场景)
    long long naive_cmp = 0, bm_cmp = 0;
    double naive_time = 0, bm_time = 0;
    for (int r = 0; r < 200; ++r) {
        int n = 100000;
        int mp = rand_int(5, 20);
        string text(n, ' '), pat(mp, ' ');
        for (char &c : text) c = 'a' + rand_int(0, 25);
        for (char &c : pat) c = 'a' + rand_int(0, 25);

        long long nc = 0;
        auto t1 = chrono::high_resolution_clock::now();
        vector<int> nv = naive_search(text, pat);
        auto t2 = chrono::high_resolution_clock::now();
        // 朴素比较次数统计
        for (int i = 0; i + mp <= n; ++i)
            for (int j = 0; j < mp; ++j) { nc++; if (text[i+j] != pat[j]) break; }

        long long bc = 0;
        vector<int> bv = boyer_moore_search(text, pat, &bc);
        auto t3 = chrono::high_resolution_clock::now();

        if (nv != bv) { failed++; total_tests++; }

        naive_cmp += nc;
        bm_cmp += bc;
        naive_time += chrono::duration<double>(t2 - t1).count();
        bm_time += chrono::duration<double>(t3 - t2).count();
    }

    cout << "================ Boyer-Moore String Search 量化验证 ================\n";
    cout << "正确性测试: " << total_tests << " 个随机用例, 失败 " << failed << " 个\n";
    cout << (failed == 0 ? "✅ 正确性验证通过 (BM 与朴素搜索结果完全一致)" : "❌ 存在不匹配") << "\n\n";

    cout << "性能基准 (200 次, 文本长度 100000, 模式长度 5~20, 字母表26):\n";
    cout << "  朴素搜索总字符比较次数: " << naive_cmp << "\n";
    cout << "  BM     总字符比较次数: " << bm_cmp << "\n";
    double cmp_ratio = naive_cmp > 0 ? (double)naive_cmp / bm_cmp : 0;
    cout << "  比较次数加速比 (朴素/BM): " << fixed << setprecision(2) << cmp_ratio << "x\n";
    cout << "  朴素搜索总耗时: " << fixed << setprecision(4) << naive_time << "s\n";
    cout << "  BM     总耗时: " << fixed << setprecision(4) << bm_time << "s\n";
    double time_ratio = naive_time > 0 ? naive_time / bm_time : 0;
    cout << "  时间加速比 (朴素/BM): " << fixed << setprecision(2) << time_ratio << "x\n\n";

    // 边界情况测试
    struct Edge { string text, pat; int expect; };
    vector<Edge> edges = {
        {"aaaaa", "aa", 4},          // 重叠匹配
        {"abc", "d", 0},             // 无匹配
        {"", "a", 0},                // 空文本
        {"abc", "", 0},              // 空模式
        {"a", "a", 1},               // 单字符
        {"banana", "ana", 2},        // "banana" 中 "ana" 出现2次（位置1和3）
        {"banana", "an", 2},         // 重叠
        {"the quick brown fox", "quick", 1},
        {"aabaaab", "aaab", 1},      // 好后缀边界
        {"GCATCGCAGAGAGTATACAGTACG", "GCAGAGAG", 1}, // 经典 Boyer-Moore 论文案例
    };
    int edge_failed = 0;
    for (auto &e : edges) {
        int got = naive_search(e.text, e.pat).size();
        int bmgot = boyer_moore_search(e.text, e.pat).size();
        if (got != e.expect) { edge_failed++; cerr << "EDGE FAIL(naive): \"" << e.text << "\"/\"" << e.pat << "\" expect " << e.expect << " got " << got << "\n"; }
        if (bmgot != e.expect) { edge_failed++; cerr << "EDGE FAIL(bm): \"" << e.text << "\"/\"" << e.pat << "\" expect " << e.expect << " got " << bmgot << "\n"; }
    }
    cout << "边界用例: " << edges.size() << " 个, 失败 " << edge_failed << " 个\n";
    cout << (edge_failed == 0 ? "✅ 边界用例通过" : "❌ 边界用例失败") << "\n";

    cout << "\n================ 最终判定 ================\n";
    bool pass = (failed == 0 && edge_failed == 0);
    cout << (pass ? "✅ ALL PASS" : "❌ FAIL") << "\n";
    return pass ? 0 : 1;
}
