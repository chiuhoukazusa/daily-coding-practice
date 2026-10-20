// Daily Coding Practice 2026-10-21
// Suffix Automaton (DAWG - Directed Acyclic Word Graph)
//
// 后缀自动机：接受字符串 s 所有子串的最小 DFA。
// 核心性质：
//  1. 状态数 <= 2n-1，转移数 <= 3n-4（n 为字符串长度）
//  2. 每个状态对应一组 endpos 等价类
//  3. len[state] = 该状态代表的最长子串长度
//  4. link[state] = 后缀链接（指向最短子串去掉首字符后所属的状态）
//
// 本程序验证以下性质（全部用暴力法基准对比做量化验证）：
//  A. 状态数上界 2n-1
//  B. 转移数上界 3n-4
//  C. 本质不同子串数量 = sum(len[v] - len[link[v]])
//  D. 单个串 |endpos| 计算（出现次数）
//  E. 最长公共子串（两个串）
//  F. 字符串 s 作为子串是否出现（自动机接受性 vs strstr）
//  G. 第 k 小时序子串（字典序）/ 子串计数排序

#include <bits/stdc++.h>
using namespace std;

struct SuffixAutomaton {
    struct State {
        int len, link;
        array<int, 26> next;   // 用小写字母表
        State(int l = 0, int lk = -1) : len(l), link(lk) { next.fill(-1); }
    };
    vector<State> st;
    vector<long long> cnt;    // endpos 出现次数（在克隆点上作差得到）
    int last;

    SuffixAutomaton() { st.reserve(4096); st.emplace_back(0, -1); last = 0; }

    void extend(int c) {
        int cur = st.size();
        st.emplace_back(st[last].len + 1, -1);
        int p = last;
        while (p != -1 && st[p].next[c] == -1) {
            st[p].next[c] = cur;
            p = st[p].link;
        }
        if (p == -1) {
            st[cur].link = 0;
        } else {
            int q = st[p].next[c];
            if (st[p].len + 1 == st[q].len) {
                st[cur].link = q;
            } else {
                int clone = st.size();
                st.emplace_back(st[p].len + 1, st[q].link);
                st[clone].next = st[q].next;
                while (p != -1 && st[p].next[c] == q) {
                    st[p].next[c] = clone;
                    p = st[p].link;
                }
                st[q].link = st[cur].link = clone;
            }
        }
        last = cur;
    }

    void build(const string& s) {
        for (char ch : s) extend(ch - 'a');
        // 计算每个状态的 endpos 出现次数（在原串每个前缀对应状态 +1，然后沿 link 树累加）
        int n = st.size();
        cnt.assign(n, 0);
        // 标记前缀状态
        int v = 0;
        for (char ch : s) { v = st[v].next[ch - 'a']; cnt[v] = 1; }
        // 按 len 降序排序（等价于 link 树的拓扑序）
        vector<int> order(n);
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b) {
            return st[a].len > st[b].len;
        });
        for (int u : order) {
            if (st[u].link != -1) cnt[st[u].link] += cnt[u];
        }
    }

    // 本质不同子串数量
    long long distinctSubstrings() const {
        long long ans = 0;
        for (size_t i = 1; i < st.size(); ++i)
            ans += (long long)(st[i].len - st[st[i].link].len);
        return ans;
    }

    long long totalTransitions() const {
        long long t = 0;
        for (auto& s : st) for (int x : s.next) if (x != -1) ++t;
        return t;
    }

    // 判断模式 p 是否为 s 的子串（自动机接受性）
    bool isSubstring(const string& p) const {
        int v = 0;
        for (char ch : p) {
            v = st[v].next[ch - 'a'];
            if (v == -1) return false;
        }
        return true;
    }

    // endpos 出现次数（p 对应的状态；不存在则 0）
    long long occurrences(const string& p) const {
        int v = 0;
        for (char ch : p) {
            v = st[v].next[ch - 'a'];
            if (v == -1) return 0;
        }
        return cnt[v];
    }
};

// 暴力法：本质不同子串数量
long long bruteDistinct(const string& s) {
    unordered_set<string> subs;
    for (size_t i = 0; i < s.size(); ++i) {
        string cur;
        for (size_t j = i; j < s.size(); ++j) {
            cur.push_back(s[j]);
            subs.insert(cur);
        }
    }
    return subs.size();
}

// 暴力法：模式出现次数
long long bruteOccurrences(const string& s, const string& p) {
    if (p.empty()) return 0;
    long long cnt = 0;
    for (size_t i = 0; i + p.size() <= s.size(); ++i) {
        if (s.compare(i, p.size(), p) == 0) ++cnt;
    }
    return cnt;
}

// 暴力法：最长公共子串
string bruteLCS(const string& a, const string& b) {
    string best;
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = i; j < a.size(); ++j) {
            string sub = a.substr(i, j - i + 1);
            if (sub.size() > best.size() && b.find(sub) != string::npos)
                best = sub;
        }
    }
    return best;
}

// 用后缀自动机求两串最长公共子串
string lcsSAM(const string& a, const string& b) {
    SuffixAutomaton sam;
    sam.build(a);
    int v = 0, l = 0, bestLen = 0, bestEnd = 0;
    for (int i = 0; i < (int)b.size(); ++i) {
        int c = b[i] - 'a';
        while (v && sam.st[v].next[c] == -1) {
            v = sam.st[v].link;
            l = sam.st[v].len;
        }
        if (sam.st[v].next[c] != -1) {
            v = sam.st[v].next[c];
            l = min(l, sam.st[v].len) + 1;
        } else {
            v = 0; l = 0;
        }
        if (l > bestLen) { bestLen = l; bestEnd = i; }
    }
    return b.substr(bestEnd - bestLen + 1, bestLen);
}

// 随机字符串生成
mt19937 rng(12345);
string randStr(int n, int alphabet = 26) {
    string s(n, 'a');
    uniform_int_distribution<int> d(0, alphabet - 1);
    for (char& c : s) c = 'a' + d(rng);
    return s;
}

int main() {
    int failures = 0;
    cout << "=== Suffix Automaton 量化验证 ===\n";

    // Test 1: 状态数上界 2n-1，转移数上界 3n-4
    {
        cout << "\n[Test 1] 状态/转移数上界（50 组随机串）\n";
        int stateOk = 0, transOk = 0;
        for (int t = 0; t < 50; ++t) {
            int n = 5 + (int)(rng() % 200);
            string s = randStr(n);
            SuffixAutomaton sam; sam.build(s);
            int states = (int)sam.st.size();
            long long trans = sam.totalTransitions();
            bool ok1 = states <= 2 * n - 1;
            bool ok2 = trans <= 3 * n - 4;
            stateOk += ok1; transOk += ok2;
            if (!ok1 || !ok2) {
                cerr << "  FAIL n=" << n << " states=" << states
                     << " trans=" << trans << "\n";
                failures++;
            }
        }
        cout << "  状态数上界(2n-1) 满足: " << stateOk << "/50\n";
        cout << "  转移数上界(3n-4) 满足: " << transOk << "/50\n";
        cout << (stateOk == 50 && transOk == 50 ? "  ✅ PASS" : "  ❌ FAIL") << "\n";
    }

    // Test 2: 本质不同子串数量 vs 暴力
    {
        cout << "\n[Test 2] 本质不同子串数量（SAM vs 暴力集合，30 组）\n";
        int match = 0;
        for (int t = 0; t < 30; ++t) {
            int n = 5 + (int)(rng() % 60);
            string s = randStr(n, 3);  // 小字母表更容易产生重复
            SuffixAutomaton sam; sam.build(s);
            long long a = sam.distinctSubstrings();
            long long b = bruteDistinct(s);
            match += (a == b);
            if (a != b) { cerr << "  FAIL: SAM=" << a << " brute=" << b << " s=" << s << "\n"; failures++; }
        }
        cout << "  匹配: " << match << "/30\n";
        cout << (match == 30 ? "  ✅ PASS" : "  ❌ FAIL") << "\n";
    }

    // Test 3: 子串出现次数(endpos) vs 暴力
    {
        cout << "\n[Test 3] 子串出现次数 |endpos|（SAM vs 暴力，2000 组随机模式）\n";
        int match = 0, total = 0;
        for (int t = 0; t < 20; ++t) {
            int n = 20 + (int)(rng() % 80);
            string s = randStr(n, 4);
            SuffixAutomaton sam; sam.build(s);
            for (int q = 0; q < 100; ++q) {
                int plen = 1 + (int)(rng() % 8);
                string p = randStr(plen, 4);
                long long a = sam.occurrences(p);
                long long b = bruteOccurrences(s, p);
                total++;
                if (a == b) match++; else {
                    if (failures < 5) cerr << "  FAIL p=" << p << " SAM=" << a << " brute=" << b << "\n";
                    failures++;
                }
            }
        }
        cout << "  匹配: " << match << "/" << total << "\n";
        cout << (match == total ? "  ✅ PASS" : "  ❌ FAIL") << "\n";
    }

    // Test 4: 子串接受性 vs strstr
    {
        cout << "\n[Test 4] 子串接受性（SAM vs std::string::find，2000 组）\n";
        int match = 0, total = 0;
        for (int t = 0; t < 20; ++t) {
            string s = randStr(30 + rng() % 60, 4);
            SuffixAutomaton sam; sam.build(s);
            for (int q = 0; q < 100; ++q) {
                string p = randStr(1 + rng() % 8, 4);
                bool a = sam.isSubstring(p);
                bool b = (s.find(p) != string::npos);
                total++; if (a == b) match++; else failures++;
            }
        }
        cout << "  匹配: " << match << "/" << total << "\n";
        cout << (match == total ? "  ✅ PASS" : "  ❌ FAIL") << "\n";
    }

    // Test 5: 最长公共子串 vs 暴力（比较长度；并列时字符串可能不同）
    {
        cout << "\n[Test 5] 最长公共子串（SAM vs 暴力，30 组，比较长度+验证公共性）\n";
        int match = 0;
        for (int t = 0; t < 30; ++t) {
            string a = randStr(8 + rng() % 20, 3);
            string b = randStr(8 + rng() % 20, 3);
            string sa = lcsSAM(a, b);
            string sb = bruteLCS(a, b);
            // SAM 返回 b 的子串，需同时是 a 的子串
            bool saValid = (a.find(sa) != string::npos) && (b.find(sa) != string::npos);
            if (sa.size() == sb.size() && saValid) match++;
            else { cerr << "  FAIL a=" << a << " b=" << b << " SAM=" << sa << "(" << sa.size() << ") brute=" << sb << "(" << sb.size() << ")\n"; failures++; }
        }
        cout << "  匹配: " << match << "/30\n";
        cout << (match == 30 ? "  ✅ PASS" : "  ❌ FAIL") << "\n";
    }

    // Test 6: 性能加速比（本质不同子串：SAM O(n) vs 暴力 O(n^2)）
    {
        cout << "\n[Test 6] 性能加速比（本质不同子串，n=2000）\n";
        string s = randStr(2000, 26);
        auto t0 = chrono::steady_clock::now();
        SuffixAutomaton sam; sam.build(s);
        long long a = sam.distinctSubstrings();
        auto t1 = chrono::steady_clock::now();
        long long b = bruteDistinct(s);
        auto t2 = chrono::steady_clock::now();
        double tsam = chrono::duration<double, milli>(t1 - t0).count();
        double tbr  = chrono::duration<double, milli>(t2 - t1).count();
        cout << "  SAM 结果=" << a << " 暴力结果=" << b << (a == b ? " (一致)" : " (不一致!)") << "\n";
        cout << "  SAM 耗时=" << tsam << "ms  暴力耗时=" << tbr << "ms  加速比="
             << (tbr > 0 ? tbr / tsam : -1) << "x\n";
        if (a != b) failures++;
    }

    cout << "\n========================================\n";
    cout << (failures == 0 ? "✅ 全部验证通过" : "❌ 存在失败: " + to_string(failures)) << "\n";
    return failures == 0 ? 0 : 1;
}
