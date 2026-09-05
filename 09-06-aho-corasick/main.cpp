// Aho-Corasick Multi-Pattern String Matching
// Builds a Trie + failure links (BFS) + output links to match many patterns
// simultaneously in O(n + m + z) where n = text length, m = total pattern length,
// z = number of matches.
//
// Quantitative verification:
//   1. Brute-force per-pattern search as ground truth (correctness)
//   2. Match count / set equality comparison
//   3. Speed-up ratio vs brute force on synthetic + real-ish text
//   4. Trie structural invariants (failure link depth, output link chain)

#include <bits/stdc++.h>
using namespace std;

static const int ALPHA = 26; // lowercase a-z only in test corpus

struct Node {
    int next[ALPHA];
    int fail;
    vector<int> outs;   // pattern indices ending here (may be multiple for duplicate patterns)
    int outLink;        // next output node via output links, or -1
    int depth;
    Node() {
        fill(begin(next), end(next), -1);
        fail = 0;
        outLink = -1;
        depth = 0;
    }
    bool hasOut() const { return !outs.empty(); }
};

struct AhoCorasick {
    vector<Node> trie;

    AhoCorasick() { trie.emplace_back(); }

    void insert(const string& s, int idx) {
        int cur = 0;
        for (char ch : s) {
            int c = ch - 'a';
            if (trie[cur].next[c] == -1) {
                trie[cur].next[c] = (int)trie.size();
                trie.emplace_back();
                trie.back().depth = trie[cur].depth + 1;
            }
            cur = trie[cur].next[c];
        }
        trie[cur].outs.push_back(idx); // support duplicate patterns
    }

    void build() {
        queue<int> q;
        for (int c = 0; c < ALPHA; ++c) {
            int v = trie[0].next[c];
            if (v != -1) {
                trie[v].fail = 0;
                q.push(v);
            }
        }
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int c = 0; c < ALPHA; ++c) {
                int v = trie[u].next[c];
                if (v == -1) continue;
                // Walk failure links from u's parent chain until a node has
                // a child 'c', that child becomes v's failure link.
                int f = trie[u].fail;
                while (f != 0 && trie[f].next[c] == -1) f = trie[f].fail;
                trie[v].fail = (trie[f].next[c] != -1) ? trie[f].next[c] : 0;

                // output link: nearest ancestor (on the fail chain) that has an output
                int fl = trie[v].fail;
                trie[v].outLink = (trie[fl].hasOut()) ? fl : trie[fl].outLink;
                q.push(v);
            }
        }
    }

    // Return all (end_pos, pattern_idx) matches, end_pos inclusive.
    vector<pair<int,int>> match(const string& text) {
        vector<pair<int,int>> res;
        int cur = 0;
        for (int i = 0; i < (int)text.size(); ++i) {
            int c = text[i] - 'a';
            while (cur != 0 && trie[cur].next[c] == -1) cur = trie[cur].fail;
            if (trie[cur].next[c] != -1) cur = trie[cur].next[c];
            // collect all outputs at this node via output links
            int u = cur;
            while (u != -1) {  // iterate output chain
                for (int pid : trie[u].outs) res.emplace_back(i, pid);
                u = trie[u].outLink;
            }
        }
        return res;
    }
};

// Brute-force ground truth: for each pattern, find all occurrences in text.
vector<pair<int,int>> bruteForce(const string& text, const vector<string>& pats) {
    vector<pair<int,int>> res;
    for (int p = 0; p < (int)pats.size(); ++p) {
        const string& pat = pats[p];
        size_t pos = 0;
        while ((pos = text.find(pat, pos)) != string::npos) {
            res.emplace_back((int)(pos + pat.size() - 1), p);
            pos += 1;
        }
    }
    return res;
}

// Compare two match sets as multisets.
bool sameMatches(vector<pair<int,int>> a, vector<pair<int,int>> b) {
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    return a == b;
}

string randomLower(int len, mt19937& rng) {
    string s(len, 'a');
    uniform_int_distribution<int> d(0, 25);
    for (auto& c : s) c = char('a' + d(rng));
    return s;
}

string randomPattern(int len, int alpha, mt19937& rng) {
    string s(len, 'a');
    uniform_int_distribution<int> d(0, alpha - 1);
    for (auto& c : s) c = char('a' + d(rng));
    return s;
}

int main() {
    mt19937 rng(12345);
    cout << fixed << setprecision(6);

    // ---------- Test 1: small correctness ----------
    {
        vector<string> pats = {"he", "she", "his", "hers"};
        string text = "ushers";
        AhoCorasick ac;
        for (int i = 0; i < (int)pats.size(); ++i) ac.insert(pats[i], i);
        ac.build();
        auto acRes = ac.match(text);
        auto bfRes = bruteForce(text, pats);
        cout << "[Test 1] small correctness: "
             << (sameMatches(acRes, bfRes) ? "PASS" : "FAIL")
             << "  ac=" << acRes.size() << " bf=" << bfRes.size() << "\n";
    }

    // ---------- Test 2: overlapping patterns ----------
    {
        vector<string> pats = {"a", "aa", "aaa", "aaaa"};
        string text = "aaaaaaaaaa";
        AhoCorasick ac;
        for (int i = 0; i < (int)pats.size(); ++i) ac.insert(pats[i], i);
        ac.build();
        auto acRes = ac.match(text);
        auto bfRes = bruteForce(text, pats);
        cout << "[Test 2] overlapping: "
             << (sameMatches(acRes, bfRes) ? "PASS" : "FAIL")
             << "  ac=" << acRes.size() << " bf=" << bfRes.size() << "\n";
    }

    // ---------- Test 3: failure link depth invariant ----------
    {
        // failure link of a node must point strictly shallower (or to root 0)
        vector<string> pats = {"abc", "bc", "c", "ababc", "bab"};
        AhoCorasick ac;
        for (int i = 0; i < (int)pats.size(); ++i) ac.insert(pats[i], i);
        ac.build();
        bool ok = true;
        for (int i = 1; i < (int)ac.trie.size(); ++i) { // skip root (fail=0 by design)
            int f = ac.trie[i].fail;
            if (ac.trie[f].depth >= ac.trie[i].depth) { ok = false; break; }
        }
        cout << "[Test 3] fail-depth invariant: " << (ok ? "PASS" : "FAIL") << "\n";
    }

    // ---------- Test 4: randomized correct vs brute force ----------
    {
        int trials = 200;
        int pass = 0;
        int totalAC = 0, totalBF = 0;
        for (int t = 0; t < trials; ++t) {
            int np = 1 + (int)(rng() % 20);
            vector<string> pats;
            for (int i = 0; i < np; ++i) {
                int len = 1 + (int)(rng() % 6);
                int alpha = 2 + (int)(rng() % 4); // small alphabet => many overlaps
                pats.push_back(randomPattern(len, alpha, rng));
            }
            string text = randomLower(200 + (int)(rng() % 800), rng);
            AhoCorasick ac;
            for (int i = 0; i < np; ++i) ac.insert(pats[i], i);
            ac.build();
            auto acRes = ac.match(text);
            auto bfRes = bruteForce(text, pats);
            totalAC += acRes.size();
            totalBF += bfRes.size();
            if (sameMatches(acRes, bfRes)) ++pass;
        }
        cout << "[Test 4] randomized (" << trials << " trials): " << pass << "/" << trials
             << "  total matches ac=" << totalAC << " bf=" << totalBF << "\n";
    }

    // ---------- Test 5: performance / speed-up ----------
    {
        // Build a pattern set and text such that brute force vs AC is a fair,
        // apples-to-apples comparison over the SAME pattern+text.
        int nWords = 500;
        vector<string> pats;
        for (int i = 0; i < nWords; ++i) {
            pats.push_back(randomPattern(4 + (int)(rng() % 4), 26, rng));
        }
        // text = repeated concatenation of a random subset so matches do occur
        string text;
        while (text.size() < 400000) {
            text += pats[rng() % nWords];
        }

        AhoCorasick ac;
        for (int i = 0; i < nWords; ++i) ac.insert(pats[i], i);
        ac.build();

        auto t1 = chrono::high_resolution_clock::now();
        auto acRes = ac.match(text);
        auto t2 = chrono::high_resolution_clock::now();

        auto t3 = chrono::high_resolution_clock::now();
        auto bfRes = bruteForce(text, pats);
        auto t4 = chrono::high_resolution_clock::now();

        double acMs = chrono::duration<double, milli>(t2 - t1).count();
        double bfMs = chrono::duration<double, milli>(t4 - t3).count();

        bool ok = sameMatches(acRes, bfRes);
        double speedup = (bfMs > 0) ? bfMs / acMs : 0.0;

        cout << "[Test 5] perf: text=" << text.size() << " chars, patterns=" << nWords
             << "\n  AC = " << acMs << " ms (" << acRes.size() << " matches)"
             << "\n  BF = " << bfMs << " ms (" << bfRes.size() << " matches)"
             << "\n  correctness: " << (ok ? "PASS" : "FAIL")
             << "  speedup = " << speedup << "x" << "\n";
    }

    cout << "\nDONE\n";
    return 0;
}
