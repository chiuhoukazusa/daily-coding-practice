#include <bits/stdc++.h>
using namespace std;

// =====================================================================
// Skip List — a probabilistic ordered-map data structure.
// Average O(log n) search/insert/erase, average O(log n) space-per-key.
// ---------------------------------------------------------------------
// Layers (levels) are composed of sorted singly-linked lists. Each node
// has a random "height" drawn from a geometric distribution (p = 0.5),
// giving an expected ~1/(1-p) = 2 levels per node and an expected top
// level of ~ log_{1/p}(n). A "sentinel" head node of MAX_LEVEL reaches
// every layer, and update[] records the predecessors at each level so
// that insert/erase can splice exactly like a multi-level linked list.
// =====================================================================

static constexpr int MAX_LEVEL = 32;
static constexpr double P = 0.5;

// Simple seeded PRNG (deterministic, std-optional, self-contained).
struct RNG {
    uint64_t s;
    explicit RNG(uint64_t seed) : s(seed) {}
    uint64_t next() { // xorshift64*
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return s * 0x2545F4914F6CDD1DULL;
    }
    uint64_t nextUniform() const { // splitmix-style top bits for quality
        return s; // replaced below in next()
    }
};

struct SkipList {
    struct Node {
        int key;
        int val;
        int level;
        vector<Node*> forward; // forward[i] = next node at level i
        Node(int k, int v, int lv) : key(k), val(v), level(lv), forward(lv + 1, nullptr) {}
    };

    Node* head;
    int sz;
    int maxLevel;      // current maximum level among all nodes
    RNG rng;
    uint64_t randCalls;

    explicit SkipList(uint64_t seed) : rng(seed), randCalls(0) {
        head = new Node(numeric_limits<int>::min(), 0, MAX_LEVEL);
        sz = 0;
        maxLevel = 0;
    }

    // Geometric random level: level increments while a random bit is 1,
    // capped at MAX_LEVEL. Returns number of forward pointers (levels+1).
    int randomLevel() {
        int lv = 0;
        // P(H>=k) = p^k. Draw until failure or cap.
        while (lv < MAX_LEVEL && (rng.next() & 1) == 1) lv++; // p=0.5
        randCalls++;
        return lv;
    }

    // Find predecessor chain for `key` into update[].
    Node** buildUpdate(int key, Node** update) const {
        Node* cur = head;
        for (int i = maxLevel; i >= 0; --i) {
            while (cur->forward[i] && cur->forward[i]->key < key)
                cur = cur->forward[i];
            update[i] = cur;
        }
        return update;
    }

    bool contains(int key) const {
        Node* cur = head;
        for (int i = maxLevel; i >= 0; --i)
            while (cur->forward[i] && cur->forward[i]->key < key)
                cur = cur->forward[i];
        cur = cur->forward[0];
        return cur && cur->key == key;
    }

    // Search: returns value if found, else -1.
    int search(int key) const {
        Node* cur = head;
        for (int i = maxLevel; i >= 0; --i)
            while (cur->forward[i] && cur->forward[i]->key < key)
                cur = cur->forward[i];
        cur = cur->forward[0];
        return (cur && cur->key == key) ? cur->val : -1;
    }

    void insert(int key, int val) {
        Node* update[MAX_LEVEL + 1];
        Node* cur = head;
        for (int i = maxLevel; i >= 0; --i) {
            while (cur->forward[i] && cur->forward[i]->key < key)
                cur = cur->forward[i];
            update[i] = cur;
        }
        cur = cur->forward[0];
        if (cur && cur->key == key) { cur->val = val; return; } // overwrite

        int lvl = randomLevel();
        if (lvl > maxLevel) {
            for (int i = maxLevel + 1; i <= lvl; ++i) update[i] = head;
            maxLevel = lvl;
        }
        Node* n = new Node(key, val, lvl);
        for (int i = 0; i <= lvl; ++i) {
            n->forward[i] = update[i]->forward[i];
            update[i]->forward[i] = n;
        }
        ++sz;
    }

    bool erase(int key) {
        Node* update[MAX_LEVEL + 1];
        Node* cur = head;
        for (int i = maxLevel; i >= 0; --i) {
            while (cur->forward[i] && cur->forward[i]->key < key)
                cur = cur->forward[i];
            update[i] = cur;
        }
        cur = cur->forward[0];
        if (!cur || cur->key != key) return false;
        for (int i = 0; i <= cur->level; ++i)
            update[i]->forward[i] = cur->forward[i];
        delete cur;
        while (maxLevel > 0 && head->forward[maxLevel] == nullptr) --maxLevel;
        --sz;
        return true;
    }

    int size() const { return sz; }
    int currentMaxLevel() const { return maxLevel; }

    // In-order traversal (via level-0 list) -> sorted ascending key list.
    vector<int> toSortedKeys() const {
        vector<int> ks;
        for (Node* n = head->forward[0]; n; n = n->forward[0]) ks.push_back(n->key);
        return ks;
    }

    // Distribution of node levels (for probabilistic-balance verification).
    map<int,int> levelHistogram() const {
        map<int,int> h;
        for (Node* n = head->forward[0]; n; n = n->forward[0]) h[n->level]++;
        return h;
    }
};

// =====================================================================
// Verification harness (fully quantitative, no visual inspection).
// =====================================================================

int main() {
    ios::sync_with_stdio(false);
    cout << fixed << setprecision(6);

    bool allOk = true;
    auto check = [&](bool cond, const string& name) {
        cout << (cond ? "PASS" : "FAIL") << "  " << name << "\n";
        if (!cond) allOk = false;
    };

    // ---------- Correctness vs std::set (random ops) ----------
    {
        const int N = 200000;
        SkipList sl(123456789ULL);
        set<int> ref;
        RNG r(987654321ULL);
        int ins = 0, del = 0, q = 0;

        for (int i = 0; i < N; ++i) {
            int op = (int)(r.next() % 3);
            int key = (int)(r.next() % 100000); // dense-ish key space -> lots of hits
            if (op == 0) { // insert
                sl.insert(key, key * 7);
                ref.insert(key);
                ins++;
            } else if (op == 1) { // erase
                bool a = sl.erase(key);
                size_t b = ref.erase(key);
                if (a != (b > 0)) { check(false, "erase-return-mismatch"); break; }
                del++;
            } else { // query contains / membership
                bool a = sl.contains(key);
                bool b = ref.count(key) > 0;
                if (a != b) { check(false, "membership-mismatch @ key=" + to_string(key)); break; }
                q++;
            }
        }

        // Final exact equality: same sorted key set + same size.
        auto sk = sl.toSortedKeys();
        vector<int> rk(ref.begin(), ref.end());
        check(sl.size() == (int)ref.size(), "size-equality");
        check(sk == rk, "sorted-keys-equality");
        check(sk.size() == sk.empty() ? true : is_sorted(sk.begin(), sk.end()), "sorted-ascending");
        cout << "  ops: insert=" << ins << " erase=" << del << " query=" << q
             << " | final size=" << sl.size() << "\n";
    }

    // ---------- Search correctness on deterministic keys ----------
    {
        SkipList sl(42ULL);
        const int M = 5000;
        for (int i = 0; i < M; ++i) sl.insert(i * 3, i * 3 + 100);
        bool ok = true;
        for (int i = 0; i < M; ++i)
            if (sl.search(i * 3) != i * 3 + 100) { ok = false; break; }
        check(ok, "search-found-values");
        bool missOk = true;
        for (int i = 0; i < M; ++i) {
            int k = i * 3 + 1; // guaranteed gaps
            if (sl.contains(k)) { missOk = false; break; }
        }
        check(missOk, "search-misses-correct");
    }

    // ---------- Probabilistic balance: level distribution ----------
    {
        const int N = 100000;
        SkipList sl(20260922ULL);
        for (int i = 0; i < N; ++i) sl.insert((int)(sl.rng.next() % 1000000), i);
        auto hist = sl.levelHistogram();
        // Each level i (0-based) should hold ~ N * (1 - p) * p^i nodes.
        // Validate level-0 ~ N/2 .. N, and geometric decay for a few levels.
        long long lvl0 = hist[0];
        check(lvl0 >= N / 3 && lvl0 <= N, "level-0-size-geometric (" + to_string(lvl0) + ")");
        for (int i = 1; i <= 6; ++i) {
            long long expect = (long long)(N * (1.0 - P) * pow(P, i));
            long long got = hist[i];
            // wide tolerance for randomness
            double ratio = (double)got / max(1LL, expect);
            check(ratio > 0.3 && ratio < 2.5,
                  "level-" + to_string(i) + "-decay (got=" + to_string(got) +
                  " expect~" + to_string(expect) + ")");
        }
        check(sl.currentMaxLevel() > 10 && sl.currentMaxLevel() < 32,
              "max-level-in-expected-range (=" + to_string(sl.currentMaxLevel()) + ")");
        cout << "  levels histogram: ";
        for (int i = 0; i <= min(8, sl.currentMaxLevel()); ++i) cout << i << ":" << hist[i] << " ";
        cout << "... maxLevel=" << sl.currentMaxLevel() << "\n";
    }

    // ---------- Performance: skip list vs naive sorted-vector lookup ----------
    {
        const int N = 500000;
        SkipList sl(555ULL);
        vector<int> keys;
        RNG r(777ULL);
        for (int i = 0; i < N; ++i) { int k = (int)(r.next() % 2000000); sl.insert(k, k); keys.push_back(k); }

        // Benchmark skip-list search
        auto t0 = chrono::high_resolution_clock::now();
        long long sink = 0;
        int Q = 1000000;
        for (int i = 0; i < Q; ++i) sink += sl.search(keys[i % N]);
        auto t1 = chrono::high_resolution_clock::now();
        double skipMs = chrono::duration<double, milli>(t1 - t0).count();

        // Benchmark naive O(n) linear search on the same keys (bounded sample)
        vector<int> sortedKeys = sl.toSortedKeys();
        int QLIN = 5000; // bounded: 5000 * 500k = 2.5e9 ops, acceptable
        t0 = chrono::high_resolution_clock::now();
        for (int i = 0; i < QLIN; ++i) {
            int target = keys[(i * 7919) % N];
            for (int x : sortedKeys) { if (x == target) { sink += x; break; } }
        }
        t1 = chrono::high_resolution_clock::now();
        double linearMs = chrono::duration<double, milli>(t1 - t0).count();
        double linearPerLookup = linearMs / QLIN;          // ms per lookup
        double skipPerLookup = skipMs / Q;                  // ms per lookup
        double speedup = linearPerLookup / skipPerLookup;
        cout << "  skip-list " << Q << " lookups: " << skipMs << " ms ("
             << skipPerLookup * 1000 << " us/lookup)\n";
        cout << "  naive linear " << QLIN << " lookups: " << linearMs << " ms ("
             << linearPerLookup * 1000 << " us/lookup)\n";
        cout << "  speedup = " << speedup << "x\n";
        check(speedup > 20.0, "skip-list-speedup>20x (=" + to_string(speedup) + "x)");
        // sanity: sink is only to prevent dead-code elimination
        if (sink == -1) cout << ""; 
    }

    // ---------- Summary ----------
    cout << "\n==========================================\n";
    cout << (allOk ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << "\n";
    cout << "==========================================\n";
    return allOk ? 0 : 1;
}
