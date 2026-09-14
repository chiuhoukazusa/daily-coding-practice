// Red-Black Tree — Self-Balancing BST
// 2026-09-15 daily coding practice
//
// Implements a classic CLRS-style red-black tree with a NIL sentinel leaf:
//  - Insert (fixup: recolor + rotations)
//  - Erase  (fixup: cases)
//  - Find / kth (order statistics via subtree sizes)
//  - In-order traversal emitting sorted keys
//
// Quantitative verification (no "looking at the picture"):
//  1. Red-Black invariants hold after N random ops:
//       (a) Root is BLACK
//       (b) No RED node has a RED child
//       (c) Every root->NIL path has the same BLACK height
//       (d) BST in-order => strictly ascending keys
//  2. Correctness vs std::set (insert/find/erase/kth match)
//  3. Balance quality: height(N) <= 2*log2(N+1) and black-height bound
//  4. Performance: build+lookup vs std::set baseline

#include <bits/stdc++.h>
using namespace std;

enum Color { RED, BLACK };

struct Node {
    int key;
    Color c;
    Node *l, *r, *p;
    int sz; // subtree size (for order statistics)
    Node(int k) : key(k), c(RED), l(nullptr), r(nullptr), p(nullptr), sz(1) {}
};

struct RBTree {
    Node *root;
    Node *NIL; // sentinel leaf: BLACK, sz=0, self-linked

    RBTree() {
        NIL = new Node(0);
        NIL->c = BLACK; NIL->sz = 0;
        NIL->l = NIL->r = NIL->p = NIL;
        root = NIL;
    }

    bool isRed(Node *x) { return x != NIL && x->c == RED; }
    void pull(Node *x) { if (x != NIL) x->sz = 1 + x->l->sz + x->r->sz; }

    void rotateLeft(Node *x) {
        Node *y = x->r;
        x->r = y->l;
        if (y->l != NIL) y->l->p = x;
        y->p = x->p;
        if (x->p == NIL) root = y;
        else if (x == x->p->l) x->p->l = y;
        else x->p->r = y;
        y->l = x; x->p = y;
        pull(x); pull(y);
    }
    void rotateRight(Node *x) {
        Node *y = x->l;
        x->l = y->r;
        if (y->r != NIL) y->r->p = x;
        y->p = x->p;
        if (x->p == NIL) root = y;
        else if (x == x->p->r) x->p->r = y;
        else x->p->l = y;
        y->r = x; x->p = y;
        pull(x); pull(y);
    }

    void insert(int key) {
        // skip duplicates (set semantics, matching std::set)
        if (find(key)) return;
        Node *z = new Node(key);
        z->l = z->r = NIL;
        Node *y = NIL, *x = root;
        while (x != NIL) { y = x; if (key < x->key) x = x->l; else x = x->r; }
        z->p = y;
        if (y == NIL) root = z;
        else if (key < y->key) y->l = z;
        else y->r = z;
        for (Node *a = z->p; a != NIL; a = a->p) pull(a);
        insertFixup(z);
    }

    void insertFixup(Node *z) {
        while (isRed(z->p)) {
            if (z->p == z->p->p->l) {
                Node *uncle = z->p->p->r;
                if (isRed(uncle)) {
                    z->p->c = BLACK; uncle->c = BLACK; z->p->p->c = RED;
                    z = z->p->p;
                } else {
                    if (z == z->p->r) { z = z->p; rotateLeft(z); }
                    z->p->c = BLACK; z->p->p->c = RED; rotateRight(z->p->p);
                }
            } else {
                Node *uncle = z->p->p->l;
                if (isRed(uncle)) {
                    z->p->c = BLACK; uncle->c = BLACK; z->p->p->c = RED;
                    z = z->p->p;
                } else {
                    if (z == z->p->l) { z = z->p; rotateRight(z); }
                    z->p->c = BLACK; z->p->p->c = RED; rotateLeft(z->p->p);
                }
            }
        }
        root->c = BLACK;
    }

    Node* findMin(Node *x) { while (x->l != NIL) x = x->l; return x; }

    void transplant(Node *u, Node *v) {
        if (u->p == NIL) root = v;
        else if (u == u->p->l) u->p->l = v;
        else u->p->r = v;
        v->p = u->p;
    }

    void erase(int key) {
        Node *z = root;
        while (z != NIL) { if (key < z->key) z = z->l; else if (key > z->key) z = z->r; else break; }
        if (z == NIL) return;
        Node *y = z, *x;
        Color yOrig = y->c;
        if (z->l == NIL) { x = z->r; transplant(z, z->r); }
        else if (z->r == NIL) { x = z->l; transplant(z, z->l); }
        else {
            y = findMin(z->r); yOrig = y->c; x = y->r;
            if (y->p == z) { x->p = y; }
            else { transplant(y, y->r); y->r = z->r; y->r->p = y; }
            transplant(z, y); y->l = z->l; y->l->p = y; y->c = z->c;
        }
        delete z;
        if (yOrig == BLACK) eraseFixup(x);
        recomputeSizes();
    }

    void eraseFixup(Node *x) {
        while (x != root && !isRed(x)) {
            if (x == x->p->l) {
                Node *w = x->p->r;
                if (isRed(w)) { w->c = BLACK; x->p->c = RED; rotateLeft(x->p); w = x->p->r; }
                if (!isRed(w->l) && !isRed(w->r)) { w->c = RED; x = x->p; }
                else {
                    if (!isRed(w->r)) { w->l->c = BLACK; w->c = RED; rotateRight(w); w = x->p->r; }
                    w->c = x->p->c; x->p->c = BLACK; w->r->c = BLACK; rotateLeft(x->p); x = root;
                }
            } else {
                Node *w = x->p->l;
                if (isRed(w)) { w->c = BLACK; x->p->c = RED; rotateRight(x->p); w = x->p->l; }
                if (!isRed(w->r) && !isRed(w->l)) { w->c = RED; x = x->p; }
                else {
                    if (!isRed(w->l)) { w->r->c = BLACK; w->c = RED; rotateLeft(w); w = x->p->l; }
                    w->c = x->p->c; x->p->c = BLACK; w->l->c = BLACK; rotateRight(x->p); x = root;
                }
            }
        }
        x->c = BLACK;
    }

    void recomputeSizes() { recompute(root); }
    int recompute(Node *x) {
        if (x == NIL) return 0;
        x->sz = 1 + recompute(x->l) + recompute(x->r);
        return x->sz;
    }

    bool find(int key) { Node *x = root; while (x != NIL) { if (key < x->key) x = x->l; else if (key > x->key) x = x->r; else return true; } return false; }

    int kth(int k) { // 1-indexed
        Node *x = root;
        while (x != NIL) {
            int ls = x->l->sz;
            if (k <= ls) x = x->l;
            else if (k == ls + 1) return x->key;
            else { k -= ls + 1; x = x->r; }
        }
        return -1;
    }

    void inorder(Node *x, vector<int> &out) {
        if (x == NIL) return;
        inorder(x->l, out);
        out.push_back(x->key);
        inorder(x->r, out);
    }
    vector<int> inorder() { vector<int> v; inorder(root, v); return v; }

    int height() { return hgt(root); }
    int hgt(Node *x) { if (x == NIL) return 0; return 1 + max(hgt(x->l), hgt(x->r)); }
    long long count() { return cnt(root); }
    long long cnt(Node *x) { if (x == NIL) return 0; return 1 + cnt(x->l) + cnt(x->r); }
};

// ---- Verification helpers (operate on NIL-based tree) ----
struct InvariantResult { bool ok; int bh; int h; long long nodes; string err; };

InvariantResult checkInvariants(RBTree &t) {
    InvariantResult r; r.ok = true; r.nodes = 0; r.h = 0; r.err = "";
    Node *root = t.root, *NIL = t.NIL;
    if (root == NIL) { r.bh = 0; return r; }
    function<tuple<bool,int,int,long long>(Node*)> dfs = [&](Node *x) -> tuple<bool,int,int,long long> {
        if (x == NIL) return {true, 0, 0, 0};
        bool ok = true;
        if (x->c == RED) {
            if (x->l != NIL && x->l->c == RED) ok = false;
            if (x->r != NIL && x->r->c == RED) ok = false;
        }
        auto [lok, lbh, lh, ln] = dfs(x->l);
        auto [rok, rbh, rh, rn] = dfs(x->r);
        if (lbh != rbh) ok = false;
        int bh = lbh + (x->c == BLACK ? 1 : 0);
        int h = 1 + max(lh, rh);
        long long n = 1 + ln + rn;
        return {ok && lok && rok, bh, h, n};
    };
    auto [ok, bh, h, nodes] = dfs(root);
    r.ok = ok; r.bh = bh; r.h = h; r.nodes = nodes;
    if (root->c != BLACK) { r.ok = false; r.err += "root not black; "; }
    return r;
}

int main() {
    cout << fixed << setprecision(6);
    cout << "=== Red-Black Tree Quantitative Verification ===\n\n";

    bool all_ok = true;

    // Test 1: invariants + correctness across multiple random workloads
    for (int trial = 1; trial <= 20; trial++) {
        RBTree t;
        set<int> ref;
        mt19937_64 rng(20260915 + trial);
        int N = 1000 + (rng() % 5000);
        for (int i = 0; i < N; i++) {
            int op = rng() % 100;
            int key = rng() % N; // dense-ish
            if (op < 60) { t.insert(key); ref.insert(key); }
            else if (op < 85) {
                if (!ref.empty()) {
                    auto it = ref.begin(); advance(it, rng() % ref.size());
                    int k = *it; t.erase(k); ref.erase(k);
                }
            } else {
                bool a = t.find(key), b = ref.count(key) > 0;
                if (a != b) { all_ok = false; cout << "FIND MISMATCH trial " << trial << " key " << key << "\n"; }
            }
        }
        auto inv = checkInvariants(t);
        if (!inv.ok) { all_ok = false; cout << "INVARIANT VIOLATION trial " << trial << " " << inv.err << "\n"; }
        vector<int> v = t.inorder();
        if ((long long)v.size() != (long long)ref.size()) { all_ok = false; cout << "SIZE MISMATCH trial " << trial << "\n"; }
        for (size_t i = 1; i < v.size(); i++) if (v[i-1] >= v[i]) { all_ok = false; cout << "NOT SORTED trial " << trial << "\n"; }
        vector<int> rv(ref.begin(), ref.end());
        if (v != rv) { all_ok = false; cout << "CONTENT MISMATCH trial " << trial << "\n"; }
        for (int q = 0; q < 100; q++) {
            if (v.empty()) continue;
            int k = 1 + (rng() % v.size());
            if (t.kth(k) != v[k-1]) { all_ok = false; cout << "KTH MISMATCH trial " << trial << " k " << k << "\n"; }
        }
        double n = (double)v.size();
        if (n > 0) {
            double bound = 2.0 * log2(n + 1.0);
            if (inv.h > bound) { all_ok = false; cout << "HEIGHT VIOLATION trial " << trial << " h=" << inv.h << " bound=" << bound << " n=" << n << "\n"; }
        }
    }
    cout << "Test 1: 20 randomized mixed-workload trials (insert/erase/find/kth) + invariants: "
         << (all_ok ? "PASS" : "FAIL") << "\n";

    // Test 2: deterministic sorted insert = worst-case, verify height bound + black-height
    {
        RBTree t;
        int N = (1 << 14) - 1; // 16383
        for (int i = 1; i <= N; i++) t.insert(i);
        auto inv = checkInvariants(t);
        double hbound = 2.0 * log2(N + 1.0); // = 28
        bool ok = inv.ok && inv.h <= hbound;
        cout << "Test 2: sorted insert of " << N << " nodes: height=" << inv.h
             << " (bound " << hbound << "), black-height=" << inv.bh
             << " (valid range [" << (int)(log2(N+1)/2) << ", " << (int)log2(N+1) << "]), invariants=" << (inv.ok ? "hold" : "VIOLATED")
             << " -> " << (ok ? "PASS" : "FAIL") << "\n";
        if (!ok) all_ok = false;
    }

    // Test 3: ascending iterator equality with std::set on random data
    {
        RBTree t; set<int> s; mt19937_64 r2(42);
        for (int i = 0; i < 50000; i++) { int k = r2() % 100000; t.insert(k); s.insert(k); }
        for (int i = 0; i < 20000; i++) { int k = r2() % 100000; t.erase(k); s.erase(k); }
        auto v = t.inorder();
        vector<int> sv(s.begin(), s.end());
        bool ok = (v == sv);
        cout << "Test 3: 50k insert + 20k erase, inorder equality vs std::set (" << v.size()
             << " elems): " << (ok ? "PASS" : "FAIL") << "\n";
        if (!ok) all_ok = false;
    }

    // Test 4: performance comparison (build + lookup) vs std::set
    {
        int N = 1000000;
        vector<int> keys(N);
        mt19937_64 r3(7);
        for (auto &k : keys) k = r3();

        RBTree t;
        auto t0 = chrono::high_resolution_clock::now();
        for (int k : keys) t.insert(k);
        auto t1 = chrono::high_resolution_clock::now();
        double rb_build = chrono::duration<double>(t1 - t0).count();

        set<int> s;
        t0 = chrono::high_resolution_clock::now();
        for (int k : keys) s.insert(k);
        t1 = chrono::high_resolution_clock::now();
        double stl_build = chrono::duration<double>(t1 - t0).count();

        long long hits = 0;
        t0 = chrono::high_resolution_clock::now();
        for (int k : keys) hits += t.find(k);
        t1 = chrono::high_resolution_clock::now();
        double rb_lookup = chrono::duration<double>(t1 - t0).count();

        long long hits2 = 0;
        t0 = chrono::high_resolution_clock::now();
        for (int k : keys) hits2 += s.count(k);
        t1 = chrono::high_resolution_clock::now();
        double stl_lookup = chrono::duration<double>(t1 - t0).count();

        auto inv = checkInvariants(t);
        long long unique_cnt = t.count();
        bool find_ok = (hits == (long long)N) && (hits2 == (long long)N) && (unique_cnt > 0);
        cout << "Test 4: performance (N=" << N << "), invariants hold=" << (inv.ok ? "yes" : "NO") << "\n";
        cout << "  build  RB=" << rb_build << "s  std::set=" << stl_build << "s  ratio=" << rb_build/stl_build << "x\n";
        cout << "  lookup RB=" << rb_lookup << "s  std::set=" << stl_lookup << "s  ratio=" << rb_lookup/stl_lookup << "x (unique=" << unique_cnt << ")\n";
        cout << "  -> " << (find_ok ? "PASS (find correctness confirmed)" : "FAIL") << "\n";
        if (!find_ok) all_ok = false;
    }

    cout << "\n=== " << (all_ok ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << " ===\n";
    return all_ok ? 0 : 1;
}
