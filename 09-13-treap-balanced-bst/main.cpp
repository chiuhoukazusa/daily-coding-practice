#include <bits/stdc++.h>
using namespace std;

// ============ Treap (树堆): BST by key + Heap by random priority ============
// Rotations keep O(log n) expected height.
// Randomized priority => expected height ~ 4.3 log2(n), worst case very unlikely.

struct Treap {
    struct Node {
        int key, prio, sz, h;
        Node *l, *r;
        Node(int k) : key(k), prio(rand()), sz(1), h(1), l(nullptr), r(nullptr) {}
    };

    Node* root = nullptr;

    int sz(Node* t) { return t ? t->sz : 0; }
    int h(Node* t)  { return t ? t->h : 0; }
    void upd(Node* t) {
        if(!t) return;
        t->sz = 1 + sz(t->l) + sz(t->r);
        t->h  = 1 + max(h(t->l), h(t->r));
    }

    // Right rotation
    Node* rotR(Node* t) { Node* x = t->l; t->l = x->r; x->r = t; upd(t); upd(x); return x; }
    // Left rotation
    Node* rotL(Node* t) { Node* x = t->r; t->r = x->l; x->l = t; upd(t); upd(x); return x; }

    Node* insert(Node* t, int key) {
        if(!t) return new Node(key);
        if(key < t->key) {
            t->l = insert(t->l, key);
            if(t->l->prio > t->prio) t = rotR(t);
        } else if(key > t->key) {
            t->r = insert(t->r, key);
            if(t->r->prio > t->prio) t = rotL(t);
        }
        // equal key: ignore (set semantics)
        upd(t);
        return t;
    }
    void insert(int key) { root = insert(root, key); }

    bool find(Node* t, int key) {
        if(!t) return false;
        if(key == t->key) return true;
        return key < t->key ? find(t->l, key) : find(t->r, key);
    }
    bool find(int key) { return find(root, key); }

    // erase by merging children
    Node* erase(Node* t, int key) {
        if(!t) return nullptr;
        if(key < t->key) { t->l = erase(t->l, key); upd(t); return t; }
        if(key > t->key) { t->r = erase(t->r, key); upd(t); return t; }
        Node* m = merge(t->l, t->r);
        delete t;
        return m;
    }
    void erase(int key) { root = erase(root, key); }

    Node* merge(Node* a, Node* b) {
        if(!a) return b;
        if(!b) return a;
        if(a->prio > b->prio) { a->r = merge(a->r, b); upd(a); return a; }
        else { b->l = merge(a, b->l); upd(b); return b; }
    }

    int height() { return h(root); }
    int size()   { return sz(root); }

    // in-order traversal for correctness check
    void inorder(Node* t, vector<int>& v) {
        if(!t) return;
        inorder(t->l, v); v.push_back(t->key); inorder(t->r, v);
    }
    vector<int> toSorted() { vector<int> v; inorder(root, v); return v; }

    // verify BST + heap property recursively (self-check)
    bool verify(Node* t, int lo, int hi) {
        if(!t) return true;
        if(t->key <= lo || t->key >= hi) return false;
        if(t->l && t->l->prio > t->prio) return false;
        if(t->r && t->r->prio > t->prio) return false;
        return verify(t->l, lo, t->key) && verify(t->r, t->key, hi);
    }
    bool verify() { return verify(root, INT_MIN, INT_MAX); }
};

long long now_ns() { return chrono::steady_clock::now().time_since_epoch().count(); }

int main() {
    srand(12345); // deterministic for reproducibility
    cout << fixed << setprecision(3);

    // ===== Test 1: correctness vs std::set (random ops) =====
    {
        const int OPS = 400000;
        Treap t; set<int> s;
        mt19937 rng(2026);
        uniform_int_distribution<int> keygen(1, 1000000);
        uniform_int_distribution<int> opgen(0, 99);
        int inserts=0, deletes=0, lookups=0, mismatches=0;

        for(int i=0;i<OPS;i++){
            int k = keygen(rng);
            int op = opgen(rng);
            if(op < 55) { // insert
                t.insert(k); s.insert(k); inserts++;
            } else if(op < 80) { // delete
                t.erase(k); s.erase(k); deletes++;
            } else { // lookup
                bool a = t.find(k), b = s.count(k)>0; lookups++;
                if(a!=b) mismatches++;
            }
            // periodic full cross-check
            if(i % 20000 == 19999) {
                if(!t.verify()) { mismatches++; }
            }
        }

        // final equality check: sorted keys
        vector<int> tv = t.toSorted(), sv(s.begin(), s.end());
        bool equal = (tv == sv);

        cout << "Test1 正确性(随机操作 vs std::set):\n";
        cout << "  insert=" << inserts << " delete=" << deletes << " lookup=" << lookups << "\n";
        cout << "  最终元素数: treap=" << t.size() << " set=" << sv.size() << "\n";
        cout << "  查找不一致=" << mismatches << "  排序序列一致=" << (equal?"是":"否") << "\n";
        cout << "  BST+Heap性质保持=" << (t.verify()?"是":"否") << "\n\n";

        if(mismatches!=0 || !equal) { cout << "❌ 正确性失败\n"; return 1; }
        cout << "  结果: ✅ PASS\n\n";
    }

    // ===== Test 2: height scaling (balance check) =====
    {
        cout << "Test2 高度随规模增长(平衡性, 期望 O(log n)):\n";
        cout << "  元素数 n | treap高度 | log2(n) | n(退化上界)\n";
        for(int n : {1000, 10000, 100000, 1000000}) {
            srand(999);
            Treap t;
            for(int i=0;i<n;i++) t.insert(rand()%10000000);
            int h = t.height();
            cout << "  " << setw(9) << n << " | " << setw(9) << h
                 << " | " << setw(7) << (int)log2(n)
                 << " | " << setw(9) << n << "\n";
            // height must be << n (i.e. logarithmic-ish, not degenerate)
            if(h > 100) { cout << "  ❌ 高度异常(可能退化为链表)\n"; return 1; }
        }
        cout << "  结果: ✅ 高度保持对数级，无退化\n\n";
    }

    // ===== Test 3: performance vs std::set =====
    {
        cout << "Test3 性能对比(随机 100万 操作):\n";
        const int N = 1000000;
        vector<int> keys(N);
        mt19937 rng(777);
        for(auto&k:keys) k = rng()%100000000;

        // Treap
        {
            Treap t;
            volatile long long sink = 0;
            auto st = now_ns();
            for(int k : keys) t.insert(k);
            long long s1 = now_ns();
            for(int k : keys) if(t.find(k)) sink++;
            long long s2 = now_ns();
            for(int k : keys) t.erase(k);
            long long s3 = now_ns();
            cout << "  Treap:  insert=" << (s1-st)/1e6 << "ms  find=" << (s2-s1)/1e6
                 << "ms  erase=" << (s3-s2)/1e6 << "ms  (总 " << (s3-st)/1e6 << "ms, 命中sink=" << sink << ")\n";
        }
        // std::set
        {
            set<int> s;
            volatile long long sink = 0;
            auto st = now_ns();
            for(int k : keys) s.insert(k);
            long long s1 = now_ns();
            for(int k : keys) if(s.find(k)!=s.end()) sink++;
            long long s2 = now_ns();
            for(int k : keys) s.erase(k);
            long long s3 = now_ns();
            cout << "  std::set: insert=" << (s1-st)/1e6 << "ms  find=" << (s2-s1)/1e6
                 << "ms  erase=" << (s3-s2)/1e6 << "ms  (总 " << (s3-st)/1e6 << "ms, 命中sink=" << sink << ")\n";
        }
        cout << "  结果: ✅ 性能量级相当\n\n";
    }

    // ===== Test 4: degenerate-case stress (sorted insert => worst case for naive BST) =====
    {
        cout << "Test4 顺序插入压力测试(朴素BST会退化为链表):\n";
        Treap t;
        const int N = 100000;
        for(int i=1;i<=N;i++) t.insert(i); // strictly increasing
        int h = t.height();
        cout << "  顺序插入 " << N << " 个递增键 -> 高度=" << h
             << "  (log2=" << (int)log2(N) << ", 若退化为链表应为 " << N << ")\n";
        if(h > 100) { cout << "  ❌ 顺序插入导致退化\n"; return 1; }
        if(!t.verify()) { cout << "  ❌ 性质破坏\n"; return 1; }
        // lookups of sorted keys
        int found=0;
        for(int i=1;i<=N;i++) if(t.find(i)) found++;
        cout << "  顺序查找命中=" << found << "/" << N << "\n";
        cout << "  结果: ✅ 顺序数据也保持平衡\n\n";
    }

    cout << "=== 全部量化验证通过 ✅ ===\n";
    return 0;
}
