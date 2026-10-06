// AVL Tree Self-Balancing Binary Search Tree
// Quantitative verification: balance-factor invariant, height bound, correctness vs std::set, performance.
#include <bits/stdc++.h>
using namespace std;

struct Node {
    int key;
    int height;
    Node* left;
    Node* right;
    Node(int k) : key(k), height(1), left(nullptr), right(nullptr) {}
};

class AVL {
    Node* root = nullptr;

    int h(Node* n) { return n ? n->height : 0; }
    int bf(Node* n) { return n ? h(n->left) - h(n->right) : 0; }
    void update(Node* n) { if (n) n->height = 1 + max(h(n->left), h(n->right)); }

    Node* rotR(Node* y) {
        Node* x = y->left; Node* T2 = x->right;
        x->right = y; y->left = T2;
        update(y); update(x); return x;
    }
    Node* rotL(Node* x) {
        Node* y = x->right; Node* T2 = y->left;
        y->left = x; x->right = T2;
        update(x); update(y); return y;
    }

    Node* insert(Node* n, int key) {
        if (!n) return new Node(key);
        if (key < n->key) n->left = insert(n->left, key);
        else if (key > n->key) n->right = insert(n->right, key);
        else return n; // no duplicates
        update(n);
        int bal = bf(n);
        // LL
        if (bal > 1 && key < n->left->key) return rotR(n);
        // RR
        if (bal < -1 && key > n->right->key) return rotL(n);
        // LR
        if (bal > 1 && key > n->left->key) { n->left = rotL(n->left); return rotR(n); }
        // RL
        if (bal < -1 && key < n->right->key) { n->right = rotR(n->right); return rotL(n); }
        return n;
    }

    Node* minNode(Node* n) { while (n->left) n = n->left; return n; }

    Node* erase(Node* n, int key) {
        if (!n) return nullptr;
        if (key < n->key) n->left = erase(n->left, key);
        else if (key > n->key) n->right = erase(n->right, key);
        else {
            if (!n->left || !n->right) {
                Node* t = n->left ? n->left : n->right;
                delete n; return t;
            }
            Node* suc = minNode(n->right);
            n->key = suc->key;
            n->right = erase(n->right, suc->key);
        }
        update(n);
        int bal = bf(n);
        if (bal > 1) {
            if (bf(n->left) >= 0) return rotR(n);       // LL
            n->left = rotL(n->left); return rotR(n);     // LR
        }
        if (bal < -1) {
            if (bf(n->right) <= 0) return rotL(n);       // RR
            n->right = rotR(n->right); return rotL(n);   // RL
        }
        return n;
    }

    bool contains(Node* n, int key) {
        if (!n) return false;
        if (key < n->key) return contains(n->left, key);
        if (key > n->key) return contains(n->right, key);
        return true;
    }

    void inorder(Node* n, vector<int>& out) {
        if (!n) return;
        inorder(n->left, out);
        out.push_back(n->key);
        inorder(n->right, out);
    }

    int validate(Node* n, bool& ok) {
        if (!n) return 0;
        int hl = validate(n->left, ok);
        int hr = validate(n->right, ok);
        int bal = hl - hr;
        if (bal < -1 || bal > 1) ok = false;                     // balance factor invariant
        if (n->left && !(n->left->key < n->key)) ok = false;     // BST ordering
        if (n->right && !(n->right->key > n->key)) ok = false;
        if (n->height != 1 + max(hl, hr)) ok = false;            // height correctness
        return n->height;
    }

public:
    void insert(int k) { root = insert(root, k); }
    void erase(int k) { root = erase(root, k); }
    bool contains(int k) { return contains(root, k); }
    int height() { return h(root); }
    vector<int> inorder() { vector<int> v; inorder(root, v); return v; }
    bool validate() { bool ok = true; validate(root, ok); return ok; }
    int size() { vector<int> v; inorder(root, v); return (int)v.size(); }
    Node* raw() { return root; }
};

int main() {
    // Seed random but deterministic for reproducibility
    mt19937 rng(42);

    int failures = 0;
    auto check = [&](bool cond, const string& msg) {
        cout << (cond ? "[PASS] " : "[FAIL] ") << msg << "\n";
        if (!cond) failures++;
    };

    // Test 1: correctness vs std::set over mixed insert/erase
    {
        AVL avl;
        set<int> ref;
        for (int i = 0; i < 200000; i++) {
            int op = rng() % 3;
            int key = int(rng() % 1000000);
            if (op < 2) { // insert
                avl.insert(key); ref.insert(key);
            } else {      // erase
                avl.erase(key); ref.erase(key);
            }
            if (i % 5000 == 0) {
                if (!avl.validate()) { check(false, "invariant broken at step " + to_string(i)); break; }
            }
        }
        vector<int> a = avl.inorder();
        vector<int> b(ref.begin(), ref.end());
        check(a == b, "std::set correctness: inorder sequence matches reference (" + to_string(b.size()) + " elements)");
    }

    // Test 2: balance factor invariant + height bound
    {
        AVL avl;
        int N = 100000;
        for (int i = 0; i < N; i++) avl.insert(int(rng() % 10000000));
        check(avl.validate(), "balance factor invariant holds after " + to_string(N) + " insertions");
        double h = avl.height();
        double bound = 1.44 * log2(N + 2) - 0.328; // theoretical AVL upper bound
        cout << "  height=" << h << "  theoretical_bound~" << bound
             << "  log2(n)=" << log2(N) << "\n";
        check(h <= bound + 0.5, "height within theoretical O(log n) bound");
    }

    // Test 3: worst-case sorted insertion (degenerate risk test)
    {
        AVL avl;
        int N = 50000;
        for (int i = 1; i <= N; i++) avl.insert(i); // strictly increasing
        check(avl.validate(), "balanced after sorted (worst-case) insertion");
        check(avl.height() <= 1.44 * log2(N + 2), "height stays logarithmic for sorted input: " + to_string(avl.height()));
    }

    // Test 4: performance vs std::set
    {
        vector<int> data(300000);
        for (auto& x : data) x = int(rng() % 10000000);

        AVL avl;
        auto t0 = chrono::high_resolution_clock::now();
        for (int x : data) avl.insert(x);
        auto t1 = chrono::high_resolution_clock::now();
        for (int x : data) avl.contains(x);
        auto t2 = chrono::high_resolution_clock::now();
        for (int x : data) avl.erase(x);
        auto t3 = chrono::high_resolution_clock::now();

        set<int> s;
        auto s0 = chrono::high_resolution_clock::now();
        for (int x : data) s.insert(x);
        auto s1 = chrono::high_resolution_clock::now();
        for (int x : data) s.count(x);
        auto s2 = chrono::high_resolution_clock::now();
        for (int x : data) s.erase(x);
        auto s3 = chrono::high_resolution_clock::now();

        auto ms = [](auto a, auto b) { return chrono::duration_cast<chrono::milliseconds>(b - a).count(); };
        cout << "  AVL:       insert=" << ms(t0,t1) << "ms lookup=" << ms(t1,t2) << "ms erase=" << ms(t2,t3) << "ms\n";
        cout << "  std::set:  insert=" << ms(s0,s1) << "ms lookup=" << ms(s1,s2) << "ms erase=" << ms(s2,s3) << "ms\n";
        check(avl.validate() && avl.size() == 0 && avl.height() == 0, "structure empty & consistent after bulk erase");
    }

    cout << "========================================\n";
    cout << (failures == 0 ? "ALL TESTS PASSED" : (to_string(failures) + " TEST(S) FAILED")) << "\n";
    return failures == 0 ? 0 : 1;
}
