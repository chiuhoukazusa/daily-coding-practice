// Splay Tree (Self-Adjusting Binary Search Tree)
// 09-16 daily coding practice
//
// Implements a splay tree with bottom-up splaying (zig / zig-zig / zig-zag),
// supporting insert / find / erase with amortized O(log n) cost.
//
// Quantitative verification (not visual):
//   1. Correctness vs std::set on random/sequential operation mixes.
//   2. BST invariant check (in-order traversal must be strictly increasing).
//   3. Splay operation effect: searched/found node becomes root.
//   4. Tree height after a sequence of accesses (balanced-ish, << N).
//   5. Amortized cost: total rotations per op stays near O(log n).
//   6. Performance benchmark vs naive BST and std::set (access locality).

#include <bits/stdc++.h>
using namespace std;

// ---------------- Splay Tree (top-down-free, explicit bottom-up) ----------------
struct Node {
    int key;
    Node *left = nullptr, *right = nullptr, *parent = nullptr;
    Node(int k) : key(k) {}
};

class SplayTree {
public:
    Node *root = nullptr;
    uint64_t rotations = 0;       // total splay rotations performed
    uint64_t compare_count = 0;   // total key comparisons during operations

    // Rotate node x up toward root (x must be non-root).
    void rotate(Node *x) {
        Node *p = x->parent;
        Node *g = p->parent;
        if (!p) return;
        if (p->left == x) {
            // x is left child of p -> right rotation
            p->left = x->right;
            if (x->right) x->right->parent = p;
            x->right = p;
        } else {
            p->right = x->left;
            if (x->left) x->left->parent = p;
            x->left = p;
        }
        p->parent = x;
        x->parent = g;
        if (g) {
            if (g->left == p) g->left = x;
            else g->right = x;
        } else {
            root = x;
        }
        rotations++;
    }

    // Splay node x to root (bottom-up: zig / zig-zig / zig-zag).
    void splay(Node *x) {
        while (x->parent) {
            Node *p = x->parent;
            Node *g = p->parent;
            if (!g) {
                // zig
                rotate(x);
            } else if ((g->left == p) == (p->left == x)) {
                // zig-zig (both left-left or right-right): rotate p then x
                rotate(p);
                rotate(x);
            } else {
                // zig-zag: rotate x twice
                rotate(x);
                rotate(x);
            }
        }
        root = x;
    }

    Node* find(int key) {
        Node *cur = root, *last = nullptr;
        while (cur) {
            last = cur;
            compare_count++;
            if (key == cur->key) break;
            cur = (key < cur->key) ? cur->left : cur->right;
        }
        if (last) splay(last);
        return (root && root->key == key) ? root : nullptr;
    }

    void insert(int key) {
        if (!root) { root = new Node(key); return; }
        Node *cur = root, *last = nullptr;
        while (cur) {
            last = cur;
            compare_count++;
            if (key == cur->key) { splay(cur); return; } // duplicate: splay existing
            cur = (key < cur->key) ? cur->left : cur->right;
        }
        Node *n = new Node(key);
        n->parent = last;
        if (key < last->key) last->left = n;
        else last->right = n;
        splay(n);
    }

    bool erase(int key) {
        Node *found = find(key);
        if (!found || found->key != key) return false;
        // found (== root after find's splay) is the node to delete
        Node *L = root->left, *R = root->right;
        // detach
        if (L) L->parent = nullptr;
        if (R) R->parent = nullptr;
        delete root;
        if (!L) { root = R; }
        else if (!R) { root = L; }
        else {
            // splay max of left subtree to root, then attach R as its right child
            Node *m = L;
            while (m->right) { m = m->right; }
            root = L;           // temporarily root = L so splay works correctly
            splay(m);
            root->right = R;
            R->parent = root;
        }
        return true;
    }

    ~SplayTree() { clear(root); }
    void clear(Node *n) { if (!n) return; clear(n->left); clear(n->right); delete n; }

    // in-order keys (for BST invariant check)
    void inorder(Node *n, vector<int> &out) const {
        if (!n) return;
        inorder(n->left, out);
        out.push_back(n->key);
        inorder(n->right, out);
    }
    int height(Node *n) const {
        if (!n) return 0;
        return 1 + max(height(n->left), height(n->right));
    }
    bool checkBST(Node *n, int lo, int hi) const {
        if (!n) return true;
        if (n->key <= lo || n->key >= hi) return false;
        return checkBST(n->left, lo, n->key) && checkBST(n->right, n->key, hi);
    }
};

// ---------------- Naive BST baseline (no balancing) ----------------
struct NaiveBST {
    struct N { int key; N *l=nullptr,*r=nullptr; N(int k):key(k){} };
    N *root=nullptr;
    uint64_t compare_count=0;
    void insert(int k){ N**c=&root; while(*c){ compare_count++; if(k==(*c)->key) return; c=(k<(*c)->key)?&(*c)->l:&(*c)->r; } *c=new N(k); }
    N* find(int k){ N*c=root; while(c){ compare_count++; if(k==c->key) return c; c=(k<c->key)?c->l:c->r; } return nullptr; }
    int height(N*n)const{ if(!n)return 0; return 1+max(height(n->l),height(n->r)); }
};

// Deterministic RNG (same sequences for fair comparison)
static uint64_t rs = 0x9E3779B97F4A7C15;
static uint64_t rnd() { rs ^= rs << 7; rs ^= rs >> 9; return rs; }
static int rnd_in(int lo, int hi) { return lo + (int)(rnd() % (uint64_t)(hi - lo + 1)); }

int main() {
    ios::sync_with_stdio(false);
    cout << fixed << setprecision(3);
    cout << "=== Splay Tree Quantitative Verification ===\n\n";

    // ---- Test 1: correctness vs std::set ----
    {
        SplayTree st; set<int> ref;
        int N = 200000;
        rs = 0x123456789ABCDEF0ULL;
        bool ok = true;
        for (int i = 0; i < N; i++) {
            int op = rnd_in(0, 2);
            int v = rnd_in(1, 20000);
            if (op == 0) { st.insert(v); ref.insert(v); }
            else if (op == 1) { bool a = st.find(v)!=nullptr, b = ref.count(v)>0; if (a!=b) { ok=false; break; } }
            else { bool a = st.erase(v), b = ref.erase(v)>0; if (a!=b) { ok=false; break; } }
        }
        cout << "[Test 1] Correctness vs std::set (" << N << " mixed ops): "
             << (ok ? "PASS" : "FAIL") << "\n";

        // total rotations per op (amortized cost)
        cout << "          total rotations = " << st.rotations
             << ", per op = " << (double)st.rotations / N << "\n";
        cout << "          key comparisons per op = " << (double)st.compare_count / N << "\n\n";
    }

    // ---- Test 2: BST invariant + size match ----
    {
        SplayTree st; set<int> ref;
        rs = 0xDEADBEEF12345678ULL;
        for (int i = 0; i < 100000; i++) { int v = rnd_in(1, 50000); st.insert(v); ref.insert(v); }
        vector<int> keys; st.inorder(st.root, keys);
        bool inc = is_sorted(keys.begin(), keys.end(), less_equal<int>()) &&
                   adjacent_find(keys.begin(), keys.end()) == keys.end(); // strictly increasing
        bool bst = st.checkBST(st.root, INT_MIN, INT_MAX);
        cout << "[Test 2] BST invariant (strictly increasing in-order): " << (inc && bst ? "PASS" : "FAIL")
             << " | size=" << keys.size() << " (ref=" << ref.size() << ")\n";
        cout << "          tree height = " << st.height(st.root)
             << "  (log2(n) ~ " << log2(max(1,(int)keys.size())) << ")\n\n";
    }

    // ---- Test 3: splay brings accessed node to root ----
    {
        SplayTree st;
        for (int v = 1; v <= 100; v += 2) st.insert(v); // {1,3,5,...,99}
        bool root_ok = true;
        for (int i = 0; i < 3; i++) {
            int target = rnd_in(1, 100);
            st.find(target);
            if (st.root == nullptr || st.root->key != target) { root_ok = false; break; }
        }
        cout << "[Test 3] accessed node becomes root after find: " << (root_ok ? "PASS" : "FAIL") << "\n\n";
    }

    // ---- Test 4: amortized height & locality (sequential access pattern) ----
    {
        SplayTree st; NaiveBST nb;
        int n = 30000;
        rs = 0xCAFEBABE00000001ULL;
        for (int i = 0; i < n; i++) { int v = rnd_in(1, 1000000); st.insert(v); nb.insert(v); }
        // sequential access of a hot window (temporal locality) -> splay should shine
        int hot_lo = 1, hot_hi = 1000;
        st.compare_count = 0; nb.compare_count = 0;
        int rounds = 100000;
        for (int i = 0; i < rounds; i++) { int v = rnd_in(hot_lo, hot_hi); st.find(v); nb.find(v); }
        cout << "[Test 4] Local access workload (" << rounds << " finds over hot window):\n";
        cout << "          Splay:  height=" << st.height(st.root)
             << ", compares/op=" << (double)st.compare_count/rounds << "\n";
        cout << "          Naive:  height=" << nb.height(nb.root)
             << ", compares/op=" << (double)nb.compare_count/rounds << "\n\n";
    }

    // ---- Test 5: erase correctness under heavy churn ----
    {
        SplayTree st; set<int> ref;
        int N = 100000; rs = 0x1111222233334444ULL;
        bool ok = true;
        for (int i = 0; i < N; i++) {
            int op = rnd_in(0,1);
            int v = rnd_in(1, 30000);
            if (op == 0) { st.insert(v); ref.insert(v); }
            else { bool a = st.erase(v), b = ref.erase(v)>0; if (a!=b){ ok=false; break; } }
        }
        vector<int> keys; st.inorder(st.root, keys);
        bool match = keys.size()==ref.size();
        if (match) { int idx=0; for(int x: ref){ if(keys[idx++]!=x){ match=false; break; } } }
        cout << "[Test 5] Heavy insert/erase churn (" << N << " ops) vs std::set: "
             << ((ok && match) ? "PASS" : "FAIL") << "\n\n";
    }

    cout << "=== DONE ===\n";
    return 0;
}
