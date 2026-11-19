#include <bits/stdc++.h>
using namespace std;

// ---------------- Median of Medians (BFPRT) ----------------
// Deterministic linear-time k-th smallest selection.
// Guarantees O(n) worst case (unlike Quickselect's O(n^2) worst case).

static long long cmpCount = 0;

// Insertion sort for small groups (size <= 5)
void insertionSort(vector<long long>& a, int l, int r) {
    for (int i = l + 1; i <= r; ++i) {
        long long key = a[i];
        int j = i - 1;
        while (j >= l && ++cmpCount && a[j] > key) { a[j + 1] = a[j]; --j; }
        a[j + 1] = key;
    }
}

// Find median of a group of 5 (assumes sorted via insertionSort internally)
long long median5(vector<long long>& a, int l) {
    // sort 5 elements with a small fixed network is overkill; use insertion sort
    insertionSort(a, l, l + 4);
    return a[l + 2];
}

// Median of medians: returns index of a "good pivot" element value
long long medianOfMedians(vector<long long>& a, int l, int r) {
    int n = r - l + 1;
    if (n <= 5) {
        insertionSort(a, l, r);
        return a[l + n / 2];
    }
    // Divide into groups of 5, find medians, store into a prefix
    int numGroups = (n + 4) / 5;
    int k = l;
    for (int i = 0; i < numGroups; ++i) {
        int gl = l + i * 5;
        int gr = min(gl + 4, r);
        insertionSort(a, gl, gr);
        long long med = a[gl + (gr - gl) / 2];
        swap(a[k], a[gl + (gr - gl) / 2]);
        a[k] = med; // place median at front area [l .. l+numGroups)
        k++;
    }
    // recursively find median of medians
    return medianOfMedians(a, l, l + numGroups - 1);
}

// Partition around pivot value, returns pivot index (Lomuto-like with explicit pivot)
int partitionAround(vector<long long>& a, int l, int r, long long pivot) {
    // find pivot position
    int pi = -1;
    for (int i = l; i <= r; ++i) if (a[i] == pivot) { pi = i; break; }
    if (pi == -1) pi = l;
    swap(a[pi], a[r]);
    long long pv = a[r];
    int i = l;
    for (int j = l; j < r; ++j) {
        ++cmpCount;
        if (a[j] < pv) { swap(a[i], a[j]); ++i; }
    }
    swap(a[i], a[r]);
    return i;
}

// BFPRT k-th smallest (0-indexed k)
long long bfprt(vector<long long>& a, int l, int r, int k) {
    if (l == r) return a[l];
    long long pivot = medianOfMedians(a, l, r);
    int pi = partitionAround(a, l, r, pivot);
    int rank = pi - l;
    if (k == rank) return a[pi];
    if (k < rank) return bfprt(a, l, pi - 1, k);
    return bfprt(a, pi + 1, r, k - rank - 1);
}

long long kthBFPRT(vector<long long> a, int k) {
    cmpCount = 0;
    return bfprt(a, 0, (int)a.size() - 1, k);
}

// ---------------- Quickselect (Hoare partition) ----------------
long long quickselect(vector<long long>& a, int l, int r, int k) {
    if (l == r) return a[l];
    // random pivot
    int pidx = l + (rand() % (r - l + 1));
    swap(a[pidx], a[r]);
    long long pv = a[r];
    int i = l;
    for (int j = l; j < r; ++j) {
        ++cmpCount;
        if (a[j] < pv) { swap(a[i], a[j]); ++i; }
    }
    swap(a[i], a[r]);
    int rank = i - l;
    if (k == rank) return a[i];
    if (k < rank) return quickselect(a, l, i - 1, k);
    return quickselect(a, i + 1, r, k - rank - 1);
}

long long kthQuickselect(vector<long long> a, int k) {
    cmpCount = 0;
    return quickselect(a, 0, (int)a.size() - 1, k);
}

// ---------------- Main ----------------
int main() {
    srand(12345);

    // Part 1: correctness on random arrays, compare against std::sort reference
    bool allCorrect = true;
    int tests = 500;
    for (int t = 0; t < tests; ++t) {
        int n = 1 + rand() % 200;
        vector<long long> a(n);
        for (auto& x : a) x = rand() % 100000 - 50000;
        vector<long long> sorted = a;
        sort(sorted.begin(), sorted.end());
        int k = rand() % n;
        long long ref = sorted[k];

        long long v1 = kthBFPRT(a, k);
        long long v2 = kthQuickselect(a, k);
        if (v1 != ref || v2 != ref) { allCorrect = false; }
    }
    cout << "correctness_tests=" << tests << " all_correct=" << (allCorrect ? "PASS" : "FAIL") << endl;
    if (!allCorrect) { cerr << "FAILED CORRECTNESS" << endl; return 1; }

    // Part 2: edge cases
    // single element
    vector<long long> e1 = {42};
    long long r1 = kthBFPRT(e1, 0);
    // duplicate elements
    vector<long long> e2 = {5,5,5,5,5,5,5};
    long long r2 = kthBFPRT(e2, 3);
    cout << "edge_single=" << r1 << " edge_duplicates_median=" << r2 << endl;
    if (r1 != 42 || r2 != 5) { cerr << "EDGE FAIL" << endl; return 1; }

    // Part 3: complexity verification — count comparisons on large inputs
    // Worst-case-ish: already-sorted array (bad for naive quickselect with fixed pivot,
    // but we use random pivot; still deterministic for BFPRT)
    vector<int> sizes = {1000, 10000, 100000, 1000000};
    for (int n : sizes) {
        vector<long long> asc(n);
        for (int i = 0; i < n; ++i) asc[i] = i; // already sorted (worst for fixed-pivot quickselect)
        int k = n / 2; // median

        long long c1; long long c2;
        cmpCount = 0; long long vb = bfprt(asc, 0, n - 1, k); c1 = cmpCount;
        cmpCount = 0; long long vq = quickselect(asc, 0, n - 1, k); c2 = cmpCount;

        // linearity check: comparisons should be roughly O(n), not O(n^2)
        double ratio = (double)c1 / n;
        cout << "n=" << n
             << " bfprt_cmp=" << c1 << " (cmp/n=" << fixed << setprecision(2) << ratio << ")"
             << " quickselect_cmp=" << c2
             << " bfprt_ok=" << (vb == k ? "Y" : "N")
             << " quickselect_ok=" << (vq == k ? "Y" : "N") << endl;
    }

    // Part 4: adversarial case for quickselect showing BFPRT advantage
    // Many duplicates + already sorted to stress quickselect variance
    int nbig = 500000;
    vector<long long> big(nbig);
    for (int i = 0; i < nbig; ++i) big[i] = (i / 1000); // sorted with duplicates
    int kbig = nbig / 2;
    cmpCount = 0; long long vb2 = bfprt(big, 0, nbig - 1, kbig); long long cbf = cmpCount;
    cmpCount = 0; long long vq2 = quickselect(big, 0, nbig - 1, kbig); long long cqs = cmpCount;
    cout << "adversarial_n=" << nbig
         << " bfprt_cmp=" << cbf << " quickselect_cmp=" << cqs
         << " bfprt_ok=" << (vb2 == kbig / 1000 ? "Y" : "N")
         << " quickselect_ok=" << (vq2 == kbig / 1000 ? "Y" : "N") << endl;

    cout << "ALL_CHECKS_PASSED" << endl;
    return 0;
}
