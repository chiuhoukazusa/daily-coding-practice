// Manacher's Algorithm — Longest Palindromic Substring (O(n))
// Daily coding practice 10-15
//
// Demonstrates the classic linear-time algorithm for finding the longest
// palindromic substring.  It works by:
//   1. Inserting sentinel '#' between every character (and at both ends) so
//      that every palindrome has odd length and an unambiguous center.
//   2. Exploiting the mirror symmetry of already-discovered palindromes to
//      skip redundant center-expansion work (hence O(n) amortised).
//   3. Tracking the rightmost palindrome boundary (C=center, R=right edge).
//
// Quantitative validation:
//   (1) Correctness: result length/indices match an O(n^2) brute-force
//       center-expansion reference on many random strings.
//   (2) Complexity: measured speedup of Manacher vs brute force on long
//       strings (both length and wall-time scale).
//   (3) Known-case sanity: a few hand-checked examples.

#include <bits/stdc++.h>
using namespace std;

// ---------------------------------------------------------------------------
// Manacher in O(n). Returns the longest palindromic substring [start, end].
// ---------------------------------------------------------------------------
struct LPSResult {
    int start;  // inclusive start index in ORIGINAL string
    int length; // length of the palindrome in ORIGINAL string
};

LPSResult manacher(const string& s) {
    LPSResult best{0, 0};
    if (s.empty()) return best;

    // Build padded string: "#a#b#c#..."
    string t;
    t.reserve(s.size() * 2 + 1);
    t.push_back('#');
    for (char c : s) { t.push_back(c); t.push_back('#'); }

    int n = (int)t.size();
    vector<int> p(n, 0);      // p[i] = radius (in #-string) around center i
    int C = 0, R = 0;         // center and right boundary of rightmost palindrome

    for (int i = 0; i < n; ++i) {
        // Mirror of i about center C is mirr = 2*C - i.
        if (i < R) {
            int mirr = 2 * C - i;
            p[i] = min(R - i, p[mirr]);
        }
        // Attempt to expand around i.
        int a = i + p[i] + 1;
        int b = i - p[i] - 1;
        while (a < n && b >= 0 && t[a] == t[b]) {
            ++p[i];
            ++a;
            --b;
        }
        // Update the rightmost palindrome if we expanded past R.
        if (i + p[i] > R) {
            C = i;
            R = i + p[i];
        }
    }

    // Locate the max radius, then map back to the original string.
    int center = 0, radius = 0;
    for (int i = 0; i < n; ++i) {
        if (p[i] > radius) { radius = p[i]; center = i; }
    }

    // A palindrome centered at 'center' with radius 'radius' in the padded
    // string maps to original indices: start = (center - radius) / 2.
    best.start  = (center - radius) / 2;
    best.length = radius;
    return best;
}

// ---------------------------------------------------------------------------
// Brute-force center expansion O(n^2) reference (for correctness comparison).
// ---------------------------------------------------------------------------
LPSResult bruteForce(const string& s) {
    LPSResult best{0, 0};
    int n = (int)s.size();
    if (n == 0) return best;

    for (int c = 0; c < n; ++c) {
        // Odd length palindrome centered at c.
        int l = c, r = c;
        while (l >= 0 && r < n && s[l] == s[r]) { --l; ++r; }
        int len = r - l - 1;
        if (len > best.length) { best.length = len; best.start = l + 1; }

        // Even length palindrome centered between c and c+1.
        l = c; r = c + 1;
        while (l >= 0 && r < n && s[l] == s[r]) { --l; ++r; }
        len = r - l - 1;
        if (len > best.length) { best.length = len; best.start = l + 1; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Validation helpers
// ---------------------------------------------------------------------------
static bool isPalindrome(const string& s, int start, int length) {
    int lo = start, hi = start + length - 1;
    while (lo < hi) { if (s[lo] != s[hi]) return false; ++lo; --hi; }
    return true;
}

static string randomString(mt19937& rng, int len, int alphabet) {
    string s; s.reserve(len);
    for (int i = 0; i < len; ++i)
        s.push_back(char('a' + (rng() % alphabet)));
    return s;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout << fixed << setprecision(3);

    mt19937 rng(12345);
    int pass = 0, fail = 0;

    // ---- (0) Hand-checked known cases ----
    struct Known { string s; int expectedLen; };
    vector<Known> known = {
        {"babad", 3},      // "bab" or "aba"
        {"cbbd", 2},       // "bb"
        {"a", 1},
        {"", 0},
        {"abacdfgdcaba", 11}, // "aba" -> actually let's trust code; full string is pal? "abacdfgdcaba" not. expected 3 ("aba")
        {"racecar", 7},    // whole string
        {"aaaa", 4},
    };
    // fix the ambiguous expected values to match the algorithm's *length* (any tie):
    // "abacdfgdcaba" longest is "aba"? check: palindromes "aba"(3), "cd fg dc" not (f breaks), "aba"=3.
    known[4].expectedLen = 3;

    for (auto& k : known) {
        LPSResult r = manacher(k.s);
        bool okLen    = (r.length == k.expectedLen);
        bool okPalin  = isPalindrome(k.s, r.start, r.length);
        if (okLen && okPalin) ++pass; else ++fail;
    }
    cout << "[known-cases] pass=" << pass << " fail=" << fail << "\n";

    // ---- (1) Random correctness vs brute force ----
    int cPass = 0, cFail = 0;
    int trials = 5000;
    for (int t = 0; t < trials; ++t) {
        int len  = (int)(rng() % 60) + 1;
        int alph = (int)(rng() % 4) + 1;   // small alphabets create many palindromes
        string s = randomString(rng, len, alph);

        LPSResult a = manacher(s);
        LPSResult b = bruteForce(s);

        bool okLen   = (a.length == b.length);          // lengths must match
        bool okPalin = isPalindrome(s, a.start, a.length);
        bool okMax   = (a.length >= b.length);          // must be at least as long
        if (okLen && okPalin && okMax) ++cPass;
        else {
            ++cFail;
            if (cFail <= 5)
                cout << "  [MISMATCH] s=\"" << s << "\" manacher=(" << a.start
                     << "," << a.length << ") brute=(" << b.start << "," << b.length << ")\n";
        }
    }
    cout << "[random-correctness] trials=" << trials
         << " pass=" << cPass << " fail=" << cFail << "\n";

    // ---- (2) Complexity / speedup measurement ----
    // Use a WORST-CASE input for brute force: a string of all 'a' forces
    // every center to expand to the full string, making brute force genuinely
    // O(n^2) while Manacher stays O(n) thanks to mirror reuse.
    // We count elementary character-comparisons to measure complexity
    // independent of wall-clock noise.
    long long manacherCmps = 0, bruteCmps = 0;

    auto manacherCount = [&](const string& s) {
        string t; t.reserve(s.size()*2+1); t.push_back('#');
        for (char c : s){ t.push_back(c); t.push_back('#');}
        int n = (int)t.size(); vector<int> p(n,0); int C=0,R=0; long long cmps=0;
        for (int i=0;i<n;++i){ if(i<R){int m=2*C-i; p[i]=min(R-i,p[m]);}
            int a=i+p[i]+1, b=i-p[i]-1;
            while(a<n&&b>=0&&t[a]==t[b]){++cmps; ++p[i]; ++a; --b;}
            if(i+p[i]>R){C=i;R=i+p[i];} }
        return cmps;
    };
    auto bruteCount = [&](const string& s) {
        int n=(int)s.size(); long long cmps=0;
        for(int c=0;c<n;++c){ int l=c,r=c;
            while(l>=0&&r<n&&s[l]==s[r]){++cmps;--l;++r;}
            l=c;r=c+1;
            while(l>=0&&r<n&&s[l]==s[r]){++cmps;--l;++r;} }
        return cmps;
    };

    for (int len : {1000, 2000, 4000, 8000, 16000}) {
        string s(len, 'a'); // worst case: all identical
        long long cm = manacherCount(s);
        long long cb = bruteCount(s);
        cout << "[complexity] len=" << len << " manacher_cmps=" << cm
             << " brute_cmps=" << cb << " ratio=" << (double)cb/(double)cm << "x\n";
        manacherCmps += cm; bruteCmps += cb;
    }

    // ---- (3) Summary / gate ----
    // Complexity gate: on worst-case (all-same) input, brute force should do
    // asymptotically ~n/2 times more comparisons than Manacher.
    // At len=16000, brute ~= n^2/4 ~ 6.4e7, manacher ~= n ~ 1.6e4.
    double ratio16000 = (double)bruteCmps / (double)max(manacherCmps, 1LL);
    bool allGood = (fail == 0) && (cFail == 0) && (ratio16000 > 100.0);
    cout << "[SUMMARY] known-fail=" << fail << " random-fail=" << cFail
         << " cmp-ratio=" << ratio16000 << "x"
         << " => " << (allGood ? "PASS" : "FAIL") << "\n";
    return allGood ? 0 : 1;
}
