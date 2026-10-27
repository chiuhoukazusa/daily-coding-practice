#include <bits/stdc++.h>
using namespace std;

// Count-Min Sketch: sub-linear space frequency estimation.
// Estimates frequency of items in a stream with a guaranteed error bound:
//   Pr[ est >= true ] = 1            (one-sided: never underestimates)
//   Pr[ est <= true + eps*N ] >= 1 - delta
// where N = sum of all frequencies, eps = e/width, delta = e^(-depth).

struct CountMinSketch {
    int width, depth;
    vector<vector<long long>> table;
    vector<uint64_t> seeds;
    mt19937_64 rng;

    CountMinSketch(int w, int d) : width(w), depth(d) {
        table.assign(depth, vector<long long>(width, 0));
        for (int i = 0; i < depth; i++) seeds.push_back(rng());
    }

    size_t idx(uint64_t x, int row) const {
        // SplitMix64-style 64-bit hash with per-row seed.
        uint64_t z = x + seeds[row];
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z = z ^ (z >> 31);
        return z % width;
    }

    void add(uint64_t x, long long cnt = 1) {
        for (int r = 0; r < depth; r++)
            table[r][idx(x, r)] += cnt;
    }

    long long estimate(uint64_t x) const {
        long long mn = LLONG_MAX;
        for (int r = 0; r < depth; r++)
            mn = min(mn, table[r][idx(x, r)]);
        return mn;
    }
};

int main() {
    const long long STREAM = 200000;   // total number of insertions
    const int DISTINCT = 5000;         // number of distinct items

    vector<pair<int,int>> configs = {
        {2000, 5},   //  eps=0.00136, delta=0.0067,  space=2000*5
        {500, 7},    //  eps=0.0054,  delta=0.0009,  space=500*7
        {160, 5},    //  tighter space -> bigger error
    };

    // Ground truth via exact unordered_map (stream generated deterministically).
    mt19937_64 gen(12345);
    uniform_int_distribution<uint64_t> item_dist(0, DISTINCT - 1);
    uniform_int_distribution<int> skewed(0, 99);

    // Generate a skewed distribution: item 0 most frequent, tail rare.
    // Simulate Zipf-ish: pick item via inverse of rank.
    auto sampleItem = [&]() -> uint64_t {
        // Zipf-ish over DISTINCT items using a biased sampler.
        // 90% mass on top ~10% items.
        if (gen() % 100 < 90) {
            return gen() % (DISTINCT / 10);          // hot set
        } else {
            return (DISTINCT / 10) + gen() % (DISTINCT - DISTINCT / 10);
        }
    };

    for (auto& [w, d] : configs) {
        CountMinSketch cms(w, d);
        unordered_map<uint64_t, long long> truth;

        for (long long i = 0; i < STREAM; i++) {
            uint64_t x = sampleItem();
            cms.add(x);
            truth[x]++;
        }

        long long totalErr = 0, totalOver = 0, totalUnder = 0;
        long long underCount = 0;
        long long maxAbsErr = 0;
        long long L1err = 0;
        long long sumTruth = 0;
        double epsTheoretical = exp(1.0) / w;
        double deltaTheoretical = exp(-d);
        long long epsN = (long long)(epsTheoretical * STREAM);

        for (auto& [x, trueVal] : truth) {
            long long est = cms.estimate(x);
            long long err = est - trueVal;
            totalErr += (err >= 0 ? err : -err);
            totalOver += max(0LL, err);
            if (err < 0) { totalUnder += -err; underCount++; }
            L1err += (err >= 0 ? err : -err);
            maxAbsErr = max(maxAbsErr, (err >= 0 ? err : -err));
            sumTruth += trueVal;
        }

        // Verification: fraction of items whose estimate exceeds eps*N guarantee.
        long long violations = 0;
        for (auto& [x, trueVal] : truth) {
            long long est = cms.estimate(x);
            if (est > trueVal + epsN) violations++;
        }
        double violationRate = (double)violations / truth.size();

        // Space comparison (bits): CMS vs exact map.
        // unordered_map node ~ key(8) + value(8) + hash overhead + ptr ~ 40+ bytes/entry.
        double cmsBits = (double)w * d * 8; // 8 bytes per counter (long long)
        double exactBits = (double)truth.size() * 48; // realistic per-entry overhead

        printf("config w=%d d=%d | eps_theory=%.4f (eps*N=%lld) delta_theory=%.4f\n",
               w, d, epsTheoretical, epsN, deltaTheoretical);
        printf("  items=%zu  total_stream=%lld\n", truth.size(), STREAM);
        printf("  mean_abs_err=%.1f  max_abs_err=%lld  L1_err=%lld\n",
               (double)totalErr / truth.size(), maxAbsErr, L1err);
        printf("  underestimate_count=%lld (should be 0 in theory) total_under=%lld\n",
               underCount, totalUnder);
        printf("  guarantee_violations=%lld/%.0f%% of items exceed eps*N\n",
               violations, violationRate * 100);
        printf("  space: cms=%lld bits (%.1f KB), exact=%lld bits (%.1f KB), ratio=%.1fx\n\n",
               (long long)cmsBits, cmsBits / 8192.0,
               (long long)exactBits, exactBits / 8192.0,
               exactBits / cmsBits);
    }

    return 0;
}
