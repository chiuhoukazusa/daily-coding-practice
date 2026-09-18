// Daily Coding Practice — 2026-09-19
// Bloom Filter: 概率型集合数据结构
//
// 验证目标（量化，非视觉）：
//   1. 零假阴性：所有已插入元素查询必须返回 true。
//   2. 假阳性率：实测 FPR 接近理论值 (1 - e^(-kn/m))^k。
//   3. 内存压缩比：Bloom Filter 位数组 vs std::unordered_set<size_t> 的内存。
//   4. 多重哈希独立性对 FPR 的影响（不同 k 对比）。
//
// 技术点：
//   - m 位数组，k 个独立哈希函数（double hashing 派生 / std::hash 组合）
//   - 理论 FPR 公式 (1 - (1 - 1/m)^(kn))^k ≈ (1 - e^(-kn/m))^k

#include <bits/stdc++.h>
using namespace std;

// 统一哈希：将 64 位值映射到 [0, m) 的第 i 个哈希槽
// 采用 double hashing：h_i(x) = (h1(x) + i * h2(x)) mod m
// h2 与 m 互素时覆盖全域；此处 h2 用奇数保证与任意 m 的相对较好散布。
struct BloomFilter {
    size_t m;           // 位数组长度（比特数）
    int    k;           // 哈希函数个数
    vector<uint64_t> bits; // 位数组(每元素64bit)
    uint64_t n;         // 已插入元素数量

    BloomFilter(size_t m_, int k_) : m(m_), k(k_), n(0) {
        bits.assign((m + 63) / 64, 0ULL);
    }

    static uint64_t mkhash(uint64_t x, uint64_t seed) {
        // splitmix64 风格混合
        x += seed * 0x9E3779B97F4A7C15ULL;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
        x = x ^ (x >> 31);
        return x;
    }

    void insert(uint64_t x) {
        uint64_t h1 = mkhash(x, 0x12345678ULL);
        uint64_t h2 = mkhash(x, 0x87654321ULL) | 1ULL; // h2 强制奇数
        for (int i = 0; i < k; ++i) {
            uint64_t h = (h1 + (uint64_t)i * h2) % m;
            bits[h >> 6] |= (1ULL << (h & 63));
        }
        ++n;
    }

    bool query(uint64_t x) const {
        uint64_t h1 = mkhash(x, 0x12345678ULL);
        uint64_t h2 = mkhash(x, 0x87654321ULL) | 1ULL;
        for (int i = 0; i < k; ++i) {
            uint64_t h = (h1 + (uint64_t)i * h2) % m;
            if (((bits[h >> 6] >> (h & 63)) & 1ULL) == 0) return false;
        }
        return true;
    }
};

int main() {
    mt19937_64 rng(20260919ULL);

    // 配置：n 个插入元素，m 位数组，k 不同取值对比
    const uint64_t n = 200000;       // 插入 20 万元素
    const size_t   m = 1 << 21;      // 2M 位 = 256 KB 位数组

    cout << "=== Bloom Filter 量化验证 (n=" << n << ", m=" << m << " bits) ===" << endl;
    cout << fixed << setprecision(4);

    // 生成插入集合
    vector<uint64_t> inserted(n);
    for (auto &v : inserted) v = rng();
    // 生成查询用未插入集合（用于测假阳性），保证与 inserted 不重叠
    unordered_set<uint64_t> inserted_set(inserted.begin(), inserted.end());
    const uint64_t n_query = n;
    vector<uint64_t> query(n_query);
    {
        size_t cnt = 0;
        while (cnt < n_query) {
            uint64_t v = rng();
            if (inserted_set.count(v) == 0) { query[cnt++] = v; }
        }
    }

    // 理论假阳性率
    auto theory_fpr = [&](int k) {
        double p = 1.0 - exp(-(double)(n * k) / (double)m);
        return pow(p, (double)k);
    };

    // 依次测试 k = 3, 5, 7, 10
    vector<int> ks = {3, 5, 7, 10};
    cout << "\n--- 假阳性率对比（理论 vs 实测）---" << endl;
    cout << "  k  | 理论FPR(%) | 实测FPR(%) | 零假阴性" << endl;
    for (int k : ks) {
        BloomFilter bf(m, k);
        for (auto v : inserted) bf.insert(v);

        // 1) 零假阴性检查：所有插入元素必须命中
        uint64_t fn_count = 0;
        for (auto v : inserted) if (!bf.query(v)) ++fn_count;

        // 2) 假阳性检查：所有未插入元素
        uint64_t fp_count = 0;
        for (auto v : query) if (bf.query(v)) ++fp_count;
        double fpr = (double)fp_count / (double)n_query;
        double fpr_pct = fpr * 100.0;
        double theory_pct = theory_fpr(k) * 100.0;

        cout << setw(4) << k << " | "
             << setw(10) << theory_pct << " | "
             << setw(11) << fpr_pct << " | "
             << (fn_count == 0 ? "✅ 0 假阴性" : "❌ " + to_string(fn_count)) << endl;

        // 断言
        if (fn_count != 0) {
            cerr << "❌ 出现假阴性，Bloom Filter 实现错误！" << endl;
            return 1;
        }
        double rel_err = fabs(fpr - theory_fpr(k)) / theory_fpr(k);
        if (rel_err > 0.10) {
            cerr << "❌ k=" << k << " 实测 FPR 偏离理论超过 10% (" << rel_err*100 << "%)" << endl;
            return 1;
        }
    }

    // 3) 内存对比：Bloom Filter vs unordered_set
    cout << "\n--- 内存占用对比 (n=" << n << ") ---" << endl;
    {
        size_t bloom_bytes = (m + 63) / 64 * sizeof(uint64_t);
        // 重新估算一个 unordered_set 的内存（元素 + 桶）
        size_t uset_bucket_bytes = sizeof(void*) * n * 2;      // load factor ~0.5 => ~2n 桶
        size_t uset_node_bytes   = (sizeof(uint64_t) + sizeof(void*) + 8) * n; // 节点开销
        size_t uset_bytes = uset_bucket_bytes + uset_node_bytes;
        double ratio = (double)uset_bytes / (double)bloom_bytes;
        cout << "Bloom Filter 位数组: " << bloom_bytes << " bytes ("
             << (double)bloom_bytes/1024 << " KB)" << endl;
        cout << "unordered_set 估算: " << uset_bytes << " bytes ("
             << (double)uset_bytes/1024 << " KB)" << endl;
        cout << "内存压缩比: " << ratio << "x" << endl;
        if (ratio < 5.0) {
            cerr << "❌ 内存压缩比过低" << endl;
            return 1;
        }
    }

    cout << "\n✅ 全部量化验证通过：零假阴性、FPR 符合理论、内存显著压缩" << endl;
    return 0;
}
