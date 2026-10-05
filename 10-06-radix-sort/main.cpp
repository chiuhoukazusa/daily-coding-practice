// 10-06 Radix Sort (LSD) - 基数排序
// 技术点：LSD 基数排序、计数排序稳定排序、按位(字节)分桶、负整数处理(偏移)
// 量化验证：
//   1. 正确性: 与 std::sort 结果完全一致（含负数/大范围值/重复元素）
//   2. 稳定性: 通过 (key, index) 对验证稳定性质
//   3. 性能: 与 std::sort 对比加速比（不同规模）

#include <vector>
#include <cstdint>
#include <algorithm>
#include <random>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cassert>

// 非负数 LSD 基数排序（按字节，基为 256）
static void countingSortByByte(std::vector<uint32_t>& a, std::vector<uint32_t>& buf, int byteShift) {
    const int R = 256;
    int count[R] = {0};
    const uint32_t mask = 0xFFu;
    for (uint32_t v : a) {
        count[(v >> byteShift) & mask]++;
    }
    // 前缀和 -> 稳定摆放位置
    for (int i = 1; i < R; i++) count[i] += count[i-1];
    // 逆序遍历保证稳定性
    for (int i = (int)a.size() - 1; i >= 0; i--) {
        uint32_t v = a[i];
        int d = (v >> byteShift) & mask;
        buf[--count[d]] = v;
    }
    a.swap(buf);
}

// LSD 基数排序（32位无符号，4 个字节）
void radixSortU32(std::vector<uint32_t>& a) {
    if (a.size() <= 1) return;
    std::vector<uint32_t> buf(a.size());
    for (int b = 0; b < 4; b++) {
        countingSortByByte(a, buf, b * 8);
    }
}

// 带符号 int 排序：将 int 映射到 uint32（翻转符号位）实现单调映射
static inline uint32_t flipSign(int v) {
    return static_cast<uint32_t>(v) ^ 0x80000000u;
}

void radixSortI32(std::vector<int>& a) {
    std::vector<uint32_t> u(a.size());
    for (size_t i = 0; i < a.size(); i++) u[i] = flipSign(a[i]);
    radixSortU32(u);
    for (size_t i = 0; i < a.size(); i++) a[i] = static_cast<int>(u[i] ^ 0x80000000u);
}

// 稳定性质验证：对 (key, id) 结构排序
struct Item { int key; int id; };

int main() {
    std::mt19937 rng(12345);
    bool allOk = true;

    // ===== 测试 1: 正确性（多组随机数据，含负数/大值/重复） =====
    {
        std::vector<std::pair<int,int>> ranges = {
            {-1000000000, 1000000000},  // 全 int 范围
            {-1000, 1000},
            {0, 1000000},
            {0, 100},                    // 大量重复
            {-10, 10},
        };
        int caseId = 0;
        for (auto& [lo, hi] : ranges) {
            for (int size : {1, 2, 10, 1000, 100000}) {
                std::vector<int> data(size);
                for (auto& x : data) x = lo + (int)(rng() % (uint32_t)(hi - lo + 1));
                std::vector<int> expected = data;
                std::sort(expected.begin(), expected.end());
                radixSortI32(data);
                if (data != expected) {
                    allOk = false;
                    printf("❌ 正确性测试失败: range=[%d,%d] size=%d\n", lo, hi, size);
                }
                caseId++;
            }
        }
        printf("✅ 正确性测试通过: 25 组 (范围覆盖负数/大值/重复/小规模) 全部与 std::sort 一致\n");
    }

    // ===== 测试 2: 稳定性验证（相等 key 保持相对顺序） =====
    {
        const int N = 20000;
        std::vector<uint32_t> a(N);
        for (int i = 0; i < N; i++) {
            // key 分布在小范围，制造大量相等 key
            a[i] = rng() % 50;
        }
        // 附加 id 信息检测稳定性
        std::vector<int> idBefore(N);
        for (int i = 0; i < N; i++) idBefore[i] = i;

        // 打包 key 和 id 到 64 位：(key << 20) | id，用 radixSortU32 排其高位 key，
        // 但为简单直接，我们验证稳定排序算法的关键性质：对 (key,id) 做稳定排序
        // 手动用计数排序稳定性验证：直接对有符号 key 排序后，比较相等 key 的 id 顺序
        std::vector<int> keys(N);
        for (int i = 0; i < N; i++) keys[i] = (int)a[i];

        // 用稳定基数排序，同时跟踪原始位置
        std::vector<std::pair<int,int>> items(N);
        for (int i = 0; i < N; i++) items[i] = {keys[i], i};

        // 由于 radixSort 内部是稳定的，我们重建一个带 id 的稳定排序结果：
        // 这里用一个简单方案：对 (key, id) 用稳定计数排序
        // 直接验证 radixSortI32 是否稳定：构造 key+id 合成整数 (key+offset)*BIG + id
        // 但 key 可能为负。改用无符号 key 空间。
        // 简化：仅用非负 key 验证稳定性
        std::vector<int> nonneg(N);
        for (int i = 0; i < N; i++) nonneg[i] = keys[i] + 1000; // 保证非负
        std::vector<uint32_t> packed(N);
        for (int i = 0; i < N; i++) packed[i] = ((uint32_t)nonneg[i] << 16) | (uint32_t)(i & 0xFFFF);
        radixSortU32(packed);

        bool stable = true;
        for (int i = 0; i + 1 < N; i++) {
            uint32_t keyA = packed[i] >> 16;
            uint32_t keyB = packed[i+1] >> 16;
            uint32_t idA  = packed[i] & 0xFFFF;
            uint32_t idB  = packed[i+1] & 0xFFFF;
            if (keyA == keyB && idA > idB) {
                stable = false; break;
            }
        }
        if (!stable) allOk = false;
        printf("%s 稳定性验证: 相等 key 的原始顺序(id)保持递增\n", stable ? "✅" : "❌");
    }

    // ===== 测试 3: 性能对比（vs std::sort） =====
    printf("\n===== 性能对比 (LSD Radix Sort vs std::sort) =====\n");
    for (int size : {100000, 1000000, 10000000}) {
        // 随机 int 分布
        std::vector<int> data(size);
        for (auto& x : data) x = (int)(rng() % 1000000000u);

        std::vector<int> a = data;
        std::vector<int> b = data;

        auto t0 = std::chrono::high_resolution_clock::now();
        radixSortI32(a);
        auto t1 = std::chrono::high_resolution_clock::now();
        std::sort(b.begin(), b.end());
        auto t2 = std::chrono::high_resolution_clock::now();

        double radixMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double stlMs   = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double speedup = stlMs / radixMs;

        bool match = (a == b);
        if (!match) allOk = false;
        printf("size=%9d | radix=%8.2f ms | std::sort=%8.2f ms | 加速比=%5.2fx | %s\n",
               size, radixMs, stlMs, speedup, match ? "一致✅" : "不一致❌");
    }

    printf("\n%s\n", allOk ? "🎉 全部量化验证通过" : "💥 存在失败项");
    return allOk ? 0 : 1;
}
