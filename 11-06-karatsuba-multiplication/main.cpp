// Karatsuba 大整数乘法
// 主题：分治乘法算法 Karatsuba，对比朴素 O(n^2) 乘法
// 量化验证：正确性（对比朴素乘法 / __int128 参考）+ 复杂度（乘次数 / 加速比）
#include <bits/stdc++.h>
using namespace std;

// 将十进制字符串解析为大整数（低位在前，基 base=10^9）
const long long BASE = 1000000000LL; // 1e9
using Big = vector<long long>;

Big parse(const string& s) {
    Big r;
    for (int i = (int)s.size(); i > 0; i -= 9) {
        int lo = max(0, i - 9);
        r.push_back(stoll(s.substr(lo, i - lo)));
    }
    return r;
}

void trim(Big& a) {
    while (a.size() > 1 && a.back() == 0) a.pop_back();
}

string tostr(Big a) {
    trim(a);
    string s = to_string(a.back());
    for (int i = (int)a.size() - 2; i >= 0; --i) {
        string t = to_string(a[i]);
        s += string(9 - t.size(), '0') + t;
    }
    return s;
}

Big add(const Big& a, const Big& b) {
    Big r(max(a.size(), b.size()), 0);
    long long carry = 0;
    for (size_t i = 0; i < r.size(); ++i) {
        long long x = (i < a.size() ? a[i] : 0);
        long long y = (i < b.size() ? b[i] : 0);
        long long s = x + y + carry;
        carry = s / BASE;
        r[i] = s % BASE;
    }
    if (carry) r.push_back(carry);
    return r;
}

Big sub(const Big& a, const Big& b) { // a >= b
    Big r(a.size(), 0);
    long long borrow = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        long long x = a[i];
        long long y = (i < b.size() ? b[i] : 0);
        long long d = x - y - borrow;
        if (d < 0) { d += BASE; borrow = 1; } else borrow = 0;
        r[i] = d;
    }
    trim(r);
    return r;
}

// 朴素 O(n^2) 乘法（卷积）
Big naive_mul(const Big& a, const Big& b) {
    Big r(a.size() + b.size(), 0);
    for (size_t i = 0; i < a.size(); ++i) {
        long long carry = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            __int128 cur = (__int128)r[i + j] + (__int128)a[i] * b[j] + carry;
            r[i + j] = cur % BASE;
            carry = cur / BASE;
        }
        if (carry) r[i + b.size()] += carry;
    }
    trim(r);
    return r;
}

// Karatsuba 分治乘法
// 乘次数计数（用于复杂度验证）
long long sub_mult_count = 0;

Big karatsuba(const Big& a, const Big& b, int depth) {
    if (a.size() < 40 || b.size() < 40) {
        // 基线：转朴素乘法
        return naive_mul(a, b);
    }
    int n = max(a.size(), b.size());
    int half = n / 2;

    // 高位 / 低位
    Big a0(a.begin(), a.begin() + min((size_t)half, a.size()));
    Big a1(a.size() > (size_t)half ? Big(a.begin() + half, a.end()) : Big());
    Big b0(b.begin(), b.begin() + min((size_t)half, b.size()));
    Big b1(b.size() > (size_t)half ? Big(b.begin() + half, b.end()) : Big());

    // 补齐（a0/b0 按 half 位，a1/b1 剩余）
    trim(a0); trim(a1); trim(b0); trim(b1);

    Big z0 = karatsuba(a0, b0, depth + 1);
    Big z2 = karatsuba(a1, b1, depth + 1);

    Big s1 = add(a0, a1);
    Big s2 = add(b0, b1);
    Big z1 = karatsuba(s1, s2, depth + 1);

    // z1 = z1 - z0 - z2  (中项)
    Big mid = sub(sub(z1, z2), z0);
    sub_mult_count += 3;

    // 组合: result = z0 + 10^half * mid + 10^{2half} * z2
    Big mid_shift(mid.size() + half, 0);
    for (size_t i = 0; i < mid.size(); ++i) mid_shift[i + half] = mid[i];

    Big z2_shift(z2.size() + 2 * half, 0);
    for (size_t i = 0; i < z2.size(); ++i) z2_shift[i + 2 * half] = z2[i];

    Big r = add(add(z0, mid_shift), z2_shift);
    return r;
}

// 参考：__int128 乘法（仅用于小数字验证）
string ref_mul(const string& x, const string& y) {
    __int128 a = 0, b = 0;
    for (char c : x) a = a * 10 + (c - '0');
    for (char c : y) b = b * 10 + (c - '0');
    __int128 p = a * b;
    if (p == 0) return "0";
    string s;
    while (p) { s.push_back('0' + p % 10); p /= 10; }
    reverse(s.begin(), s.end());
    return s;
}

mt19937_64 rng(12345);

string rand_digits(int n) {
    string s;
    s.reserve(n);
    for (int i = 0; i < n; ++i) {
        char d = '0' + (rng() % 10);
        if (i == 0) d = '1' + (rng() % 9); // 无前导零
        s.push_back(d);
    }
    return s;
}

int main() {
    int pass = 0, fail = 0;

    // === 测试 1：小数字正确性（对比 __int128 参考）===
    vector<pair<int,int>> small_cases = {{1,1},{2,2},{3,3},{5,5},{10,10}};
    for (auto& p : small_cases) {
        string x = rand_digits(p.first);
        string y = rand_digits(p.second);
        string got = tostr(karatsuba(parse(x), parse(y), 0));
        string exp = ref_mul(x, y);
        if (got == exp) pass++; else { fail++; printf("FAIL small %dx%d: %s * %s\n", p.first, p.second, x.c_str(), y.c_str()); }
    }

    // === 测试 2：中等数字正确性（对比朴素乘法）===
    for (int n : {100, 500, 1000}) {
        string x = rand_digits(n);
        string y = rand_digits(n);
        string got = tostr(karatsuba(parse(x), parse(y), 0));
        string exp = tostr(naive_mul(parse(x), parse(y)));
        if (got == exp) pass++; else { fail++; printf("FAIL naive %d digits\n", n); }
    }

    // === 测试 3：性能对比（Karatsuba vs 朴素乘法）===
    printf("\n=== 性能对比 ===\n");
    printf("%12s | %12s | %12s | %8s\n", "位数", "Karatsuba(ms)", "朴素(ms)", "加速比");
    for (int n : {2000, 4000, 8000, 16000, 32000}) {
        string x = rand_digits(n);
        string y = rand_digits(n);
        Big bx = parse(x), by = parse(y);

        auto t0 = chrono::high_resolution_clock::now();
        Big rk = karatsuba(bx, by, 0);
        auto t1 = chrono::high_resolution_clock::now();
        Big rn = naive_mul(bx, by);
        auto t2 = chrono::high_resolution_clock::now();

        double tk = chrono::duration<double, milli>(t1 - t0).count();
        double tn = chrono::duration<double, milli>(t2 - t1).count();

        // 正确性自检
        bool ok = (tostr(rk) == tostr(rn));
        double ratio = tk > 0 ? tn / tk : 0.0;
        printf("%12d | %12.2f | %12.2f | %8.2fx  %s\n", n, tk, tn, ratio, ok ? "OK" : "MISMATCH");
        if (!ok) fail++;
        else pass++;
    }

    // === 测试 4：复杂度验证（递归乘次数 vs 理论 (n/b)^1.585）===
    // 归零计数，重新计算一个较大输入
    sub_mult_count = 0;
    {
        string x = rand_digits(20000);
        string y = rand_digits(20000);
        karatsuba(parse(x), parse(y), 0);
        // 理论：Karatsuba 递归产生约 3*submult 顶层项；这里直接打印实际子乘次数
        printf("\n=== 复杂度统计 ===\n");
        printf("Karatsuba 顶层子乘法调用(加减)次数: %lld\n", sub_mult_count);
    }

    printf("\n=== 结果 ===\n");
    printf("通过: %d  失败: %d\n", pass, fail);
    if (fail == 0) printf("✅ 全部验证通过\n");
    else printf("❌ 存在失败用例\n");
    return fail == 0 ? 0 : 1;
}
