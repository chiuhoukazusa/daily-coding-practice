// FFT (Cooley-Tukey) + 量化验证
// 实现：迭代式 Radix-2 Cooley-Tukey FFT（位逆序 + 蝴蝶运算）
// 验证项（全部量化，不靠眼睛）：
//  1. FFT vs 朴素 DFT 逐点误差（随机信号 & 确定性信号）
//  2. IFFT(FFT(x)) == x 往返重建误差
//  3. Parseval 定理：时域能量 == 频域能量（能量守恒）
//  4. FFT 线性性：FFT(a)+FFT(b) == FFT(a+b)
//  5. 圆卷积定理：FFT 卷积 == 时域循环卷积
//  6. 已知频谱验证：纯正弦波的频谱尖峰位置与幅度
//  7. 性能：N=1024 vs N=65536 的时间复杂度缩放

#include <complex>
#include <cmath>
#include <cstdio>
#include <vector>
#include <random>
#include <chrono>
#include <numeric>

using cd = std::complex<double>;
static const double PI = 3.14159265358979323846;

// ---------- 位逆序置换 ----------
static int reverseBits(int x, int log2n) {
    int r = 0;
    for (int i = 0; i < log2n; i++) {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    return r;
}

// ---------- 迭代式 Cooley-Tukey FFT ----------
// invert=false: 正变换; invert=true: 逆变换（不除 N）
static void fft(std::vector<cd>& a, bool invert) {
    int n = (int)a.size();
    int log2n = 0;
    while ((1 << log2n) < n) log2n++;

    // 位逆序置换
    for (int i = 0; i < n; i++) {
        int j = reverseBits(i, log2n);
        if (i < j) std::swap(a[i], a[j]);
    }

    // 蝴蝶运算
    for (int len = 2; len <= n; len <<= 1) {
        double ang = 2 * PI / len * (invert ? 1 : -1);
        cd wlen(std::cos(ang), std::sin(ang));
        for (int i = 0; i < n; i += len) {
            cd w(1, 0);
            for (int j = 0; j < len / 2; j++) {
                cd u = a[i + j];
                cd v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    if (invert) {
        for (auto& x : a) x /= n;
    }
}

// ---------- 朴素 DFT（基准） ----------
static std::vector<cd> dft(const std::vector<cd>& a, bool invert) {
    int n = (int)a.size();
    std::vector<cd> out(n);
    double sign = invert ? 1 : -1;
    for (int k = 0; k < n; k++) {
        cd sum(0, 0);
        for (int t = 0; t < n; t++) {
            double ang = sign * 2 * PI * k * t / n;
            sum += a[t] * cd(std::cos(ang), std::sin(ang));
        }
        out[k] = invert ? sum / (double)n : sum;
    }
    return out;
}

// ---------- 最大逐点误差 ----------
static double maxPointError(const std::vector<cd>& x, const std::vector<cd>& y) {
    double m = 0;
    for (int i = 0; i < (int)x.size(); i++) m = std::max(m, std::abs(x[i] - y[i]));
    return m;
}

int main() {
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    std::vector<std::string> results;
    bool allPass = true;
    auto check = [&](bool ok, const std::string& name, const std::string& detail) {
        results.push_back((ok ? "PASS" : "FAIL") + std::string(" | ") + name + " | " + detail);
        if (!ok) allPass = false;
    };

    // ========== 1. FFT vs 朴素 DFT ==========
    {
        const int N = 64;
        std::vector<cd> x(N);
        for (auto& v : x) v = cd(dist(rng), dist(rng));
        std::vector<cd> f = x; fft(f, false);
        std::vector<cd> d = dft(x, false);
        double err = maxPointError(f, d);
        check(err < 1e-9, "FFT vs DFT 逐点误差",
              "max_err=" + std::to_string(err) + " (需 < 1e-9)");
    }

    // ========== 2. IFFT(FFT(x)) == x ==========
    {
        const int N = 256;
        std::vector<cd> x(N);
        for (auto& v : x) v = cd(dist(rng), dist(rng));
        std::vector<cd> y = x; fft(y, false); fft(y, true);
        double err = maxPointError(x, y);
        check(err < 1e-9, "IFFT(FFT(x)) 往返重建",
              "max_err=" + std::to_string(err) + " (需 < 1e-9)");
    }

    // ========== 3. Parseval 能量守恒 ==========
    {
        const int N = 1024;
        std::vector<cd> x(N);
        double tE = 0;
        for (auto& v : x) { v = cd(dist(rng), dist(rng)); tE += std::norm(v); }
        std::vector<cd> f = x; fft(f, false);
        double fE = 0;
        for (auto& v : f) fE += std::norm(v);
        // Parseval: sum|x|^2 == (1/N) sum|X|^2
        double fScaled = fE / N;
        double rel = std::abs(tE - fScaled) / tE;
        check(rel < 1e-9, "Parseval 能量守恒",
              "time_E=" + std::to_string(tE) + " freq_E/N=" + std::to_string(fScaled) + " rel_err=" + std::to_string(rel));
    }

    // ========== 4. 线性性：FFT(a)+FFT(b) == FFT(a+b) ==========
    {
        const int N = 128;
        std::vector<cd> a(N), b(N), s(N);
        for (int i = 0; i < N; i++) { a[i] = cd(dist(rng), dist(rng)); b[i] = cd(dist(rng), dist(rng)); s[i] = a[i] + b[i]; }
        fft(a, false); fft(b, false); fft(s, false);
        std::vector<cd> sum(N);
        for (int i = 0; i < N; i++) sum[i] = a[i] + b[i];
        double err = maxPointError(sum, s);
        check(err < 1e-9, "FFT 线性性 FFT(a)+FFT(b)=FFT(a+b)",
              "max_err=" + std::to_string(err));
    }

    // ========== 5. 圆卷积定理 ==========
    {
        const int N = 128;
        std::vector<cd> a(N), b(N);
        for (int i = 0; i < N; i++) { a[i] = cd(dist(rng), 0); b[i] = cd(dist(rng), 0); }
        // 时域循环卷积（直接 O(N^2)）
        std::vector<cd> cDirect(N);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                cDirect[i] += a[j] * b[((i - j) % N + N) % N];
        // 频域：IFFT(FFT(a) * FFT(b))
        std::vector<cd> fa = a, fb = b;
        fft(fa, false); fft(fb, false);
        std::vector<cd> fc(N);
        for (int i = 0; i < N; i++) fc[i] = fa[i] * fb[i];
        fft(fc, true);
        double err = maxPointError(fc, cDirect);
        check(err < 1e-6, "圆卷积定理 FFT卷积 == 时域循环卷积",
              "max_err=" + std::to_string(err));
    }

    // ========== 6. 纯正弦波频谱尖峰 ==========
    {
        const int N = 512;
        double freq = 16.0; // 在第 16 个 bin 处有尖峰
        double amp = 3.0;
        std::vector<cd> x(N);
        for (int i = 0; i < N; i++) x[i] = cd(amp * std::cos(2 * PI * freq * i / N), 0);
        std::vector<cd> f = x; fft(f, false);
        // 期望：在 k=16 和 k=N-16 处幅度各为 (amp*N/2)
        double expected = amp * N / 2.0;
        double valPos = std::abs(f[(int)freq]);
        double valNeg = std::abs(f[N - (int)freq]);
        // 其它 bin 应接近 0
        double maxOther = 0;
        for (int k = 0; k < N; k++) {
            if (k == (int)freq || k == N - (int)freq) continue;
            maxOther = std::max(maxOther, std::abs(f[k]));
        }
        check(std::abs(valPos - expected) / expected < 1e-9 && std::abs(valNeg - expected) / expected < 1e-9,
              "正弦波频谱尖峰位置与幅度",
              "bin16=" + std::to_string(valPos) + " bin496=" + std::to_string(valNeg) + " expected=" + std::to_string(expected) + " max_other=" + std::to_string(maxOther));
        check(maxOther < 1e-9, "正弦波旁瓣抑制",
              "max_other_bin=" + std::to_string(maxOther) + " (需 ~0)");
    }

    // ========== 7. 性能 & 复杂度缩放 O(N log N) ==========
    {
        auto bench = [&](int N) {
            std::vector<cd> x(N);
            for (auto& v : x) v = cd(dist(rng), dist(rng));
            auto t0 = std::chrono::high_resolution_clock::now();
            fft(x, false);
            auto t1 = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::micro>(t1 - t0).count();
        };
        int N1 = 1024, N2 = 65536;
        // warmup
        bench(128);
        double t1 = bench(N1);
        double t2 = bench(N2);
        // O(N log N) 理论比: (N2 log2 N2) / (N1 log2 N1) = (65536*16)/(1024*10)
        double theory = (double)(N2 * 16) / (double)(N1 * 10);
        double actual = t2 / t1;
        check(actual < theory * 3.0, "时间复杂度 O(N log N) 缩放",
              "t(1024)=" + std::to_string(t1) + "us t(65536)=" + std::to_string(t2) + "us ratio=" + std::to_string(actual) + " theory=" + std::to_string(theory) + " (需 < 3x theory)");
    }

    // ========== 输出 ==========
    printf("=== FFT (Cooley-Tukey) 量化验证结果 ===\n\n");
    for (auto& r : results) printf("%s\n", r.c_str());
    printf("\n=== 总结果: %s ===\n", allPass ? "ALL PASS" : "SOME FAILED");

    FILE* fp = fopen("fft_output.txt", "w");
    for (auto& r : results) fprintf(fp, "%s\n", r.c_str());
    fprintf(fp, "\nTOTAL: %s\n", allPass ? "ALL PASS" : "SOME FAILED");
    fclose(fp);

    return allPass ? 0 : 1;
}
