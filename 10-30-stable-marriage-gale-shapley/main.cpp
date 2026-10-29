// Stable Marriage Problem — Gale-Shapley (Deferred Acceptance) Algorithm
//
// 目标：
//   1. 实现 Gale-Shapley 延迟接受算法，输出稳定匹配
//   2. 量化验证：
//      a) 匹配是「完美匹配」(每个男/女恰好一个伴侣)
//      b) 匹配是「稳定」的 (不存在阻滞对 blocking pair)
//      c) 提议方(男)最优性：每个男都得到其可达稳定伴侣中排名最高者
//      d) 与暴力枚举所有完美匹配、统计稳定匹配数量做基准对比
//
// 复杂度：Gale-Shapley 为 O(n^2)，暴力枚举为 O(n!)
//
// 编译：g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra

#include <bits/stdc++.h>
using namespace std;

struct GaleShapley {
    int n;                            // 男女数量
    vector<vector<int>> menPref;      // menPref[i] = 男 i 对女性的偏好（女性编号升序，优先往前）
    vector<vector<int>> womenRank;    // womenRank[w][m] = 女性 w 对男 m 的排名（越小越喜欢）
    vector<int> womenMatch;           // womenMatch[w] = 与女 w 匹配的男 (-1 未匹配)
    vector<int> menMatch;             // menMatch[m] = 与男 m 匹配的女 (-1 未匹配)

    // 传入女性偏好表（独立于男性偏好），据此构造女性排名表
    GaleShapley(int n_, const vector<vector<int>>& menPref_,
                const vector<vector<int>>& womenPref_)
        : n(n_), menPref(menPref_) {
        womenRank.assign(n, vector<int>(n, -1));
        for (int w = 0; w < n; ++w) {
            for (int r = 0; r < n; ++r) {
                womenRank[w][womenPref_[w][r]] = r;  // 女 w 眼里，男 womenPref_[w][r] 排名为 r
            }
        }
        womenMatch.assign(n, -1);
        menMatch.assign(n, -1);
    }

    void run() {
        // 每个男性维护"下一个要追求的偏好位置"
        vector<int> next(n, 0);
        queue<int> free;               // 自由男性队列
        for (int m = 0; m < n; ++m) free.push(m);

        while (!free.empty()) {
            int m = free.front(); free.pop();
            if (next[m] >= n) continue;         // 理论上不会发生，配对一定成功
            int w = menPref[m][next[m]++];      // 男 m 当前最想要且未拒绝他的女性

            if (womenMatch[w] == -1) {
                // 该女性单身，直接匹配
                womenMatch[w] = m;
                menMatch[m] = w;
            } else {
                int cur = womenMatch[w];        // 当前伴侣
                if (womenRank[w][m] < womenRank[w][cur]) {
                    // 女性更喜欢新来者 m，替换
                    womenMatch[w] = m;
                    menMatch[m] = w;
                    menMatch[cur] = -1;
                    free.push(cur);             // 被抛弃的男重新进入队列
                } else {
                    // 女性偏爱当前伴侣，拒绝 m
                    free.push(m);
                }
            }
        }
    }
};

// 检查匹配中 (m, w) 是否是阻滞对：双方都更偏爱对方甚于当前伴侣
bool isBlockingPair(int m, int w,
                    const vector<vector<int>>& menPref,
                    const vector<vector<int>>& womenRank,
                    const vector<int>& menMatch,
                    const vector<int>& womenMatch) {
    int m_cur = menMatch[m];       // 男 m 当前伴侣
    int w_cur = womenMatch[w];     // 女 w 当前伴侣

    // 男 m 是否更喜欢 w 甚于当前伴侣 m_cur
    auto rankOf = [](const vector<int>& pref, int x) -> int {
        for (int i = 0; i < (int)pref.size(); ++i) if (pref[i] == x) return i;
        return INT_MAX;
    };
    bool mPrefersW = (m_cur == -1) || (rankOf(menPref[m], w) < rankOf(menPref[m], m_cur));
    bool wPrefersM = (w_cur == -1) || (womenRank[w][m] < womenRank[w][w_cur]);
    return mPrefersW && wPrefersM;
}

// 检查匹配是否稳定（无阻滞对） + 是否完美匹配
bool isStableAndPerfect(int n,
                        const vector<vector<int>>& menPref,
                        const vector<vector<int>>& womenRank,
                        const vector<int>& menMatch,
                        const vector<int>& womenMatch,
                        long long& blockingCount) {
    // 完美性检查
    for (int m = 0; m < n; ++m) if (menMatch[m] == -1) return false;
    for (int w = 0; w < n; ++w) if (womenMatch[w] == -1) return false;

    // 稳定性检查
    blockingCount = 0;
    for (int m = 0; m < n; ++m) {
        for (int w = 0; w < n; ++w) {
            if (isBlockingPair(m, w, menPref, womenRank, menMatch, womenMatch)) {
                ++blockingCount;
            }
        }
    }
    return blockingCount == 0;
}

// 验证男 m 是否得到其「最优稳定伴侣」：在所有稳定匹配中，其伴侣排名最小（最靠前）
// 用暴力枚举所有完美匹配并筛选稳定匹配，找男 m 可达到的最优排名
// 返回男 m 在所有稳定匹配中的最优伴侣排名（暴力枚举 n! 个完美匹配）
// 注意：男最优性定理 —— Gale-Shapley（男提议）的输出中，每个男的伴侣排名
// 应等于该最优排名。这里用暴力枚举作为基准进行量化验证。
int bestPossibleRankForMen(int n, int m,
                           const vector<vector<int>>& menPref,
                           const vector<vector<int>>& womenRank) {
    vector<int> perm(n);
    iota(perm.begin(), perm.end(), 0);  // perm[w] = 女 w 配对的男
    vector<int> menMatch(n), womenMatch(n);

    int best = n;  // 不可达时返回 n
    do {
        for (int w = 0; w < n; ++w) {
            womenMatch[w] = perm[w];
            menMatch[perm[w]] = w;
        }
        long long bc = 0;
        if (isStableAndPerfect(n, menPref, womenRank, menMatch, womenMatch, bc)) {
            int w = menMatch[m];
            int r = 0;
            for (; r < n; ++r) if (menPref[m][r] == w) break;
            best = min(best, r);
        }
    } while (next_permutation(perm.begin(), perm.end()));

    return best;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // 多组 n 测试，从小 (暴力可枚举) 到大 (只验证 Gale-Shapley)
    vector<int> testSizes = {4, 5, 8, 20};

    cout << "==================================================\n";
    cout << " Stable Marriage — Gale-Shapley 量化验证报告\n";
    cout << "==================================================\n\n";
    cout << fixed << setprecision(4);

    mt19937 rng(12345);

    for (int n : testSizes) {
        cout << "----- n = " << n << " -----\n";

        // 生成随机偏好：男女偏好独立生成，各是随机排列
        vector<vector<int>> menPref(n);
        vector<vector<int>> womenPref(n);
        for (int m = 0; m < n; ++m) {
            menPref[m].resize(n);
            iota(menPref[m].begin(), menPref[m].end(), 0);
            shuffle(menPref[m].begin(), menPref[m].end(), rng);
        }
        for (int w = 0; w < n; ++w) {
            womenPref[w].resize(n);
            iota(womenPref[w].begin(), womenPref[w].end(), 0);
            shuffle(womenPref[w].begin(), womenPref[w].end(), rng);
        }

        GaleShapley gs(n, menPref, womenPref);
        gs.run();

        // 定量验证 1：完美匹配
        int unmatched = 0;
        for (int m = 0; m < n; ++m) if (gs.menMatch[m] == -1) ++unmatched;
        for (int w = 0; w < n; ++w) if (gs.womenMatch[w] == -1) ++unmatched;

        cout << "  完美匹配检查: 未匹配数 = " << unmatched
             << (unmatched == 0 ? " [PASS]" : " [FAIL]") << "\n";

        // 定量验证 2：稳定性（无阻滞对）
        long long bc = 0;
        bool stable = isStableAndPerfect(n, menPref, gs.womenRank,
                                         gs.menMatch, gs.womenMatch, bc);
        cout << "  稳定性检查:  阻滞对数 = " << bc
             << (stable ? " [PASS]" : " [FAIL]") << "\n";

        // 定量验证 3：提议方最优性（仅对可枚举的小 n）
        if (n <= 9) {
            int allOptimal = 1;
            int sumBest = 0, sumActual = 0;
            for (int m = 0; m < n; ++m) {
                int best = bestPossibleRankForMen(n, m, menPref, gs.womenRank);
                int w = gs.menMatch[m];
                int actual = 0;
                for (; actual < n; ++actual) if (menPref[m][actual] == w) break;
                sumBest += best;
                sumActual += actual;
                if (actual != best) allOptimal = 0;
            }
            cout << "  男最优性检查: 每个男的实际排名 == 其稳定可达最优排名 "
                 << (allOptimal ? "[PASS]" : "[FAIL]") << "\n";
            cout << "    (sumBest=" << sumBest << ", sumActual=" << sumActual << ")\n";
        }

        // 定量验证 4：接受方（女）最劣性 —— 每个女得到其最差稳定伴侣
        //   这里只做方向性说明：Gale-Shapley 对女性是最劣的
        //   量化：统计女性平均伴侣排名（应比女性平均更差，即排名更靠后）
        double avgMenHappy = 0, avgWomenHappy = 0;
        for (int m = 0; m < n; ++m) {
            int w = gs.menMatch[m];
            int r = 0; for (; r < n; ++r) if (menPref[m][r] == w) break;
            avgMenHappy += r;
        }
        for (int w = 0; w < n; ++w) {
            avgWomenHappy += gs.womenRank[w][gs.womenMatch[w]];
        }
        avgMenHappy /= n;
        avgWomenHappy /= n;
        cout << "  幸福度统计: 男平均伴侣排名=" << avgMenHappy
             << " (0最优), 女平均伴侣排名=" << avgWomenHappy << "\n";

        cout << "\n";
    }

    cout << "==================================================\n";
    cout << " 全部量化验证完成\n";
    cout << "==================================================\n";
    return 0;
}
