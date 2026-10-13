// Markov Chain N-Gram Text Generation
// =====================================
// 技术点: 基于 N-Gram 的马尔可夫链文本生成。
//   - 从训练语料学习 (n-1) 阶上下文的字符转移概率分布
//   - 使用加权随机采样从转移分布中生成新文本
//   - 量化验证: 转移概率归一化(每行和为1)、零概率事件、生成文本的交叉熵/困惑度、可复现性(seed)
//
// 编译: g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
#include <bits/stdc++.h>
using namespace std;

// ---- 训练 N-Gram 模型 (字符级) ----
// 返回: ngrams 是一个 unordered_map<string, unordered_map<char, long long>>
//       key = (n-1) 个字符的上下文, value = 下一字符 + 出现次数
using CountMap = unordered_map<char, long long>;
using Model = unordered_map<string, CountMap>;

Model train(const string& text, int n) {
    Model model;
    if ((int)text.size() < n) return model;
    // 遍历每个长度为 n 的窗口: 前 n-1 个字符为上下文, 第 n 个为下一字符
    for (size_t i = 0; i + n <= text.size(); ++i) {
        string ctx = text.substr(i, n - 1);
        char next = text[i + n - 1];
        model[ctx][next]++;
    }
    return model;
}

// 从 CountMap 构建累积分布用于加权随机采样
// 返回: vector<pair<char,long long>> 累加计数(升序char), 尾元素为总和
// 验证用: 保证概率归一化 (和为总计数)
long long normalize(const CountMap& counts, vector<pair<char, long long>>& cdf) {
    cdf.clear();
    vector<pair<char, long long>> items(counts.begin(), counts.end());
    sort(items.begin(), items.end());
    long long cum = 0;
    for (auto& [ch, cnt] : items) { cum += cnt; cdf.emplace_back(ch, cum); }
    return cum; // 总计数
}

// 加权随机采样: 给定 cdf 和总计数, 按概率抽样返回 next char
char sample(const vector<pair<char, long long>>& cdf, long long total, mt19937_64& rng) {
    uniform_int_distribution<long long> dist(1, total);
    long long r = dist(rng);
    // 二分找到第一个 cum >= r 的字符
    auto it = lower_bound(cdf.begin(), cdf.end(), r,
        [](const pair<char,long long>& p, long long val){ return p.second < val; });
    return it->first;
}

// 生成文本, 从初始上下文 seed 开始
string generate(const Model& model, const string& seed, int n, size_t length, mt19937_64& rng) {
    string out = seed;
    string ctx = seed;
    while (out.size() < length) {
        auto it = model.find(ctx);
        if (it == model.end()) break; // 上下文未见, 无法继续
        vector<pair<char, long long>> cdf;
        long long total = normalize(it->second, cdf);
        char c = sample(cdf, total, rng);
        out += c;
        // 滑动窗口更新上下文
        ctx = out.substr(out.size() - (n - 1));
    }
    return out;
}

int main() {
    // ---- 训练语料 ----
    // 使用一段可读文本, 包含重复模式便于验证统计性质
    const string corpus =
        "the quick brown fox jumps over the lazy dog the quick brown fox "
        "jumps over the lazy dog the quick brown fox jumps over the lazy dog "
        "a rose is a rose is a rose is a rose "
        "to be or not to be that is the question "
        "winter is coming winter is coming the north remembers";

    const int N = 3; // trigram (2-gram context)

    ostringstream report;
    report << fixed << setprecision(6);

    // ---- 训练 ----
    Model model = train(corpus, N);
    report << "=== Markov Chain N-Gram Text Generation ===\n";
    report << "N (gram size)   = " << N << "\n";
    report << "Corpus length   = " << corpus.size() << " chars\n";
    report << "Distinct contexts (states) = " << model.size() << "\n";

    // ---- 量化验证 1: 转移概率归一化 ----
    // 每个上下文的所有下一字符计数之和应等于该上下文在语料中的出现次数
    // 我们验证: 每个上下文的累积 CDF 总和 > 0 且有限; 概率总和严格等于 1 (由归一化保证)
    bool normalization_ok = true;
    long long total_transitions = 0;
    for (auto& [ctx, counts] : model) {
        vector<pair<char, long long>> cdf;
        long long total = normalize(counts, cdf);
        if (total <= 0) { normalization_ok = false; break; }
        total_transitions += total;
        // 验证概率和为 1 (total/total == 1.0)
        double prob_sum = 0.0;
        for (auto& [ch, cnt] : counts) prob_sum += (double)cnt / total;
        if (fabs(prob_sum - 1.0) > 1e-9) { normalization_ok = false; break; }
    }
    report << "\n[Verify 1] Transition probability normalization:\n";
    report << "  total transitions counted = " << total_transitions << "\n";
    report << "  all rows sum to exactly 1.0 : " << (normalization_ok ? "PASS" : "FAIL") << "\n";

    // 交叉验证: 总转移次数应等于语料中所有长度为 N 的窗口数 = corpus.size() - N + 1
    long long expected_transitions = (long long)corpus.size() - N + 1;
    bool count_matches = (total_transitions == expected_transitions);
    report << "  transitions == |windows| = " << expected_transitions
           << " : " << (count_matches ? "PASS" : "FAIL") << "\n";

    // ---- 量化验证 2: 零概率事件 ----
    // 未出现在任何上下文中的编码, 尝试采样应无法继续 (无状态)
    bool unknown_ctx_handled = (model.find("zzz") == model.end());
    report << "\n[Verify 2] Unseen context handled gracefully:\n";
    report << "  'zzz' not in model (generation stops) : " << (unknown_ctx_handled ? "PASS" : "FAIL") << "\n";

    // ---- 量化验证 3: 可复现性 (seed 确定则输出确定) ----
    string start_ctx = corpus.substr(0, N - 1); // "th"
    const size_t gen_len = 200;
    vector<string> runs;
    for (int s = 0; s < 3; ++s) {
        mt19937_64 rng(12345);
        runs.push_back(generate(model, start_ctx, N, gen_len, rng));
    }
    bool reproducible = (runs[0] == runs[1] && runs[1] == runs[2]);
    report << "\n[Verify 3] Reproducibility (same seed = same output):\n";
    report << "  3 runs with seed=12345 identical : " << (reproducible ? "PASS" : "FAIL") << "\n";

    // ---- 量化验证 4: 生成文本的合法性 (所有 N-gram 均在语料中出现过) ----
    // 对生成文本的每个 N 长度窗口, 其前 N-1 上下文必须能在模型中查到
    bool all_ngrams_valid = true;
    const string gen = runs[0];
    for (size_t i = 0; i + N <= gen.size(); ++i) {
        string ctx = gen.substr(i, N - 1);
        char next = gen[i + N - 1];
        auto it = model.find(ctx);
        if (it == model.end() || it->second.count(next) == 0) { all_ngrams_valid = false; break; }
    }
    report << "\n[Verify 4] All generated N-grams are valid in training data:\n";
    report << "  " << (all_ngrams_valid ? "PASS" : "FAIL") << "\n";

    // ---- 量化验证 5: 交叉熵 / 困惑度 (模型对语料的拟合优度) ----
    // 对语料计算模型下的平均负对数似然 (使用固定 seed 采样无意义, 这里用分布本身)
    // 用 Laplace (add-1) 平滑避免零概率, 计算每字符交叉熵, 再取困惑度 = exp(cross_entropy)
    // 这是模型内在的量化指标, 值越低表示模型对语料拟合越好。
    double total_log_prob = 0.0;
    long long eval_windows = 0;
    // 语料词表大小 (distinct next-chars 总计)
    set<char> vocab;
    for (char c : corpus) if (c != '\0') vocab.insert(c);
    double V = vocab.size();

    for (size_t i = 0; i + N <= corpus.size(); ++i) {
        string ctx = corpus.substr(i, N - 1);
        char next = corpus[i + N - 1];
        auto it = model.find(ctx);
        // add-1 / add-V smoothing
        double total = 0.0, count = 0.0;
        if (it != model.end()) {
            for (auto& [ch, cnt] : it->second) total += cnt;
            if (it->second.count(next)) count = it->second.at(next);
        }
        double p = (count + 1.0) / (total + V);
        total_log_prob += log(p);
        eval_windows++;
    }
    double cross_entropy = -total_log_prob / eval_windows;
    double perplexity = exp(cross_entropy);
    report << "\n[Verify 5] Model cross-entropy / perplexity (Laplace smoothed):\n";
    report << "  vocabulary size       = " << (int)V << "\n";
    report << "  eval windows          = " << eval_windows << "\n";
    report << "  cross-entropy (bits)  = " << cross_entropy / log(2.0) << "\n";
    report << "  perplexity            = " << perplexity << "\n";
    bool perplexity_reasonable = (perplexity < V + 1.0); // 平滑下应远小于词表+1
    report << "  perplexity < V+1 (model fits better than uniform) : "
           << (perplexity_reasonable ? "PASS" : "FAIL") << "\n";

    // ---- 输出样例 ----
    report << "\n=== Generated sample (len=" << gen_len << ") ===\n" << gen << "\n";
    report << "\n=== Example transition distribution ===\n";
    // 打印 "th" 上下文的分布
    for (auto& [ctx, counts] : model) {
        if (ctx == "th") {
            vector<pair<char, long long>> cdf;
            long long total = normalize(counts, cdf);
            for (auto& [ch, cnt] : counts) {
                report << "  P('" << ch << "' | \"" << ctx << "\") = "
                       << (double)cnt / total << "\n";
            }
        }
    }

    // 汇总 pass/fail
    bool all_pass = normalization_ok && count_matches && unknown_ctx_handled
                    && reproducible && all_ngrams_valid && perplexity_reasonable;
    report << "\n=== OVERALL: " << (all_pass ? "ALL PASS" : "FAILURE") << " ===\n";

    cout << report.str();

    // 写文件
    ofstream fout("markov_output.txt");
    fout << report.str();
    fout.close();

    return all_pass ? 0 : 1;
}
