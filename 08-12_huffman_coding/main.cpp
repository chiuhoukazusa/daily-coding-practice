#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <map>
#include <string>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iomanip>

// Huffman tree node
struct Node {
    char ch;
    int freq;
    Node *left, *right;
    Node(char c, int f) : ch(c), freq(f), left(nullptr), right(nullptr) {}
};

// Min-heap comparator
struct Compare {
    bool operator()(Node* a, Node* b) { return a->freq > b->freq; }
};

// Build Huffman tree from frequency map
Node* buildHuffmanTree(const std::unordered_map<char, int>& freqMap) {
    std::priority_queue<Node*, std::vector<Node*>, Compare> pq;
    for (auto& p : freqMap) {
        pq.push(new Node(p.first, p.second));
    }
    if (pq.empty()) return nullptr;
    while (pq.size() > 1) {
        Node* left = pq.top(); pq.pop();
        Node* right = pq.top(); pq.pop();
        Node* parent = new Node('\0', left->freq + right->freq);
        parent->left = left;
        parent->right = right;
        pq.push(parent);
    }
    return pq.top();
}

// Generate codes by DFS
void generateCodes(Node* root, const std::string& code,
                   std::unordered_map<char, std::string>& codeMap) {
    if (!root) return;
    if (!root->left && !root->right) {
        codeMap[root->ch] = code.empty() ? "0" : code;
    }
    generateCodes(root->left, code + "0", codeMap);
    generateCodes(root->right, code + "1", codeMap);
}

// Encode input string using code map
std::string encode(const std::string& input,
                   const std::unordered_map<char, std::string>& codeMap) {
    std::string result;
    for (char c : input) {
        result += codeMap.at(c);
    }
    return result;
}

// Decode bit string using tree
std::string decode(const std::string& bits, Node* root) {
    std::string result;
    Node* cur = root;
    for (char b : bits) {
        cur = (b == '0') ? cur->left : cur->right;
        if (!cur->left && !cur->right) {
            result += cur->ch;
            cur = root;
        }
    }
    return result;
}

// Free tree
void freeTree(Node* root) {
    if (!root) return;
    freeTree(root->left);
    freeTree(root->right);
    delete root;
}

// Calculate Shannon entropy
double entropy(const std::unordered_map<char, int>& freqMap, int total) {
    double H = 0.0;
    for (auto& p : freqMap) {
        double prob = (double)p.second / total;
        H -= prob * std::log2(prob);
    }
    return H;
}

// Calculate average code length
double avgCodeLength(const std::unordered_map<char, std::string>& codeMap,
                     const std::unordered_map<char, int>& freqMap, int total) {
    double avg = 0.0;
    for (auto& p : codeMap) {
        double prob = (double)freqMap.at(p.first) / total;
        avg += prob * p.second.size();
    }
    return avg;
}

// Serialize tree for header overhead estimation
std::string serializeTree(Node* root) {
    if (!root) return "";
    if (!root->left && !root->right) {
        return "1" + std::string(1, root->ch);
    }
    return "0" + serializeTree(root->left) + serializeTree(root->right);
}

// PPM generation for frequency visualization
void generatePPM(const std::unordered_map<char, int>& freqMap,
                 const std::unordered_map<char, std::string>& codeMap) {
    int W = 800, H = 600;
    std::vector<unsigned char> img(W * H * 3, 255);
    
    // Sort chars by frequency descending
    std::vector<std::pair<char, int>> sorted(freqMap.begin(), freqMap.end());
    std::sort(sorted.begin(), sorted.end(),
              [](auto& a, auto& b) { return a.second > b.second; });
    
    int maxFreq = sorted[0].second;
    int barAreaX = 50, barAreaY = 50, barAreaW = 700, barAreaH = 350;
    int n = sorted.size();
    int barW = barAreaW / n;
    
    auto drawRect = [&](int x, int y, int w, int h, int r, int g, int b) {
        for (int dy = 0; dy < h; dy++) {
            for (int dx = 0; dx < w; dx++) {
                int px = x + dx, py = y + dy;
                if (px >= 0 && px < W && py >= 0 && py < H) {
                    int idx = (py * W + px) * 3;
                    img[idx] = r; img[idx+1] = g; img[idx+2] = b;
                }
            }
        }
    };
    
    auto drawText = [&](int x, int y, const std::string& s, int r, int g, int b) {
        // Simple 5x7 font placeholder - use colored pixel blocks
        int scale = 2;
        for (size_t i = 0; i < s.size(); i++) {
            // Draw a small colored block as character placeholder
            for (int dy = 0; dy < 7; dy++) {
                for (int dx = 0; dx < 5; dx++) {
                    int px = x + i * 6 * scale + dx, py = y + dy;
                    if (px >= 0 && px < W && py >= 0 && py < H && px < W-1 && py < H-1) {
                        int idx = (py * W + px) * 3;
                        img[idx] = r; img[idx+1] = g; img[idx+2] = b;
                    }
                }
            }
        }
    };
    
    // Title
    drawText(50, 10, "Huffman Coding - Frequency Distribution & Code Lengths", 0, 0, 0);
    
    // Draw bars
    for (int i = 0; i < n; i++) {
        int barH = (int)((double)sorted[i].second / maxFreq * barAreaH);
        int x = barAreaX + i * barW;
        int y = barAreaY + barAreaH - barH;
        
        // Color based on frequency (red = high, blue = low)
        double ratio = (double)sorted[i].second / maxFreq;
        int r = (int)(ratio * 200 + 55);
        int g = (int)((1.0 - abs(ratio - 0.5) * 2.0) * 150 + 50);
        int bcol = (int)((1.0 - ratio) * 200 + 55);
        
        drawRect(x, y, barW - 2, barH, r, g, bcol);
        
        // Draw code length below
        int codeLen = (int)codeMap.at(sorted[i].first).size();
        std::string label;
        if (sorted[i].first >= 32 && sorted[i].first < 127)
            label = std::string(1, sorted[i].first) + ":" + std::to_string(codeLen);
        else
            label = "0x" + std::to_string((int)(unsigned char)sorted[i].first) + ":" + std::to_string(codeLen);
        
        // Draw small label
        int lx = x + barW / 2 - 10;
        int ly = barAreaY + barAreaH + 5;
        for (size_t j = 0; j < label.size(); j++) {
            for (int dy = 0; dy < 5; dy++) {
                for (int dx = 0; dx < 3; dx++) {
                    int px = lx + (int)j * 4 + dx, py = ly + dy;
                    if (px >= 0 && px < W && py >= 0 && py < H) {
                        int idx = (py * W + px) * 3;
                        img[idx] = 0; img[idx+1] = 0; img[idx+2] = 0;
                    }
                }
            }
        }
    }
    
    // Statistics panel
    int panelY = barAreaY + barAreaH + 60;
    drawText(barAreaX, panelY, "Huffman Coding Analysis Results:", 0, 0, 80);
    
    std::ofstream out("huffman_output.ppm", std::ios::binary);
    out << "P6\n" << W << " " << H << "\n255\n";
    out.write((char*)img.data(), img.size());
    out.close();
}

// Generate test data with known distribution
std::string generateTestData(const std::string& prefix, int size,
                              const std::vector<std::pair<char, double>>& dist) {
    std::string result = prefix;
    std::vector<double> cum(dist.size());
    double total = 0;
    for (auto& p : dist) total += p.second;
    for (size_t i = 0; i < dist.size(); i++) {
        cum[i] = (i > 0 ? cum[i-1] : 0) + dist[i].second / total;
    }
    
    // Simple PRNG
    unsigned seed = 42;
    auto rand01 = [&]() -> double {
        seed = seed * 1103515245 + 12345;
        return (double)(seed & 0x7FFFFFFF) / 0x7FFFFFFF;
    };
    
    for (int i = 0; i < size; i++) {
        double r = rand01();
        for (size_t j = 0; j < cum.size(); j++) {
            if (r <= cum[j]) {
                result += dist[j].first;
                break;
            }
        }
    }
    return result;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Huffman Coding Data Compression" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Test 1: Skewed distribution (English-like)
    std::cout << "\n--- Test 1: Skewed Distribution (English-like) ---" << std::endl;
    
    std::vector<std::pair<char, double>> englishDist = {
        {'e', 0.127}, {'t', 0.091}, {'a', 0.082}, {'o', 0.075}, {'i', 0.070},
        {'n', 0.067}, {'s', 0.063}, {'h', 0.061}, {'r', 0.060}, {'d', 0.043},
        {'l', 0.040}, {'c', 0.028}, {'u', 0.028}, {'m', 0.024}, {'w', 0.024},
        {'f', 0.022}, {'g', 0.020}, {'y', 0.020}, {'p', 0.019}, {'b', 0.015}
    };
    
    std::string text = generateTestData("", 50000, englishDist);
    
    // Build frequency map
    std::unordered_map<char, int> freqMap;
    for (char c : text) freqMap[c]++;
    int totalChars = (int)text.size();
    
    // Build tree and codes
    Node* root = buildHuffmanTree(freqMap);
    std::unordered_map<char, std::string> codeMap;
    generateCodes(root, "", codeMap);
    
    // Encode
    std::string encoded = encode(text, codeMap);
    
    // Decode and verify
    std::string decoded = decode(encoded, root);
    assert(decoded == text);
    
    // Compute metrics
    double H = entropy(freqMap, totalChars);
    double avgLen = avgCodeLength(codeMap, freqMap, totalChars);
    double originalBits = totalChars * 8.0;
    double compressedBits = (double)encoded.size();
    double treeOverheadBits = (double)serializeTree(root).size();
    double totalCompressedBits = compressedBits + treeOverheadBits;
    double compressionRatio = originalBits / compressedBits;
    double netCompressionRatio = originalBits / totalCompressedBits;
    
    std::cout << "  Input size:      " << totalChars << " bytes (" << originalBits << " bits)" << std::endl;
    std::cout << "  Unique chars:    " << freqMap.size() << std::endl;
    std::cout << "  Shannon entropy: " << std::fixed << std::setprecision(4) << H << " bits/symbol" << std::endl;
    std::cout << "  Avg code length: " << std::fixed << std::setprecision(4) << avgLen << " bits/symbol" << std::endl;
    std::cout << "  Entropy bound:   " << std::fixed << std::setprecision(4) << H << " (avgLen >= H: " << (avgLen >= H - 0.001 ? "PASS" : "FAIL") << ")" << std::endl;
    std::cout << "  Compressed:      " << compressedBits << " bits" << std::endl;
    std::cout << "  Tree overhead:   " << treeOverheadBits << " bits" << std::endl;
    std::cout << "  Total w/overhead:" << totalCompressedBits << " bits" << std::endl;
    std::cout << "  Compression ratio (payload): " << std::fixed << std::setprecision(2) << compressionRatio << "x" << std::endl;
    std::cout << "  Compression ratio (w/tree):  " << std::fixed << std::setprecision(2) << netCompressionRatio << "x" << std::endl;
    
    std::cout << "\n  Prefix property check: ";
    bool prefixOk = true;
    std::vector<std::string> codes;
    for (auto& p : codeMap) codes.push_back(p.second);
    for (size_t i = 0; i < codes.size() && prefixOk; i++) {
        for (size_t j = i+1; j < codes.size() && prefixOk; j++) {
            if (codes[i].size() <= codes[j].size()) {
                if (codes[j].substr(0, codes[i].size()) == codes[i]) prefixOk = false;
            } else {
                if (codes[i].substr(0, codes[j].size()) == codes[j]) prefixOk = false;
            }
        }
    }
    std::cout << (prefixOk ? "PASS" : "FAIL") << std::endl;
    
    bool roundtrip = (decoded == text);
    std::cout << "  Roundtrip:       " << (roundtrip ? "PASS" : "FAIL") << std::endl;
    
    freeTree(root);
    
    // Test 2: Uniform distribution
    std::cout << "\n--- Test 2: Uniform Distribution ---" << std::endl;
    
    std::vector<std::pair<char, double>> uniformDist;
    for (char c = 'A'; c <= 'P'; c++) {
        uniformDist.push_back({c, 1.0 / 16.0});
    }
    std::string text2 = generateTestData("", 50000, uniformDist);
    
    std::unordered_map<char, int> freqMap2;
    for (char c : text2) freqMap2[c]++;
    int totalChars2 = (int)text2.size();
    
    Node* root2 = buildHuffmanTree(freqMap2);
    std::unordered_map<char, std::string> codeMap2;
    generateCodes(root2, "", codeMap2);
    
    std::string encoded2 = encode(text2, codeMap2);
    std::string decoded2 = decode(encoded2, root2);
    assert(decoded2 == text2);
    
    double H2 = entropy(freqMap2, totalChars2);
    double avgLen2 = avgCodeLength(codeMap2, freqMap2, totalChars2);
    double originalBits2 = totalChars2 * 8.0;
    double compressedBits2 = (double)encoded2.size();
    
    std::cout << "  Input size:      " << totalChars2 << " bytes (" << originalBits2 << " bits)" << std::endl;
    std::cout << "  Unique chars:    " << freqMap2.size() << std::endl;
    std::cout << "  Shannon entropy: " << std::fixed << std::setprecision(4) << H2 << " bits/symbol" << std::endl;
    std::cout << "  Avg code length: " << std::fixed << std::setprecision(4) << avgLen2 << " bits/symbol" << std::endl;
    std::cout << "  Entropy bound:   " << (avgLen2 >= H2 - 0.001 ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Compression ratio:" << std::fixed << std::setprecision(2) << (originalBits2 / compressedBits2) << "x" << std::endl;
    std::cout << "  Roundtrip:       " << ((decoded2 == text2) ? "PASS" : "FAIL") << std::endl;
    
    freeTree(root2);
    
    // Test 3: Highly skewed (power-law)
    std::cout << "\n--- Test 3: Power-Law Distribution ---" << std::endl;
    
    std::string text3 = "aaaaaaa" + std::string(30000, 'a');
    // Add some variation with decreasing probability
    std::vector<std::pair<char, double>> powerLawDist;
    for (int i = 0; i < 10; i++) {
        powerLawDist.push_back({(char)('a' + i), 1.0 / (i + 1)});
    }
    text3 += generateTestData("", 20000, powerLawDist);
    
    std::unordered_map<char, int> freqMap3;
    for (char c : text3) freqMap3[c]++;
    int totalChars3 = (int)text3.size();
    
    Node* root3 = buildHuffmanTree(freqMap3);
    std::unordered_map<char, std::string> codeMap3;
    generateCodes(root3, "", codeMap3);
    
    std::string encoded3 = encode(text3, codeMap3);
    std::string decoded3 = decode(encoded3, root3);
    assert(decoded3 == text3);
    
    double H3 = entropy(freqMap3, totalChars3);
    double avgLen3 = avgCodeLength(codeMap3, freqMap3, totalChars3);
    double originalBits3 = totalChars3 * 8.0;
    double compressedBits3 = (double)encoded3.size();
    
    std::cout << "  Input size:      " << totalChars3 << " bytes (" << originalBits3 << " bits)" << std::endl;
    std::cout << "  Unique chars:    " << freqMap3.size() << std::endl;
    std::cout << "  Shannon entropy: " << std::fixed << std::setprecision(4) << H3 << " bits/symbol" << std::endl;
    std::cout << "  Avg code length: " << std::fixed << std::setprecision(4) << avgLen3 << " bits/symbol" << std::endl;
    std::cout << "  Entropy bound:   " << (avgLen3 >= H3 - 0.001 ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Compression ratio:" << std::fixed << std::setprecision(2) << (originalBits3 / compressedBits3) << "x" << std::endl;
    std::cout << "  Roundtrip:       " << ((decoded3 == text3) ? "PASS" : "FAIL") << std::endl;
    
    // Generate PPM visualization using test 1
    std::unordered_map<char, int> freqMap_viz;
    for (char c : text) freqMap_viz[c]++;
    Node* root_viz = buildHuffmanTree(freqMap_viz);
    std::unordered_map<char, std::string> codeMap_viz;
    generateCodes(root_viz, "", codeMap_viz);
    generatePPM(freqMap_viz, codeMap_viz);
    freeTree(root_viz);
    freeTree(root3);
    
    // Final summary
    std::cout << "\n========================================" << std::endl;
    std::cout << "  OVERALL SUMMARY" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Verify all assertions
    bool allPass = true;
    for (int t = 1; t <= 3; t++) {
        // Check entropy bound
        double ent, avg;
        if (t == 1) { ent = H; avg = avgLen; }
        else if (t == 2) { ent = H2; avg = avgLen2; }
        else { ent = H3; avg = avgLen3; }
        
        if (!(avg >= ent - 0.001)) {
            std::cout << "  FAIL: Test " << t << " violates entropy bound" << std::endl;
            allPass = false;
        }
    }
    
    if (prefixOk && allPass) {
        std::cout << "  ALL CHECKS PASSED" << std::endl;
        std::cout << "  Prefix property:     OK" << std::endl;
        std::cout << "  Entropy lower bound: OK" << std::endl;
        std::cout << "  Lossless roundtrip:  OK" << std::endl;
        std::cout << "  PPM visualization:   huffman_output.ppm" << std::endl;
    } else {
        std::cout << "  SOME CHECKS FAILED!" << std::endl;
        return 1;
    }
    
    return 0;
}
