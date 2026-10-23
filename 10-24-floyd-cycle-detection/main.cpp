#include <iostream>
#include <vector>
#include <unordered_set>
#include <cstdint>
#include <random>
#include <chrono>
#include <iomanip>
#include <cassert>

// Floyd's Cycle Detection (Tortoise & Hare)
// - detect whether a singly-linked list has a cycle
// - find the entry node of the cycle (if any)
// - compute the cycle length
// All in O(mu + lambda) time and O(1) extra space.

struct Node {
    int value;
    Node* next = nullptr;
    Node(int v) : value(v) {}
};

// Build a list of `total` nodes. If cycleStart >= 0, the last node points back
// to node index `cycleStart` (0-based), forming a cycle of length
// (total - cycleStart). Returns head.
Node* buildList(int total, int cycleStart, std::vector<Node*>& nodes) {
    nodes.clear();
    nodes.reserve(total);
    for (int i = 0; i < total; ++i) nodes.push_back(new Node(i));
    for (int i = 0; i + 1 < total; ++i) nodes[i]->next = nodes[i + 1];
    if (cycleStart >= 0 && total > 0) {
        nodes[total - 1]->next = nodes[cycleStart];
    }
    return total > 0 ? nodes[0] : nullptr;
}

// Returns true if a cycle exists.
bool hasCycleFloyd(Node* head) {
    if (!head) return false;
    Node* slow = head;
    Node* fast = head;
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
        if (slow == fast) return true;
    }
    return false;
}

// Returns the entry node of the cycle, or nullptr if acyclic.
// If a cycle exists, also fills cycleLength.
Node* detectCycleFloyd(Node* head, int& cycleLength) {
    cycleLength = 0;
    if (!head) return nullptr;
    Node* slow = head;
    Node* fast = head;
    Node* meet = nullptr;
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
        if (slow == fast) { meet = slow; break; }
    }
    if (!meet) return nullptr;

    // Measure cycle length: advance one pointer around the loop.
    Node* p = meet;
    do {
        p = p->next;
        cycleLength++;
    } while (p != meet);

    // Find entry: start one pointer at head, other at meeting point.
    Node* a = head;
    Node* b = meet;
    while (a != b) {
        a = a->next;
        b = b->next;
    }
    return a;
}

// Baseline using a hash set: O(n) time, O(n) space, and gives entry + length.
Node* detectCycleHashSet(Node* head, int& cycleLength) {
    cycleLength = 0;
    if (!head) return nullptr;
    std::unordered_set<Node*> seen;
    Node* p = head;
    while (p) {
        if (seen.count(p)) {
            // p is the entry node; measure length from here.
            Node* q = p;
            do {
                q = q->next;
                cycleLength++;
            } while (q != p);
            return p;
        }
        seen.insert(p);
        p = p->next;
    }
    return nullptr;
}

void cleanup(std::vector<Node*>& nodes) {
    for (Node* n : nodes) delete n;
}

int main() {
    std::mt19937 rng(123456789u);
    int testCases = 0, passed = 0;

    auto runCase = [&](int total, int cycleStart) {
        std::vector<Node*> nodes;
        Node* head = buildList(total, cycleStart, nodes);

        bool fdHasCycle = hasCycleFloyd(head);
        int fdLen = 0;
        Node* fdEntry = detectCycleFloyd(head, fdLen);

        int hsLen = 0;
        Node* hsEntry = detectCycleHashSet(head, hsLen);
        bool hsHasCycle = (hsEntry != nullptr);

        bool ok = true;
        if (fdHasCycle != hsHasCycle) { ok = false; }
        if (hsHasCycle) {
            if (fdEntry != hsEntry) ok = false;
            if (fdLen != hsLen) ok = false;
            int expectedLen = total - cycleStart;
            if (fdLen != expectedLen) ok = false;
        }

        testCases++;
        if (ok) passed++;
        else {
            std::cout << "FAIL total=" << total << " cycleStart=" << cycleStart
                      << " fdHas=" << fdHasCycle << " hsHas=" << hsHasCycle
                      << " fdLen=" << fdLen << " hsLen=" << hsLen << "\n";
        }
        cleanup(nodes);
    };

    // Acyclic cases.
    runCase(0, -1);
    runCase(1, -1);
    runCase(2, -1);
    runCase(100, -1);

    // Cyclic cases with varied entry points.
    runCase(1, 0);          // self loop
    runCase(2, 0);          // full cycle of 2
    runCase(2, 1);          // tail node cycles
    runCase(5, 0);          // whole list is a cycle
    runCase(5, 2);          // mu=2 lambda=3
    runCase(9, 4);          // mu=4 lambda=5
    runCase(100, 1);
    runCase(100, 50);
    runCase(100, 99);       // tiny loop at tail

    // Random fuzz: random total and random cycle start (or acyclic).
    for (int i = 0; i < 200; ++i) {
        int total = (int)(rng() % 200) + 1;
        int cycleStart = (rng() % 2 == 0) ? (int)(rng() % total) : -1;
        runCase(total, cycleStart);
    }

    std::cout << "=== Correctness ===\n";
    std::cout << "Test cases: " << testCases << "\n";
    std::cout << "Passed:     " << passed << "\n";
    assert(passed == testCases);

    // Performance benchmark: O(1) space vs O(n) hashset on a large cyclic list.
    const int N = 2000000;
    int benchCycles = 5;
    std::vector<Node*> big;
    Node* bigHead = buildList(N, N / 3, big); // mu=666666, lambda=1333334

    auto timeFloyd = [&]() {
        auto t0 = std::chrono::steady_clock::now();
        int len = 0;
        Node* e = detectCycleFloyd(bigHead, len);
        auto t1 = std::chrono::steady_clock::now();
        (void)e; (void)len;
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };
    auto timeHash = [&]() {
        auto t0 = std::chrono::steady_clock::now();
        int len = 0;
        Node* e = detectCycleHashSet(bigHead, len);
        auto t1 = std::chrono::steady_clock::now();
        (void)e; (void)len;
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    double floydSum = 0, hashSum = 0;
    for (int i = 0; i < benchCycles; ++i) { floydSum += timeFloyd(); hashSum += timeHash(); }
    double floydAvg = floydSum / benchCycles;
    double hashAvg = hashSum / benchCycles;
    double speedup = hashAvg / floydAvg;

    std::cout << "\n=== Performance (N=" << N << ", mu=" << N / 3 << ", lambda=" << N - N / 3 << ") ===\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Floyd avg:   " << floydAvg << " ms\n";
    std::cout << "HashSet avg: " << hashAvg << " ms\n";
    std::cout << "Speedup (Floyd vs HashSet): " << speedup << "x\n";

    std::cout << "\nALL CHECKS PASSED\n";
    cleanup(big);
    return 0;
}
