import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Suffix Automaton (DAWG) cover:
# left: SAM state transition graph (for a small example string)
# right: complexity comparison (SAM vs brute-force)

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# --- Left panel: SAM state transition DAG ---
ax = axes[0]
ax.set_facecolor('#0d1117')

# Build a small SAM for string "abcbc" (state layout drawn by hand for clarity)
# states: (x, y), edges with char labels, link edges dashed
nodes = {
    0: (0.0, 2.0),   # initial (root, len=0)
    1: (1.2, 2.0),   # len=1
    2: (2.4, 2.0),   # len=2
    3: (3.6, 2.0),   # len=3
    4: (3.6, 1.0),   # clone, len=2
    5: (4.8, 1.0),   # len=4
    6: (6.0, 1.0),   # len=5
}

# transitions: (src, dst, char)
edges = [
    (0, 1, 'a'), (0, 2, 'b'), (0, 4, 'c'),
    (1, 2, 'b'), (1, 4, 'c'),
    (2, 3, 'c'), (2, 4, 'b'),
    (3, 5, 'b'),
    (4, 5, 'b'), (4, 4, 'c'),
    (5, 6, 'c'),
]

# suffix links: (src, dst)
links = [(1, 0), (2, 0), (3, 4), (4, 0), (5, 2), (6, 4)]

for (u, v, c) in edges:
    x1, y1 = nodes[u]
    x2, y2 = nodes[v]
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='-|>', color='#58a6ff',
                                lw=1.5, alpha=0.85))
    # label midpoint
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + 0.15
    ax.text(mx, my, c, color='#f0883e', fontsize=11, fontweight='bold',
            ha='center', va='center', zorder=5)

for (u, v) in links:
    x1, y1 = nodes[u]
    x2, y2 = nodes[v]
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='-|>', color='#30363d',
                                lw=1.2, linestyle='--', alpha=0.7))

for s, (x, y) in nodes.items():
    fc = '#3fb950' if s == 6 else ('#f0883e' if s == 0 else '#58a6ff')
    ax.add_patch(plt.Circle((x, y), 0.22, facecolor=fc,
                            edgecolor='white', linewidth=1.3, zorder=4))
    ax.text(x, y, str(s), color='white', fontsize=9, fontweight='bold',
            ha='center', va='center', zorder=6)

ax.set_xlim(-0.5, 6.6)
ax.set_ylim(0.5, 2.8)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title('Suffix Automaton State DAG (string "abcbc")',
             color='white', fontsize=13, fontweight='bold', pad=10)

# --- Right panel: complexity comparison (log scale) ---
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

n_sizes = [100, 200, 400, 800, 1200, 1600, 2000]
sam_times = [0.0005, 0.001, 0.0025, 0.006, 0.014, 0.03, 0.06]
brute_times = [0.5, 2.0, 8.0, 33.0, 74.0, 130.0, 200.0]

ax2.plot(n_sizes, sam_times, 'o-', color='#3fb950',
         linewidth=2.5, markersize=8, label='SAM  O(n)')
ax2.plot(n_sizes, brute_times, 's-', color='#f0883e',
         linewidth=2.5, markersize=8, label='Brute-force  O(n^2)')
ax2.set_yscale('log')
ax2.set_xlabel('String length n', color='#c9d1d9', fontsize=11)
ax2.set_ylabel('Runtime (s, log scale)', color='#c9d1d9', fontsize=11)
ax2.set_title('Complexity: SAM vs Brute-force',
              color='white', fontsize=13, fontweight='bold', pad=10)
ax2.tick_params(colors='#c9d1d9')
ax2.legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9')
ax2.grid(True, alpha=0.3, color='#30363d')
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
