import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Mo's Algorithm cover:
# left: sqrt-decomposition / block ordering visualization
# right: pointer movement total complexity vs naive re-scan

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# --- Left panel: block ordering / sqrt decomposition ---
ax = axes[0]
ax.set_facecolor('#0d1117')

# Draw the array split into blocks of size sqrt(n)
n = 20
block = 5
for i in range(n):
    x0 = i
    x1 = i + 1
    color = '#161b22' if (i // block) % 2 == 0 else '#1f2833'
    ax.add_patch(plt.Rectangle((x0, 0), 1, 1, facecolor=color,
                               edgecolor='#30363d', linewidth=1))

# Query ranges as horizontal brackets (a few Mo queries)
queries = [
    ((0, 6), '#58a6ff', 0.9),
    ((1, 8), '#3fb950', 1.1),
    ((0, 12), '#f0883e', 1.3),
    ((13, 19), '#d29922', 0.9),
    ((10, 18), '#bc8cff', 1.1),
]
for (l, r), color, y in queries:
    ax.plot([l, r-0.1], [y, y], color=color, lw=2, alpha=0.9)
    ax.scatter([l, r-0.1], [y, y], color=color, s=40, zorder=5)
    ax.text((l + r) / 2, y + 0.22, f'[{l},{r})', color=color,
            fontsize=9, ha='center', fontweight='bold')

# block separators
for b in range(block, n, block):
    ax.axvline(b, color='#f85149', lw=1, alpha=0.5, linestyle='--')

ax.set_xlim(-1, n + 0.5)
ax.set_ylim(-0.5, 1.8)
ax.set_aspect('auto')
ax.set_xticks(range(0, n, 5))
ax.set_yticks([])
ax.set_title('Sqrt Decomposition: Ordering Queries by Block',
             color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# --- Right panel: total pointer movement complexity ---
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

n_sizes = np.array([100, 500, 1000, 5000, 10000, 50000, 100000])
# naive: O(q * n), Mo: O((n+q) sqrt(n))
q = 100000
nav = n_sizes.astype(float) * q / 1e9   # scaled for display (arbitrary units)
mo = (n_sizes.astype(float) + q) * np.sqrt(n_sizes.astype(float)) / 1e9

ax2.plot(n_sizes, nav, 's-', color='#f0883e', linewidth=2.5, markersize=8,
         label='Naive re-scan  O(q·n)')
ax2.plot(n_sizes, mo, 'o-', color='#3fb950', linewidth=2.5, markersize=8,
         label="Mo's  O((n+q)·√n)")
ax2.set_yscale('log')
ax2.set_xscale('log')
ax2.set_xlabel('Array size n', color='#c9d1d9', fontsize=11)
ax2.set_ylabel('Total pointer moves (log scale)', color='#c9d1d9', fontsize=11)
ax2.set_title('Pointer Movement: Mo vs Naive',
              color='white', fontsize=13, fontweight='bold', pad=10)
ax2.tick_params(colors='#c9d1d9')
ax2.legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9')
ax2.grid(True, alpha=0.3, color='#30363d')
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
