import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import random

# Heavy-Light Decomposition cover (English labels, matching repo convention):
#   left  : a rooted tree with heavy edges (red thick) = child with largest
#           subtree; light edges (blue dashed) = everything else.
#   right : heavy-chain decomposition -> dfn order -> consecutive ranges on
#           a segment tree; any path query decomposes into O(log n) ranges.

random.seed(42)
BG = '#0d1117'
HEAVY = '#f85149'
LIGHT = '#58a6ff'
GREY = '#c9d1d9'
GREEN = '#3fb950'
ORANGE = '#f0883e'
PURPLE = '#a371f7'

fig, axes = plt.subplots(1, 2, figsize=(15, 6.5), facecolor=BG)

# ==================== Left: tree + heavy/light decomposition ====================
ax = axes[0]
ax.set_facecolor(BG)

pos = {
    1: (0.0, 2.4),
    2: (-1.8, 1.2),
    3: (0.0, 1.2),
    4: (1.8, 1.2),
    5: (-2.6, 0.0),
    6: (-1.0, 0.0),
    7: (0.0, 0.0),
    8: (1.8, 0.0),
    9: (-2.6, -1.2),
    10: (-1.0, -1.2),
    11: (0.0, -1.2),
    12: (1.8, -1.2),
}

edges = [
    (1, 2, False),
    (1, 3, True),
    (1, 4, False),
    (2, 5, False),
    (2, 6, True),
    (3, 7, True),
    (4, 8, True),
    (6, 9, False),
    (6, 10, True),
    (7, 11, True),
    (8, 12, True),
]

for (u, v, heavy) in edges:
    x0, y0 = pos[u]
    x1, y1 = pos[v]
    color = HEAVY if heavy else LIGHT
    lw = 4.0 if heavy else 2.2
    style = '-' if heavy else '--'
    alpha = 0.95 if heavy else 0.55
    ax.plot([x0, x1], [y0, y1], color=color, lw=lw, linestyle=style,
            zorder=1, alpha=alpha)

for v, (x, y) in pos.items():
    color = GREEN if v == 1 else (ORANGE if v in (3, 7, 11) else GREY)
    ax.scatter(x, y, s=520, color=color, zorder=3, edgecolors='white',
               linewidths=1.8)
    ax.text(x, y, str(v), color='#0d1117', fontsize=11, ha='center',
            va='center', zorder=4, fontweight='bold')
    w = (v * 37) % 97 - 48
    ax.text(x, y - 0.42, f'w={w}', color='white', fontsize=8.5,
            ha='center', va='center', zorder=4)

from matplotlib.lines import Line2D
legend = [
    Line2D([0], [0], color=HEAVY, lw=4.0, label='heavy edge (largest subtree)'),
    Line2D([0], [0], color=LIGHT, lw=2.2, linestyle='--', label='light edge'),
]
ax.legend(handles=legend, loc='lower right', framealpha=0.2, facecolor=BG,
          edgecolor='#30363d', labelcolor='white', fontsize=10)

ax.set_xlim(-3.4, 3.4)
ax.set_ylim(-1.9, 3.0)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
ax.set_title('Heavy-Light Decomposition: pick the child with the largest subtree',
             color='white', fontsize=12.5, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# ==================== Right: dfn order -> segment tree ranges ====================
ax2 = axes[1]
ax2.set_facecolor(BG)

chains = [
    ('Chain C1', [1, 3, 7, 11], GREEN),
    ('Chain C2', [2, 6, 10], ORANGE),
    ('Chain C3', [4, 8, 12], PURPLE),
    ('singleton', [5, 9], GREY),
]

y_levels = [2.6, 1.6, 0.6, -0.4]
label_x = -2.8
bar_left = -1.6
dfn = 1
for (name, vs, color) in chains:
    y = y_levels.pop(0)
    for v in vs:
        ax2.scatter(bar_left + (dfn - 1) * 0.52, y, s=430, color=color,
                    zorder=3, edgecolors='white', linewidths=1.6)
        ax2.text(bar_left + (dfn - 1) * 0.52, y, f'{v}', color='#0d1117',
                 fontsize=9, ha='center', va='center', zorder=4, fontweight='bold')
        dfn += 1
    a, b = dfn - len(vs), dfn - 1
    ax2.text(label_x, y, name, color=color, fontsize=10.5, ha='right',
             va='center', fontweight='bold')
    ax2.text(bar_left + (a - 0.5) * 0.52, y - 0.45, f'[dfn {a}..{b}]',
             color='white', fontsize=8.5, ha='center', va='center')

ax2.text(-2.8, -1.3, 'Segment Tree', color='#c9d1d9', fontsize=11,
         ha='right', va='center', fontweight='bold')
seg_y = -1.9
xs = np.linspace(-1.6, 4.4, 17)
for x in xs:
    ax2.scatter(x, seg_y, s=40, color='#30363d', zorder=1)
ax2.plot([-1.6, 4.4], [seg_y, seg_y], color='#30363d', lw=2, zorder=1)

def hl(x0, x1, color):
    ax2.plot([x0, x1], [seg_y, seg_y], color=color, lw=5, zorder=2, alpha=0.8)
hl(-1.6, 0.1, GREEN)
hl(0.9, 1.6, ORANGE)

ax2.text(3.4, -1.3,
         'path query = sum of a few\ncontinuous ranges\nO(log n) chains x O(log n)\n= O(log^2 n) per query',
         color='#f85149', fontsize=10.5, ha='center', va='center', fontweight='bold')

ax2.set_xlim(-4.6, 6.0)
ax2.set_ylim(-2.5, 3.2)
ax2.set_aspect('equal')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title('Chain decomposition -> consecutive dfn -> segment-tree range',
              color='white', fontsize=12.5, fontweight='bold', pad=10)
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor=BG, bbox_inches='tight')
print('cover.png generated')
