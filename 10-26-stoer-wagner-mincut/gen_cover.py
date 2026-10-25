import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import random

# Stoer-Wagner Global Minimum Cut cover:
# left  : a weighted undirected graph with the global min-cut highlighted.
#         The dashed red line splits the vertices into two groups such that
#         the total weight of crossing edges is minimal.
# right : Stoer-Wagner principle —— Maximum Adjacency Search (MAS) grows a
#         set A one vertex at a time; the last two vertices (s,t) give the
#         cut-of-the-phase, then s is contracted into t.

random.seed(7)

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

BG = '#0d1117'
EDGE = '#58a6ff'
CUT = '#f85149'
GREY = '#c9d1d9'

# ---------------- Left panel: weighted graph with min-cut ----------------
ax = axes[0]
ax.set_facecolor(BG)

# Two clusters A (left) and B (right), connected by a narrow "bottleneck"
# of 3 edges whose total weight is the global min-cut.
posA = {  # left cluster
    0: (-2.0, 1.2),
    1: (-2.0, -1.2),
    2: (-0.8, 1.6),
    3: (-0.8, -1.6),
}
posB = {  # right cluster
    4: (2.0, 1.2),
    5: (2.0, -1.2),
    6: (0.8, 1.6),
    7: (0.8, -1.6),
}
pos = {}
pos.update(posA)
pos.update(posB)

# intra-cluster edges (heavy, keep each cluster strongly connected)
intra = [
    (0, 2, 8), (0, 3, 7), (1, 2, 6), (1, 3, 9), (0, 1, 10), (2, 3, 11),
    (4, 6, 8), (4, 7, 7), (5, 6, 9), (5, 7, 6), (4, 5, 10), (6, 7, 11),
]
# bottleneck edges crossing the cut (small weights -> min cut)
cut_edges = [
    (2, 6, 2),   # crossing
    (3, 7, 3),   # crossing
    (3, 6, 1),   # crossing
]

def draw_edge(ax, s, t, w, color, lw, style='-', rad=0.0):
    sx, sy = pos[s]
    dx, dy = pos[t]
    ax.plot([sx, dx], [sy, dy], color=color, lw=lw, linestyle=style,
            zorder=1, alpha=0.9)
    mx, my = (sx + dx) / 2, (sy + dy) / 2
    ax.text(mx, my + 0.06, str(w), color='white', fontsize=9,
            ha='center', va='center', zorder=3, fontweight='bold')

for (s, t, w) in intra:
    draw_edge(ax, s, t, w, EDGE, 2.0)
for (s, t, w) in cut_edges:
    draw_edge(ax, s, t, w, CUT, 3.0)

# vertices
for v, (x, y) in pos.items():
    c = '#3fb950' if v in posA else '#f0883e'
    ax.scatter(x, y, s=340, color=c, zorder=4, edgecolors='white', linewidths=1.5)
    ax.text(x, y, str(v), color='#0d1117', fontsize=11, ha='center',
            va='center', zorder=5, fontweight='bold')

# dashed cut line through the bottleneck
ax.plot([0.0, 0.0], [-2.4, 2.4], color=CUT, lw=2.5, linestyle='--', zorder=2)
ax.text(0.0, 2.65, 'min-cut (2+3+1 = 6)', color=CUT, fontsize=12,
        ha='center', fontweight='bold')

ax.text(-1.4, -2.55, 'side A', color='#3fb950', fontsize=12,
        ha='center', fontweight='bold')
ax.text(1.4, -2.55, 'side B', color='#f0883e', fontsize=12,
        ha='center', fontweight='bold')

ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-3.0, 3.1)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
ax.set_title('Global Min-Cut: split vertices to minimize crossing weight',
             color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# ---------------- Right panel: MAS + contraction principle ----------------
ax2 = axes[1]
ax2.set_facecolor(BG)

# Show MAS growing a set A across phases. We draw a small graph where
# vertices are selected in Maximum-Adjacency order, and s,t get contracted.
pp = {
    0: (-0.4, 2.0),
    1: (-2.0, 0.4),
    2: (1.8, 0.6),
    3: (-0.6, -0.6),
}
ee = [(0, 1, 5), (0, 2, 8), (0, 3, 6), (1, 3, 4), (2, 3, 7)]

for (s, t, w) in ee:
    sx, sy = pp[s]; dx, dy = pp[t]
    ax2.plot([sx, dx], [sy, dy], color=EDGE, lw=2.0, zorder=1, alpha=0.85)
    ax2.text((sx+dx)/2, (sy+dy)/2 + 0.05, str(w), color='white', fontsize=9,
             ha='center', va='center', zorder=3, fontweight='bold')

# highlight the last two selected vertices s (to contract) and t
ax2.scatter(pp[2][0], pp[2][1], s=420, color='#f85149', zorder=4,
            edgecolors='white', linewidths=2)
ax2.text(pp[2][0], pp[2][1], 's', color='#0d1117', fontsize=12, ha='center',
         va='center', zorder=5, fontweight='bold')
ax2.scatter(pp[0][0], pp[0][1], s=340, color='#f0883e', zorder=4,
            edgecolors='white', linewidths=1.5)
ax2.text(pp[0][0], pp[0][1], 't', color='#0d1117', fontsize=12, ha='center',
         va='center', zorder=5, fontweight='bold')
for v in (1, 3):
    ax2.scatter(pp[v][0], pp[v][1], s=340, color='#c9d1d9', zorder=4,
                edgecolors='white', linewidths=1.5)
    ax2.text(pp[v][0], pp[v][1], str(v), color='#0d1117', fontsize=12,
             ha='center', va='center', zorder=5, fontweight='bold')

ax2.annotate('cut-of-the-phase = w[s]', xy=pp[2], xytext=(3.2, 1.4),
             color='#f85149', fontsize=11, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#f85149', lw=1.8))
ax2.annotate('contract s into t', xy=pp[0], xytext=(-3.6, 1.2),
             color='#f0883e', fontsize=11, fontweight='bold',
             arrowprops=dict(arrowstyle='->', color='#f0883e', lw=1.8))

ax2.text(0.4, -2.2,
         'Maximum Adjacency Search: grow set A by max\n'
         'connection weight; last two vertices (s, t) give\n'
         'the phase cut, then merge s into t  →  O(n³)',
         color='#c9d1d9', fontsize=11, ha='center')

ax2.set_xlim(-4.2, 4.2)
ax2.set_ylim(-2.9, 2.9)
ax2.set_aspect('equal')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title('Stoer-Wagner: MAS + Phase Contraction (n-1 phases)',
              color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor=BG, bbox_inches='tight')
print('cover.png generated')
