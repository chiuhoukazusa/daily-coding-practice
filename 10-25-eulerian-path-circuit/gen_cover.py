import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Eulerian Path & Circuit (Hierholzer's Algorithm) cover:
# left  : a directed graph with a single Eulerian trail (the classic
#         "one-stroke" drawing). Red vertices show the unique start/end
#         (out-in = +1 / in-out = +1); the trail visits every edge once.
# right : Hierholzer 原理示意 —— 贪心 DFS 走到死胡同后回边拼接子回路。

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# ---------------- Left panel: one-stroke Eulerian path ----------------
ax = axes[0]
ax.set_facecolor('#0d1117')

# Classic "house with X" (5 vertices, 8 edges) admitting an Eulerian path
# vertex layout
pos = {
    0: (0.0, 1.6),   # top
    1: (0.0, 0.0),   # bottom-left
    2: (2.0, 0.0),   # bottom-right
    3: (2.0, 1.6),   # top-right
    4: (1.0, 2.6),   # roof peak
}
edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),   # outer square (counterclockwise)
    (0, 2),                           # diagonal 0->2
    (1, 3),                           # diagonal 1->3
]
# To make a valid Eulerian PATH, add 4->0 and 3->4 (roof)
edges += [(4, 0), (3, 4)]

def draw_edge(ax, src, dst, color='#58a6ff', lw=2.4):
    sx, sy = pos[src]
    dx, dy = pos[dst]
    # slightly shorten for arrowhead
    ax.annotate('', xy=(dx, dy), xytext=(sx, sy),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw,
                                shrinkA=10, shrinkB=10,
                                connectionstyle='arc3,rad=0.12'))

for (s, t) in edges:
    draw_edge(ax, s, t, color='#58a6ff', lw=2.4)

# vertices
for v, (x, y) in pos.items():
    c = '#c9d1d9'
    s = 260
    if v == 4:      # unique start (out-in = +1)
        c = '#3fb950'; s = 360
    elif v == 3:    # unique end (in-out = +1)
        c = '#f0883e'; s = 360
    ax.scatter(x, y, s=s, color=c, zorder=3, edgecolors='white', linewidths=1.5)
    ax.text(x, y, str(v), color='#0d1117', fontsize=11, ha='center',
            va='center', zorder=4, fontweight='bold')

ax.text(1.0, 1.6, 'one-stroke:\nvisit every edge exactly once',
        color='white', fontsize=11, ha='center', va='center', zorder=2,
        fontweight='bold', alpha=0.9)

ax.annotate('start (out-in=+1)', xy=pos[4], xytext=(1.0, 3.0),
            color='#3fb950', fontsize=10, fontweight='bold', ha='center',
            arrowprops=dict(arrowstyle='->', color='#3fb950', lw=1.5))
ax.annotate('end (in-out=+1)', xy=pos[3], xytext=(2.9, 2.1),
            color='#f0883e', fontsize=10, fontweight='bold', ha='center',
            arrowprops=dict(arrowstyle='->', color='#f0883e', lw=1.5))

ax.set_xlim(-0.8, 3.6)
ax.set_ylim(-0.7, 3.4)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
ax.set_title('Eulerian Path: Traverse Every Edge Exactly Once',
             color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# ---------------- Right panel: Hierholzer principle ----------------
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

# Show a cycle that splits into two sub-cycles which Hierholzer splices.
# Two overlapping loops: big outer loop C1 and a smaller inner loop C2
# sharing the vertex 'v'.
cx, cy, R = 0.0, 0.4, 1.5
theta = np.linspace(0, 2 * np.pi, 200)
ax2.plot(cx + R * np.cos(theta), cy + R * np.sin(theta),
         color='#58a6ff', lw=3, label='sub-circuit C1')

# inner loop C2 (shares vertex v at angle 0)
cx2, cy2, R2 = cx, cy, 0.75
theta2 = np.linspace(0, 2 * np.pi, 200)
ax2.plot(cx2 + R2 * np.cos(theta2), cy2 + R2 * np.sin(theta2),
         color='#3fb950', lw=3, label='sub-circuit C2')

# shared vertex v (rightmost point)
vx, vy = cx + R, cy
ax2.scatter(vx, vy, s=220, color='#f0883e', zorder=4,
            edgecolors='white', linewidths=2)
ax2.text(vx + 0.15, vy + 0.15, 'v (splice)', color='#f0883e',
         fontsize=11, fontweight='bold')

# start point on C1 (leftmost)
sx0, sy0 = cx - R, cy
ax2.scatter(sx0, sy0, s=220, color='#3fb950', zorder=4,
            edgecolors='white', linewidths=2)
ax2.text(sx0 - 0.2, sy0 - 0.45, 'start', color='#3fb950',
         fontsize=11, fontweight='bold', ha='center')

# arrow markers to show direction
ax2.annotate('', xy=(cx + R - 0.4, cy - 0.1), xytext=(cx - R + 0.5, cy - 0.1),
             arrowprops=dict(arrowstyle='->', color='#58a6ff', lw=2.4))
ax2.annotate('', xy=(vx - 0.15, cy + R2 - 0.1), xytext=(vx - 0.15, cy - R2 + 0.15),
             arrowprops=dict(arrowstyle='->', color='#3fb950', lw=2.4))

ax2.text(0.0, -1.1,
         'Hierholzer: greedy DFS until dead-end,\n'
         'backtrack & splice sub-circuits  →  O(E)',
         color='#c9d1d9', fontsize=11, ha='center')

ax2.set_xlim(-2.6, 2.4)
ax2.set_ylim(-2.4, 2.2)
ax2.set_aspect('equal')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title('Hierholzer: Splice Sub-Circuits (O(E)) vs Fleury O(E²)',
              color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax2.spines.values():
    spine.set_color('#30363d')

ax2.legend(loc='lower left', frameon=False, fontsize=10,
           labelcolor='white')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
