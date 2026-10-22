import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Edmonds' Blossom Algorithm cover:
# left: a graph with an odd cycle (blossom) being contracted to a single vertex
# right: max matching size on the "sunflower" graph (overlapping blossoms)

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# --- Left panel: blossom contraction (odd cycle -> single vertex) ---
ax = axes[0]
ax.set_facecolor('#0d1117')

# Draw a pentagon (odd cycle C5) with a dangling edge
# pentagon vertices
pts = {
    0: (0.0, 0.0),
    1: (1.0, 0.0),
    2: (1.309, 0.951),
    3: (0.5, 1.539),
    4: (-0.309, 0.951),
}
# dangling vertex attached to vertex 0
pts[5] = (-1.0, 0.0)

# edges of the pentagon
pent_edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)]
dangling_edges = [(0, 5)]

# draw blossom edges (the odd cycle) in green, dangling edge in blue
for (u, v) in pent_edges:
    x = [pts[u][0], pts[v][0]]
    y = [pts[u][1], pts[v][1]]
    ax.plot(x, y, color='#3fb950', lw=2.5, zorder=1)
for (u, v) in dangling_edges:
    x = [pts[u][0], pts[v][0]]
    y = [pts[u][1], pts[v][1]]
    ax.plot(x, y, color='#58a6ff', lw=2.5, zorder=1)

# draw vertices
for v, (x, y) in pts.items():
    color = '#f0883e' if v == 0 else '#c9d1d9'
    ax.scatter(x, y, s=180, color=color, zorder=3, edgecolors='white', linewidths=1.5)
    ax.text(x, y, str(v), color='#0d1117', fontsize=10,
            ha='center', va='center', zorder=4, fontweight='bold')

# annotate the blossom (odd cycle)
ax.text(1.35, 1.9, 'odd cycle (blossom)', color='#3fb950',
        fontsize=12, fontweight='bold', ha='center')
ax.annotate('', xy=(0.9, 1.3), xytext=(1.2, 1.75),
            arrowprops=dict(arrowstyle='->', color='#f85149', lw=2))
ax.text(1.55, -0.4, 'contract \u2192 1 vertex', color='#f85149',
        fontsize=12, fontweight='bold', ha='center')

ax.set_xlim(-1.6, 2.6)
ax.set_ylim(-0.9, 2.3)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
ax.set_title('Blossom Contraction: Odd Cycle \u2192 Single Vertex',
             color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# --- Right panel: sunflower graph (overlapping blossoms) ---
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

# sunflower: k triangles sharing center vertex 0
k = 5
center = (0.0, 0.0)
petals = []
for i in range(k):
    ang1 = 2 * np.pi * i / k
    ang2 = 2 * np.pi * (i + 0.5) / k
    r1, r2 = 1.0, 1.35
    a = (r1 * np.cos(ang1), r1 * np.sin(ang1))
    b = (r2 * np.cos(ang2), r2 * np.sin(ang2))
    petals.append((a, b))

for (a, b) in petals:
    # center-a, center-b (edges to center), a-b (petal edge) -> triangle
    ax2.plot([center[0], a[0]], [center[1], a[1]], color='#58a6ff', lw=2, alpha=0.85)
    ax2.plot([center[0], b[0]], [center[1], b[1]], color='#58a6ff', lw=2, alpha=0.85)
    ax2.plot([a[0], b[0]], [a[1], b[1]], color='#bc8cff', lw=2)

ax2.scatter(*center, s=260, color='#f0883e', zorder=3, edgecolors='white', linewidths=1.5)
ax2.text(*center, '0', color='#0d1117', fontsize=11, ha='center', va='center',
         zorder=4, fontweight='bold')
for idx, (a, b) in enumerate(petals):
    ax2.scatter(*a, s=120, color='#c9d1d9', zorder=3, edgecolors='white', linewidths=1)
    ax2.scatter(*b, s=120, color='#c9d1d9', zorder=3, edgecolors='white', linewidths=1)

ax2.set_xlim(-1.8, 1.8)
ax2.set_ylim(-1.8, 1.8)
ax2.set_aspect('equal')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title('Sunflower Graph: Overlapping Odd Cycles (center 0)',
              color='white', fontsize=13, fontweight='bold', pad=10)
ax2.text(0, -1.65, 'max matching = #petals (each triangle contributes 1 edge)',
         color='#c9d1d9', fontsize=11, ha='center')
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
