import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Floyd's Cycle Detection (Tortoise & Hare) cover:
# left : a linked list shaped like the Greek letter rho (ρ)
#        with a straight tail (mu) leading into a loop (lambda),
#        showing the slow/fast pointers meeting inside the loop.
# right: a small "meeting point -> entry -> length" schematic.

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# ---------------- Left panel: rho-shaped list ----------------
ax = axes[0]
ax.set_facecolor('#0d1117')

# Build a rho shape: straight tail from left into a circular loop on the right.
# Loop center and radius
cx, cy, R = 3.2, 0.0, 1.6
# Loop nodes (lambda = 6)
loop_angles = np.linspace(0, 2 * np.pi, 7)[:-1]  # 6 points, leaving a gap at the "entry"
loop_pts = [(cx + R * np.cos(a), cy + R * np.sin(a)) for a in loop_angles]
# entry node is the rightmost point of the loop (angle 0)
entry = loop_pts[0]

# Tail nodes (mu = 3) going from far left to the entry
tail_pts = [(-2.5, 0.0), (-1.2, 0.2), (0.1, 0.35), entry]

def draw_arrow(ax, src, dst, color, lw=2, style='-', head=True):
    ax.annotate('', xy=dst, xytext=src,
                arrowprops=dict(arrowstyle='->' if head else '-',
                                color=color, lw=lw, linestyle=style,
                                shrinkA=6, shrinkB=6))

# Draw tail segments
for i in range(len(tail_pts) - 1):
    draw_arrow(ax, tail_pts[i], tail_pts[i + 1], color='#58a6ff', lw=2.5)

# Draw loop segments
for i in range(len(loop_pts)):
    draw_arrow(ax, loop_pts[i], loop_pts[(i + 1) % len(loop_pts)],
               color='#3fb950', lw=2.5)

# Nodes
node_color = '#c9d1d9'
ax.scatter(*zip(*tail_pts[:-1]), s=140, color=node_color, zorder=3,
           edgecolors='white', linewidths=1.2)
# entry node highlighted orange
ax.scatter(entry[0], entry[1], s=220, color='#f0883e', zorder=3,
           edgecolors='white', linewidths=2)
for i, (x, y) in enumerate(tail_pts[:-1]):
    ax.text(x, y, f'{i}', color='#0d1117', fontsize=9, ha='center',
            va='center', zorder=4, fontweight='bold')
ax.text(entry[0], entry[1], 'mu', color='#0d1117', fontsize=9, ha='center',
        va='center', zorder=4, fontweight='bold')

# slow/fast pointers inside the loop (meeting point)
meet = loop_pts[3]
ax.scatter(meet[0], meet[1], s=180, color='#bc8cff', zorder=5,
           edgecolors='white', linewidths=2)
ax.text(meet[0], meet[1], 'meet', color='#0d1117', fontsize=8, ha='center',
        va='center', zorder=6, fontweight='bold')

# labels
ax.text(cx, cy + R + 0.7, 'cycle (lambda)', color='#3fb950',
        fontsize=12, fontweight='bold', ha='center')
ax.text(2.0, -R - 0.8, 'tail (mu)\nslow (1 step) / fast (2 steps)',
        color='#58a6ff', fontsize=10, ha='center')

ax.set_xlim(-3.3, 5.6)
ax.set_ylim(-3.4, 3.0)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
ax.set_title('Rho-shaped List: Slow/Fast Meet Inside the Cycle',
             color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_color('#30363d')

# ---------------- Right panel: entry-length schematic ----------------
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

# A simple horizontal segment split into: head --mu--> entry --lambda--> (back to entry)
# visual ruler showing mu and lambda
x0 = 0.0
mu = 3.0
lam = 4.0
y = 0.6

# head point
ax2.scatter(x0, y, s=140, color='#f0883e', zorder=3,
            edgecolors='white', linewidths=1.5)
ax2.text(x0, y + 0.35, 'head', color='white', fontsize=10, ha='center', fontweight='bold')

# segment mu
draw_arrow(ax2, (x0, y), (x0 + mu, y), color='#58a6ff', lw=3)
ax2.text(x0 + mu / 2, y + 0.3, 'mu (tail length)', color='#58a6ff',
         fontsize=11, ha='center', fontweight='bold')

# entry node
ax2.scatter(x0 + mu, y, s=140, color='#f0883e', zorder=3,
            edgecolors='white', linewidths=1.5)
ax2.text(x0 + mu, y + 0.35, 'entry', color='#f0883e', fontsize=10,
         ha='center', fontweight='bold')

# loop (lambda) drawn as a rounded arc back to entry
lam_y = y
# draw an arc from entry going right and looping back (semi-ellipse below)
theta = np.linspace(0, np.pi, 100)
arc_x = x0 + mu + lam / 2 + (lam / 2) * np.cos(theta)
arc_y = y - 0.5 - 0.8 * np.sin(theta)
ax2.plot(arc_x, arc_y, color='#3fb950', lw=3)
ax2.annotate('', xy=(x0 + mu, y), xytext=(arc_x[-2], arc_y[-2]),
             arrowprops=dict(arrowstyle='->', color='#3fb950', lw=3,
                             shrinkA=6, shrinkB=6))
ax2.text(x0 + mu + lam / 2, y - 1.4, 'lambda (cycle length)',
         color='#3fb950', fontsize=11, ha='center', fontweight='bold')

# key formula
ax2.text(0.5, -2.4,
         'entry: head→entry = meet→entry (mod lambda)\n'
         'length: one pass around loop from meet',
         color='#c9d1d9', fontsize=11, ha='center')

ax2.set_xlim(-0.6, x0 + mu + lam + 0.6)
ax2.set_ylim(-3.0, 1.8)
ax2.set_aspect('equal')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title('Entry (mu) & Length (lambda) Recovery',
              color='white', fontsize=13, fontweight='bold', pad=10)
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
