import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Viterbi HMM Decoding cover: HMM trellis diagram + complexity comparison

fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor='#0d1117')

# --- Left panel: HMM trellis (Viterbi decoding) ---
ax = axes[0]
ax.set_facecolor('#0d1117')

N = 3  # hidden states
T = 6  # time steps

state_colors = ['#f0883e', '#58a6ff', '#3fb950']
state_labels = ['sunny', 'cloudy', 'rainy']

# draw states
for t in range(T):
    for i in range(N):
        x, y = t, N - 1 - i
        ax.add_patch(plt.Circle((x, y), 0.28, facecolor=state_colors[i],
                                edgecolor='white', linewidth=1.5, zorder=3))

# transitions; highlight the optimal (Viterbi) path
best_path = [0, 2, 2, 1, 0, 0]
for t in range(T-1):
    for i in range(N):
        for j in range(N):
            x1, y1 = t, N - 1 - i
            x2, y2 = t + 1, N - 1 - j
            on_path = (best_path[t] == i and best_path[t+1] == j)
            ax.plot([x1, x2], [y1, y2],
                    color='#f0883e' if on_path else '#30363d',
                    linewidth=3 if on_path else 1,
                    alpha=0.9 if on_path else 0.3, zorder=1)

ax.set_xlim(-0.6, T - 0.4)
ax.set_ylim(-0.6, N - 0.4)
ax.set_xticks(range(T))
ax.set_xticklabels([f't={t}' for t in range(T)], color='#c9d1d9', fontsize=9)
ax.set_yticks(range(N))
ax.set_yticklabels(state_labels[::-1], color='#c9d1d9', fontsize=9)
ax.set_title('Viterbi Trellis — Optimal Hidden State Path',
             color='white', fontsize=13, fontweight='bold', pad=10)
ax.set_aspect('equal')
for spine in ax.spines.values():
    spine.set_color('#30363d')

# --- Right panel: complexity comparison (log scale) ---
ax2 = axes[1]
ax2.set_facecolor('#0d1117')

trial_sizes = [2, 3, 4, 5, 6]
viterbi_times = [0.0001, 0.0002, 0.0004, 0.0009, 0.0021]
brute_times = [0.0002, 0.001, 0.008, 0.05, 0.35]

ax2.plot(trial_sizes, viterbi_times, 'o-', color='#3fb950',
         linewidth=2.5, markersize=8, label='Viterbi  O(N^2 * T)')
ax2.plot(trial_sizes, brute_times, 's-', color='#f0883e',
         linewidth=2.5, markersize=8, label='Brute-force  O(N^T)')
ax2.set_yscale('log')
ax2.set_xlabel('Observation sequence length T', color='#c9d1d9', fontsize=11)
ax2.set_ylabel('Runtime (s, log scale)', color='#c9d1d9', fontsize=11)
ax2.set_title('Complexity: Viterbi vs Brute-force',
              color='white', fontsize=13, fontweight='bold', pad=10)
ax2.tick_params(colors='#c9d1d9')
ax2.legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9')
ax2.grid(True, alpha=0.3, color='#30363d')
for spine in ax2.spines.values():
    spine.set_color('#30363d')

plt.tight_layout()
plt.savefig('cover.png', dpi=130, facecolor='#0d1117', bbox_inches='tight')
print('cover.png generated')
