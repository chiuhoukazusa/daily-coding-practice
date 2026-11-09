import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 复现 C++ 的 RANSAC 演示数据，绘制对比图作为封面
rng = np.random.default_rng(42)

def ransac_line_fit(x, y, thresh, iters, seed):
    r = np.random.default_rng(seed)
    n = len(x)
    best = None; best_cnt = 0
    for _ in range(iters):
        i1, i2 = r.integers(0, n, 2)
        if i1 == i2: continue
        dx = x[i2] - x[i1]
        if abs(dx) < 1e-9: continue
        a = (y[i2] - y[i1]) / dx
        b = y[i1] - a*x[i1]
        d = np.abs(a*x - y + b) / np.sqrt(a*a + 1.0)
        cnt = np.sum(d < thresh)
        if cnt > best_cnt:
            best_cnt = cnt; best = (a, b)
    a, b = best
    mask = np.abs(a*x - y + b)/np.sqrt(a*a+1.0) < thresh
    aa, bb = np.polyfit(x[mask], y[mask], 1)
    return aa, bb, mask

def ransac_circle_fit(x, y, thresh, iters, seed):
    r = np.random.default_rng(seed)
    n = len(x)
    best = None; best_cnt = 0
    for _ in range(iters):
        idx = r.integers(0, n, 3)
        if len(set(idx)) < 3: continue
        x1,y1,x2,y2,x3,y3 = x[idx[0]],y[idx[0]],x[idx[1]],y[idx[1]],x[idx[2]],y[idx[2]]
        d0 = 2*(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))
        if abs(d0) < 1e-9: continue
        ux = ((x1*x1+y1*y1)*(y2-y3)+(x2*x2+y2*y2)*(y3-y1)+(x3*x3+y3*y3)*(y1-y2))/d0
        uy = ((x1*x1+y1*y1)*(x3-x2)+(x2*x2+y2*y2)*(x1-x3)+(x3*x3+y3*y3)*(x2-x1))/d0
        rr = np.sqrt((x1-ux)**2+(y1-uy)**2)
        dist = np.abs(np.hypot(x-ux, y-uy) - rr)
        cnt = np.sum(dist < thresh)
        if cnt > best_cnt:
            best_cnt = cnt; best = (ux, uy, rr)
    ux, uy, rr = best
    mask = np.abs(np.hypot(x-ux, y-uy)-rr) < thresh
    # Kasa 圆拟合精修
    xx, yy = x[mask], y[mask]
    z = xx*xx + yy*yy
    A = np.array([[np.sum(xx*xx), np.sum(xx*yy), np.sum(xx)],
                  [np.sum(xx*yy), np.sum(yy*yy), np.sum(yy)],
                  [np.sum(xx), np.sum(yy), len(xx)]])
    B = np.array([np.sum(xx*z), np.sum(yy*z), np.sum(z)])
    sol = np.linalg.solve(A, B)
    cx, cy = sol[0]/2, sol[1]/2
    cr = np.sqrt(sol[2] + cx*cx + cy*cy)
    return cx, cy, cr, mask

# ---- 数据生成（与 C++ 一致）----
def gen_line(n_in, n_out, a, b, sigma, out_range, seed):
    r = np.random.default_rng(seed)
    n = n_in + n_out
    x = np.zeros(n); y = np.zeros(n)
    perm = r.permutation(n)
    for k in range(n):
        i = perm[k]
        xx = (r.random()-0.5)*20.0
        if k < n_in:
            x[i] = xx; y[i] = a*xx + b + sigma*r.standard_normal()
        else:
            x[i] = xx; y[i] = (r.random()-0.5)*out_range
    return x, y

def gen_circle(n_in, n_out, cx, cy, rr, sigma, out_range, seed):
    r = np.random.default_rng(seed)
    n = n_in + n_out
    x = np.zeros(n); y = np.zeros(n)
    perm = r.permutation(n)
    for k in range(n):
        i = perm[k]
        if k < n_in:
            th = r.random()*2*np.pi
            rrad = rr + sigma*r.standard_normal()
            x[i] = cx + rrad*np.cos(th); y[i] = cy + rrad*np.sin(th)
        else:
            x[i] = (r.random()-0.5)*out_range; y[i] = (r.random()-0.5)*out_range
    return x, y

fig, axes = plt.subplots(1, 2, figsize=(16, 7))

# ---- 左图：直线拟合（40% 离群点）----
x, y = gen_line(100, 67, 3.0, 1.0, 0.3, 30.0, 12345)  # ~40% outliers
a_ransac, b_ransac, mask_ransac = ransac_line_fit(x, y, 1.0, 1000, 42)
a_ols, b_ols = np.polyfit(x, y, 1)

ax = axes[0]
ax.scatter(x[~mask_ransac], y[~mask_ransac], c='#d9d9d9', s=12, label='outliers', zorder=2)
ax.scatter(x[mask_ransac], y[mask_ransac], c='#1f77b4', s=18, label='inliers (RANSAC)', zorder=3)
xs = np.linspace(-10, 10, 400)
ax.plot(xs, 3.0*xs + 1.0, 'k-', lw=2, label='ground truth y=3x+1', zorder=5)
ax.plot(xs, a_ransac*xs + b_ransac, 'g--', lw=2, label=f'RANSAC y={a_ransac:.2f}x{b_ransac:+.2f}', zorder=4)
ax.plot(xs, a_ols*xs + b_ols, 'r:', lw=2.5, label=f'OLS y={a_ols:.2f}x{b_ols:+.2f}', zorder=4)
ax.set_xlim(-10, 10); ax.set_ylim(-15, 35)
ax.set_title('Line fitting with ~40% outliers')
ax.set_xlabel('x'); ax.set_ylabel('y')
ax.legend(loc='upper left', fontsize=8)
ax.grid(alpha=0.3)

# ---- 右图：圆拟合（50% 离群点）----
x, y = gen_circle(100, 100, 2.0, -1.0, 5.0, 0.3, 30.0, 777)
cx_r, cy_r, cr_r, mask_r = ransac_circle_fit(x, y, 1.0, 2000, 99)

ax = axes[1]
ax.scatter(x[~mask_r], y[~mask_r], c='#d9d9d9', s=12, label='outliers', zorder=2)
ax.scatter(x[mask_r], y[mask_r], c='#ff7f0e', s=16, label='inliers (RANSAC)', zorder=3)
th = np.linspace(0, 2*np.pi, 400)
ax.plot(2.0+5*np.cos(th), -1.0+5*np.sin(th), 'k-', lw=2, label='ground truth r=5', zorder=5)
ax.plot(cx_r+cr_r*np.cos(th), cy_r+cr_r*np.sin(th), 'g--', lw=2,
        label=f'RANSAC r={cr_r:.2f}', zorder=4)
ax.set_xlim(-15, 15); ax.set_ylim(-15, 15)
ax.set_aspect('equal')
ax.set_title('Circle fitting with 50% outliers')
ax.set_xlabel('x'); ax.set_ylabel('y')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

plt.suptitle('RANSAC Robust Model Fitting vs. Least-Squares under Heavy Outliers', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('cover.png', dpi=110, bbox_inches='tight')
import os
print('cover.png saved:', os.path.getsize('cover.png'), 'bytes')
