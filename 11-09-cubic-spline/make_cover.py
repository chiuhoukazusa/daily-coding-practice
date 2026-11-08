import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 复现 C++ 中的自然三次样条（用同样的算法，确保数据一致）
def thomas(a, b, c, d):
    n = len(d)
    bp = np.array(b, dtype=float).copy()
    cp = np.array(c, dtype=float).copy()
    dp = np.array(d, dtype=float).copy()
    x = np.zeros(n)
    for i in range(1, n):
        m = a[i] / bp[i-1]
        bp[i] -= m * cp[i-1]
        dp[i] -= m * dp[i-1]
    x[n-1] = dp[n-1] / bp[n-1]
    for i in range(n-2, -1, -1):
        x[i] = (dp[i] - cp[i] * x[i+1]) / bp[i]
    return x

def natural_spline(X, Y):
    n = len(X)
    h = np.diff(X)
    lo, diag, up, rhs = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    diag[0] = 1.0; rhs[0] = 0.0
    diag[n-1] = 1.0; rhs[n-1] = 0.0
    for i in range(1, n-1):
        lo[i] = h[i-1]
        diag[i] = 2.0*(h[i-1]+h[i])
        up[i] = h[i]
        dy1 = (Y[i+1]-Y[i])/h[i]
        dy0 = (Y[i]-Y[i-1])/h[i-1]
        rhs[i] = 6.0*(dy1-dy0)
    M = thomas(lo, diag, up, rhs)
    a = Y[:-1].astype(float)
    b = (Y[1:]-Y[:-1])/h - h*(2.0*M[:-1]+M[1:])/6.0
    c = M[:-1]/2.0
    d = (M[1:]-M[:-1])/(6.0*h)
    return X, h, a, b, c, d

def eval_spline(XX, X, h, a, b, c, dd, x):
    i = np.searchsorted(X, x, side='right') - 1
    i = max(0, min(i, len(a)-1))
    t = x - X[i]
    return a[i] + b[i]*t + c[i]*t*t + dd[i]*t*t*t

# --- 图 1: Runge 现象对照 ---
runge = lambda x: 1.0/(1.0+25.0*x*x)
X11 = np.linspace(-1, 1, 11)
Y11 = runge(X11)
Xs, h, a, b, c, d = natural_spline(X11, Y11)

# 全局多项式 (用 numpy polyfit 简化以匹配 C++ 的 Lagrange)
poly = np.polyfit(X11, Y11, 10)

xs = np.linspace(-1, 1, 800)
ys_spline = np.array([eval_spline(X11, Xs, h, a, b, c, d, x) for x in xs])
ys_poly = np.polyval(poly, xs)
ys_true = runge(xs)

fig, axes = plt.subplots(1, 2, figsize=(16, 7))

ax = axes[0]
ax.plot(xs, ys_true, 'k-', lw=2, label='ground truth', zorder=3)
ax.plot(xs, ys_poly, 'r--', lw=1.5, label='global polynomial (Runge)')
ax.plot(xs, ys_spline, 'b-', lw=2, label='natural cubic spline')
ax.scatter(X11, Y11, c='k', s=40, zorder=5, label='nodes')
ax.set_title('Runge function $f(x)=1/(1+25x^2)$, 11 equidistant nodes')
ax.set_xlabel('x'); ax.set_ylabel('f(x)')
ax.set_ylim(-1, 1.6)
ax.legend(loc='upper center', fontsize=9)
ax.grid(alpha=0.3)

ax = axes[1]
ax.plot(xs, np.abs(ys_spline - ys_true), 'b-', lw=2, label='|spline error|')
ax.plot(xs, np.abs(ys_poly - ys_true), 'r--', lw=1.5, label='|polynomial error|')
ax.set_yscale('log')
ax.set_title('Absolute interpolation error (log scale)')
ax.set_xlabel('x'); ax.set_ylabel('|error|')
ax.legend(fontsize=9)
ax.grid(alpha=0.3, which='both')

plt.tight_layout()
plt.savefig('cover.png', dpi=110, bbox_inches='tight')
print('cover.png saved:', __import__('os').path.getsize('cover.png'), 'bytes')
