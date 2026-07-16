import math

pts = [
    (250 + 200*math.cos(2*math.pi*i/6 - math.pi/2), 250 + 200*math.sin(2*math.pi*i/6 - math.pi/2))
    for i in range(6)
]
print("Convex Hexagon points:")
for i, p in enumerate(pts):
    print(f"  {i}: ({p[0]:.2f}, {p[1]:.2f})")

# Check ear at vertex 0: vertices 5,0,1 form a triangle
def cross(ax, ay, bx, by):
    return ax*by - ay*bx

def is_convex(poly, i):
    n = len(poly)
    prev = poly[(i-1)%n]
    curr = poly[i]
    next_p = poly[(i+1)%n]
    # cross of edges (prev->curr) and (curr->next)
    e1 = (curr[0]-prev[0], curr[1]-prev[1])
    e2 = (next_p[0]-curr[0], next_p[1]-curr[1])
    c = cross(e1[0], e1[1], e2[0], e2[1])
    return c

def point_in_triangle(px, py, a, b, c):
    d1 = cross(b[0]-a[0], b[1]-a[1], px-a[0], py-a[1])
    d2 = cross(c[0]-b[0], c[1]-b[1], px-b[0], py-b[1])
    d3 = cross(a[0]-c[0], a[1]-c[1], px-c[0], py-c[1])
    has_neg = d1 < -1e-9 or d2 < -1e-9 or d3 < -1e-9
    has_pos = d1 > 1e-9 or d2 > 1e-9 or d3 > 1e-9
    print(f"    d1={d1:.6f} d2={d2:.6f} d3={d3:.6f} has_neg={has_neg} has_pos={has_pos}")
    return not (has_neg and has_pos)

for i in range(6):
    c = is_convex(pts, i)
    print(f"Vertex {i}: cross={c:.6f} {'convex' if c > 0 else 'reflex'}")

# Test ear at vertex 0
print("\nTesting ear at vertex 0:")
print(f"  Triangle: {pts[5]}, {pts[0]}, {pts[1]}")
a, b, c = pts[5], pts[0], pts[1]
for j in range(6):
    if j in (5, 0, 1): continue
    p = pts[j]
    print(f"  Checking vertex {j} ({p[0]:.2f}, {p[1]:.2f})")
    inside = point_in_triangle(p[0], p[1], a, b, c)
    print(f"    inside: {inside}")

# The issue: with CCW polygon, vertices that form a convex ear at index 0
# have triangle (5,0,1). All other vertices should be outside.
# But since this is convex, ALL vertices 2,3,4 form a reflex arc...
# Wait - they're all outside the triangle, so ear should be found.
# Let me check if the triangle itself covers everything
print("\nTriangle area:", abs(cross(b[0]-a[0], b[1]-a[1], c[0]-a[0], c[1]-a[1]))*0.5)
