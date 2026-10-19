"""Small 2-D geometry helpers (Gazebo ENU metres)."""

import math


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rect(cx, cy, yaw, length, width):
    """Corners of a length x width rectangle centred at (cx, cy) with heading yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def point_in_poly(p, poly):
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def seg_point_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def seg_intersect(p0, p1, a, b):
    """(t, s) where p0 + t (p1 - p0) == a + s (b - a), both in [0, 1]; None otherwise."""
    rx, ry = p1[0] - p0[0], p1[1] - p0[1]
    sx, sy = b[0] - a[0], b[1] - a[1]
    den = rx * sy - ry * sx
    if abs(den) < 1e-12:
        return None
    qx, qy = a[0] - p0[0], a[1] - p0[1]
    t = (qx * sy - qy * sx) / den
    s = (qx * ry - qy * rx) / den
    if 0.0 <= t <= 1.0 and 0.0 <= s <= 1.0:
        return t, s
    return None


def poly_circle(poly, c, r):
    if point_in_poly(c, poly):
        return True
    n = len(poly)
    return any(seg_point_dist(c, poly[i], poly[(i + 1) % n]) <= r for i in range(n))


def poly_poly(pa, pb):
    if any(point_in_poly(p, pb) for p in pa) or any(point_in_poly(p, pa) for p in pb):
        return True
    na, nb = len(pa), len(pb)
    for i in range(na):
        for j in range(nb):
            if seg_intersect(pa[i], pa[(i + 1) % na], pb[j], pb[(j + 1) % nb]):
                return True
    return False
