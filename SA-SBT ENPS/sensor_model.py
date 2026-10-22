"""
Geometric reconstruction of E-puck IR sensor readings.

Webots applies a reflectance factor f before the lookup table:
    f = 0.2 + 0.8 * red_level * (1 - 0.5*roughness) * (1 - 0.5*occlusion)

For the road border f = 0.3086, so the effective detection range is
0.07 * 0.3086 = 2.16 cm geometric — not the 7 cm the table suggests.
"""

import csv
import math

import numpy as np

REFLECTANCE_ROAD = 0.3086     # empirically measured for the road border
SENSOR_RANGE = 0.07
MAX_REPORTED = 0.06           # UtilWebots.get() returns 0 beyond this
GRID = [0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]

LOOKUP = [(0.0, 4095.0), (0.005, 2133.33), (0.01, 1465.73), (0.015, 601.46),
          (0.02, 383.84), (0.03, 234.93), (0.04, 158.03), (0.05, 120.0),
          (0.06, 104.09), (0.07, 67.19)]

WHEEL_RADIUS = 0.0205
AXLE = 0.052
KV   = 0.977                  # linear velocity scale, calibrated against GPS traces
KW   = 0.946                  # angular velocity scale, calibrated against GPS traces
DT = 0.032
GPS_OFFSET = -0.0095          # GPS is mounted 9.5 mm behind the robot centre

# mounting positions from E-puck.proto, robot frame (x forward, y left)
MOUNTS = [("s1", (0.030, 0.010)), ("s2", (0.022, 0.025)), ("s3", (0.0, 0.031)),
          ("s4", (0.030, -0.010)), ("s5", (0.022, -0.025)), ("s6", (0.0, -0.031))]


def distance_to_raw(distance):
    if distance >= LOOKUP[-1][0]:
        return LOOKUP[-1][1]
    for i in range(len(LOOKUP) - 1):
        d0, r0 = LOOKUP[i]
        d1, r1 = LOOKUP[i + 1]
        if d0 <= distance <= d1:
            return r0 + (distance - d0) / (d1 - d0) * (r1 - r0)
    return LOOKUP[0][1]


def raw_to_value(raw):
    reported = LOOKUP[-1][0]
    for level, threshold in reversed(LOOKUP):
        if raw <= threshold:
            reported = level
            break
    if reported >= MAX_REPORTED:
        return 0.0
    return MAX_REPORTED - reported


def geometric_to_value(distance, reflectance=REFLECTANCE_ROAD):
    effective = distance / reflectance
    if effective >= SENSOR_RANGE:
        return 0.0
    return raw_to_value(distance_to_raw(effective))


def read_spine(path):
    points = []
    for line in open(path).read().replace("\r\n", "\n").strip().split("\n"):
        parts = line.split()
        if len(parts) >= 2:
            points.append((float(parts[0]), float(parts[1])))
    return points


def borders(spine, half_width):
    left, right = [], []
    for i, p in enumerate(spine):
        a = spine[max(i - 1, 0)]
        b = spine[min(i + 1, len(spine) - 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(tx, ty) or 1.0
        tx, ty = tx / norm, ty / norm
        left.append((p[0] - ty * half_width, p[1] + tx * half_width))
        right.append((p[0] + ty * half_width, p[1] - tx * half_width))
    return left, right


def as_segments(left, right):
    segs = []
    for line in (left, right):
        for i in range(len(line) - 1):
            segs.append((line[i][0], line[i][1], line[i + 1][0], line[i + 1][1]))
    return np.array(segs)


def ray_distance(ox, oy, dx, dy, segs, max_range):
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    sx, sy = bx - ax, by - ay
    den = dx * sy - dy * sx
    ok = np.abs(den) > 1e-12
    qx, qy = ax - ox, ay - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qx * sy - qy * sx) / den
        u = (qx * dy - qy * dx) / den
    hit = ok & (t >= 0) & (t <= max_range) & (u >= 0) & (u <= 1)
    return t[hit].min() if hit.any() else max_range


def sense(x, y, heading, segs, reflectance=REFLECTANCE_ROAD):
    # (x, y) is the robot centre, not the GPS position
    limit = SENSOR_RANGE * reflectance      # beyond this, no obstacle can be detected
    out = []
    for _, (mx, my) in MOUNTS:
        ca, sa = math.cos(heading), math.sin(heading)
        ox = x + mx * ca - my * sa
        oy = y + mx * sa + my * ca
        angle = heading + math.atan2(my, mx)
        distance = ray_distance(ox, oy, math.cos(angle), math.sin(angle),
                                segs, limit)
        out.append(geometric_to_value(distance, reflectance))
    return out


def gps_to_center(gps_x, gps_y, heading):
    return (gps_x - GPS_OFFSET * math.cos(heading),
            gps_y - GPS_OFFSET * math.sin(heading))


def reconstruct_pose(lw, rw, start, heading0, n):
    x, y = start
    th = heading0
    positions, headings = [(x, y)], [th]
    for i in range(n - 1):
        vl, vr = lw[i] * WHEEL_RADIUS, rw[i] * WHEEL_RADIUS
        v = (vl + vr) / 2.0 * KV
        omega = (vr - vl) / AXLE * KW
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += omega * DT
        positions.append((x, y))
        headings.append(th)
    return positions, headings


def load_trace(prefix):
    rows = list(csv.reader(open(prefix + ".csv")))
    header = [h.strip() for h in rows[1]]
    idx = {h: i for i, h in enumerate(header)}
    data = [r for r in rows[2:] if len(r) > 10]
    lw = [float(r[idx["lw"]]) for r in data]
    rw = [float(r[idx["rw"]]) for r in data]
    sensors = np.array([[float(r[idx[name]]) for name, _ in MOUNTS] for r in data])
    return lw, rw, sensors
