# venv/animation.py
import math
import numpy as np
from kinematics import delta_calcInverse, delta_calcForward
import collision


# ----------------------------
# Primitive shape generators
# ----------------------------
def generate_spiral_descend(start, turns=4, pts=1440, radius=60.0, z_span=100.0):
    cx, cy, cz = float(start[0]), float(start[1]), float(start[2])
    pts_out = []
    for i in range(pts):
        t = float(i) / max(1.0, (pts - 1))
        ang = 2.0 * math.pi * turns * t
        r = radius * (1.0 - 0.9 * t)
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        z = cz - t * z_span
        pts_out.append((x, y, z))
    return pts_out


def generate_spiral_flat(start, turns=4, pts=1440, radius=60.0):
    cx, cy, cz = float(start[0]), float(start[1]), float(start[2])
    pts_out = []
    for i in range(pts):
        t = float(i) / max(1.0, (pts - 1))
        ang = 2.0 * math.pi * turns * t
        r = radius * (0.5 + 0.5 * math.cos(t * math.pi))
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        z = cz
        pts_out.append((x, y, z))
    return pts_out


def generate_lissajous(center=(0, 0, -220), A=40, B=40, a=3, b=2, delta=math.pi / 2, pts=1440, zbob=20):
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    out = []
    for i in range(pts):
        t = float(i) / max(1.0, pts) * 2.0 * math.pi
        x = cx + A * math.sin(a * t + delta)
        y = cy + B * math.sin(b * t)
        z = cz + zbob * math.sin(a * t * 0.5)
        out.append((x, y, z))
    return out


def generate_circle(center, radius=40, pts=720):
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    out = []
    for i in range(pts):
        ang = 2.0 * math.pi * (i / pts)
        x = cx + radius * math.cos(ang)
        y = cy + radius * math.sin(ang)
        out.append((x, y, cz))
    return out


# --- SEGMENTED SHAPES (For Sharp Corners) ---

def generate_zigzag(center=(0, 0, -220), pts=960, width=30, height=40, lines=8):
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    segments = []
    pts_per_line = pts // lines

    for seg in range(lines):
        line_pts = []
        for j in range(pts_per_line):
            frac = j / pts_per_line
            # Local t for this segment (0 to 1)
            # Global t approximation
            angle = math.pi * seg / max(1, lines)

            # Logic to zigzag: we alternate direction or just draw lines?
            # Standard zigzag logic:
            x = cx + width * math.cos(angle)
            y = cy + width * math.sin(angle)

            # Z goes up/down based on global progress or segment parity?
            # Let's keep original logic but break into segments
            z_start = cz + height * (-1) if (seg % 2 == 0) else cz + height * (1)
            z_end = cz + height * (1) if (seg % 2 == 0) else cz + height * (-1)

            z = z_start + (z_end - z_start) * frac
            line_pts.append((x, y, z))
        segments.append(line_pts)
    return segments


def generate_square(center, side=60, z_lift=0, pts=600):
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    half = side / 2.0
    corners = [(-half, -half), (half, -half), (half, half), (-half, half), (-half, -half)]
    steps_per_side = pts // 4

    segments = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[i + 1]
        side_pts = []
        for j in range(steps_per_side):
            t = j / steps_per_side
            x = cx + p1[0] * (1 - t) + p2[0] * t
            y = cy + p1[1] * (1 - t) + p2[1] * t
            side_pts.append((x, y, cz))
        segments.append(side_pts)
    return segments


def generate_triangle(center, side=60, pts=600):
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    h = side * math.sqrt(3) / 2
    v = [(0, h * 2 / 3), (-side / 2, -h / 3), (side / 2, -h / 3), (0, h * 2 / 3)]

    segments = []
    steps = pts // 3
    for i in range(3):
        p1 = v[i]
        p2 = v[i + 1]
        side_pts = []
        for j in range(steps):
            t = j / steps
            x = cx + p1[0] * (1 - t) + p2[0] * t
            y = cy + p1[1] * (1 - t) + p2[1] * t
            side_pts.append((x, y, cz))
        segments.append(side_pts)
    return segments


def generate_square_x(center, side=60, pts=800):
    # Get square segments first
    sq_segments = generate_square(center, side, pts=pts // 2)

    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    h = side / 2.0

    # Diagonals
    diag1 = [(-h, -h), (h, h)]
    diag2 = [(-h, h), (h, -h)]
    steps = pts // 4

    # Seg 1: Diagonal /
    seg_diag1 = []
    for j in range(steps):
        t = j / steps
        seg_diag1.append(
            (cx + diag1[0][0] * (1 - t) + diag1[1][0] * t, cy + diag1[0][1] * (1 - t) + diag1[1][1] * t, cz))

    # Seg 2: Travel top edge (Move without delay? Or with delay?)
    # We treat it as a move segment to position for next diagonal
    seg_travel = []
    for j in range(steps // 2):
        t = j / (steps // 2)
        seg_travel.append(
            (cx + diag1[1][0] * (1 - t) + diag2[0][0] * t, cy + diag1[1][1] * (1 - t) + diag2[0][1] * t, cz))

    # Seg 3: Diagonal \
    seg_diag2 = []
    for j in range(steps):
        t = j / steps
        seg_diag2.append(
            (cx + diag2[0][0] * (1 - t) + diag2[1][0] * t, cy + diag2[0][1] * (1 - t) + diag2[1][1] * t, cz))

    # Combine all
    return sq_segments + [seg_diag1, seg_travel, seg_diag2]


# ----------------------------
# Utility helpers
# ----------------------------
def _points_arc_lengths(pts):
    pts = np.asarray(pts, dtype=float)
    if pts.shape[0] < 2:
        return np.array([0.0]), np.array([])
    diffs = np.diff(pts, axis=0)
    seglens = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seglens)))
    return cum, seglens


def resample_by_arclength(pts, step):
    pts = np.asarray(pts, dtype=float)
    if pts.shape[0] < 2:
        return pts.tolist()
    cum, _ = _points_arc_lengths(pts)
    total = cum[-1]
    if total <= 0.0:
        return pts.tolist()
    n_samples = max(2, int(math.ceil(total / max(step, 1e-6))) + 1)
    new_s = np.linspace(0.0, total, n_samples)
    resampled = []
    idx = 0
    for s in new_s:
        while idx < len(cum) - 2 and s > cum[idx + 1]:
            idx += 1
        if idx >= len(cum) - 1:
            resampled.append(pts[-1].tolist());
            continue
        s0 = cum[idx];
        s1 = cum[idx + 1]
        if abs(s1 - s0) < 1e-9:
            resampled.append(pts[idx].tolist());
            continue
        frac = (s - s0) / (s1 - s0)
        p = (1.0 - frac) * pts[idx] + frac * pts[idx + 1]
        resampled.append(p.tolist())
    return resampled


def _clamp(v, a, b):
    return max(a, min(b, v))


# ----------------------------
# Validation + adaptive resampling
# ----------------------------
def validate_points(points, geom, vel=120.0, dt=0.02, collision_check=False, min_step_mm=0.5):
    if not points:
        return []
    pts = np.asarray(points, dtype=float)

    per_step = max(0.2, vel * dt)
    per_step = _clamp(per_step, min_step_mm, 5.0)
    sampled = resample_by_arclength(pts, step=per_step)

    out = []
    e = float(geom.get('e', 115.0));
    f = float(geom.get('f', 457.3));
    re = float(geom.get('re', 232.0));
    rf = float(geom.get('rf', 112.0))
    min_alpha = float(geom.get('min_alpha_deg', -90.0))
    max_alpha = float(geom.get('max_alpha_deg', 90.0))
    clearance = float(geom.get('clearance', 1.0))

    for p in sampled:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        st, _, _, _ = delta_calcInverse(x, y, z, e, f, re, rf, min_alpha, max_alpha)
        if st != 0:
            continue

        if collision_check and collision.check_pose_self_collision(x, y, z, e, f, re, rf, clearance=clearance):
            continue

        out.append({'x': x, 'y': y, 'z': z, 'vel': float(vel), 'dt': float(dt), 'mode': 'linear'})
    return out


# ----------------------------
# Public: validated animation generator
# ----------------------------
def generate_validated_animation(name, params, geom):
    name = (name or "").lower()
    start = params.get('start_pos', [0.0, 0.0, -220.0])
    pts = int(params.get('pts', 720))
    turns = int(params.get('turns', 4))
    vel = float(params.get('vel', 120.0))
    dt = float(params.get('dt', 0.02))
    mode = params.get('mode', 'linear')
    collision_check = bool(geom.get('collision_check', False))

    raw = []
    is_segmented = False

    if "descend" in name:
        raw = generate_spiral_descend(start, turns=turns, pts=pts)
    elif "flat" in name:
        raw = generate_spiral_flat(start, turns=turns, pts=pts)
    elif "lissajous" in name:
        raw = generate_lissajous(center=start, pts=pts)
    elif "zigzag" in name:
        raw = generate_zigzag(center=start, pts=pts)
        is_segmented = True
    elif "square_x" in name:
        raw = generate_square_x(center=start, pts=pts)
        is_segmented = True
    elif "square" in name:
        raw = generate_square(center=start, pts=pts)
        is_segmented = True
    elif "circle" in name:
        raw = generate_circle(center=start, pts=pts)
    elif "triangle" in name:
        raw = generate_triangle(center=start, pts=pts)
        is_segmented = True
    else:
        raw = generate_spiral_flat(start, turns=max(1, turns), pts=pts, radius=40.0)

    # Process based on type
    validated = []

    if is_segmented:
        # raw is list of lists
        for segment in raw:
            v_seg = validate_points(segment, geom, vel=vel, dt=dt, collision_check=collision_check, min_step_mm=0.25)
            if v_seg:
                validated.extend(v_seg)
                # Add a delay marker between segments for sharp turns
                validated.append({'type': 'delay', 'dt': 0.5})  # 0.5 second pause at corners
    else:
        # raw is list of points
        validated = validate_points(raw, geom, vel=vel, dt=dt, collision_check=collision_check, min_step_mm=0.25)

    for v in validated:
        if v.get('type') == 'delay': continue  # Skip delays for param update
        v['mode'] = mode
        v['vel'] = float(vel)
        v['dt'] = float(dt)
        if 'accel' in params:
            v['accel'] = params['accel']

    return validated


def _quintic_smoothstep(t):
    return ((6.0 * t ** 5) - (15.0 * t ** 4) + (10.0 * t ** 3))


def _generate_joint_space_return(a_start, a_end, e, f, re, rf, vel, dt, collision_check, min_alpha, max_alpha):
    a_start = np.array(a_start, dtype=float);
    a_end = np.array(a_end, dtype=float)
    ang_delta = np.abs(a_end - a_start)
    max_delta = float(np.max(ang_delta))
    target_deg_step = 0.25
    n_by_angle = max(16, int(math.ceil(max_delta / target_deg_step)))
    n_time = max(24, int(math.ceil((max_delta / 180.0) * (1.0 / max(1e-6, dt)))))
    n = int(_clamp(max(n_by_angle, n_time), 48, 4000))
    out = []
    for i in range(n + 1):
        t = float(i) / n
        s = _quintic_smoothstep(t)
        a_interp = (1.0 - s) * a_start + s * a_end
        ok, x, y, z = delta_calcForward(a_interp[0], a_interp[1], a_interp[2], e, f, re, rf)
        if ok != 0: return None
        st_ik, _, _, _ = delta_calcInverse(x, y, z, e, f, re, rf, min_alpha, max_alpha)
        if st_ik != 0: return None
        if collision_check and collision.check_pose_self_collision(x, y, z, e, f, re, rf, clearance=1.0):
            return None
        out.append({'x': float(x), 'y': float(y), 'z': float(z), 'vel': float(vel), 'dt': float(dt), 'mode': 'joint'})
    return out


def _generate_cartesian_smooth_return(start_xyz, home_xyz, e, f, re, rf, vel, dt, collision_check, min_alpha,
                                      max_alpha):
    start = np.array(start_xyz, dtype=float);
    home = np.array(home_xyz, dtype=float)
    dist = np.linalg.norm(home - start)
    if dist < 1e-6: return [{'x': home[0], 'y': home[1], 'z': home[2], 'vel': vel, 'dt': dt, 'mode': 'linear'}]
    time_total = max(0.02, dist / max(1e-6, vel))
    nominal_steps = max(48, int(math.ceil(time_total / max(1e-4, dt))))
    n = int(_clamp(nominal_steps, 48, 5000))
    pts = []
    for i in range(n + 1):
        t = float(i) / n
        s = _quintic_smoothstep(t)
        p = (1.0 - s) * start + s * home
        st, _, _, _ = delta_calcInverse(p[0], p[1], p[2], e, f, re, rf, min_alpha, max_alpha)
        if st != 0: continue
        if collision_check and collision.check_pose_self_collision(p[0], p[1], p[2], e, f, re, rf, clearance=1.0):
            continue
        pts.append(p.tolist())
    res = resample_by_arclength(pts if pts else [start.tolist(), home.tolist()], vel * dt)
    out = [{'x': p[0], 'y': p[1], 'z': p[2], 'vel': vel, 'dt': dt, 'mode': 'linear'} for p in res]
    if not any(np.allclose([m['x'], m['y'], m['z']], home) for m in out):
        out.append({'x': home[0], 'y': home[1], 'z': home[2], 'vel': vel, 'dt': dt, 'mode': 'linear'})
    return out


def generate_smooth_return_path(last_pos, home_pos, e, f, re, rf, vel=120.0, dt=0.02, collision_check=False,
                                min_alpha=-90.0, max_alpha=90.0):
    last = np.array(last_pos, dtype=float)
    home = np.array(home_pos, dtype=float)
    s1, a1, a2, a3 = delta_calcInverse(last[0], last[1], last[2], e, f, re, rf, min_alpha, max_alpha)
    s2, b1, b2, b3 = delta_calcInverse(home[0], home[1], home[2], e, f, re, rf, min_alpha, max_alpha)
    if s1 == 0 and s2 == 0:
        jpath = _generate_joint_space_return([a1, a2, a3], [b1, b2, b3], e, f, re, rf, vel, dt, collision_check,
                                             min_alpha, max_alpha)
        if jpath is not None and len(jpath) > 0:
            return jpath
    return _generate_cartesian_smooth_return(last, home, e, f, re, rf, vel, dt, collision_check, min_alpha, max_alpha)