"""Shared, wrap-aware joint limits for Reachy.

Most joints can turn all the way round, and going the wrong way round winds up
the wires. So a limit is not a single min/max: it is the arc you recorded as
safe, stored as one or more intervals of the angle the SDK reports. There are
two intervals when that arc crosses the point where the reported angle wraps
(e.g. +180 -> -180). Everything outside them is the forbidden zone.

joint_limits.json (written by record_joint_limits.ipynb), degrees:

    {"r_shoulder_pitch": {"safe": [[-172.0, 157.0]], ...}, ...}

A move is allowed only if the target is in a safe interval AND it is the same
interval the joint is in now. goto() moves each joint in a straight line from
its present number to the target number, so a move between two different
intervals would sweep through the gap between them, which is the forbidden zone
(or the long way round past it).

Typical use:

    from joint_limits import safe_goto, check_limits
    q_goal = ARM.inverse_kinematics(T_goal, q0=current_q())
    safe_goto(CHAIN, q_goal, duration=2)    # raises JointLimitError, nothing moves
"""

import json
from pathlib import Path

LIMITS_FILE = Path(__file__).with_name('joint_limits.json')
WRAP_JUMP_DEG = 180   # a jump this big between two readings means the reported angle wrapped


class JointLimitError(ValueError):
    pass


# ---------- recording ----------

def intervals_from_samples(samples, margin=3.0):
    """Turn a stream of reported angles (one joint, recorded while you moved it
    to its safe extremes) into safe intervals.

    Returns (intervals, travel): intervals is a sorted list of [lo, hi];
    travel is the total arc covered in degrees, unwrapped. travel close to 360
    means you went all the way round and no forbidden zone was found.

    margin is taken off the real ends you stopped at, not off the ends created
    by the reported angle wrapping (those aren't physical limits).
    """
    samples = list(samples)
    if not samples:
        return [], 0.0

    runs = []                      # [lo, hi, lo_is_wrap_edge, hi_is_wrap_edge]
    prev = samples[0]
    run = [prev, prev, False, False]
    u = u_lo = u_hi = 0.0          # unwrapped angle relative to the first sample
    for p in samples[1:]:
        d = p - prev
        if d < -WRAP_JUMP_DEG:     # went off the top, came back in at the bottom
            run[3] = True
            runs.append(run)
            run = [p, p, True, False]
            d += 360
        elif d > WRAP_JUMP_DEG:    # went off the bottom, came back in at the top
            run[2] = True
            runs.append(run)
            run = [p, p, False, True]
            d -= 360
        run[0], run[1] = min(run[0], p), max(run[1], p)
        u += d
        u_lo, u_hi = min(u_lo, u), max(u_hi, u)
        prev = p
    runs.append(run)

    runs.sort(key=lambda r: r[0])
    merged = []
    for lo, hi, lw, hw in runs:
        if merged and lo <= merged[-1][1]:
            m = merged[-1]
            if lo == m[0]:
                m[2] = m[2] or lw
            if hi > m[1]:
                m[1], m[3] = hi, hw
            elif hi == m[1]:
                m[3] = m[3] or hw
        else:
            merged.append([lo, hi, lw, hw])

    intervals = []
    for lo, hi, lw, hw in merged:
        lo2 = lo if lw else lo + margin
        hi2 = hi if hw else hi - margin
        if lo2 < hi2:
            intervals.append([round(lo2, 2), round(hi2, 2)])
    return intervals, u_hi - u_lo


# ---------- loading ----------

def load_limits(path=LIMITS_FILE):
    """Return {joint_name: {'safe': [[lo, hi], ...], ...}}."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f'{path} not found. Record limits first with record_joint_limits.ipynb')
    with open(path) as f:
        return json.load(f)


def _name(joint):
    return joint if isinstance(joint, str) else joint.name


def _fmt(safe):
    return ' + '.join(f'[{lo:.1f}, {hi:.1f}]' for lo, hi in safe)


def interval_index(safe, angle):
    """Index of the safe interval containing angle, or None."""
    for i, (lo, hi) in enumerate(safe):
        if lo <= angle <= hi:
            return i
    return None


# ---------- checking ----------

def resolve(name, target, current, limits=None):
    """Same physical angle, written so goto() reaches it without leaving the safe arc.

    IK may hand back e.g. +200 for a joint that reports -160. Of target,
    target-360 and target+360, return the one in the same safe interval as
    current. If none is, return target unchanged (check_limits will refuse it).
    """
    limits = load_limits() if limits is None else limits
    lim = limits.get(name)
    if lim is None:
        return target
    here = interval_index(lim['safe'], current)
    if here is None:
        return target
    for t in (target, target - 360, target + 360):
        if interval_index(lim['safe'], t) == here:
            return t
    return target


def violations(joints, angles, current=None, limits=None, require_all=True):
    """List of human-readable problems; empty means OK.

    joints: Joint objects or names. angles: targets in degrees, same order.
    current: present angles, same order. Without it only the targets are
    checked, not the path to them, so pass it whenever the arm is about to move.
    require_all: a joint with no recorded limit counts as a problem.
    """
    limits = load_limits() if limits is None else limits
    joints, angles = list(joints), list(angles)
    if len(joints) != len(angles):
        raise ValueError(f'{len(joints)} joints but {len(angles)} angles')
    current = [None] * len(joints) if current is None else list(current)

    problems = []
    for joint, a, c in zip(joints, angles, current):
        name = _name(joint)
        lim = limits.get(name)
        if lim is None:
            if require_all:
                problems.append(f'{name}: no recorded limit')
            continue
        safe = lim['safe']
        there = interval_index(safe, a)
        if there is None:
            problems.append(f'{name}: target {a:.1f} outside safe {_fmt(safe)}')
            continue
        if c is None:
            continue
        here = interval_index(safe, c)
        if here is None:
            problems.append(f'{name}: currently at {c:.1f}, outside safe {_fmt(safe)}; move it back by hand')
        elif here != there:
            problems.append(f'{name}: {c:.1f} -> {a:.1f} would pass through the forbidden zone '
                            f'(safe {_fmt(safe)})')
    return problems


def check_limits(joints, angles, current=None, limits=None, require_all=True):
    """Raise JointLimitError if any target, or the path to it, leaves the safe arc."""
    problems = violations(joints, angles, current, limits, require_all)
    if problems:
        raise JointLimitError('joint limit violation:\n  ' + '\n  '.join(problems))


def clamp(joints, angles, current, limits=None):
    """Pull each target to the nearest end of the safe interval the joint is in now.

    For small manual/teleop nudges only: clamping an IK result changes where the
    hand ends up. Joints with no limit, or currently outside their safe arc, are
    left unchanged.
    """
    limits = load_limits() if limits is None else limits
    out = []
    for joint, a, c in zip(joints, angles, current):
        lim = limits.get(_name(joint))
        i = None if lim is None else interval_index(lim['safe'], c)
        if i is None:
            out.append(a)
        else:
            lo, hi = lim['safe'][i]
            out.append(min(max(a, lo), hi))
    return out


def safe_goto(joints, angles, duration=2.0, limits=None, **goto_kwargs):
    """resolve + check_limits (target and path) + reachy_sdk goto. joints must be Joint objects."""
    from reachy_sdk.trajectory import goto
    from reachy_sdk.trajectory.interpolation import InterpolationMode

    limits = load_limits() if limits is None else limits
    joints = list(joints)
    current = [j.present_position for j in joints]
    angles = [resolve(j.name, a, c, limits) for j, a, c in zip(joints, angles, current)]
    check_limits(joints, angles, current, limits)
    goto_kwargs.setdefault('interpolation_mode', InterpolationMode.MINIMUM_JERK)
    return goto({j: a for j, a in zip(joints, angles)}, duration=duration, **goto_kwargs)
