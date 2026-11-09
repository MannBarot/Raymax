"""Shared joint limits for Reachy.

Limits live in joint_limits.json (written by record_joint_limits.ipynb), keyed by
joint name, in degrees:

    {"r_shoulder_pitch": {"min": -170.0, "max": -10.0, ...}, ...}

Typical use:

    from joint_limits import check_limits, safe_goto
    q_goal = ARM.inverse_kinematics(T_goal, q0=current_q())
    check_limits(CHAIN, q_goal)          # raises before anything moves
    safe_goto(CHAIN, q_goal, duration=2) # same check, then moves

Checking only the target is enough for goto(): each joint moves monotonically
from its start angle to its target, so if both ends are inside [min, max] the
whole path is too.
"""

import json
from pathlib import Path

LIMITS_FILE = Path(__file__).with_name('joint_limits.json')


class JointLimitError(ValueError):
    pass


def load_limits(path=LIMITS_FILE):
    """Return {joint_name: {'min': deg, 'max': deg, ...}}."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f'{path} not found. Record limits first with record_joint_limits.ipynb')
    with open(path) as f:
        return json.load(f)


def _name(joint):
    return joint if isinstance(joint, str) else joint.name


def violations(joints, angles, limits=None, require_all=True):
    """List of human-readable problems; empty list means OK.

    joints: joint objects or names. angles: degrees, same order.
    require_all: a joint with no recorded limit counts as a violation.
    """
    limits = load_limits() if limits is None else limits
    joints = list(joints)
    angles = list(angles)
    if len(joints) != len(angles):
        raise ValueError(f'{len(joints)} joints but {len(angles)} angles')

    problems = []
    for joint, a in zip(joints, angles):
        name = _name(joint)
        lim = limits.get(name)
        if lim is None:
            if require_all:
                problems.append(f'{name}: no recorded limit')
            continue
        if lim.get('wrapped'):
            problems.append(f'{name}: limit unusable (angle wrapped past +/-180 while recording)')
            continue
        if not lim['min'] <= a <= lim['max']:
            problems.append(f'{name}: {a:.1f} not in [{lim["min"]:.1f}, {lim["max"]:.1f}]')
    return problems


def check_limits(joints, angles, limits=None, require_all=True):
    """Raise JointLimitError if any angle is outside its limit."""
    problems = violations(joints, angles, limits, require_all)
    if problems:
        raise JointLimitError('joint limit violation:\n  ' + '\n  '.join(problems))


def clamp(joints, angles, limits=None):
    """Return angles pushed inside the limits (joints without a limit are unchanged).

    Prefer check_limits for IK results: clamping one joint changes where the
    hand ends up. Clamp is meant for small manual/teleop nudges.
    """
    limits = load_limits() if limits is None else limits
    out = []
    for joint, a in zip(joints, angles):
        lim = limits.get(_name(joint))
        if lim is None or lim.get('wrapped'):
            out.append(a)
        else:
            out.append(min(max(a, lim['min']), lim['max']))
    return out


def safe_goto(joints, angles, duration=2.0, limits=None, **goto_kwargs):
    """check_limits, then reachy_sdk goto. joints must be Joint objects."""
    from reachy_sdk.trajectory import goto
    from reachy_sdk.trajectory.interpolation import InterpolationMode

    joints = list(joints)
    check_limits(joints, angles, limits)
    goto_kwargs.setdefault('interpolation_mode', InterpolationMode.MINIMUM_JERK)
    return goto({j: a for j, a in zip(joints, angles)}, duration=duration, **goto_kwargs)
