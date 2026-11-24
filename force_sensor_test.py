"""Gripper force sensor test for Reachy (reachy_sdk).

Run on the robot (or anywhere that can reach it):
    python force_sensor_test.py                  # list sensors, then log them for 15 s (pinch the gripper by hand)
    python force_sensor_test.py --seconds 30
    python force_sensor_test.py --squeeze right  # also close that gripper slowly and log force vs angle
    python force_sensor_test.py --host 169.254.50.2

It prints what it finds at each step and writes force_log.csv, so you can see which of these is the problem:
  - no force sensors found at all            -> SDK version / robot config (see the attribute dump)
  - reading is None / NaN                    -> sensor declared but no data reaching the SDK
  - reading never changes                    -> stale value: sensor or its board not publishing
  - reading changes but not when you pinch   -> wrong sensor (left vs right) or a mounting/cable problem
  - reading changes when you pinch           -> sensor works; use the numbers to set FORCE_CONTACT / FORCE_HOLD
"""
import argparse
import csv
import math
import time

import numpy as np
from reachy_sdk import ReachySDK


def find_force_sensors(reachy):
    # Returns {name: sensor}. Tries the documented container first, then searches the arms.
    sensors = {}
    container = getattr(reachy, 'force_sensors', None)
    if container is not None:
        items = container.values() if hasattr(container, 'values') else \
            [v for k, v in vars(container).items() if not k.startswith('_')]
        for s in items:
            sensors[getattr(s, 'name', repr(s))] = s
    for arm_name in ('l_arm', 'r_arm'):
        arm = getattr(reachy, arm_name, None)
        if arm is None:
            continue
        for attr in dir(arm):
            if 'force' in attr.lower() and not attr.startswith('_'):
                s = getattr(arm, attr)
                if hasattr(s, 'force'):
                    sensors.setdefault(getattr(s, 'name', f'{arm_name}.{attr}'), s)
    return sensors


def read(sensor):
    try:
        v = sensor.force
    except Exception as e:  # report, don't crash: an exception here is itself a finding
        return f'ERROR {type(e).__name__}: {e}'
    return v


def as_float(v):
    try:
        f = float(v)
        return None if math.isnan(f) else f
    except (TypeError, ValueError):
        return None


def gripper_of(reachy, side):
    p = side[0]
    arm = getattr(reachy, f'{p}_arm', None)
    return None if arm is None else getattr(arm, f'{p}_gripper', None)


def summarize(name, values, duration):
    nums = [v for v in (as_float(x) for x in values) if v is not None]
    bad = len(values) - len(nums)
    print(f'\n{name}:')
    if not nums:
        print(f'  no numeric readings ({len(values)} samples, first: {values[0] if values else None!r})')
        print('  -> the sensor exists in the SDK but no data is coming through.')
        return
    arr = np.array(nums)
    changes = int(np.count_nonzero(np.diff(arr)))
    print(f'  samples {len(nums)} (+{bad} None/NaN/errors)   min {arr.min():.2f}   max {arr.max():.2f}   '
          f'mean {arr.mean():.2f}   std {arr.std():.2f}')
    print(f'  distinct values {len(np.unique(arr))}   value changed {changes} times '
          f'(~{changes / duration:.1f} updates/s)')
    if changes == 0:
        print('  -> NEVER CHANGED: stale reading. The sensor (or its board) is not publishing.')
    elif arr.max() - arr.min() < 5 * max(arr[: max(len(arr) // 5, 2)].std(), 1e-6):
        print('  -> only noise: did you pinch this gripper? If yes, it may be the other side\'s sensor or not connected.')
    else:
        print(f'  -> responds. Range {arr.max() - arr.min():.1f}. Suggested FORCE_CONTACT ~ '
              f'{(arr.max() - arr.min()) / 3:.0f}, FORCE_HOLD ~ {(arr.max() - arr.min()) / 6:.0f} (raw units).')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--seconds', type=float, default=15)
    ap.add_argument('--rate', type=float, default=20, help='samples per second')
    ap.add_argument('--squeeze', choices=['left', 'right'], help='also close this gripper slowly while logging')
    ap.add_argument('--out', default='force_log.csv')
    args = ap.parse_args()

    print(f'connecting to {args.host} ...')
    reachy = ReachySDK(host=args.host)
    time.sleep(1)  # let the first sensor messages arrive

    # ---- 1. What does the SDK expose? ----
    sensors = find_force_sensors(reachy)
    if not sensors:
        print('No force sensors found.')
        print('  reachy attributes:', [a for a in dir(reachy) if not a.startswith('_')])
        fs = getattr(reachy, 'force_sensors', None)
        print('  reachy.force_sensors =', repr(fs))
        return
    print(f'found {len(sensors)} force sensor(s):')
    for name, s in sensors.items():
        print(f'  {name:25s} {type(s).__name__:20s} force = {read(s)!r}')

    names = list(sensors)
    grippers = {side: gripper_of(reachy, side) for side in ('left', 'right')}

    # ---- 2. Passive log: pinch each gripper by hand ----
    print(f'\nLogging for {args.seconds:.0f} s. Leave the grippers empty for ~3 s, '
          'then squeeze each gripper\'s fingers (or an object between them) by hand.')
    rows, dt = [], 1 / args.rate
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        t = time.time() - t0
        vals = [read(sensors[n]) for n in names]
        gpos = [getattr(g, 'present_position', None) if g else None for g in grippers.values()]
        rows.append([round(t, 3), 'passive', *vals, *gpos])
        print('\r' + f'{t:5.1f}s  ' + '  '.join(f'{n}: {v if isinstance(v, str) else f"{as_float(v) or 0:8.1f}"}'
                                              for n, v in zip(names, vals)), end='', flush=True)
        time.sleep(dt)
    print()
    for i, n in enumerate(names):
        summarize(n, [r[2 + i] for r in rows], args.seconds)

    # ---- 3. Optional: close one gripper slowly and watch the force ----
    if args.squeeze:
        g = grippers[args.squeeze]
        if g is None:
            print(f'\nno {args.squeeze} gripper found; skipping the squeeze test.')
        else:
            sign = 1 if args.squeeze == 'left' else -1       # right gripper turns the other way
            g_open, g_closed = 35 * sign, -12 * sign         # same values as reachy_grasping.ipynb
            print(f'\nSqueeze test ({args.squeeze}): put a soft object (sponge, paper cup) between the fingers '
                  'or leave it empty. Keep fingers clear. Starting in 3 s ...')
            time.sleep(3)
            g.goal_position = g.present_position
            g.compliant = False
            try:
                for target, secs in ((g_open, 1.5), (g_closed, 6.0)):
                    start, ts = g.present_position, time.time()
                    while time.time() - ts < secs:
                        tau = (time.time() - ts) / secs
                        g.goal_position = start + (target - start) * tau
                        vals = [read(sensors[n]) for n in names]
                        rows.append([round(time.time() - t0, 3), f'squeeze->{target}', *vals,
                                     *[getattr(x, 'present_position', None) if x else None for x in grippers.values()]])
                        time.sleep(dt)
                squeeze = [r for r in rows if str(r[1]).startswith(f'squeeze->{g_closed}')]
                for i, n in enumerate(names):
                    summarize(f'{n} (while closing {args.squeeze})', [r[2 + i] for r in squeeze], 6.0)
            finally:
                g.goal_position = g_open
                time.sleep(1.5)
                g.compliant = True
                print('gripper reopened and set compliant.')

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'phase', *names, 'l_gripper_deg', 'r_gripper_deg'])
        w.writerows(rows)
    print(f'\nwrote {len(rows)} rows to {args.out}')


if __name__ == '__main__':
    main()
