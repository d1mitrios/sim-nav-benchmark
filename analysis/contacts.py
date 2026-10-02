"""Walker contacts: moments when a walking person physically pushed the robot.

The walkers follow scripted routes and do not yield. When one walks into the robot, the
physics engine displaces the robot, and its logged position moves faster than the
navigator can drive it (at most 0.4 m/s). A contact event is a logged speed above
PUSH_SPEED with a person closer than NEAR_M (centre to centre) within the preceding second.
Samples less than MERGE_S apart belong to the same event.
"""
import numpy as np

PUSH_SPEED = 1.0   # m/s, 2.5 times the navigator's top speed
NEAR_M = 0.5       # m, centre to centre (the 0.25 m person cylinder touches the robot's side at about 0.45 m)
MERGE_S = 3.0      # s


def events(t, x, y, people):
    """people: list of (px, py), arrays or constants. Returns [(t, closest distance)] per event."""
    speed = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)
    dist = np.min([np.hypot(x - px, y - py) for px, py in people], axis=0)
    out, last = [], -1e9
    for j in np.where(speed > PUSH_SPEED)[0]:
        near = float(dist[max(0, j - 10):j + 2].min())
        if near >= NEAR_M:
            continue
        if t[j] - last >= MERGE_S:
            out.append((float(t[j]), near))
        last = t[j]
    return out
