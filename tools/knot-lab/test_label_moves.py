#!/usr/bin/env python3
"""Checks for label_moves.py on hand-made rope motions whose moves are known.

    python3 tools/knot-lab/test_label_moves.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from label_moves import crossings, determinant, label_episode  # noqa: E402

SEG = 7.0   # node spacing (table units), like a rope of thickness 14


def polyline(pieces):
    """Nodes along straight pieces [(start, end, nodes), ...] (each piece's start is skipped
    except for the first), with z rising slowly along the rope: later strands lie on top."""
    pts = []
    for k, (p, q, m) in enumerate(pieces):
        for i in range(0 if k == 0 else 1, m + 1):
            pts.append(np.asarray(p, float) + (np.asarray(q, float) - np.asarray(p, float)) * i / m)
    P = np.zeros((len(pts), 3))
    P[:, :2] = pts
    P[:, 2] = 7 + np.arange(len(pts)) * 0.01
    return P


def episode(frames_per_action, every=1):
    """An episode from a list of actions, each a list of rope frames; the gripper holds node 0
    during the first half of each action."""
    frames = [f for a in frames_per_action for f in a]
    T = len(frames)
    grip = np.full((T, 7), -1.0)
    k = 0
    for a in frames_per_action:
        grip[k:k + max(1, len(a) // 2), 5] = 0
        k += len(a)
    idx = np.arange(0, T, every)
    if idx[-1] != T - 1:
        idx = np.append(idx, T - 1)
    t = np.arange(T) * 0.05
    return {"t": t, "grip": grip, "rope": np.asarray(frames)[idx], "rope_t": t[idx], "rope_idx": idx}


def run(frames_per_action, every=1):
    return label_episode(episode(frames_per_action, every), {"round": {"rad": SEG}})[0]


def loop(d, nodes=121):
    """A rope with one twist loop in the middle (a trochoid), loop size set by d (> 1: a loop)."""
    th = np.linspace(-3 * np.pi, 3 * np.pi, nodes)
    dd = 0.5 + (d - 0.5) * np.exp(-th ** 2 / 2)
    P = np.zeros((len(th), 3))
    P[:, 0] = 300 + 25 * (th - dd * np.sin(th))
    P[:, 1] = 200 - 25 * dd * np.cos(th)
    P[:, 2] = 7 + th * 0.5
    return P


def bump(amp, clasp=False):
    """Strand 1 along y = 0, a U-turn, strand 2 back along y = 120 with a bump of height amp
    towards strand 1; strand 2 is on top (with clasp: only where x < 0, below it where x > 0)."""
    xs = np.linspace(200, -200, 58)
    back = [(x, 120 - amp * np.exp(-x ** 2 / (2 * 40 ** 2))) for x in xs]
    P = polyline([((-200, 0), (200, 0), 57), ((200, 0), (260, 0), 9), ((260, 0), (260, 120), 17),
                  ((260, 120), (200, 120), 9)])
    tail = np.zeros((len(back) - 1, 3))
    tail[:, :2] = back[1:]
    tail[:, 2] = P[-1, 2] + 0.01 * np.arange(1, len(back))
    if clasp:
        tail[:, 2] = np.where(tail[:, 0] < 0, 21, 0)
    return np.vstack([P, tail])


def three(c):
    """Strand 1 along y = c, then strand 2 along y = x, then strand 3 along y = -x: moving c
    from below the crossing of strands 2 and 3 to above it is an R3."""
    return polyline([((-100, c), (300, c), 57), ((300, c), (300, 150), 22 if c < 150 else 1),
                     ((300, 150), (150, 150), 21), ((150, 150), (-150, -150), 61),
                     ((-150, -150), (-150, 150), 43), ((-150, 150), (150, -150), 61)])


def tail(e):
    """Strand 1 along x = 0, a turn, then the end of the rope along y = 0 from x = 200 to x = e."""
    return polyline([((0, -150), (0, 150), 43), ((0, 150), (200, 150), 29), ((200, 150), (200, 0), 21),
                     ((200, 0), (e, 0), 40)])


def trefoil(flip=False):
    """An open trefoil (overhand knot); with flip, its first crossing has the other strand on top."""
    th = np.linspace(0.15, 2 * np.pi - 0.15, 160)
    P = np.zeros((len(th), 3))
    P[:, 0] = 320 + 50 * (np.sin(th) + 2 * np.sin(2 * th))
    P[:, 1] = 200 + 50 * (np.cos(th) - 2 * np.cos(2 * th))
    P[:, 2] = 20 - 14 * np.sin(3 * th)
    if flip:
        c = crossings(P)[0]
        top, low = (int(c[0]), int(c[1])) if c[2] > 0 else (int(c[1]), int(c[0]))
        P[top - 3:top + 5, 2] = P[low, 2] - 15
    return P


def check(name, cond, info=""):
    print(("ok   " if cond else "FAIL ") + name + (" " + str(info) if info else ""))
    return bool(cond)


def main():
    good = True
    lin = np.linspace

    a = run([[loop(d) for d in lin(1.8, 0.6, 30)]])
    good &= check("R1-: twist loop pulled out", [x["label"] for x in a] == ["R1"] and a[0]["counts"] == {"R1-": 1}, a[0]["counts"])
    a = run([[loop(d) for d in lin(0.6, 1.8, 30)]])
    good &= check("R1+: twist loop made", a[0]["label"] == "R1" and a[0]["counts"] == {"R1+": 1}, a[0]["counts"])

    a = run([[loop(d, 31) for d in lin(0.6, 1.8, 30)]])
    good &= check("kink: a loop never longer than a few nodes is not an R1", a[0]["label"] == "none" and a[0]["counts"] == {"kink+": 1}, a[0]["counts"])

    a = run([[bump(h) for h in lin(0, 200, 30)], [bump(h) for h in lin(200, 0, 30)]])
    good &= check("R2+ then R2-: strand pushed over another, then pulled back off",
                  [x["label"] for x in a] == ["R2", "R2"] and a[0]["counts"] == {"R2+": 1} and a[1]["counts"] == {"R2-": 1},
                  [x["counts"] for x in a])
    a = run([[bump(h) for h in np.r_[lin(0, 200, 30), lin(200, 0, 30)]]])
    good &= check("R2+ and R2- in one action cancel: none", a[0]["label"] == "none" and a[0]["undone"] == 2, a[0]["counts"])

    a = run([[three(c) for c in lin(-60, 60, 40)]])
    good &= check("R3: strand slid across a crossing", a[0]["label"] == "R3" and a[0]["counts"] == {"R3": 1}
                  and a[0]["crossings_before"] == a[0]["crossings_after"] == 3, a[0]["counts"])
    a = run([[three(c) for c in np.r_[lin(-60, 60, 40), lin(60, -60, 40)]]])
    good &= check("R3 there and back in one action: none", a[0]["label"] == "none", a[0]["counts"])

    a = run([[tail(e) for e in lin(-80, 80, 30)]])
    good &= check("End-: end pulled out from under a strand", a[0]["label"] == "End" and a[0]["counts"] == {"End-": 1}, a[0]["counts"])
    a = run([[tail(e) for e in lin(80, -80, 30)]])
    good &= check("End+: end pushed across a strand", a[0]["label"] == "End" and a[0]["counts"] == {"End+": 1}, a[0]["counts"])

    a = run([[tail(e) for e in lin(-80, 80, 30)], [tail(80)] * 4], every=4)
    good &= check("thinned rope frames (every 4th): End- still found", a[0]["counts"] == {"End-": 1}, a[0]["counts"])
    a = run([[loop(d) for d in lin(1.8, 0.6, 12)]], every=2)
    good &= check("fast and thinned: R1- still found", a[0]["counts"] == {"R1-": 1}, a[0]["counts"])

    a = run([[bump(150, True)] * 3 + [bump(h, True) for h in lin(150, 0, 30)]])
    good &= check("hooked strands pulled apart through each other, knot unchanged: unclassified", a[0]["counts"] == {"unclassified-": 1} and a[0]["label"] == "none", a[0]["counts"])

    still = run([[bump(150)] * 10])
    good &= check("rope that does not move: none", still[0]["label"] == "none" and not still[0]["moves"], still[0]["counts"])
    flip = bump(150)
    flip[57:, 2] -= 20     # strand 2 now under strand 1: both crossings swap in place
    a = run([[bump(150)] * 5 + [flip] * 5])
    good &= check("crossings swapped in place, knot unchanged: no move (cannot be told from a mix-up)", a[0]["counts"] == {} and a[0]["label"] == "none", a[0]["counts"])
    good &= check("trefoil: determinant 3, 1 with a crossing flipped", determinant(trefoil()) == 3 and determinant(trefoil(True)) == 1,
                  (determinant(trefoil()), determinant(trefoil(True))))
    a = run([[trefoil()] * 5 + [trefoil(True)] * 5])
    good &= check("a crossing of a knot swapped in place (the knot changes): pass", a[0]["counts"] == {"pass": 1} and a[0]["label"] == "none", a[0]["counts"])

    print("all passed" if good else "SOME FAILED")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
