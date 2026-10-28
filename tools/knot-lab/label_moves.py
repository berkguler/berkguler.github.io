#!/usr/bin/env python3
"""Label what every action in the Knot Lab runs did to the rope's crossing diagram.

Seen from above, a rope is a diagram: the places where it crosses itself, each with one strand on
top. Every rope motion shows up in the diagram as a sequence of a few local moves:

  R1   a twist loop appears or disappears: one crossing whose loop holds no other crossing
  R2   one strand slides onto or off another: two neighbouring crossings, same strand on top
  R3   a strand slides across a crossing: no crossing appears or vanishes, but they reorder
       along the rope
  End  an end of the rope slides onto or off a strand: one crossing at the end of the rope;
       the only move that can change the knot

The script follows the diagram from rope frame to rope frame in the runs written by
export_episodes.py, finds these moves (with + when they add crossings, - when they remove them),
and groups them into actions: from one grasp to the next, including the rope settling after the
release. Moves an action undid itself (a strand slid onto another and off again, back and forth
across a crossing) cancel out. Every action gets one label, the most significant move it kept
(End > R2 > R1 > R3), or "none" if it left the diagram as it was.

Three more kinds are counted but do not make a label: "kink" (an R1 or R2 made by the rope bending
sharply over itself, never a loop longer than KINK), "pass" (strands went through each other: a
crossing whose top strand changed, or two crossings side by side with different strands on top
that appeared or vanished together) and "unclassified" (a change that fits none of the moves).

    python3 label_moves.py data/knot-lab                  # the folder export_episodes.py wrote
    python3 label_moves.py data/knot-lab --out moves      # default: data/knot-lab/moves

Writes moves/episode_<id>_moves.json (every action with its moves) and moves/actions.csv (one row
per action), and prints a summary.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

TYPES = ("End", "R2", "R1", "R3")   # label priority: the knot type, then crossings removed or added
COUNTS = ("End-", "End+", "R2-", "R2+", "R1-", "R1+", "R3", "kink-", "kink+", "pass", "unclassified-", "unclassified+")
MIN_LOOP = 3                        # crossings closer than this along the rope (nodes) are a strand
                                    # standing on end seen from above, not a loop
KINK = 8                            # an R1 or R2 whose crossings' passes never got this far apart
                                    # along the rope (nodes, one rope radius apart: 4 thicknesses) is
                                    # the rope bent sharply over itself, a kink
_pairs = {}


def crossings(P):
    """Crossings of the rope P (n, 3), seen from above, as an array of rows (a, b, over, x, y):
    a < b are the two positions along the rope (node index + fraction), over is +1 if the strand
    at a is on top and -1 if the one at b is, and (x, y) is where they cross."""
    m = len(P) - 1
    if m not in _pairs:
        _pairs[m] = np.triu_indices(m, k=2)
    I, J = _pairs[m]
    A, D = P[:-1], P[1:] - P[:-1]
    rx, ry, sx, sy = D[I, 0], D[I, 1], D[J, 0], D[J, 1]
    qx, qy = A[J, 0] - A[I, 0], A[J, 1] - A[I, 1]
    d = rx * sy - ry * sx
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qx * sy - qy * sx) / d
        u = (qx * ry - qy * rx) / d
    ok = (np.abs(d) > 1e-12) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
    i, j, t, u = I[ok], J[ok], t[ok], u[ok]
    a, b = i + t, j + u
    keep = b - a >= MIN_LOOP
    i, j, t, u, a, b = i[keep], j[keep], t[keep], u[keep], a[keep], b[keep]
    za = P[i, 2] + D[i, 2] * t
    zb = P[j, 2] + D[j, 2] * u
    x, y = A[i, 0] + D[i, 0] * t, A[i, 1] + D[i, 1] * t
    over = np.where(za >= zb, 1.0, -1.0)
    out = np.stack([a, b, over, x, y], axis=1) if len(a) else np.zeros((0, 5))
    return out[np.argsort(out[:, 0])] if len(out) else out


def match(prev, cur, tol):
    """Pair crossings of two frames that are the same crossing (closest first, within tol nodes)."""
    cand = []
    for p in range(len(prev)):
        for c in range(len(cur)):
            cost = abs(prev[p, 0] - cur[c, 0]) + abs(prev[p, 1] - cur[c, 1])
            if cost <= tol:
                cand.append((cost, p, c))
    cand.sort()
    pairs, used = {}, set()
    for _, p, c in cand:
        if p not in pairs and c not in used:
            pairs[p] = c
            used.add(c)
    return pairs


def free(C, lo, hi, skip):
    """No crossing of C other than those in skip passes strictly between rope positions lo and hi."""
    for k in range(len(C)):
        if k in skip:
            continue
        for q in C[k, :2]:
            if lo < q < hi:
                return False
    return True


def split(C, idx, n, near, close, small):
    """The moves by which the crossings C[idx] (all appearing, or all vanishing) appeared or vanished;
    C is the whole diagram of the frame they are in."""
    left, moves = sorted(idx), []
    # Two crossings side by side on both strands, at the same spot: an R2 if the same strand is on
    # top at both; if not, the strands went through each other (a pass). Decided later, when the
    # top strands are known.
    for p in list(left):
        if p not in left:
            continue
        for q in left:
            if q == p:
                continue
            cp, cq = C[p], C[q]
            if (np.hypot(cp[3] - cq[3], cp[4] - cq[4]) <= close
                    and free(C, min(cp[0], cq[0]), max(cp[0], cq[0]), {p, q})
                    and free(C, min(cp[1], cq[1]), max(cp[1], cq[1]), {p, q})):
                moves.append(("pair", [p, q]))
                left.remove(p)
                left.remove(q)
                break
    for p in left:
        a, b = C[p, 0], C[p, 1]
        # R1: its loop (the rope between its two passes) holds no other crossing and is about to
        # close up, or has just opened.
        loop = b - a <= small and free(C, a, b, {p})
        # End: at an end of the rope, with nothing between it and that end.
        end = (a <= near and free(C, 0, a, {p})) or (b >= n - 1 - near and free(C, b, n - 1, {p}))
        moves.append(("R1" if loop else "End" if end else "unclassified", [p]))
    return moves


def reorders(prev, cur, pairs, gap):
    """Number of R3 moves between two diagrams: crossings that exist in both (pairs maps rows of
    prev to rows of cur) but changed order along the rope. An R3 swaps the order of two crossings
    on each of its three strands. Two crossings whose other strands are less than gap nodes apart
    are the two ends of one overlap, which can trade places without an R3."""
    keys = sorted(pairs)
    pos = [(prev[k, e], cur[pairs[k], e], prev[k, 1 - e], k) for k in keys for e in (0, 1)]
    swaps = 0
    for x in range(len(pos)):
        for y in range(x + 1, len(pos)):
            if (pos[x][3] != pos[y][3] and (pos[x][0] - pos[y][0]) * (pos[x][1] - pos[y][1]) < 0
                    and abs(pos[x][2] - pos[y][2]) > gap):
                swaps += 1
    return (swaps + 2) // 3


def as_list(C, rows):
    return [[round(float(C[r, 0]), 2), round(float(C[r, 1]), 2), int(C[r, 2])] for r in rows]


def label_episode(npz, meta):
    """Moves and labelled actions of one run (the arrays and metadata export_episodes.py wrote)."""
    rope, rope_t, t, grip = npz["rope"], npz["rope_t"], npz["t"], npz["grip"]
    n, K = rope.shape[1], rope.shape[0]
    rad = (meta.get("round") or {}).get("rad") or (meta.get("params") or {}).get("rad") or 7

    moves = []                         # dicts: t, type, sign, ids, crossings
    state = {"C": crossings(rope[0])}
    state["ids"], state["next"] = list(range(len(state["C"]))), len(state["C"])
    span = {i: c[1] - c[0] for i, c in zip(state["ids"], state["C"])}   # longest loop of each crossing

    def advance(P0, P1, tk, seg, depth=0):
        """Move the diagram on from the rope P0 to the rope P1. If that takes more than one move,
        or one that fits none, go through the rope halfway between them first."""
        C, ids, cur = state["C"], state["ids"], crossings(P1)
        step = float(np.abs(P1[:, :2] - P0[:, :2]).max())
        slide = step / seg            # how far (nodes) a crossing can have slid along the rope
        tol, near, small, close = 2 + 3 * slide, 1.5 + 1.5 * slide, MIN_LOOP + 2 + 2 * slide, 4 * rad + 2 * step
        pairs = match(C, cur, tol)
        for p, c in pairs.items():
            cur[c, 2] = C[p, 2]       # a crossing keeps its top strand (checked at the next rope frame)
        dead = [p for p in range(len(C)) if p not in pairs]
        born = [c for c in range(len(cur)) if c not in pairs.values()]
        # (kind, sign, rows of the diagram they are in, that diagram, its crossing ids)
        found = [(kind, "-", rows, C, ids) for kind, rows in split(C, dead, n, near, close, small)]
        found += [(kind, "+", rows, cur, None) for kind, rows in split(cur, born, n, near, close, small)]
        found += [("R3", "0", [], cur, None)] * reorders(C, cur, pairs, MIN_LOOP + tol)
        if depth < 8 and step > 0.01 and (len(found) > 1 or any(f[0] == "unclassified" for f in found)):
            mid = (P0 + P1) / 2
            advance(P0, mid, tk, seg, depth + 1)
            advance(mid, P1, tk, seg, depth + 1)
            return
        cur_ids = [None] * len(cur)
        for p, c in pairs.items():
            cur_ids[c] = ids[p]
        for c in born:
            cur_ids[c], state["next"] = state["next"], state["next"] + 1
        for kind, sign, rows, D, I in found:
            I = I or cur_ids
            moves.append({"t": tk, "type": kind, "sign": sign, "ids": [I[r] for r in rows], "crossings": as_list(D, rows)})
        for i, c in zip(cur_ids, cur):
            span[i] = max(span.get(i, 0), c[1] - c[0])
        state["C"], state["ids"] = cur, cur_ids

    # Between two rope frames (20 Hz, or fewer if the run was thinned out) a fast pull can make
    # several moves; going through the ropes in between, moved in straight lines, sorts them out.
    # Which strand is on top is only taken from the rope frames themselves: in between, heights
    # moved in straight lines can put a strand that lies across another one below it.
    last = rope[0]
    frames = [(float(rope_t[0]), state["C"], state["ids"])]   # the diagram after every frame that moved
    for k in range(1, K):
        if np.abs(rope[k] - last).max() < 0.05:   # the rope did not move: same diagram
            continue
        seg = max(float(np.median(np.linalg.norm(np.diff(rope[k], axis=0), axis=1))), 1e-3)
        sub = int(min(32, max(1, np.ceil(np.abs(rope[k, :, :2] - last[:, :2]).max() / (0.5 * seg)))))
        tk, first, before = float(rope_t[k]), len(moves), dict(zip(state["ids"], state["C"][:, 2]))
        ropes = [last + (rope[k] - last) * (j / sub) for j in range(sub)] + [rope[k]]
        for j in range(sub):
            advance(ropes[j], ropes[j + 1], tk, seg)
        top = crossings(rope[k])[:, 2]   # the same crossings, in the same order, as state["C"]
        over = dict(zip(state["ids"], top))
        mine = moves[first:]
        for m in mine:
            for i, c in zip(m["ids"], m["crossings"]):
                if m["sign"] == "+" and i in over:
                    c[2] = int(over[i])
            if m["type"] == "pair":
                m["type"] = "R2" if m["crossings"][0][2] == m["crossings"][1][2] else "pass"
        # Crossings that vanished and came back in the same place, of the same kind, between two
        # rope frames slid there: the straight-line ropes in between just lost track of them.
        tol = 2 + 3 * float(np.abs(rope[k, :, :2] - last[:, :2]).max()) / seg
        same = {}
        for m in mine:
            if m["sign"] != "-":
                continue
            for b in mine:
                if (b["sign"] == "+" and b["type"] == m["type"] and len(b["ids"]) == len(m["ids"]) and "gone" not in b
                        and all(abs(p[0] - q[0]) + abs(p[1] - q[1]) <= tol and p[2] == q[2]
                                for p, q in zip(m["crossings"], b["crossings"]))):
                    m["gone"] = b["gone"] = True
                    same.update(zip(b["ids"], m["ids"]))
                    for i, j in zip(b["ids"], m["ids"]):
                        span[j] = max(span[j], span[i])
                    break
        moves[first:] = [m for m in mine if "gone" not in m]
        state["ids"] = [same.get(i, i) for i in state["ids"]]
        C, ids = state["C"], state["ids"]
        flipped = [r for r, i in enumerate(ids) if i in before and before[i] != top[r]]
        for r in list(flipped):       # two crossings side by side that only traded places
            for q in flipped:
                if q != r and r in flipped and top[q] != top[r] and abs(C[q, 0] - C[r, 0]) + abs(C[q, 1] - C[r, 1]) <= tol:
                    ids[q], ids[r] = ids[r], ids[q]
                    flipped.remove(q)
                    flipped.remove(r)
                    break
        for r in flipped:             # the top strand changed: strands passed through each other
            moves.append({"t": tk, "type": "pass", "sign": "0", "ids": [ids[r]], "crossings": as_list(C, [r])})
        C[:, 2] = top
        last = rope[k]
        frames.append((tk, state["C"], state["ids"]))

    for m in moves:
        if m["type"] in ("R1", "R2") and all(span[i] < KINK for i in m["ids"]):
            m["type"] = "kink"

    def diagram(at):
        """The diagram just before time `at`."""
        return next((f for f in reversed(frames) if f[0] < at), frames[0])

    # Actions: from each grasp to the next one (the last one to the end of the run).
    held = grip[:, 5] >= 0
    starts = [i for i in range(len(t)) if held[i] and (i == 0 or not held[i - 1])]
    actions = []
    for a_i, s in enumerate(starts):
        e = starts[a_i + 1] if a_i + 1 < len(starts) else len(t)
        t0, t1 = float(t[s]), float(t[e]) if e < len(t) else float(t[-1]) + 1e-3
        rel = next((i for i in range(s, e) if not held[i]), e)
        mine = [m for m in moves if t0 <= m["t"] < t1]
        # Moves the action undid: crossings it made and removed again by the same kind of move.
        born = {}
        for m in mine:
            key = (m["type"], tuple(sorted(m["ids"])))
            if m["sign"] == "+" and m["type"] in TYPES + ("kink",):
                born[key] = m
            elif m["sign"] == "-" and key in born:
                born.pop(key)["undone"] = m["undone"] = True
        # R3: count only how far the order along the rope changed from the start to the end.
        start, end = diagram(t0), diagram(t1)
        where = {i: r for r, i in enumerate(end[2])}
        r3_all = sum(m["type"] == "R3" for m in mine)
        r3 = min(r3_all, reorders(start[1], end[1], {r: where[i] for r, i in enumerate(start[2]) if i in where}, MIN_LOOP + 2))
        counts = {k: 0 for k in COUNTS}
        for m in mine:
            if not m.get("undone") and m["type"] != "R3":
                counts["pass" if m["type"] == "pass" else m["type"] + m["sign"].strip("0")] += 1
        counts["R3"] = r3
        undone = sum(1 for m in mine if m.get("undone")) + r3_all - r3
        label = next((ty for ty in TYPES if any(counts[k] for k in counts if k.rstrip("+-") == ty)), "none")
        actions.append({
            "index": a_i, "t_grasp": round(t0, 3), "t_release": round(float(t[rel]) if rel < len(t) else t1, 3),
            "t_end": round(t1, 3), "node": int(grip[s, 5]),
            "crossings_before": len(start[1]), "crossings_after": len(end[1]),
            "label": label, "counts": {k: v for k, v in counts.items() if v}, "undone": undone,
            "moves": [{"t": round(m["t"], 3), "type": m["type"], "sign": m["sign"], "crossings": m["crossings"],
                       **({"undone": True} if m.get("undone") else {})} for m in mine],
        })
    before_first = [m for m in moves if not starts or m["t"] < float(t[starts[0]])]
    return actions, moves, before_first


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("folder", help="folder written by export_episodes.py")
    ap.add_argument("--out", help="where to write (default: <folder>/moves)")
    args = ap.parse_args()
    out = args.out or os.path.join(args.folder, "moves")
    os.makedirs(out, exist_ok=True)
    files = sorted(glob.glob(os.path.join(args.folder, "episode_*.npz")), key=lambda f: int(f.split("_")[-1][:-4]))
    if not files:
        sys.exit("no episode_*.npz in " + args.folder)

    rows, totals, labels = [], {}, {}
    for f in files:
        rid = int(os.path.basename(f)[8:-4])
        with open(f[:-4] + ".json", encoding="utf-8") as fh:
            meta = json.load(fh)
        actions, moves, early = label_episode(np.load(f), meta)
        r = meta.get("round") or {}
        with open(os.path.join(out, "episode_%d_moves.json" % rid), "w", encoding="utf-8") as fh:
            json.dump({"id": rid, "knot": r.get("knot"), "assist": r.get("assist", "off"), "sim": meta.get("sim"),
                       "actions": actions, "moves_before_first_grasp": len(early)}, fh, indent=1)
        for a in actions:
            labels[a["label"]] = labels.get(a["label"], 0) + 1
            for key, v in a["counts"].items():
                totals[key] = totals.get(key, 0) + v
            rows.append({"episode": rid, "action": a["index"], "label": a["label"], "knot": r.get("knot"),
                         "assist": r.get("assist", "off"), "sim": meta.get("sim") or "",
                         "t_grasp": a["t_grasp"], "t_release": a["t_release"], "t_end": a["t_end"],
                         "crossings_before": a["crossings_before"], "crossings_after": a["crossings_after"],
                         **{k: a["counts"].get(k, 0) for k in COUNTS}, "undone": a["undone"]})

    with open(os.path.join(out, "actions.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else ["episode"])
        w.writeheader()
        w.writerows(rows)
    print("%d episodes, %d actions -> %s" % (len(files), len(rows), out))
    print("actions by label: " + ", ".join("%s %d" % (k, labels.get(k, 0)) for k in TYPES + ("none",)))
    print("moves kept: " + ", ".join("%s %d" % (k, totals[k]) for k in COUNTS if totals.get(k)))


if __name__ == "__main__":
    main()
