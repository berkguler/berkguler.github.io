#!/usr/bin/env python3
"""Export Knot Lab runs to one .npz (arrays) and one .json (metadata) per run.

Reads the knot_episodes table from Supabase with the service_role key
(SUPABASE_URL, SUPABASE_SERVICE_KEY), or a JSON / CSV export of that table.
The recording format is described in README.md next to this file.

    python3 export_episodes.py --out data/knot-lab
    python3 export_episodes.py --json rows.json --out data/knot-lab
    python3 export_episodes.py --csv knot_episodes.csv --out data/knot-lab --anonymize
"""
import argparse
import base64
import csv
import gzip
import json
import os
import struct
import sys
import urllib.error
import urllib.parse
import urllib.request

import numpy as np

PAGE = 20


def decode(data):
    """Decode one recording (the `data` column) into (header, arrays)."""
    raw = base64.b64decode(data)
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    if raw[:4] != b"KNT1":
        raise ValueError("not a knot-lab recording")
    (hlen,) = struct.unpack_from("<I", raw, 4)
    header = json.loads(raw[8:8 + hlen].decode("utf-8"))
    if header.get("format") != "knot-lab/1":
        raise ValueError("unknown format %r" % header.get("format"))
    T, n, res = header["samples"], header["n"], header["res"]
    idx = np.asarray(header["ropeFrames"], dtype=np.int32)
    K = len(idx)
    off = 8 + hlen + (4 - (8 + hlen) % 4) % 4
    expected = off + 4 * T * (1 + 7 + 4) + 2 * 3 * n * K
    if len(raw) != expected:
        raise ValueError("size %d, expected %d" % (len(raw), expected))

    def take(dtype, count):
        nonlocal off
        a = np.frombuffer(raw, dtype=dtype, count=count, offset=off)
        off += a.nbytes
        return a

    t = take("<f4", T)
    grip = take("<f4", 7 * T).reshape(T, 7)
    hand = take("<f4", 4 * T).reshape(T, 4)
    rope = take("<i2", 3 * n * K).reshape(K, n, 3)
    rope = np.cumsum(rope.astype(np.int32), axis=0).astype(np.float32) / res
    arrays = {"t": t, "grip": grip, "hand": hand, "rope": rope, "rope_t": t[idx], "rope_idx": idx}
    return header, arrays


def fetch_rows(url, key, since):
    """Rows with id > since from Supabase, oldest first, a page at a time."""
    while True:
        query = urllib.parse.urlencode({"select": "*", "id": "gt.%d" % since, "order": "id.asc", "limit": PAGE})
        headers = {"apikey": key}
        if key.startswith("eyJ"):  # a legacy service_role key (a JWT); a secret key (sb_secret_...) goes in apikey only
            headers["Authorization"] = "Bearer " + key
        req = urllib.request.Request(url.rstrip("/") + "/rest/v1/knot_episodes?" + query, headers=headers)
        with urllib.request.urlopen(req) as res:
            rows = json.load(res)
        for row in rows:
            yield row
        if len(rows) < PAGE:
            return
        since = rows[-1]["id"]


def read_csv(path):
    csv.field_size_limit(sys.maxsize)
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            row["id"] = int(row["id"])
            if isinstance(row.get("meta"), str) and row["meta"]:
                row["meta"] = json.loads(row["meta"])
            yield row


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="output folder")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--json", help="JSON export of knot_episodes (a list of rows)")
    src.add_argument("--csv", help="CSV export of knot_episodes")
    ap.add_argument("--anonymize", action="store_true", help="replace usernames by player_001, player_002, ...")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    done = {int(f[8:-4]) for f in os.listdir(args.out) if f.startswith("episode_") and f.endswith(".npz")}
    names_path = os.path.join(args.out, "players.json")
    names = {}
    if args.anonymize and os.path.exists(names_path):
        with open(names_path, encoding="utf-8") as f:
            names = json.load(f)

    if args.json:
        with open(args.json, encoding="utf-8") as f:
            rows = json.load(f)
    elif args.csv:
        rows = read_csv(args.csv)
    else:
        url, key = os.environ.get("SUPABASE_URL"), os.environ.get("SUPABASE_SERVICE_KEY")
        if not url or not key:
            ap.error("set SUPABASE_URL and SUPABASE_SERVICE_KEY, or pass --json / --csv")
        rows = fetch_rows(url, key, max(done, default=0))

    wrote = failed = 0
    for row in rows:
        rid = int(row["id"])
        if rid in done:
            continue
        try:
            header, arrays = decode(row["data"])
        except Exception as e:  # keep going; one bad row should not stop the export
            print("episode %d: %s" % (rid, e), file=sys.stderr)
            failed += 1
            continue
        user = row.get("username")
        if args.anonymize and user is not None:
            key = user.lower()
            if key not in names:
                names[key] = "player_%03d" % (len(names) + 1)
            user = names[key]
        meta = {k: row.get(k) for k in ("id", "created_at", "knot", "crossings", "seconds", "grasps", "misses", "hints", "points")}
        meta["username"] = user
        meta.update({k: header[k] for k in ("format", "rate", "res", "table", "n", "samples", "round", "params", "events", "result", "device")})
        base = os.path.join(args.out, "episode_%d" % rid)
        np.savez_compressed(base + ".npz", **arrays)
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=1, ensure_ascii=False)
        wrote += 1

    if args.anonymize:
        with open(names_path, "w", encoding="utf-8") as f:
            json.dump(names, f, indent=1)
    print("wrote %d episode(s) to %s%s" % (wrote, args.out, ", %d failed" % failed if failed else ""))


if __name__ == "__main__":
    try:
        main()
    except urllib.error.HTTPError as e:
        sys.exit("Supabase answered HTTP %d; check SUPABASE_URL and SUPABASE_SERVICE_KEY" % e.code)
