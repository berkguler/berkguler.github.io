# Knot Lab backend and data

[berkguler.github.io/knot](https://berkguler.github.io/knot/) is the knot-untangling game
(`assets/js/assistdlo-game.js`) on its own page, set up for data collection:

- every round deals a **shuffled, verified knot**: a prime knot from the Rolfsen table (3₁ to 7₁,
  weighted towards the easier ones) with a random thickness (10 to 18 mm), bending stiffness
  (limp to stiff), mirror image, start direction and rotation. Before a rope is dealt the game
  checks that it lies on the table, that no two strands overlap, that it has as many crossings as
  the drawing it was made from (the braid word's length, which for 5₂ and 6₁ is one more than the
  crossing number) and the knot's determinant, and that both still hold after settling for 30
  simulation steps; otherwise it draws again;
- players sign in with a **username and a 4-digit PIN** (no email). The first sign-in with a free
  name claims it;
- every **untied** knot is recorded (rope state, gripper, pointer, events) and saved with the
  username; skipped or unfinished rounds are not kept;
- a **leaderboard** ranks players by points.

The page talks to a [Supabase](https://supabase.com) project (hosted Postgres with an HTTP API)
through three database functions. Without one configured, the page shows a note and the game is
playable but nothing is saved.

## Setup (once)

1. Create a project at [supabase.com](https://supabase.com) (the free plan is enough to start;
   check its current limits on database size and pausing of inactive projects). A run of a
   minute or so takes roughly 50 to 400 kB, so export the data now and then.
2. In the project, open **SQL Editor**, paste [`schema.sql`](schema.sql) and run it. Running it
   again later is safe.
3. Open **Project Settings → API Keys** and copy the **publishable** key (`sb_publishable_...`;
   the older **anon** key, `eyJ...`, works too), and from **Project Settings → Data API** the
   **Project URL**, into `_config.yml`:

   ```yaml
   knot_lab:
     supabase_url: https://YOUR-PROJECT.supabase.co
     supabase_anon_key: sb_publishable_...
   ```

   This key is meant to be public. With `schema.sql` it can do nothing except call
   `knot_login`, `knot_submit` and `knot_leaderboard`: the tables have row-level security on and
   no grants for the anon role, so nobody can read the recordings or PIN hashes through it.
   **Never** put a secret key (`sb_secret_...`, or the older `service_role` key) in the site; it
   is only for the export script below.
4. Push. The page picks the settings up at the next GitHub Pages build.

### What the backend checks

- Usernames: 3 to 20 letters, digits, `-` or `_`, unique regardless of case.
- PINs: exactly 4 digits, stored as bcrypt hashes. Five wrong PINs in a row lock the name for
  15 minutes.
- Submissions: valid PIN, at most one run per 5 seconds per player, sane values (3 to 9 crossings,
  1 to 900 s, at least one grasp), recording between 100 bytes and 6 MB.
- Points are computed on the server from the submitted values:
  `10·c + 10·c·max(0, 1 − seconds / max(180, 60·c))` for a knot with `c` crossings, halved if a
  hint was used. A browser game cannot be made cheat-proof; the recordings let you spot and drop
  implausible runs before using the data.

### Useful SQL

```sql
-- runs per knot
select knot, count(*), round(avg(seconds)) as avg_s from knot_episodes group by knot order by knot;
-- remove a player and all of their runs (for example on request)
delete from knot_players where lower(username) = lower('some_name');
```

## Exporting the data

[`export_episodes.py`](export_episodes.py) (Python 3 with numpy) downloads the runs and writes
one `.npz` with arrays and one `.json` with the metadata per run:

```sh
export SUPABASE_URL=https://YOUR-PROJECT.supabase.co
export SUPABASE_SERVICE_KEY=sb_secret_... # secret (or legacy service_role) key, keep it private
python3 tools/knot-lab/export_episodes.py --out data/knot-lab
```

It only fetches runs that are not in `--out` yet, so it can be re-run to pick up new ones. It can
also read a JSON or CSV export of the `knot_episodes` table (`--json rows.json` or
`--csv knot_episodes.csv`). `--anonymize` replaces usernames by `player_001`, `player_002`, …
(stable within one output folder) for sharing.

Per run, `episode_<id>.npz` holds, in table units (1 unit = 1 mm = 1 px of the 640 × 400 table,
x to the right, y down, z up from the table):

| array      | shape       | content |
|------------|-------------|---------|
| `t`        | (T,)        | time in seconds since the first press (about 20 Hz, see below) |
| `grip`     | (T, 7)      | gripper x, y, z, yaw (degrees), open (1/0), grasped rope node (−1: none), phase |
| `hand`     | (T, 4)      | pointer x, y (the commanded target), button held (1/0), pointer over the table (1/0) |
| `rope`     | (K, n, 3)   | rope node positions x, y, z |
| `rope_t`   | (K,)        | time of each rope frame |
| `rope_idx` | (K,)        | sample index of each rope frame (all of them unless the run was long, see below) |

A sample is taken at the first simulation frame on or after each 50 ms tick, so the spacing
varies by up to one display frame (about 17 ms at 60 Hz); `t` holds the exact times, so resample
to a fixed rate if a model needs one.

Gripper phases: 0 idle, 1 travel, 2 descend, 3 close, 4 carry, 5 lift. The `.json` file holds
the round (knot, thickness `rad` as the rope radius, stiffness from 0 to 1, mirror, flip, rotation, crossings
as dealt, determinant, and `minSep`, the closest approach of two strands at the deal in rope
diameters), the
simulation parameters, the event list (press, grasp, miss, release, up, hint, with time, position
and rope node), the result (seconds, grasps, misses, hints) and the row's username, time and
points.

Training on the data (for example with ACT) is not part of this folder.

## Recording format (`knot-lab/1`)

The `data` column is base64 of a gzip-compressed binary file (raw, not gzipped, if the browser
had no `CompressionStream`). All numbers are little-endian.

| bytes | content |
|-------|---------|
| 4     | magic `KNT1` |
| 4     | `L`, uint32: length of the header |
| L     | header, UTF-8 JSON |
| 0–3   | zero padding to a multiple of 4 |
| 4·T   | float32 `t` |
| 4·7·T | float32 `grip`, row-major (T, 7) |
| 4·4·T | float32 `hand`, row-major (T, 4) |
| 2·3·n·K | int16 rope frames (K, n, 3) in 1/`res` units: the first frame as is, every other frame as the difference to the frame before it |

Header fields: `format`, `rate` (Hz), `res` (rope positions are stored in 1/`res` units),
`table`, `n` (rope nodes), `samples` (T), `ropeFrames` (the sample indices of the K stored rope
frames), `round`, `params`, `events`, `result`, `device` (`mouse`, `touch` or `pen`).

The rope is stored for every sample unless the compressed run would exceed about 1.3 MB; then
only every 2nd (or 4th, 8th) rope frame is kept, always including the last. Gripper and pointer
are always stored at the full rate.
