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

`knot_lab.music_youtube_ids` in `_config.yml` sets the background music: a list of YouTube video
ids (the part after `youtu.be/`), played in a small player under the game, or an empty list for no
music. Each visit picks one and starts at a random point; the shuffle button under the game moves
to another video, or with a single video to another part of it. The videos must allow embedding;
otherwise the player and the music buttons hide themselves.

### What the backend checks

- Usernames: 3 to 20 letters, digits, `-` or `_`, unique regardless of case.
- PINs: exactly 4 digits, stored as bcrypt hashes. Five wrong PINs in a row lock the name for
  15 minutes.
- Submissions: valid PIN, at most one run per 5 seconds per player, sane values (3 to 9 crossings,
  1 to 900 s, at least one grasp), recording between 100 bytes and 6 MB.
- Points are computed on the server from the submitted values:
  `10·c + 10·c·max(0, 1 − seconds / max(180, 60·c))` for a knot with `c` crossings (halved if
  `hints` > 0; the game no longer has hints and always sends 0). A browser game cannot be made
  cheat-proof; the recordings let you spot and drop implausible runs before using the data.

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
simulation parameters, the table, the event list (press, grasp, miss, release, up, with
time, position and rope node; `blocked` when SA-CBF kept the gripper up after a click away from
the rope; `table` when the table changed size), the result (seconds, grasps,
misses) and the row's username, time and points. Runs recorded before hints were removed may
also have `hint` events and a `hints` count.

**Assistance.** `round.assist` (also in the row's `meta`) is `off` or `sa-cbf`. With SA-CBF a
press near the rope sends the gripper to the nearest rope point (the upper strand at a crossing)
instead of the pressed point, and a press away from the rope does not lower the gripper, as on
the AssistDLO page (the page does not draw the funnels there). The pointer channel (`hand`) is
where the player pressed; the gripper channel (`grip`) is where the gripper went.

**The table.** Knots are dealt in the middle 640 × 400 (`[0, 0, 640, 400]`), or, on screens
taller than wide such as phones held upright, in an upright 400 × 640 (`[120, -120, 520, 520]`)
with the drawing turned a quarter; the round records which (`upright`, `area`), and `rot`
includes the quarter turn. The table then extends on both sides to fit the screen, so positions
can be negative or beyond 640 / 400.
`table` is `[x0, y0, x1, y1]` when recording starts; during a run it can change (the player went
full screen or turned the phone), always keeping the rope on it, which shows up as a `table`
event with the new `box`. The simulation keeps the rope inside the table at all times.

Training on the data (for example with ACT) is not part of this folder.

## Simulation

The rope (in `assets/js/assistdlo-game.js`) is simulated with position-based dynamics (PBD) in 3D:
a chain of nodes one rope radius apart, moved by Verlet integration under gravity, at 3 substeps
per frame with 6 solver passes each. In every pass:

- **links** keep neighbouring nodes one radius apart; a hard limit between nodes i and i+2 stops
  kinks, and the bending stiffness pulls those towards straight;
- **tether** (long-range attachments): while the gripper holds node k, no node i may be farther
  from k than the rope between them is long, so pulling drags the rope instead of stretching it;
- **self-collision**: spheres of the rope's radius on every node push apart (found through a grid),
  then the links are solved again so that a tight knot does not stretch the rope;
- **gripper**: the held node follows the gripper; the finger pads push other strands aside;
- **crossing guard** (last pass of each substep): wherever two segments cross in the table plan,
  the upper one is kept at least 0.9 rope thicknesses above the lower one, on the side it is on,
  so strands can press together but never swap over and under;
- **table**: nodes stay above it and inside the table; resting nodes are heavier and lose half
  their speed per substep (friction).

With the tether and the guard, pulling knots tight in tests (all knots, thicknesses and stiffnesses,
pulled by an end or the middle, and random fast drags) gives no strands passing through each other,
and links stretch at most about 2× in the tightest knots.

The gripper is not a rigid body in the simulation: it moves at most 330 units/s sideways and
240 units/s up and down, closes on the rope node between its jaws, and the held node then follows
it exactly. With SA-CBF it is sent to the chosen rope point instead of the pressed point.

## Recording format (`knot-lab/2`)

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
`table` (`[x0, y0, x1, y1]`), `n` (rope nodes), `samples` (T), `ropeFrames` (the sample indices of the K stored rope
frames), `round`, `params`, `events`, `result`, `device` (`mouse`, `touch` or `pen`).

The rope is stored for every sample unless the compressed run would exceed about 1.3 MB; then
only every 2nd (or 4th, 8th) rope frame is kept, always including the last. Gripper and pointer
are always stored at the full rate.

`knot-lab/1` (the first runs) is the same, except that the table was always 640 × 400 and
`table` is `[640, 400]`; the export script turns it into `[0, 0, 640, 400]`.
