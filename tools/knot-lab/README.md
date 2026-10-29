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

The game page does not load Google Analytics (`analytics: false` in `knot.html`; the rest of the site
does), so the page's "What is recorded" note is complete. Keep it that way, or update the note.

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
the rope; `table` when the table changed size; `slip` when the rope slipped out of the gripper,
see Simulation), the result (seconds, grasps,
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

## Labelling the moves

[`label_moves.py`](label_moves.py) sorts every action of every run (from one grasp to the next,
including the rope settling after the release) into one of four kinds of move, named after
Reidemeister's moves on the rope seen from above (its diagram):

| label | what the action did to the diagram |
|---|---|
| `End` | an end of the rope slid onto or off a strand: one crossing at the end. The only one of these moves that can change the knot |
| `R2` | a strand slid onto or off another one: two neighbouring crossings, the same strand on top at both |
| `R1` | a twist loop opened or closed: one crossing whose loop holds no other crossing |
| `R3` | a strand slid across a crossing: no crossing appeared or vanished, but their order along the rope changed |
| `none` | the diagram is the same as before (the rope only moved, or the action undid what it did) |

```sh
python3 tools/knot-lab/label_moves.py data/knot-lab     # the folder export_episodes.py wrote
```

It writes `data/knot-lab/moves/actions.csv`, one row per action: the label, the knot, assistance,
simulation version, times (grasp, release, next grasp), crossings before and after, and how many
moves of each kind the action kept (`+` adds crossings, `-` removes them) and how many it undid. For each run,
`moves/episode_<id>_moves.json` lists every move with its time and crossings (positions along the
rope in nodes, and which strand was on top).

How it works: it follows the diagram from one rope frame to the next, keeps track of each crossing,
and names every crossing that appears or vanishes, and every change in their order, by the move it
fits. Between two frames a fast pull can make several moves, so it goes through the rope in
between (positions moved in straight lines, halved again until one move is left), but takes which
strand is on top only from the recorded frames. An action can make several moves; its label is the
most significant one it kept (`End` before `R2` before `R1` before `R3`), and the moves it undid
itself (a strand pushed onto another and pulled back off, slid across a crossing and back) cancel
out. The counts are there for other labellings. Three more kinds are counted but never make a label:
`kink` (an R1 or R2 from the rope bending sharply over itself: the loop was never longer than
4 rope thicknesses), `pass` (strands went through each other: a crossing whose top strand
changed, or two crossings side by side with different strands on top that appeared or vanished
together, while the knot changed without an End move) and `unclassified` (fits no move). The
signs of a pass without the knot changing are mostly a fast pull fooling the tracking (a crossing
taken for its neighbour), so those count as `unclassified`, or not at all when two crossings only
traded places.

[`test_label_moves.py`](test_label_moves.py) checks it on hand-made rope motions with known moves.
On 60 simulated runs (knots untied by a planner, mixed with fast random drags; 433 actions):

- the knot's determinant, which R1, R2 and R3 cannot change, changed 118 times: 93 times with an
  `End` move between the same two rope frames, 23 times with a `pass`, once with an `unclassified`
  change and once with none of them (a pass-through one frame before the labeller saw it);
- for every action, the crossings before and after add up with the moves it kept;
- about 2% of the moves were `unclassified`;
- with only every 2nd rope frame kept (as for long runs, see below), 96% of the actions got the
  same label as at the full rate; with every 4th, 91%.

A `pass` is a change of the knot that no End move explains, almost always strands really pushed
through each other in the simulation (see the end of Simulation below for how often that happens),
so drop or check runs with `pass` before training on them. The first version of the labeller did
not check the knot and reported 55 passes in the same runs instead of 27.

## Simulation

The rope (in `assets/js/assistdlo-game.js`) is simulated with position-based dynamics (PBD) in 3D:
a chain of nodes one rope radius apart, moved by Verlet integration under gravity, at 3 substeps
per frame with 6 solver passes each. In every pass:

- **links** keep neighbouring nodes one radius apart, and a hard limit between nodes i and i+2
  stops kinks;
- **bending**: each node is pulled toward the midpoint of the nodes 1, 2 and 4 places away on
  either side (weights 1, ½, ¼), in the table plane, by the fraction `k = 0.03·s^1.75` per pass,
  where `s` is the round's `stiffness` (0 limp to 1 stiff) and `k` its `stiff`. Dragging the
  middle of a rope (thickness 14) then bends it about 56° per node when limp and 21° when stiff,
  and a stiff, thick rope laid in a loop springs partly open;
- **tether** (long-range attachments): while the gripper holds node k, no node i may be farther
  from k than the rope between them is long, so pulling drags the rope instead of stretching it;
- **self-collision**: spheres of the rope's radius on every node push apart (found through a grid),
  then the links are solved again so that a tight knot does not stretch the rope;
- **gripper**: the held node follows the gripper; the finger pads push other strands aside;
- **crossing guard** (last pass of each substep): wherever two segments cross in the table plan,
  the upper one is kept at least 0.9 rope thicknesses above the lower one, on the side it is on,
  so strands can press together but never swap over and under;
- **table**: nodes stay above it and inside the table; resting nodes are heavier and lose half
  their speed per substep (sliding friction);
- **static friction** (end of each substep): a node lying on the table or on another strand that
  moved less than 0.04 units in the substep is put back where it was. Without it, small steady
  pushes (bending, strands settling on each other) made ropes nobody touched creep across the
  table; with it, an untouched rope comes to rest at once, and real pulls are far stronger;
- **knot check** (end of each substep): the knot, as its determinant from the crossing guard's
  crossings, must be the one after the substep before, unless a crossing on the rope's first or
  last three segments came or went (an end moving past a strand). If it changed anyway, the
  constraints have pushed one strand through another (they can work against each other when a
  pull meets a tight knot, or when a stretched rope springs back): the simulation goes back two
  frames and starts from rest, with no node moving more than a quarter radius per substep for the
  next 10 frames so the stretch comes out gradually. If the gripper was holding the rope, it lets
  go: the rope slipped out, as a real one pulled too hard through a tight spot would, and the game
  says so and records a `slip` event. After five undos in a row it lets the change stand rather
  than freeze the rope.

With the tether and the guard, pulling knots tight in tests (all knots, thicknesses and stiffnesses,
pulled by an end or the middle, and random fast drags) gives no crossings that swap over and under
in place, and links stretch at most about 2× in the tightest knots. Strands passing through each
other were measured in simulated play like the game's (the gripper's speeds and lifts, untying
moves mixed with fast random drags; 32 games), checked after every substep as the knot's
determinant changing with no crossing at the rope's ends changing:

| `sim` | passes through | in frames | per 1,000 frames | slips |
|---|---|---|---|---|
| `pbd-4` | 19 | 42,800 | 0.44 | none |
| `pbd-5` (knot check) | 6 | 45,200 | 0.13 | 42, about one per 18 s of play |

The ones left are almost all right after a slip, when the rope let go springs back through itself
more than five times in a row. Tried and dropped, as they did not help clearly: more solver
passes or substeps, self-collision twice per pass or 10% thicker, links twice per pass, capsule
(segment) collision, a speed limit, a capped tether, finger pads off while holding, and a crossing
guard that keeps the order from the start of the substep. The move labels above mark what is left
as `pass`.

The gripper is not a rigid body in the simulation: it moves at most 330 units/s sideways and
240 units/s up and down, closes on the rope node between its jaws, and the held node then follows
it exactly. With SA-CBF it is sent to the chosen rope point instead of the pressed point.

### Simulation versions

The physics has changed several times; mix runs from different versions only on purpose.

| `sim` | in use | what changed |
|---|---|---|
| `pbd-5` | from the knot-check release | knot check (above): instead of passing through, the rope slips out of the gripper (`slip` event) or goes back two frames; about a third as many passes through |
| `pbd-4` | 2026-10-27, about 21:40 UTC, to `pbd-5` | static friction (above): untouched ropes no longer creep |
| `pbd-3` | 2026-10-27, about 20:25 UTC, to `pbd-4` | multi-scale bending (above), stiffness `k = 0.03·s^1.75`; untouched ropes crept (stiff ones by up to about 40 units in 10 s) |
| (none) | 2026-10-27, about 19:40 to the `pbd-3` release (UTC) | tether and crossing guard: no pass-through; bending only between nodes i and i+2, `k = 0.06·s²` |
| (none) | before 2026-10-27 about 19:40 UTC | first version: pulled tight, the rope could stretch and strands could pass through each other |

Runs without `sim` can be told apart by the row's `created_at` (allow a few minutes around the
switch for browsers that still had the old script).

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

Header fields: `format`, `sim` (simulation version, below), `rate` (Hz), `res` (rope positions are stored in 1/`res` units),
`table` (`[x0, y0, x1, y1]`), `n` (rope nodes), `samples` (T), `ropeFrames` (the sample indices of the K stored rope
frames), `round`, `params`, `events`, `result`, `device` (`mouse`, `touch` or `pen`).

The rope is stored for every sample unless the compressed run would exceed about 1.3 MB; then
only every 2nd (or 4th, 8th) rope frame is kept, always including the last. Gripper and pointer
are always stored at the full rate.

`knot-lab/1` (the first runs) is the same, except that the table was always 640 × 400 and
`table` is `[640, 400]`; the export script turns it into `[0, 0, 640, 400]`.
