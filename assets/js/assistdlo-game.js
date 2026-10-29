/* Knot-untangling game for the AssistDLO project page.
 *
 * A knot lies on a table, seen from above: the overhand knot, or another
 * prime knot from the Rolfsen table drawn as a closed braid and cut open on
 * its outer strand in the same way (an open rope only mimics the knot's
 * diagram; it is never knotted in the mathematical sense). Click a strand to
 * grasp it, drag to carry it, and release to put it down. The knot counts as
 * untied when the rope no longer crosses itself, as in the paper's study.
 * The rope is a small 3D simulation (gravity, table friction, bending
 * stiffness and self-collision), so strands cannot pass through each other,
 * even when pulled tight; its thickness and stiffness can be changed. The gripper is teleoperated
 * directly, or with SA-CBF assistance in simplified form: a barrier keeps the
 * gripper above the rope except in funnels around rope points, so a click
 * near the rope is guided onto the nearest point (the upper strand at a
 * crossing) and a click away from it does not descend. The tutorial shows
 * the controls with a hand on a practice knot. Rendering uses rope.js
 * (window.RopeDraw).
 */
(function () {
  'use strict';

  var W = 640, H = 400;
  var SIM_VERSION = 'pbd-5';             // recorded with every run: pbd-5 = slips instead of passing through (see README)
  var GRAV = 1800, SUB = 3, ITER = 6;    // px/s², substeps per frame, solver iterations
  var W_TABLE = 0.2;                     // rope resting on the table is harder to move (friction)
  var FINGER = 4;                        // finger pad radius (px)
  var Z_HOVER = 80;                      // gripper height while travelling (px above the table)
  var V_XY = 330, V_Z = 240;             // gripper speed limits (px/s)
  var STICK = 0.04;                      // static friction: smallest move of a resting node per substep (px)
  var MAX_STIFF = 0.03;                  // bending stiffness (see solveBending) at the top of the slider

  // Everything that scales with the rope: rad is the radius in px, stiff the
  // bending stiffness (0: limp, MAX_STIFF: springy).
  // Knots are dealt in the middle 640 x 400 of the table, or in an upright
  // 400 x 640 on screens taller than wide (phones held upright).
  var DEAL = [0, 0, W, H], DEAL_UPRIGHT = [W / 2 - H / 2, H / 2 - W / 2, W / 2 + H / 2, H / 2 + W / 2];

  // box is the table [x0, y0, x1, y1]: the deal area, extended on both sides
  // to the screen's shape.
  function ropeParams(rad, stiff, box) {
    var cell = 2 * rad + 2;
    box = box || DEAL;
    return {
      rad: rad, seg: rad, bend: 1.45 * rad, stiff: stiff, box: box,
      cell: cell, cols: Math.ceil((box[2] - box[0]) / cell), rows: Math.ceil((box[3] - box[1]) / cell),
      gapOpen: rad + 4.5, gapShut: rad,          // finger centres from the jaw centre
      zCarry: 4 * rad + 6,                       // carrying height
      funnel: Math.max(22, 2.6 * rad + 8)        // SA-CBF: clicks this close to the rope are guided onto it
    };
  }

  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
  function f1(x) { return Math.round(x * 10) / 10; }
  function orient(ax, ay, bx, by, cx, cy) { return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax); }

  // Crossings of a polyline in the top view, as [i, j] segment pairs (i < j).
  function crossings(x, y, n) {
    var out = [];
    for (var i = 0; i < n - 1; i++) {
      var ax = x[i], ay = y[i], bx = x[i + 1], by = y[i + 1];
      var x0 = Math.min(ax, bx), x1 = Math.max(ax, bx), y0 = Math.min(ay, by), y1 = Math.max(ay, by);
      for (var j = i + 2; j < n - 1; j++) {
        var cx = x[j], cy = y[j], dx = x[j + 1], dy = y[j + 1];
        if (Math.max(cx, dx) < x0 || Math.min(cx, dx) > x1 || Math.max(cy, dy) < y0 || Math.min(cy, dy) > y1) continue;
        if ((orient(cx, cy, dx, dy, ax, ay) > 0) !== (orient(cx, cy, dx, dy, bx, by) > 0) &&
            (orient(ax, ay, bx, by, cx, cy) > 0) !== (orient(ax, ay, bx, by, dx, dy) > 0)) out.push([i, j]);
      }
    }
    return out;
  }

  // Segment intersection parameters [t, u], or null.
  function hit(ax, ay, bx, by, cx, cy, dx, dy) {
    var rx = bx - ax, ry = by - ay, sx = dx - cx, sy = dy - cy, den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-9) return null;
    var t = ((cx - ax) * sy - (cy - ay) * sx) / den, u = ((cx - ax) * ry - (cy - ay) * rx) / den;
    return t > 0 && t < 1 && u > 0 && u < 1 ? [t, u] : null;
  }

  function det(M) {
    var n = M.length, d = 1;
    M = M.map(function (r) { return r.slice(); });
    for (var c = 0; c < n; c++) {
      var p = c;
      for (var r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-9) return 0;
      if (p !== c) { var tmp = M[p]; M[p] = M[c]; M[c] = tmp; d = -d; }
      d *= M[c][c];
      for (var r2 = c + 1; r2 < n; r2++) {
        var f = M[r2][c] / M[c][c];
        for (var k = c; k < n; k++) M[r2][k] -= f * M[c][k];
      }
    }
    return d;
  }

  // Is the rope still knotted? Close it with an arc from end to end that
  // passes over everything (as if both ends were lifted and pulled apart)
  // and return the knot determinant: 3 for the overhand knot, 1 when the
  // knot is gone. Over/under at each crossing comes from the simulated heights.
  function knotDeterminant(x, y, z, n) {
    var cr = [];
    for (var i = 0; i < n - 1; i++) {
      for (var j = i + 2; j < n - 1; j++) {
        var h = hit(x[i], y[i], x[i + 1], y[i + 1], x[j], y[j], x[j + 1], y[j + 1]);
        if (!h) continue;
        var zi = z[i] + (z[i + 1] - z[i]) * h[0], zj = z[j] + (z[j + 1] - z[j]) * h[1];
        cr.push(zi > zj ? { over: i + h[0], under: j + h[1] } : { over: j + h[1], under: i + h[0] });
      }
    }
    return determinantOf(cr, x, y, n);
  }

  // The same for a list of the rope's own crossings {over, under} (positions
  // along the rope), found elsewhere.
  function determinantOf(cr, x, y, n) {
    cr = cr.slice();
    // The closing arc is "segment" n - 1, from the last node back to the first.
    for (var k = 1; k < n - 2; k++) {
      var c = hit(x[k], y[k], x[k + 1], y[k + 1], x[n - 1], y[n - 1], x[0], y[0]);
      if (c) cr.push({ over: n - 1 + c[1], under: k + c[0] });
    }
    var m = cr.length;
    if (m < 2) return 1;
    var unders = cr.map(function (c) { return c.under; }).sort(function (a, b) { return a - b; });
    var arcOf = function (pos) {   // arcs run from one under-pass to the next, cyclically
      for (var a = m - 1; a >= 0; a--) if (pos > unders[a]) return a;
      return m - 1;
    };
    var M = [];
    cr.forEach(function (c) {
      var row = new Array(m).fill(0), out = unders.indexOf(c.under);
      row[arcOf(c.over)] += 2;
      row[(out + m - 1) % m] -= 1;
      row[out] -= 1;
      M.push(row);
    });
    M.pop();
    M = M.map(function (r) { return r.slice(0, m - 1); });
    return Math.round(Math.abs(det(M)));
  }

  // ---------- rope simulation (position-based dynamics) ----------

  // pts: [x, y] or [x, y, h] per node. With h, the strand with the larger h is
  // on top at each crossing; without it, crossings alternate along the rope.
  function Sim(pts, p, overFirst) {
    var n = this.n = pts.length;
    this.p = p;
    var A = function () { return new Float64Array(n); };
    this.x = A(); this.y = A(); this.z = A(); this.px = A(); this.py = A(); this.pz = A();
    for (var i = 0; i < n; i++) { this.x[i] = pts[i][0]; this.y[i] = pts[i][1]; }
    var cr = crossings(this.x, this.y, n), over = [];
    if (pts[0].length > 2) {
      cr.forEach(function (c) { over.push(pts[c[0]][2] + pts[c[0] + 1][2] > pts[c[1]][2] + pts[c[1] + 1][2] ? c[0] : c[1]); });
    } else {
      var events = [], pick = {};
      cr.forEach(function (c, k) { events.push({ at: c[0], k: k }, { at: c[1], k: k }); });
      events.sort(function (a, b) { return a.at - b.at; });
      events.forEach(function (e, idx) {
        if (pick[e.k] !== undefined) return;
        var first = (idx % 2 === 0) === overFirst;
        pick[e.k] = first ? e.at : (e.at === cr[e.k][0] ? cr[e.k][1] : cr[e.k][0]);
      });
      for (var key in pick) over.push(pick[key]);
    }
    // Lift the upper strand where it crosses.
    for (var c = 0; c < over.length; c++) {
      for (var d = -4; d <= 5; d++) {
        var m = over[c] + d;
        if (m >= 0 && m < n) this.z[m] = Math.max(this.z[m], 2 * p.rad * Math.exp(-(d - 0.5) * (d - 0.5) / 5));
      }
    }
    this.px.set(this.x); this.py.set(this.y); this.pz.set(this.z);
    this.grab = -1;
    this.undos = 0; this.calm = 0; this.ease = 0;
    this.head = new Int32Array(p.cols * p.rows);
    this.next = new Int32Array(n);
    for (var s = 0; s < 40; s++) this.step(1 / 60, null);
  }

  Sim.prototype.buildGrid = function () {
    var head = this.head, next = this.next, p = this.p;
    head.fill(-1);
    for (var i = 0; i < this.n; i++) {
      var c = clamp(Math.floor((this.x[i] - p.box[0]) / p.cell), 0, p.cols - 1) + p.cols * clamp(Math.floor((this.y[i] - p.box[1]) / p.cell), 0, p.rows - 1);
      next[i] = head[c];
      head[c] = i;
    }
  };

  // One frame. grip: {x, y, z, yaw, open, hold} or null.
  Sim.prototype.step = function (dt, grip) {
    var x = this.x, y = this.y, z = this.z, px = this.px, py = this.py, pz = this.pz, n = this.n;
    var h = dt / SUB, g = GRAV * h * h;
    this.moved = 0;
    for (var s = 0; s < SUB; s++) {
      this.snapshot();
      for (var i = 0; i < n; i++) {
        var vx = x[i] - px[i], vy = y[i] - py[i], vz = z[i] - pz[i];
        var keep = z[i] < 0.6 ? 0.5 : 0.99;   // table friction, air drag
        px[i] = x[i]; py[i] = y[i]; pz[i] = z[i];
        if (i === this.grab) continue;
        x[i] += vx * keep; y[i] += vy * keep; z[i] += vz * 0.99 - g;
        this.moved = Math.max(this.moved, Math.abs(vx) + Math.abs(vy) + Math.abs(vz));
      }
      if (this.grab >= 0 && grip) { x[this.grab] = grip.x; y[this.grab] = grip.y; z[this.grab] = grip.z; }
      this.buildGrid();
      for (var it = 0; it < ITER; it++) {
        this.solveLinks();
        this.solveBending();
        if (this.grab >= 0) this.solveTether();
        this.solveContacts();
        this.solveLinks();   // again: in a tight knot the contacts would otherwise stretch the rope
        if (grip) this.solveGripper(grip);
        if (it === ITER - 1) this.solveCrossings();   // once per substep: motion within one is far too small to swap strands
        this.solveBounds();
      }
      this.easeOut();
      this.stick();
      this.keepKnot();
    }
  };

  // The solver's constraints can work against each other: a pull through a
  // tight knot, or a stretched rope springing back once let go, now and then
  // pushes one strand through another. Catch it: after every substep the knot
  // (its determinant, from the crossing guard's crossings) must be the same as
  // after the one before, unless a crossing at one of the rope's ends came or
  // went (an end slipped past a strand). If not, go back UNDO substeps (two
  // frames, to before the strands were pressed into each other) and start
  // from rest; while the gripper holds the rope, it slips out, as a real rope
  // pulled too hard through a tight spot would (the game shows it and records
  // a 'slip'). The state it goes back to is still stretched tight: let go, the
  // links would snap shut at once and push the strands through again, so for
  // EASE substeps no node moves more than a quarter radius per substep and the
  // stretch comes out gradually. After a few undos in a row it lets the change
  // stand rather than freeze the rope.
  var UNDO = 6, UNDO_MAX = 5, EASE = 30;
  Sim.prototype.snapshot = function () {
    var n = this.n, r = this.ring || (this.ring = { at: 0, count: 0, x: [], y: [], z: [] });
    if (!r.x[r.at] || r.x[r.at].length !== n) { r.x[r.at] = new Float64Array(n); r.y[r.at] = new Float64Array(n); r.z[r.at] = new Float64Array(n); }
    r.x[r.at].set(this.x); r.y[r.at].set(this.y); r.z[r.at].set(this.z);
    r.at = (r.at + 1) % UNDO;
    r.count = Math.min(UNDO, r.count + 1);
  };
  Sim.prototype.keepKnot = function () {
    if (!this.crossList) return;
    var n = this.n, ends = 0, cr = this.crossList, h = this.kept;
    for (var c = 0; c < cr.length; c++) {
      var a = Math.min(cr[c].over, cr[c].under), b = Math.max(cr[c].over, cr[c].under);
      if (a < 3 || b > n - 4) ends++;   // on the first or last three segments
    }
    var d = determinantOf(cr, this.x, this.y, n);
    if (h && d !== h.det && ends === h.ends && this.undos < UNDO_MAX) {
      var r = this.ring, o = (r.at - r.count + UNDO) % UNDO;   // the oldest snapshot
      this.x.set(r.x[o]); this.y.set(r.y[o]); this.z.set(r.z[o]);
      this.px.set(r.x[o]); this.py.set(r.y[o]); this.pz.set(r.z[o]);   // and at rest
      r.count = 0;
      this.undos++;
      this.calm = 0;
      this.ease = EASE;
      if (this.grab >= 0) { this.grab = -1; this.slipped = true; }
      return;
    }
    if (++this.calm > 30) this.undos = 0;
    this.kept = { det: d, ends: ends };
  };

  // After an undo (above): nodes move at most a quarter radius per substep.
  Sim.prototype.easeOut = function () {
    if (!this.ease) return;
    this.ease--;
    var x = this.x, y = this.y, z = this.z, px = this.px, py = this.py, pz = this.pz, lim = 0.25 * this.p.rad;
    for (var i = 0; i < this.n; i++) {
      if (i === this.grab) continue;
      var dx = x[i] - px[i], dy = y[i] - py[i], dz = z[i] - pz[i], d = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (d <= lim) continue;
      x[i] = px[i] + dx * lim / d; y[i] = py[i] + dy * lim / d; z[i] = pz[i] + dz * lim / d;
    }
  };

  // Inverse mass for the solver: zero when held, small when resting on the table.
  Sim.prototype.weight = function (i) {
    return i === this.grab ? 0 : this.z[i] < 0.6 ? W_TABLE : 1;
  };

  // Fixed length between neighbours, and between nodes i and i+2 a hard limit
  // on how sharply the rope can kink.
  Sim.prototype.solveLinks = function () {
    var x = this.x, y = this.y, z = this.z, n = this.n, p = this.p;
    for (var pass = 0; pass < 2; pass++) {
      var gap = pass === 0 ? 1 : 2;
      for (var i = 0; i + gap < n; i++) {
        var j = i + gap;
        var dx = x[j] - x[i], dy = y[j] - y[i], dz = z[j] - z[i];
        var L = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-6;
        var wi = this.weight(i), wj = this.weight(j);
        if (!wi && !wj) continue;
        var k;
        if (gap === 1) k = (L - p.seg) / L / (wi + wj);
        else if (L < p.bend) k = (L - p.bend) / L / (wi + wj);
        else continue;
        x[i] += dx * k * wi; y[i] += dy * k * wi; z[i] += dz * k * wi;
        x[j] -= dx * k * wj; y[j] -= dy * k * wj; z[j] -= dz * k * wj;
      }
    }
  };

  // Bending stiffness: every node is pulled toward the midpoint of the nodes
  // 1, 2 and 4 places away on either side (with weight 1, 1/2, 1/4 of
  // p.stiff), a discrete curvature penalty at three scales. A stiff rope so
  // resists gentle curves as well as sharp kinks, and holds smooth loops.
  // Only in the table plane: flattening a strand that rides over another
  // would press it down into the one below.
  var BEND_SPANS = [[1, 1], [2, 0.5], [4, 0.25]];
  Sim.prototype.solveBending = function () {
    var k0 = this.p.stiff, x = this.x, y = this.y, n = this.n;
    if (!(k0 > 0)) return;
    for (var q = 0; q < BEND_SPANS.length; q++) {
      var m = BEND_SPANS[q][0], k = k0 * BEND_SPANS[q][1];
      for (var i = m; i < n - m; i++) {
        var a = i - m, c = i + m;
        var wa = this.weight(a), wb = this.weight(i), wc = this.weight(c), den = wb + (wa + wc) / 4;
        if (!den) continue;
        var f = k / den;
        var cx = x[i] - (x[a] + x[c]) / 2, cy = y[i] - (y[a] + y[c]) / 2;
        x[i] -= wb * f * cx; y[i] -= wb * f * cy;
        x[a] += wa * f / 2 * cx; y[a] += wa * f / 2 * cy;
        x[c] += wc * f / 2 * cx; y[c] += wc * f / 2 * cy;
      }
    }
  };

  // Rope against rope: spheres of the rope's radius on every node; neighbours
  // along the rope are skipped. Nodes resting on the table push each other sideways.
  Sim.prototype.solveContacts = function () {
    var x = this.x, y = this.y, z = this.z, head = this.head, next = this.next, p = this.p;
    var D = 2 * p.rad, D2 = D * D;
    for (var i = 0; i < this.n; i++) {
      var cx = clamp(Math.floor((x[i] - p.box[0]) / p.cell), 0, p.cols - 1), cy = clamp(Math.floor((y[i] - p.box[1]) / p.cell), 0, p.rows - 1);
      for (var oy = -1; oy <= 1; oy++) {
        var ry = cy + oy;
        if (ry < 0 || ry >= p.rows) continue;
        for (var ox = -1; ox <= 1; ox++) {
          var rx = cx + ox;
          if (rx < 0 || rx >= p.cols) continue;
          for (var j = head[rx + p.cols * ry]; j >= 0; j = next[j]) {
            if (j <= i + 2) continue;
            var dx = x[i] - x[j], dy = y[i] - y[j], dz = z[i] - z[j];
            if (z[i] < 0.5 && z[j] < 0.5) dz = 0;
            var d2 = dx * dx + dy * dy + dz * dz;
            if (d2 >= D2) continue;
            var d = Math.sqrt(d2);
            if (d < 1e-6) { dx = 0.01; d = 0.01; }
            var wi = this.weight(i), wj = this.weight(j);
            if (!wi && !wj) continue;
            var k = (D - d) / d / (wi + wj);
            x[i] += dx * k * wi; y[i] += dy * k * wi; z[i] += dz * k * wi;
            x[j] -= dx * k * wj; y[j] -= dy * k * wj; z[j] -= dz * k * wj;
          }
        }
      }
    }
  };

  // The rope does not stretch: no node may be farther from the held one than
  // the rope between them is long (long-range attachments). Pulling drags the
  // rope instead of opening gaps between its nodes for other strands to slip
  // through.
  Sim.prototype.solveTether = function () {
    var x = this.x, y = this.y, z = this.z, k = this.grab, seg = 1.02 * this.p.seg;
    for (var i = 0; i < this.n; i++) {
      if (i === k) continue;
      var dx = x[i] - x[k], dy = y[i] - y[k], dz = z[i] - z[k];
      var d = Math.sqrt(dx * dx + dy * dy + dz * dz), max = Math.abs(i - k) * seg;
      if (d <= max) continue;
      var f = (d - max) / d;
      x[i] -= dx * f; y[i] -= dy * f; z[i] -= dz * f;
    }
  };

  // Where two strands cross on the table plan, the upper one stays at least
  // most of a rope's thickness above the lower one, on whichever side it is
  // now: a pull can press strands together but never swap over and under.
  // Candidate strands come from the contact grid: a segment crossing segment
  // i has an end within (length of i + half the longest segment) of node i,
  // which is the next cells over unless the rope is stretched.
  Sim.prototype.solveCrossings = function () {
    var x = this.x, y = this.y, z = this.z, px = this.px, py = this.py, pz = this.pz;
    var head = this.head, next = this.next, p = this.p, n = this.n, D = 1.8 * p.rad, i, L = [], Lmax = 0;
    var seen = this.seen || (this.seen = new Int32Array(n)), list = this.crossList = [];
    seen.fill(-1);
    for (i = 0; i < n - 1; i++) { L[i] = Math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]); Lmax = Math.max(Lmax, L[i]); }
    for (i = 0; i < n - 1; i++) {
      var cx = clamp(Math.floor((x[i] - p.box[0]) / p.cell), 0, p.cols - 1), cy = clamp(Math.floor((y[i] - p.box[1]) / p.cell), 0, p.rows - 1);
      var reach = Math.min(3, Math.ceil((L[i] + Lmax / 2) / p.cell));
      for (var oy = -reach; oy <= reach; oy++) {
        var ry = cy + oy;
        if (ry < 0 || ry >= p.rows) continue;
        for (var ox = -reach; ox <= reach; ox++) {
          var rx = cx + ox;
          if (rx < 0 || rx >= p.cols) continue;
          for (var m = head[rx + p.cols * ry]; m >= 0; m = next[m]) {
            for (var j = m - 1; j <= m; j++) {   // the two segments at node m
              if (j < i + 2 || j >= n - 1 || seen[j] === i) continue;
              seen[j] = i;
              var h = hit(x[i], y[i], x[i + 1], y[i + 1], x[j], y[j], x[j + 1], y[j + 1]);
              if (!h) continue;
              var s = h[0], t = h[1];
              var dz = z[i] + (z[i + 1] - z[i]) * s - z[j] - (z[j + 1] - z[j]) * t;
              if (Math.abs(dz) >= D) { list.push(dz > 0 ? { over: i + s, under: j + t } : { over: j + t, under: i + s }); continue; }
              // Which strand is on top: as now, or as a moment ago, or (both on
              // the table) the one moving more rides over.
              var up = dz > 1e-3 ? 1 : dz < -1e-3 ? -1 : 0;
              if (!up) {
                var was = pz[i] + (pz[i + 1] - pz[i]) * s - pz[j] - (pz[j + 1] - pz[j]) * t;
                up = was > 1e-3 ? 1 : was < -1e-3 ? -1 : 0;
              }
              if (!up) {
                var vi = Math.abs(x[i] - px[i]) + Math.abs(y[i] - py[i]), vj = Math.abs(x[j] - px[j]) + Math.abs(y[j] - py[j]);
                up = vi >= vj ? 1 : -1;
              }
              list.push(up > 0 ? { over: i + s, under: j + t } : { over: j + t, under: i + s });
              // Lift the upper segment, press the lower one down unless it lies
              // on the table; split by inverse mass at the crossing point.
              var a = up > 0 ? i : j, b = up > 0 ? j : i, sa = up > 0 ? s : t, sb = up > 0 ? t : s;
              var wa0 = this.weight(a), wa1 = this.weight(a + 1);
              var wb0 = z[b] < 0.6 ? 0 : this.weight(b), wb1 = z[b + 1] < 0.6 ? 0 : this.weight(b + 1);
              var den = wa0 * (1 - sa) * (1 - sa) + wa1 * sa * sa + wb0 * (1 - sb) * (1 - sb) + wb1 * sb * sb;
              if (!den) continue;
              var lam = (D - Math.abs(dz)) / den;
              z[a] += lam * wa0 * (1 - sa); z[a + 1] += lam * wa1 * sa;
              z[b] -= lam * wb0 * (1 - sb); z[b + 1] -= lam * wb1 * sb;
            }
          }
        }
      }
    }
  };

  // Gripper fingers against rope: two pads on the jaw axis push rope away, so
  // a strand between them is centred and one outside them is shoved aside.
  // The strand around grip.hold (held or just let go) is left alone.
  Sim.prototype.solveGripper = function (grip) {
    var x = this.x, y = this.y, z = this.z, p = this.p, D = FINGER + p.rad;
    var a = grip.yaw * Math.PI / 180, ax = -Math.sin(a), ay = Math.cos(a);
    var gap = grip.open ? p.gapOpen : p.gapShut;
    for (var f = -1; f <= 1; f += 2) {
      var fx = grip.x + f * gap * ax, fy = grip.y + f * gap * ay;
      for (var i = 0; i < this.n; i++) {
        if (grip.hold >= 0 && Math.abs(i - grip.hold) <= 8) continue;   // the strand hanging from the jaws
        if (z[i] + p.rad <= grip.z - 6) continue;
        var dx = x[i] - fx, dy = y[i] - fy, d = Math.sqrt(dx * dx + dy * dy);
        if (d >= D) continue;
        if (d < 1e-6) { dx = -f * ax; dy = -f * ay; d = 1; }
        var k = (D - d) / d;
        x[i] += dx * k; y[i] += dy * k;
      }
    }
  };

  Sim.prototype.solveBounds = function () {
    var r = this.p.rad, b = this.p.box;
    for (var i = 0; i < this.n; i++) {
      if (this.z[i] < 0) this.z[i] = 0;
      this.x[i] = clamp(this.x[i], b[0] + r, b[2] - r);
      this.y[i] = clamp(this.y[i], b[1] + r, b[3] - r);
    }
  };

  // Static friction: rope lying on the table (or on another strand) that moved
  // less than STICK this substep stays where it was. Small steady pushes
  // (bending, strands settling on each other) then cannot make a rope that
  // nobody touches creep across the table; real pulls are far stronger.
  Sim.prototype.stick = function () {
    var x = this.x, y = this.y, z = this.z, px = this.px, py = this.py, high = 2.5 * this.p.rad, lim2 = STICK * STICK;
    for (var i = 0; i < this.n; i++) {
      if (i === this.grab || z[i] > high) continue;
      var dx = x[i] - px[i], dy = y[i] - py[i];
      if (dx * dx + dy * dy < lim2) { x[i] = px[i]; y[i] = py[i]; }
    }
  };

  // Put the rope on another table.
  Sim.prototype.setBox = function (box) {
    var p = {}, k;
    for (k in this.p) p[k] = this.p[k];
    p.box = box;
    p.cols = Math.ceil((box[2] - box[0]) / p.cell);
    p.rows = Math.ceil((box[3] - box[1]) / p.cell);
    this.p = p;
    this.head = new Int32Array(p.cols * p.rows);
  };

  Sim.prototype.crossingCount = function () { return crossings(this.x, this.y, this.n).length; };

  // The rope point nearest to (px, py) within r, preferring the upper strand.
  Sim.prototype.pick = function (px, py, r) {
    var best = -1, bs = Infinity;
    for (var i = 0; i < this.n; i++) {
      var d = Math.hypot(this.x[i] - px, this.y[i] - py);
      if (d > r) continue;
      var score = d - 0.4 * this.z[i];
      if (score < bs) { bs = score; best = i; }
    }
    return best;
  };

  // ---------- knots ----------

  // Prime knots from the Rolfsen table as closed braids (braid words as in
  // KnotInfo: generator i crosses strands i and i+1, its sign says which is
  // on top). The overhand knot (3_1) keeps its familiar drawing. det is the
  // knot determinant from the table, weight how often the Knot Lab deals it.
  var KNOTS = {
    '3_1': { name: '3₁ overhand', det: 3, weight: 30 },
    '4_1': { name: '4₁ figure-eight', word: [1, -2, 1, -2], strands: 3, det: 5, weight: 15 },
    '5_1': { name: '5₁ cinquefoil', word: [1, 1, 1, 1, 1], strands: 2, det: 5, weight: 15 },
    '5_2': { name: '5₂ three-twist', word: [1, 1, 1, 2, -1, 2], strands: 3, det: 7, weight: 12 },
    '6_1': { name: '6₁ stevedore', word: [1, 1, 2, -1, -3, 2, -3], strands: 4, det: 9, weight: 4 },
    '6_2': { name: '6₂', word: [1, 1, 1, -2, 1, -2], strands: 3, det: 11, weight: 8 },
    '6_3': { name: '6₃', word: [1, 1, -2, 1, -2, -2], strands: 3, det: 13, weight: 8 },
    '7_1': { name: '7₁ septafoil', word: [1, 1, 1, 1, 1, 1, 1], strands: 2, det: 7, weight: 8 }
  };

  // Points [x, y(, h)] every `step` px along a polyline.
  function resample(pts, step) {
    var out = [pts[0].slice()], acc = 0;
    for (var i = 1; i < pts.length; i++) {
      var a = pts[i - 1], b = pts[i], L = Math.hypot(b[0] - a[0], b[1] - a[1]), t = step - acc;
      while (t <= L) {
        out.push(a.map(function (v, d) { return v + (b[d] - v) * t / L; }));
        t += step;
      }
      acc = L - (t - step);
    }
    return out;
  }

  function smooth(u) { u = clamp(u, 0, 1); return u * u * (3 - 2 * u); }

  // A closed braid drawn on concentric rings, cut open on the outer ring
  // between two crossings (like an overhand knot cut from a trefoil), turned
  // so that the cut faces up, with a tail on either side. Returns [x, y, h]
  // with h > 0 on the upper and h < 0 on the lower strand at each crossing.
  function braidRope(word, strands, p) {
    var m = word.length, step = 2 * Math.PI / m, hw = 0.36 * step, S = 16, GAP = 3;
    var R0 = Math.max(5.2 * p.rad, 36, (strands === 2 ? 9 : 0) * m), D = 2 * p.rad + 16;
    var ringR = function (j) { return R0 + j * D; };
    var path = [], cut = -1, cutAngle = 0, pos = 0, laps = 0;
    do {
      for (var k = 0; k < m; k++) {
        var g = Math.abs(word[k]), next = pos === g - 1 ? g : pos === g ? g - 1 : pos;
        var up = next === pos ? 0 : (next > pos) === (word[k] > 0) ? 1 : -1;
        for (var s = 0; s < S; s++) {
          var a = (k - 0.5 + (s + 0.5) / S) * step;   // never exactly on a crossing
          var r = ringR(pos) + (ringR(next) - ringR(pos)) * smooth((a - k * step + hw) / (2 * hw));
          path.push([r * Math.cos(a), r * Math.sin(a), up * Math.max(0, 1 - Math.abs(a - k * step) / hw)]);
        }
        pos = next;
        if (cut < 0 && pos === strands - 1) { cut = path.length; cutAngle = (k + 0.5) * step; }
      }
      laps++;
    } while (pos !== 0 && laps <= strands);
    var loop = path.slice(cut).concat(path.slice(0, cut)).slice(GAP, -GAP);
    // Turn the cut to the top (SVG y points down) and add the tails.
    var rot = -Math.PI / 2 - cutAngle, c = Math.cos(rot), sn = Math.sin(rot);
    loop = loop.map(function (q) { return [q[0] * c - q[1] * sn, q[0] * sn + q[1] * c, q[2]]; });
    var tail = function (e, side) {
      var len = Math.hypot(e[0], e[1]), ux = e[0] / len + 0.45 * side, uy = e[1] / len, ul = Math.hypot(ux, uy);
      return [1, 2, 3, 4].map(function (i) { return [e[0] + ux / ul * 16 * i, e[1] + uy / ul * 16 * i - 2 * i * i, 0]; });
    };
    var a0 = loop[0], a1 = loop[loop.length - 1];
    var side0 = a0[0] > a1[0] ? 1 : -1;
    var pts = tail(a0, side0).reverse().concat(loop, tail(a1, -side0));
    return resample(pts, p.seg);
  }

  // Loose overhand knot with the tails cut to TAIL nodes beyond the
  // outermost crossings.
  var TAIL = 12;
  function overhandRope(R, p) {
    var pts = resample(R.spline(R.overhand(0, 0, 42, 0, 0), 10), p.seg);
    var xs = pts.map(function (q) { return q[0]; }), ys = pts.map(function (q) { return q[1]; });
    var cr = crossings(xs, ys, pts.length), lo = Infinity, hi = -Infinity;
    cr.forEach(function (c) { lo = Math.min(lo, c[0]); hi = Math.max(hi, c[1] + 1); });
    return pts.slice(Math.max(0, lo - TAIL), Math.min(pts.length, hi + TAIL + 1));
  }

  // The rope for a knot from KNOTS, centred on the table, optionally mirrored
  // and turned by rot (radians).
  function knotPoints(R, id, p, mirror, flip, rot) {
    var K = KNOTS[id];
    var pts = K.word ? braidRope(K.word, K.strands, p) : overhandRope(R, p);
    if (flip && K.word) pts = pts.map(function (q) { return [q[0], q[1], -q[2]]; });
    if (rot) {
      var c = Math.cos(rot), sn = Math.sin(rot);
      pts = pts.map(function (q) { var r = [q[0] * c - q[1] * sn, q[0] * sn + q[1] * c]; if (q.length > 2) r.push(q[2]); return r; });
    }
    var x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    pts.forEach(function (q) { x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]); });
    var dx = W / 2 - (x0 + x1) / 2, dy = (H - 4) / 2 - (y0 + y1) / 2;
    return pts.map(function (q) {
      var out = [mirror ? W - (q[0] + dx) : q[0] + dx, q[1] + dy];
      if (q.length > 2) out.push(q[2]);
      return out;
    });
  }

  // Crossings in the drawing of a knot: the braid word's length, 3 for the overhand.
  function drawnCrossings(id) { return KNOTS[id].word ? KNOTS[id].word.length : 3; }

  // Build a rope for the given settings and check it before anyone sees it:
  // it lies on the table, strands keep their distance, it has the drawing's
  // crossings and the knot's determinant, and still has both after settling.
  // Returns {sim, ...checks} or null when a check fails.
  function verifiedRope(R, id, p, mirror, flip, rot) {
    var pts = knotPoints(R, id, p, mirror, flip, rot), r = p.rad, area = p.box;
    for (var i = 0; i < pts.length; i++) {
      if (pts[i][0] < area[0] + r + 2 || pts[i][0] > area[2] - r - 2 || pts[i][1] < area[1] + r + 2 || pts[i][1] > area[3] - r - 2) return null;
    }
    var sim = new Sim(pts, p, flip), want = drawnCrossings(id), det = KNOTS[id].det;
    var check = function () {
      return sim.crossingCount() === want && knotDeterminant(sim.x, sim.y, sim.z, sim.n) === det;
    };
    if (!check()) return null;
    var minSep = Infinity;
    for (var a = 0; a < sim.n; a++) {
      for (var b = a + 3; b < sim.n; b++) {
        minSep = Math.min(minSep, Math.hypot(sim.x[a] - sim.x[b], sim.y[a] - sim.y[b], sim.z[a] - sim.z[b]));
      }
    }
    if (minSep < 1.8 * r) return null;
    for (var f = 0; f < 30; f++) sim.step(1 / 60, null);
    if (!check()) return null;
    return { sim: sim, crossings: want, det: det, minSep: minSep / (2 * r) };
  }

  // ---------- page widget ----------

  var NS = 'http://www.w3.org/2000/svg';
  var C = { orange: '#FE7743', navy: '#273F4F', blue: '#447D9B', gray: '#D7D7D7' };

  function el(name, attrs, parent) {
    var node = document.createElementNS(NS, name);
    for (var k in attrs) node.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(node);
    return node;
  }

  function Game(root) {
    this.root = root;
    this.svg = root.querySelector('.game-svg');
    this.stage = root.querySelector('.game-stage');
    this.runs = [];
    this.assist = false;
    this.knot = '3_1';
    this.rad = 7;
    this.stiffness = 0.2;   // slider position, 0..1
    this.lab = root.hasAttribute('data-lab');   // Knot Lab: every round is shuffled and recorded
    this.simVersion = SIM_VERSION;
    this.hideFunnels = root.hasAttribute('data-hide-funnels');   // SA-CBF without drawing its funnels
    this.visible = false;
    this.build();
    this.bind();
    this.newKnot();
  }

  Game.prototype.q = function (s) { return this.root.querySelector(s); };

  // Tell listeners (the Knot Lab recorder) what happens: 'new', 'frame',
  // 'action' and 'finish', as knot:* events on the game's root element.
  Game.prototype.emit = function (type, detail) {
    this.root.dispatchEvent(new CustomEvent('knot:' + type, { detail: detail || {} }));
  };

  Game.prototype.build = function () {
    var svg = this.svg;
    this.funnels = el('path', { fill: 'none', stroke: C.blue, 'stroke-opacity': 0.13, 'stroke-linecap': 'round', 'stroke-linejoin': 'round' }, svg);
    this.ropeSlot = el('g', {}, svg);
    this.intentRing = el('circle', { r: 12, fill: 'none', stroke: C.orange, 'stroke-width': 2.5 }, svg);
    this.cursor = el('circle', { r: 10, fill: 'none', stroke: C.blue, 'stroke-width': 2, 'stroke-dasharray': '3 3' }, svg);
    var g = this.grip = el('g', {}, svg);
    this.bar = el('rect', { x: -3, width: 6, rx: 3, fill: C.navy }, g);
    this.f1 = el('rect', { x: -5, width: 10, height: 7, rx: 2, fill: C.navy }, g);
    this.f2 = el('rect', { x: -5, width: 10, height: 7, rx: 2, fill: C.navy }, g);
    el('circle', { r: 3.5, fill: C.orange, stroke: '#fff', 'stroke-width': 1.2 }, g);
    // Tutorial hand: a fingertip showing how to press, hold, drag and let go.
    this.hand = el('g', { 'pointer-events': 'none', opacity: 0 }, svg);
    this.handRipple = el('circle', { fill: 'none', stroke: C.orange, 'stroke-width': 3 }, this.hand);
    this.handTip = el('circle', { stroke: C.navy, 'stroke-width': 3 }, this.hand);
    this.flash = el('text', { x: 320, y: 384, 'text-anchor': 'middle', 'font-family': 'Roboto, sans-serif', 'font-size': 15, 'font-weight': 700, fill: C.navy }, svg);
    var root = this.root;
    this.out = {};
    ['time', 'knot', 'cross', 'grasps', 'misses', 'stiff', 'thick'].forEach(function (k) { this.out[k] = root.querySelector('[data-out="' + k + '"]'); }, this);
  };

  Game.prototype.bind = function () {
    var self = this, stage = this.stage;
    function local(ev) {
      var pt = self.svg.createSVGPoint();
      pt.x = ev.clientX; pt.y = ev.clientY;
      var p = pt.matrixTransform(self.svg.getScreenCTM().inverse()), b = self.sim.p.box;
      return [clamp(p.x, b[0], b[2]), clamp(p.y, b[1], b[3])];
    }
    // self.ptr keeps the last hand position (the carry target after a release);
    // self.hover says whether to draw it.
    // While the tutorial's hand is showing the controls, it has them.
    stage.addEventListener('pointermove', function (ev) { if (self.demo) return; self.ptr = local(ev); self.hover = true; self.wake(); });
    stage.addEventListener('pointerleave', function () { if (!self.held && !self.demo) { self.hover = false; self.wake(); } });
    stage.addEventListener('pointerdown', function (ev) {
      if (ev.button > 0 || self.demo) return;
      stage.setPointerCapture(ev.pointerId);
      self.ptr = local(ev);
      self.hover = true;
      self.press();
      ev.preventDefault();
    });
    // On phones a long press would start text selection, the loupe or a callout.
    stage.addEventListener('touchstart', function (ev) { ev.preventDefault(); }, { passive: false });
    ['selectstart', 'contextmenu'].forEach(function (t) {
      self.root.addEventListener(t, function (ev) {
        var el = ev.target.nodeType === 3 ? ev.target.parentNode : ev.target;
        if (!el.closest('input, select, textarea')) ev.preventDefault();
      });
    });
    ['pointerup', 'pointercancel'].forEach(function (t) {
      stage.addEventListener(t, function () {
        if (self.demo) return;
        if (self.held) self.emit('action', { type: 'up', x: self.ptr[0], y: self.ptr[1] });
        self.held = false;
        self.wake();
      });
    });
    // Keyboard: arrows move the hand, Space or Enter holds the grasp.
    stage.addEventListener('keydown', function (ev) {
      if (self.demo) return;
      var k = ev.key, d = ev.shiftKey ? 24 : 8;
      var step = { ArrowLeft: [-d, 0], ArrowRight: [d, 0], ArrowUp: [0, -d], ArrowDown: [0, d] }[k];
      if (step) {
        var p = self.ptr || [W / 2, H / 2], b = self.sim.p.box;
        self.ptr = [clamp(p[0] + step[0], b[0], b[2]), clamp(p[1] + step[1], b[1], b[3])];
        self.hover = true;
        self.wake();
        ev.preventDefault();
      } else if (k === ' ' || k === 'Enter') {
        if (!ev.repeat) { self.ptr = self.ptr || [W / 2, H / 2]; self.hover = true; self.press(); }
        ev.preventDefault();
      }
    });
    stage.addEventListener('keyup', function (ev) {
      if (self.demo || (ev.key !== ' ' && ev.key !== 'Enter')) return;
      if (self.held) self.emit('action', { type: 'up', x: self.ptr[0], y: self.ptr[1] });
      self.held = false;
      self.wake();
    });

    this.root.addEventListener('click', function (ev) {
      var b = ev.target.closest('button');
      if (!b) return;
      if (b.hasAttribute('data-new')) self.newKnot();
      if (b.hasAttribute('data-full')) self.toggleFull(!self.full);
      if (b.hasAttribute('data-tutorial')) { if (self.tut) self.endTutorial(); else self.startTutorial(); }
      if (b.hasAttribute('data-assist')) {
        self.assist = b.getAttribute('data-assist') === 'cbf';
        self.root.querySelectorAll('[data-assist]').forEach(function (o) { o.setAttribute('aria-pressed', String(o === b)); });
        self.newKnot();
      }
    });
    var knot = this.q('[data-knot]'), stiff = this.q('[data-stiff]'), thick = this.q('[data-thick]');
    // A setting changed during the tutorial ends it (and keeps the new setting).
    if (knot) knot.addEventListener('change', function () { if (self.tut) self.closeTutorial(); self.knot = knot.value; self.newKnot(); });
    // Stiffness applies at once; a new thickness needs a new rope.
    if (stiff) stiff.addEventListener('input', function () {
      self.stiffness = stiff.value / 100;
      if (self.tut) self.tut.saved.stiffness = self.stiffness;
      self.sim.p.stiff = self.stiffValue();
      self.showSettings();
      self.wake();
    });
    if (thick) {
      thick.addEventListener('input', function () { self.rad = +thick.value; self.showSettings(); });
      thick.addEventListener('change', function () { if (self.tut) self.closeTutorial(); self.rad = +thick.value; self.newKnot(); });
    }

    // Leaving the browser's full screen (Esc, back gesture) leaves ours too.
    ['fullscreenchange', 'webkitfullscreenchange'].forEach(function (t) {
      document.addEventListener(t, function () {
        if (self.fsApi && !(document.fullscreenElement || document.webkitFullscreenElement)) { self.fsApi = false; self.toggleFull(false); }
        self.fitTable();
      });
    });
    document.addEventListener('keydown', function (ev) { if (ev.key === 'Escape' && self.full) self.toggleFull(false); });
    if ('ResizeObserver' in window) new ResizeObserver(function () { self.fitTable(); }).observe(stage);
    window.addEventListener('resize', function () { self.fitTable(); });

    if ('IntersectionObserver' in window) {
      new IntersectionObserver(function (entries) {
        self.visible = entries[0].isIntersecting;
        self.wake();
      }).observe(this.stage);
    } else {
      this.visible = true;
    }
  };

  // Slider (0..1) to bending stiffness, curved so that equal steps of the
  // slider change how sharply a dragged rope bends by about equal amounts.
  Game.prototype.stiffValue = function () { return MAX_STIFF * Math.pow(this.stiffness, 1.75); };

  Game.prototype.showSettings = function () {
    var s = this.stiffness;
    if (this.out.stiff) this.out.stiff.textContent = s < 0.05 ? 'limp' : s < 0.35 ? 'soft' : s < 0.7 ? 'medium' : 'stiff';
    if (this.out.thick) this.out.thick.textContent = 2 * this.rad + ' mm';
  };

  // A random knot (by weight), thickness, stiffness, mirror image and turn.
  Game.prototype.shuffle = function () {
    var ids = Object.keys(KNOTS), total = 0, pick = Math.random();
    ids.forEach(function (id) { total += KNOTS[id].weight; });
    for (var i = 0, acc = 0; i < ids.length; i++) {
      acc += KNOTS[ids[i]].weight / total;
      if (pick < acc || i === ids.length - 1) { this.knot = ids[i]; break; }
    }
    this.rad = 5 + Math.floor(Math.random() * 5);
    this.stiffness = Math.random();
  };

  // opts.tutorial: the tutorial's practice knot, always drawn the same way.
  Game.prototype.newKnot = function (opts) {
    var practice = !!(opts && opts.tutorial);
    if (this.tut && !practice) this.closeTutorial();
    var R = window.RopeDraw, made = null, tries = 0;
    // On a screen taller than wide, deal upright (the drawing turned a quarter).
    var want = this.wantBox(), upright = want[2] - want[0] < want[3] - want[1], area = upright ? DEAL_UPRIGHT : DEAL, turn = upright ? Math.PI / 2 : 0;
    // Shuffle (in the Knot Lab) and build until a rope passes every check.
    while (!made && tries < 25) {
      if (this.lab && !practice && tries % 5 === 0) this.shuffle();
      tries++;
      var p = ropeParams(this.rad, this.stiffValue(), area);
      var mirror = !practice && Math.random() < 0.5, flip = !practice && Math.random() < 0.5;
      var rot = ((this.lab && !practice ? Math.random() * 2 * Math.PI : 0) + turn) % (2 * Math.PI);
      made = verifiedRope(R, this.knot, p, mirror, flip, rot);
    }
    if (!made) {   // never seen in testing; fall back to the plain overhand knot
      this.knot = '3_1';
      p = ropeParams(this.rad, this.stiffValue(), area);
      mirror = flip = false; rot = turn;
      made = { sim: new Sim(knotPoints(R, '3_1', p, false, false, rot), p, false), crossings: 3, det: 3, minSep: 1 };
    }
    this.sim = made.sim;
    this.round = {
      knot: this.knot, name: KNOTS[this.knot].name, rad: this.rad, stiffness: this.stiffness, stiff: p.stiff,
      mirror: mirror, flip: flip, rot: rot, crossings: made.crossings, det: made.det, minSep: made.minSep, tries: tries,
      upright: upright, area: area.slice(), assist: this.assist ? 'sa-cbf' : 'off'
    };
    if (practice) this.round.tutorial = true;   // not recorded by the Knot Lab
    var thick = this.q('[data-thick]'), stiffInput = this.q('[data-stiff]'), knotInput = this.q('[data-knot]');
    if (thick) thick.value = this.rad;
    if (stiffInput) stiffInput.value = Math.round(this.stiffness * 100);
    if (knotInput) knotInput.value = this.knot;
    // The renderer has one shape per rope segment, so every rope gets its own.
    this.ropeSlot.textContent = '';
    this.rope = new R.Rope(this.ropeSlot, { width: 2 * p.rad });
    this.funnels.setAttribute('stroke-width', 2 * p.funnel);
    this.g = { x: W - 34, y: H - 34, z: Z_HOVER, open: true, yaw: 0, hold: -1 };
    this.fitTable(true);
    this.phase = 'idle';
    this.target = null;
    this.held = false;
    this.running = false;
    this.finished = false;
    this.t = 0;
    this.grasps = 0;
    this.misses = 0;
    this.flashT = 0;
    this.solvedFor = 0;
    this.demo = null;
    this.nCross = this.sim.crossingCount();
    // Knot Lab keeps going until the knot is untied (up to the 15 minutes the server accepts).
    this.timeLimit = this.lab ? 900 : Math.max(180, 60 * this.nCross);
    this.tied = true;
    this.showSettings();
    this.status('Untie the ' + KNOTS[this.knot].name + ' knot: click a strand to grasp it, drag to move it, release to put it down.');
    this.render();
    this.emit('new', { round: this.round });
    this.wake();
  };

  Game.prototype.status = function (text) { this.q('.game-status').textContent = text; };

  Game.prototype.flashText = function (text, color) {
    // Same on-screen size on a phone as on a desktop.
    var b = this.sim.p.box, w = this.svg.getBoundingClientRect().width || W, upp = (b[2] - b[0]) / w;
    var size = 15 * clamp(upp, 1, 2.4);
    // In full screen the buttons cover the bottom of the table, so go above them.
    var foot = this.full ? (this.q('.comanip-footer').offsetHeight + 16) * upp : 8;
    this.flash.setAttribute('font-size', f1(size));
    this.flash.setAttribute('x', f1((b[0] + b[2]) / 2));
    this.flash.setAttribute('y', f1(b[3] - foot - size * 0.3));
    this.flash.textContent = text;
    this.flash.setAttribute('fill', color);
    this.flashT = 1.4;
  };

  // Full screen: the game covers the window (that alone works on iPhones,
  // which have no full screen for page elements), and where the browser
  // allows it, it also goes full screen and, on phones, turns to landscape.
  Game.prototype.toggleFull = function (on) {
    if (!on === !this.full) return;
    var root = this.root, doc = document, self = this;
    this.full = on;
    root.classList.toggle('is-full', on);
    doc.documentElement.classList.toggle('game-full', on);
    var b = this.q('[data-full]');
    if (b) {
      var label = on ? 'Exit full screen' : 'Full screen';
      b.setAttribute('aria-pressed', String(on));
      b.setAttribute('aria-label', label);
      b.title = label;
      b.querySelector('i').className = 'fa-solid ' + (on ? 'fa-compress' : 'fa-expand');
    }
    var req = root.requestFullscreen || root.webkitRequestFullscreen;
    var current = doc.fullscreenElement || doc.webkitFullscreenElement;
    if (on && req && !current) {
      try {
        var pr = req.call(root, { navigationUI: 'hide' });
        var turn = function () {
          self.fsApi = true;
          var o = screen.orientation;
          if (o && o.lock && window.matchMedia('(pointer: coarse)').matches) o.lock('landscape').catch(function () {});
        };
        if (pr && pr.then) pr.then(turn, function () {}); else turn();
      } catch (e) { /* the window-filling layout is enough */ }
    } else if (!on && current) {
      this.fsApi = false;
      var exit = doc.exitFullscreen || doc.webkitExitFullscreen;
      try { var px = exit.call(doc); if (px && px.catch) px.catch(function () {}); } catch (e) { /* already out */ }
    }
    this.fitTable();
    if (on) this.stage.focus({ preventScroll: true });
    this.wake();
  };

  // The table for the space the game has: in full screen the whole window, on
  // the page the column's width at a height that keeps the buttons in view on
  // wide screens. The 640 x 400 table is extended on both sides to that shape.
  Game.prototype.wantBox = function () {
    var w = this.stage.clientWidth || W, a;
    if (this.full) a = clamp(w / (this.stage.clientHeight || H), 0.45, 3.2);
    else a = clamp(w / Math.max(240, window.innerHeight - (w < 600 ? 220 : 260)), 0.62, 2.4);
    // The deal area for that shape (upright below 1:1), extended to it in
    // steps of 20, so small changes (a phone's address bar) leave it alone.
    var d = a < 1 ? DEAL_UPRIGHT : DEAL, k = (d[2] - d[0]) / (d[3] - d[1]);
    var ex = a > k ? 20 * Math.round(((d[3] - d[1]) * a - (d[2] - d[0])) / 40) : 0;
    var ey = a < k ? 20 * Math.round(((d[2] - d[0]) / a - (d[3] - d[1])) / 40) : 0;
    return [d[0] - ex, d[1] - ey, d[2] + ex, d[3] + ey];
  };

  // Fit the table to the screen. A new round gets exactly that; during a
  // round the table also keeps whatever the rope still covers.
  Game.prototype.fitTable = function (fresh) {
    var s = this.sim, g = this.g, b = this.wantBox();
    if (!s) return;
    if (!fresh) {
      var m = 2 * s.p.rad + 4;
      for (var i = 0; i < s.n; i++) {
        b[0] = Math.min(b[0], Math.floor(s.x[i] - m)); b[1] = Math.min(b[1], Math.floor(s.y[i] - m));
        b[2] = Math.max(b[2], Math.ceil(s.x[i] + m)); b[3] = Math.max(b[3], Math.ceil(s.y[i] + m));
      }
    }
    var old = s.p.box;
    if (!fresh && old[0] === b[0] && old[1] === b[1] && old[2] === b[2] && old[3] === b[3]) return;
    s.setBox(b);
    this.svg.setAttribute('viewBox', b[0] + ' ' + b[1] + ' ' + (b[2] - b[0]) + ' ' + (b[3] - b[1]));
    // Parked until the first press: in the corner, or in full screen (where
    // the buttons cover the corners) halfway down the right edge.
    if (fresh) this.parked = true;
    if (this.parked) { g.x = b[2] - 34; g.y = this.full ? (b[1] + b[3]) / 2 : b[3] - 34; }
    else if (s.grab < 0) { g.x = clamp(g.x, b[0] + 34, b[2] - 34); g.y = clamp(g.y, b[1] + 34, b[3] - 34); }   // carrying, it is on the rope
    if (this.ptr) this.ptr = [clamp(this.ptr[0], b[0], b[2]), clamp(this.ptr[1], b[1], b[3])];
    if (!fresh) this.emit('action', { type: 'table', box: b.slice() });
    this.wake();
  };

  // Hand pressed: send the gripper to that point (with SA-CBF, to the rope
  // point nearest to it).
  Game.prototype.press = function () {
    if (this.locked && !this.tut) { this.flashText(this.locked, C.navy); this.emit('locked'); this.wake(); return; }   // e.g. Knot Lab before signing in
    this.held = true;
    this.wake();
    if (this.finished || (this.phase !== 'idle' && this.phase !== 'lift')) return;
    var node = -1;
    if (this.assist) {
      node = this.sim.pick(this.ptr[0], this.ptr[1], this.sim.p.funnel);
      if (node < 0) {
        this.flashText('Barrier: no rope here, the gripper stays up', C.navy);
        this.emit('action', { type: 'blocked', x: this.ptr[0], y: this.ptr[1] });
        return;
      }
    }
    if (!this.running) this.running = true;
    this.parked = false;
    this.target = { node: node, x: this.ptr[0], y: this.ptr[1] };
    this.emit('action', { type: 'press', x: this.ptr[0], y: this.ptr[1], node: node });
    // Like an operator would, turn the jaws across the strand below.
    var below = node >= 0 ? node : this.sim.pick(this.target.x, this.target.y, 30);
    if (below >= 0) this.g.yaw = this.tangentAngle(below);
    this.phase = 'travel';
  };

  // Where the gripper is heading: the guided rope point, or the clicked point.
  Game.prototype.targetPos = function () {
    var T = this.target, s = this.sim;
    return T.node >= 0 ? [s.x[T.node], s.y[T.node], s.z[T.node]] : [T.x, T.y, 0];
  };

  // Move the gripper horizontally toward (x, y) at the speed limit; returns the remaining distance.
  Game.prototype.moveXY = function (x, y, dt) {
    var g = this.g, dx = x - g.x, dy = y - g.y, d = Math.hypot(dx, dy), step = V_XY * dt;
    if (d <= step) { g.x = x; g.y = y; return 0; }
    g.x += dx / d * step; g.y += dy / d * step;
    return d - step;
  };

  Game.prototype.moveZ = function (z, dt) {
    var g = this.g, step = V_Z * dt;
    g.z = Math.abs(z - g.z) <= step ? z : g.z + Math.sign(z - g.z) * step;
    return Math.abs(z - g.z);
  };

  // Highest rope point under the open jaws (0: table).
  Game.prototype.floorUnder = function () {
    var s = this.sim, top = 0, r = s.p.gapOpen - FINGER;
    for (var i = 0; i < s.n; i++) {
      if (Math.hypot(s.x[i] - this.g.x, s.y[i] - this.g.y) < r) top = Math.max(top, s.z[i]);
    }
    return top;
  };

  Game.prototype.update = function (dt) {
    var g = this.g, s = this.sim, p = s.p;
    if (this.running) this.t += dt;
    if (this.flashT) this.flashT = Math.max(0, this.flashT - dt);
    if (this.demo) this.demoStep(dt);

    if (this.phase === 'travel') {
      var T = this.targetPos();
      this.moveZ(Z_HOVER, dt);
      if (this.moveXY(T[0], T[1], dt) === 0 && g.z === Z_HOVER) this.phase = 'descend';
    } else if (this.phase === 'descend') {
      var floor = this.floorUnder();
      if (this.target.node >= 0) {
        // Down the funnel: follow the rope point, jaws across the rope.
        var P = this.targetPos();
        g.x = P[0]; g.y = P[1]; floor = P[2];
        g.yaw = this.tangentAngle(this.target.node);
      }
      if (this.moveZ(floor, dt) < 0.5) this.phase = 'close';
    } else if (this.phase === 'close') {
      this.close();
    } else if (this.phase === 'carry') {
      // Lift clear of the strands below first, then follow the hand.
      var c = [clamp(this.ptr[0] + this.carryOff[0], p.box[0] + p.rad, p.box[2] - p.rad), clamp(this.ptr[1] + this.carryOff[1], p.box[1] + p.rad, p.box[3] - p.rad)];
      this.moveZ(p.zCarry, dt);
      var left = g.z < 3 * p.rad ? 1 : this.moveXY(c[0], c[1], dt);
      if (!this.held && left === 0) this.release();
    } else if (this.phase === 'lift') {
      g.open = true;
      if (this.moveZ(Z_HOVER, dt) === 0) this.phase = 'idle';
    }

    if (this.phase === 'carry') g.hold = s.grab;
    else if (this.phase !== 'lift') g.hold = -1;
    s.step(dt, g);
    if (s.slipped) {
      s.slipped = false;
      if (this.phase === 'carry') this.slip();
    }

    this.nCross = s.crossingCount();
    this.tied = this.nCross > 0;
    if (this.running) this.emit('frame', { t: this.t, dt: dt });
    if (this.running) {
      if (!this.tied && s.grab < 0) this.solvedFor += dt; else this.solvedFor = 0;
      if (this.solvedFor > 0.5) this.finish(true);
      else if (this.t > this.timeLimit) this.finish(false);
    }
  };

  // Direction of the rope at node k (degrees), folded so the jaws turn the short way.
  Game.prototype.tangentAngle = function (k) {
    var s = this.sim, a = Math.max(0, k - 2), b = Math.min(s.n - 1, k + 2);
    var ang = Math.atan2(s.y[b] - s.y[a], s.x[b] - s.x[a]) * 180 / Math.PI;
    while (ang - this.g.yaw > 90) ang -= 180;
    while (ang - this.g.yaw < -90) ang += 180;
    return ang;
  };

  // Close the jaws: grasp the rope point between them, if any.
  Game.prototype.close = function () {
    var s = this.sim, g = this.g, best = -1, bd = Infinity, r = s.p.gapOpen - FINGER;
    for (var i = 0; i < s.n; i++) {
      var d = Math.hypot(s.x[i] - g.x, s.y[i] - g.y);
      if (d >= r || Math.abs(s.z[i] - g.z) > s.p.rad + 3) continue;
      if (i === this.target.node) { best = i; break; }
      if (d < bd) { bd = d; best = i; }
    }
    if (best < 0) {
      this.misses++;
      this.flashText('Missed: click right on the rope', '#b4451f');
      this.phase = 'lift';
      this.emit('action', { type: 'miss', x: g.x, y: g.y });
      return;
    }
    this.grasps++;
    this.emit('action', { type: 'grasp', node: best, x: s.x[best], y: s.y[best] });
    s.grab = best;
    g.open = false;
    g.x = s.x[best]; g.y = s.y[best];
    this.carryOff = [g.x - this.ptr[0], g.y - this.ptr[1]];
    this.phase = 'carry';
  };

  // The simulation let go of the rope (see Sim.keepKnot).
  Game.prototype.slip = function () {
    this.emit('action', { type: 'slip', x: this.g.x, y: this.g.y });
    this.flashText('Too tight: the rope slipped out of the gripper', '#b4451f');
    this.g.open = true;
    this.phase = 'lift';
  };

  Game.prototype.release = function () {
    this.emit('action', { type: 'release', node: this.sim.grab, x: this.g.x, y: this.g.y });
    this.sim.grab = -1;
    this.g.open = true;
    this.phase = 'lift';
  };

  Game.prototype.finish = function (ok) {
    this.running = false;
    this.finished = true;
    if (this.sim.grab >= 0) this.release();
    this.emit('finish', { ok: ok, t: this.t, grasps: this.grasps, misses: this.misses, round: this.round, timeLimit: this.timeLimit });
    if (ok && this.tut) {
      // Practice knot untied: on to a real round.
      var self = this, tut = this.tut;
      setTimeout(function () { if (self.tut === tut) self.endTutorial(); }, 1600);
    } else if (ok) {
      var key = this.knot + (this.assist ? ' cbf' : '');
      this.runs.push({ t: this.t, key: key });
      var mine = this.runs.filter(function (r) { return r.key === key; });
      var best = Math.min.apply(null, mine.map(function (r) { return r.t; }));
      var count = function (k, word) { return k + ' ' + word + (k === 1 ? '' : word.slice(-1) === 's' ? 'es' : 's'); };
      this.status('Untied the ' + KNOTS[this.knot].name + ' in ' + this.t.toFixed(1) + ' s' + (this.assist ? ' with SA-CBF' : '') + ', ' +
        count(this.grasps, 'grasp') + ' and ' + count(this.misses, 'miss') + (mine.length > 1 ? ' (your best here: ' + best.toFixed(1) + ' s)' : '') +
        (this.lab ? '. Press Next knot for another one.' : '. Press New knot, or pick a harder knot.'));
    } else {
      this.status(this.lab ? 'Time is up. Press Next knot for another one.' : 'Time is up. Press New knot to try again.');
    }
    this.flashText(ok ? 'Untied!' : 'Time is up', ok ? '#2f7d4f' : '#b4451f');
  };

  // ---------- tutorial ----------

  // Shown, not told: on a practice overhand knot a hand presses on one end of
  // the rope, holds while the gripper comes down and closes, drags it out and
  // lets go. Then the player unties the practice knot, and a real round
  // starts. Practice rounds carry round.tutorial, so the Knot Lab does not
  // record them, and they can be played before signing in.
  var SEEN = 'dloTutorialSeen';

  Game.prototype.tutorialSeen = function () {
    try { return !!window.localStorage.getItem(SEEN); } catch (e) { return true; }   // no storage: do not insist
  };

  Game.prototype.startTutorial = function () {
    if (this.tut) this.closeTutorial();
    var saved = { knot: this.knot, rad: this.rad, stiffness: this.stiffness };
    this.knot = '3_1'; this.rad = 8; this.stiffness = 0.3;
    this.newKnot({ tutorial: true });
    this.tut = { saved: saved };
    this.tutorialButton(true);
    this.status('');
    this.startDemo();
  };

  // Leave the tutorial; the caller deals the next round.
  Game.prototype.closeTutorial = function () {
    var saved = this.tut.saved;
    this.tut = null;
    this.demo = null;
    this.held = false;
    this.hand.setAttribute('opacity', 0);
    this.tutorialButton(false);
    this.knot = saved.knot; this.rad = saved.rad; this.stiffness = saved.stiffness;
    try { window.localStorage.setItem(SEEN, '1'); } catch (e) { /* private mode */ }
  };

  Game.prototype.endTutorial = function () { if (this.tut) this.newKnot(); };

  // "How to play" turns into "Skip tutorial" while it runs.
  Game.prototype.tutorialButton = function (on) {
    var b = this.q('[data-tutorial]');
    if (!b) return;
    b.lastChild.textContent = on ? 'Skip tutorial' : 'How to play';
    b.querySelector('i').className = 'fa-solid ' + (on ? 'fa-forward-step' : 'fa-circle-question');
  };

  // The demonstration on the end of the rope whose pull outward stays
  // farthest inside the table.
  Game.prototype.startDemo = function () {
    var s = this.sim, n = s.n, b = s.p.box, cx = 0, cy = 0, best = null, i;
    for (i = 0; i < n; i++) { cx += s.x[i] / n; cy += s.y[i] / n; }
    [3, n - 4].forEach(function (k) {
      var dx = s.x[k] - cx, dy = s.y[k] - cy, L = Math.hypot(dx, dy) || 1;
      var tx = s.x[k] + dx / L * 130, ty = s.y[k] + dy / L * 130;
      var room = Math.min(tx - b[0], b[2] - tx, ty - b[1], b[3] - ty);
      if (!best || room > best.room) best = { k: k, dx: tx - s.x[k], dy: ty - s.y[k], room: room };
    });
    var x0 = (cx + this.g.x) / 2, y0 = (cy + this.g.y) / 2;
    this.demo = { phase: 'move', t: 0, node: best.k, dx: best.dx, dy: best.dy, x0: x0, y0: y0, x: x0, y: y0, down: false, ripple: -1 };
    this.wake();
  };

  // One frame of the demonstration: glide to the rope, press and hold until
  // the gripper has it, drag it out, let go, and hand over.
  Game.prototype.demoStep = function (dt) {
    var d = this.demo, s = this.sim, ease = function (u) { return u * u * (3 - 2 * u); }, u;
    d.t += dt;
    if (d.ripple >= 0) d.ripple += dt;
    if (d.phase === 'move') {
      u = ease(Math.min(1, d.t / 0.9));
      d.x = d.x0 + (s.x[d.node] - d.x0) * u; d.y = d.y0 + (s.y[d.node] - d.y0) * u;
      if (d.t >= 1.2) {
        this.ptr = [s.x[d.node], s.y[d.node]]; this.hover = true;
        this.press();
        d.phase = 'hold'; d.t = 0; d.down = true; d.ripple = 0;
      }
    } else if (d.phase === 'hold') {
      d.x = this.ptr[0]; d.y = this.ptr[1];
      if (this.phase === 'carry' && d.t > 0.3) { d.phase = 'drag'; d.t = 0; d.from = this.ptr.slice(); }
      else if (d.t > 4) { this.held = false; d.down = false; d.phase = 'move'; d.t = 0; d.x0 = d.x; d.y0 = d.y; }   // missed: again
    } else if (d.phase === 'drag') {
      u = ease(Math.min(1, d.t / 1.5));
      this.ptr = [d.from[0] + d.dx * u, d.from[1] + d.dy * u];
      d.x = this.ptr[0]; d.y = this.ptr[1];
      if (d.t >= 1.7) { this.held = false; d.down = false; d.ripple = 0; d.phase = 'let'; d.t = 0; }
    } else if (d.phase === 'let' && this.phase === 'idle' && d.t > 0.8) {
      this.demo = null;   // the player's turn
      this.hover = false;
      this.hand.setAttribute('opacity', 0);
    }
  };

  // Run frames while the stage is on screen and something is happening.
  Game.prototype.wake = function () {
    if (this.raf || !this.visible) return;
    var self = this, last = performance.now();
    function frame(now) {
      var dt = Math.min((now - last) / 1000, 1 / 30);
      last = now;
      self.update(dt);
      self.render();
      var busy = self.running || self.phase !== 'idle' || self.flashT > 0 || self.demo || self.sim.moved > 0.002;
      self.raf = self.visible && busy ? requestAnimationFrame(frame) : null;
    }
    this.raf = requestAnimationFrame(frame);
  };

  Game.prototype.render = function () {
    var s = this.sim, g = this.g, n = s.n;
    var pts = new Array(n);
    for (var i = 0; i < n; i++) pts[i] = [s.x[i], s.y[i]];
    // Painter's order by height, so the upper strand covers the lower one.
    var order = [];
    for (var k = 0; k < n - 1; k++) order.push(k);
    order.sort(function (a, b) { return (s.z[a] + s.z[a + 1]) - (s.z[b] + s.z[b + 1]) || a - b; });
    this.rope.render(pts, order);

    // SA-CBF: the funnels around the rope, and the rope point being approached.
    var showFunnels = this.assist && !this.hideFunnels;
    if (showFunnels) this.funnels.setAttribute('d', 'M' + pts.map(function (q) { return f1(q[0]) + ' ' + f1(q[1]); }).join('L'));
    this.funnels.setAttribute('opacity', showFunnels ? 1 : 0);
    var guided = this.target && this.target.node >= 0 && (this.phase === 'travel' || this.phase === 'descend');
    if (guided) {
      var T = this.targetPos();
      this.intentRing.setAttribute('cx', f1(T[0])); this.intentRing.setAttribute('cy', f1(T[1]));
    }
    this.intentRing.setAttribute('opacity', guided ? 1 : 0);

    // Tutorial hand, the same size on screen whatever the table's scale.
    var d = this.demo;
    if (d) {
      var b = s.p.box, upp = clamp((b[2] - b[0]) / (this.svg.getBoundingClientRect().width || W), 1, 2.4), r = 11 * upp;
      this.hand.setAttribute('opacity', 1);
      this.handTip.setAttribute('cx', f1(d.x)); this.handTip.setAttribute('cy', f1(d.y)); this.handTip.setAttribute('r', f1(r));
      this.handTip.setAttribute('fill', d.down ? C.orange : '#fff');
      this.handTip.setAttribute('fill-opacity', d.down ? 0.6 : 0.9);   // pressed: the jaws show through
      var rp = d.ripple >= 0 && d.ripple < 0.6 ? d.ripple / 0.6 : -1;
      this.handRipple.setAttribute('opacity', rp >= 0 ? f1(1 - rp) : 0);
      if (rp >= 0) { this.handRipple.setAttribute('cx', f1(d.x)); this.handRipple.setAttribute('cy', f1(d.y)); this.handRipple.setAttribute('r', f1(r * (1 + 1.6 * rp))); }
    }

    var p = this.hover && !d ? this.ptr : null;   // the tutorial hand stands in for the pointer
    this.cursor.setAttribute('opacity', p ? 0.8 : 0);
    if (p) { this.cursor.setAttribute('cx', f1(p[0])); this.cursor.setAttribute('cy', f1(p[1])); }

    // Gripper seen from above: larger when higher.
    var scale = 1 + g.z / 150, gap = (g.open ? s.p.gapOpen : s.p.gapShut) - 3.5;
    this.bar.setAttribute('y', f1(-gap - 7)); this.bar.setAttribute('height', f1(2 * gap + 14));
    this.f1.setAttribute('y', f1(-gap - 7)); this.f2.setAttribute('y', f1(gap));
    this.grip.setAttribute('transform', 'translate(' + f1(g.x) + ' ' + f1(g.y) + ') rotate(' + f1(g.yaw) + ') scale(' + f1(scale * 100) / 100 + ')');

    var out = this.out;
    out.time.textContent = this.t.toFixed(1) + ' s';
    out.knot.textContent = this.tied ? 'tied' : 'untied';
    out.knot.className = this.tied ? '' : 'is-ok';
    out.cross.textContent = this.nCross;
    out.grasps.textContent = this.grasps;
    out.misses.textContent = this.misses;
    this.flash.setAttribute('opacity', this.flashT ? Math.min(1, this.flashT * 2) : 0);
  };

  function init() {
    if (!window.RopeDraw) return;
    Array.prototype.forEach.call(document.querySelectorAll('[data-dlo-game]'), function (root) { root.game = new Game(root); });
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }
  if (typeof module !== 'undefined') module.exports = { Sim: Sim, crossings: crossings, knotPoints: knotPoints, knotDeterminant: knotDeterminant, ropeParams: ropeParams, verifiedRope: verifiedRope, KNOTS: KNOTS };
})();
