/* Knot-untangling game for the AssistDLO project page.
 *
 * A knot lies on a table, seen from above: the overhand knot, or another
 * prime knot from the Rolfsen table drawn as a closed braid and cut open on
 * its outer strand in the same way (an open rope only mimics the knot's
 * diagram; it is never knotted in the mathematical sense). Click a strand to
 * grasp it, drag to carry it, and release to put it down. The knot counts as
 * untied when the rope no longer crosses itself, as in the paper's study.
 * The rope is a small 3D simulation (gravity, table friction, bending
 * stiffness and self-collision), so strands cannot pass through each other;
 * its thickness and stiffness can be changed. The gripper is teleoperated
 * directly, or with SA-CBF assistance in simplified form: a barrier keeps the
 * gripper above the rope except in funnels around rope points, so a click
 * near the rope is guided onto the nearest point (the upper strand at a
 * crossing) and a click away from it does not descend. The hint tries
 * candidate moves on copies of the rope and shows the most useful one.
 * Rendering uses rope.js (window.RopeDraw).
 */
(function () {
  'use strict';

  var W = 640, H = 400;
  var GRAV = 1800, SUB = 3, ITER = 6;    // px/s², substeps per frame, solver iterations
  var W_TABLE = 0.2;                     // rope resting on the table is harder to move (friction)
  var FINGER = 4;                        // finger pad radius (px)
  var Z_HOVER = 80;                      // gripper height while travelling (px above the table)
  var V_XY = 330, V_Z = 240;             // gripper speed limits (px/s)
  var MAX_STIFF = 0.06;                  // bending stiffness at the top of the slider

  // Everything that scales with the rope: rad is the radius in px, stiff the
  // bending stiffness (0: limp, MAX_STIFF: springy).
  // box is the table [x0, y0, x1, y1]: the 640 x 400 table, extended on wide
  // or tall screens (knots are always dealt in the middle 640 x 400).
  function ropeParams(rad, stiff, box) {
    var cell = 2 * rad + 2;
    box = box || [0, 0, W, H];
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
        if (it % 2 === 1) this.solveContacts();
        if (grip) this.solveGripper(grip);
        this.solveBounds();
      }
    }
  };

  // Inverse mass for the solver: zero when held, small when resting on the table.
  Sim.prototype.weight = function (i) {
    return i === this.grab ? 0 : this.z[i] < 0.6 ? W_TABLE : 1;
  };

  // Fixed length between neighbours; between nodes i and i+2 a hard limit on
  // how sharply the rope can kink, plus a soft pull toward straight (the
  // bending stiffness).
  Sim.prototype.solveLinks = function () {
    var x = this.x, y = this.y, z = this.z, n = this.n, p = this.p, straight = 2 * p.seg;
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
        else if (p.stiff > 0) k = p.stiff * (L - straight) / L / (wi + wj);
        else continue;
        x[i] += dx * k * wi; y[i] += dy * k * wi; z[i] += dz * k * wi;
        x[j] -= dx * k * wj; y[j] -= dy * k * wj; z[j] -= dz * k * wj;
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

  // Put the rope on another table: a copy of the parameters, so the moves
  // being tried out for a hint keep theirs.
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
  Sim.prototype.knotted = function () { return knotDeterminant(this.x, this.y, this.z, this.n) !== 1; };

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

  // ---------- move planning (for the hint) ----------

  // An independent copy of the rope, for trying out moves.
  Sim.prototype.clone = function () {
    var c = Object.create(Sim.prototype);
    c.n = this.n; c.p = this.p; c.grab = -1; c.moved = 0;
    ['x', 'y', 'z', 'px', 'py', 'pz'].forEach(function (k) { c[k] = this[k].slice(); }, this);
    c.head = new Int32Array(this.head.length);
    c.next = new Int32Array(this.n);
    return c;
  };

  // Carry node k to (tx, ty) the way the gripper does (lift, move at the
  // speed limit, put down), then let the rope settle.
  Sim.prototype.tryMove = function (k, tx, ty) {
    var dt = 1 / 30, step = V_XY * dt, f;   // coarser than the game's frames: planning only
    var g = { x: this.x[k], y: this.y[k], z: this.z[k], yaw: 0, open: false, hold: k };
    this.grab = k;
    for (f = 0; f < 6; f++) { g.z = Math.min(this.p.zCarry, g.z + V_Z * dt); this.step(dt, g); }
    for (f = 0; f < 120; f++) {
      var dx = tx - g.x, dy = ty - g.y, d = Math.hypot(dx, dy);
      if (d <= step) { g.x = tx; g.y = ty; this.step(dt, g); break; }
      g.x += dx / d * step; g.y += dy / d * step;
      this.step(dt, g);
    }
    this.grab = -1;
    for (f = 0; f < 15; f++) this.step(dt, null);
  };

  // Candidate moves {k: node to grab, x, y: where to put it}: free each end
  // through its nearest crossing (pull the rope just past the crossing away,
  // or carry the end back across the strand it lies on), pull the end
  // straight out, and pull small loops open.
  Sim.prototype.candidateMoves = function () {
    var x = this.x, y = this.y, n = this.n, p = this.p, out = [], cr = [];
    var box = p.box, inside = function (q) { return [clamp(q[0], box[0] + 20, box[2] - 20), clamp(q[1], box[1] + 20, box[3] - 20)]; };
    var add = function (k, q) { q = inside(q); out.push({ k: k, x: q[0], y: q[1] }); };
    for (var i = 0; i < n - 1; i++) {
      for (var j = i + 2; j < n - 1; j++) {
        var h = hit(x[i], y[i], x[i + 1], y[i + 1], x[j], y[j], x[j + 1], y[j + 1]);
        if (h) cr.push({ a: i + h[0], b: j + h[1], x: x[i] + (x[i + 1] - x[i]) * h[0], y: y[i] + (y[i + 1] - y[i]) * h[0] });
      }
    }
    if (!cr.length) return out;
    var cx = 0, cy = 0;
    for (var m = 0; m < n; m++) { cx += x[m]; cy += y[m]; }
    cx /= n; cy /= n;
    [0, n - 1].forEach(function (e) {
      var c = cr.reduce(function (best, q) { return e === 0 ? (q.a < best.a ? q : best) : (q.b > best.b ? q : best); });
      var pos = e === 0 ? c.a : c.b, tail = (e === 0 ? pos : n - 1 - pos) * p.seg;
      var I = e === 0 ? Math.min(n - 1, Math.ceil(pos) + 4) : Math.max(0, Math.floor(pos) - 4);
      var dx = x[I] - c.x, dy = y[I] - c.y, L = Math.hypot(dx, dy) || 1;
      [tail + 30, tail + 90].forEach(function (d) {
        [0, 0.5, -0.5].forEach(function (rot) {
          var ux = (dx * Math.cos(rot) - dy * Math.sin(rot)) / L, uy = (dx * Math.sin(rot) + dy * Math.cos(rot)) / L;
          add(I, [x[I] + ux * d, y[I] + uy * d]);
        });
      });
      // Carry the end back across the other strand at this crossing: mirror
      // it over that strand's direction.
      var other = e === 0 ? c.b : c.a, o = Math.round(other), a = Math.max(0, o - 2), b = Math.min(n - 1, o + 2);
      var tx = x[b] - x[a], ty = y[b] - y[a], tl = Math.hypot(tx, ty) || 1, ux2 = tx / tl, uy2 = ty / tl;
      var vx = x[e] - c.x, vy = y[e] - c.y, along = vx * ux2 + vy * uy2;
      add(e, [c.x + (2 * along * ux2 - vx) * 1.15, c.y + (2 * along * uy2 - vy) * 1.15]);
      var ex = x[e] - cx, ey = y[e] - cy, el = Math.hypot(ex, ey) || 1;
      add(e, [x[e] + ex / el * 120, y[e] + ey / el * 120]);
    });
    cr.slice().sort(function (q, r) { return (q.b - q.a) - (r.b - r.a); }).slice(0, 6).forEach(function (c) {
      var k = Math.round((c.a + c.b) / 2), dx = x[k] - c.x, dy = y[k] - c.y, L = Math.hypot(dx, dy) || 1;
      [120, 220, 320].forEach(function (d) { add(k, [x[k] + dx / L * d, y[k] + dy / L * d]); });
    });
    // Lift the upper strand off a crossing, sideways to the strand below.
    var z = this.z;
    cr.slice(0, 8).forEach(function (c) {
      var ia = Math.round(c.a), ib = Math.round(c.b), top = z[ia] > z[ib] ? ia : ib, low = top === ia ? ib : ia;
      var a = Math.max(0, low - 2), b = Math.min(n - 1, low + 2), tx = x[b] - x[a], ty = y[b] - y[a], tl = Math.hypot(tx, ty) || 1;
      [1, -1].forEach(function (sg) { add(top, [x[top] + sg * tx / tl * 70, y[top] + sg * ty / tl * 70]); });
    });
    return out;
  };

  // How good a rope state is (lower is better): fewer crossings, no knot
  // left, and long free tails (crossings far from the ends).
  Sim.prototype.score = function () {
    var cr = crossings(this.x, this.y, this.n);
    if (!cr.length) return 0;
    var first = Infinity, last = -Infinity;
    cr.forEach(function (c) { first = Math.min(first, c[0]); last = Math.max(last, c[1]); });
    return 10 * cr.length + (this.knotted() ? 5 : 0) - 3 * (first + (this.n - 1 - last)) / this.n;
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
    var pts = knotPoints(R, id, p, mirror, flip, rot), r = p.rad;
    for (var i = 0; i < pts.length; i++) {
      if (pts[i][0] < r + 2 || pts[i][0] > W - r - 2 || pts[i][1] < r + 2 || pts[i][1] > H - r - 2) return null;
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
    // The hint goes above the gripper, so its target is never hidden.
    this.hintG = el('g', { 'pointer-events': 'none' }, svg);
    var defs = el('defs', {}, svg);
    var marker = el('marker', { id: 'dlo-hint-head', viewBox: '0 0 10 10', refX: 6, refY: 5, markerWidth: 5, markerHeight: 5, orient: 'auto' }, defs);
    el('path', { d: 'M0 0L10 5L0 10z', fill: C.orange }, marker);
    this.hintArrow = el('line', { stroke: C.orange, 'stroke-width': 3, 'stroke-dasharray': '7 5', 'marker-end': 'url(#dlo-hint-head)' }, this.hintG);
    this.hintRing = el('circle', { r: 14, fill: 'none', stroke: C.orange, 'stroke-width': 3 }, this.hintG);
    this.hintEnd = el('circle', { r: 12, fill: 'none', stroke: C.orange, 'stroke-width': 2.5, 'stroke-dasharray': '4 3' }, this.hintG);
    this.flash = el('text', { x: 320, y: 384, 'text-anchor': 'middle', 'font-family': 'Roboto, sans-serif', 'font-size': 15, 'font-weight': 700, fill: C.navy }, svg);
    // Tutorial panel, just above the table.
    var coach = this.coach = document.createElement('div');
    coach.className = 'game-coach';
    coach.hidden = true;
    coach.innerHTML = '<p class="game-coach-text" aria-live="polite"></p><div class="game-coach-actions">' +
      '<button type="button" class="btn btn--sm" data-coach-skip>Skip tutorial</button>' +
      '<button type="button" class="btn btn--sm btn--solid" data-coach-next>Next</button></div>';
    this.stage.parentNode.insertBefore(coach, this.stage);
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
    stage.addEventListener('pointermove', function (ev) { self.ptr = local(ev); self.hover = true; self.wake(); });
    stage.addEventListener('pointerleave', function () { if (!self.held) { self.hover = false; self.wake(); } });
    stage.addEventListener('pointerdown', function (ev) {
      if (ev.button > 0) return;
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
        if (self.held) self.emit('action', { type: 'up', x: self.ptr[0], y: self.ptr[1] });
        self.held = false;
        self.wake();
      });
    });
    // Keyboard: arrows move the hand, Space or Enter holds the grasp.
    stage.addEventListener('keydown', function (ev) {
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
      if (ev.key !== ' ' && ev.key !== 'Enter') return;
      if (self.held) self.emit('action', { type: 'up', x: self.ptr[0], y: self.ptr[1] });
      self.held = false;
      self.wake();
    });

    this.root.addEventListener('click', function (ev) {
      var b = ev.target.closest('button');
      if (!b) return;
      if (b.hasAttribute('data-new')) self.newKnot();
      if (b.hasAttribute('data-hint')) self.showHint();
      if (b.hasAttribute('data-full')) self.toggleFull(!self.full);
      if (b.hasAttribute('data-tutorial')) self.startTutorial();
      if (b.hasAttribute('data-coach-next')) self.tutNext();
      if (b.hasAttribute('data-coach-skip')) self.endTutorial();
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

    this.root.addEventListener('knot:action', function (ev) { if (self.tut) self.tutAction(ev.detail); });

    // Leaving the browser's full screen (Esc, back gesture) leaves ours too.
    ['fullscreenchange', 'webkitfullscreenchange'].forEach(function (t) {
      document.addEventListener(t, function () {
        if (self.fsApi && !(document.fullscreenElement || document.webkitFullscreenElement)) { self.fsApi = false; self.toggleFull(false); }
        self.fitTable();
      });
    });
    document.addEventListener('keydown', function (ev) { if (ev.key === 'Escape' && self.full) self.toggleFull(false); });
    if ('ResizeObserver' in window) {
      var ro = new ResizeObserver(function () { self.coachSpace(); self.fitTable(); });
      ro.observe(stage);
      ro.observe(this.coach);
    }
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

  Game.prototype.stiffValue = function () { return MAX_STIFF * this.stiffness * this.stiffness; };

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
    // Shuffle (in the Knot Lab) and build until a rope passes every check.
    while (!made && tries < 25) {
      if (this.lab && !practice && tries % 5 === 0) this.shuffle();
      tries++;
      var p = ropeParams(this.rad, this.stiffValue());
      var mirror = !practice && Math.random() < 0.5, flip = !practice && Math.random() < 0.5;
      var rot = this.lab && !practice ? Math.random() * 2 * Math.PI : 0;
      made = verifiedRope(R, this.knot, p, mirror, flip, rot);
    }
    if (!made) {   // never seen in testing; fall back to the plain overhand knot
      this.knot = '3_1';
      p = ropeParams(this.rad, this.stiffValue());
      mirror = flip = false; rot = 0;
      made = { sim: new Sim(knotPoints(R, '3_1', p, false, false, 0), p, false), crossings: 3, det: 3, minSep: 1 };
    }
    this.sim = made.sim;
    this.round = {
      knot: this.knot, name: KNOTS[this.knot].name, rad: this.rad, stiffness: this.stiffness, stiff: p.stiff,
      mirror: mirror, flip: flip, rot: rot, crossings: made.crossings, det: made.det, minSep: made.minSep, tries: tries
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
    this.hints = 0;
    this.flashT = 0;
    this.solvedFor = 0;
    this.hinted = null;
    this.guide = null;
    this.planning = null;
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
    this.coachSpace();
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
    else a = clamp(w / Math.max(240, window.innerHeight - 260), W / H, 2.4);
    // In steps of 20, so small changes (a phone's address bar) leave it alone.
    var ex = a >= W / H ? 20 * Math.round((H * a - W) / 40) : 0, ey = a < W / H ? 20 * Math.round((W / a - H) / 40) : 0;
    return [-ex, -ey, W + ex, H + ey];
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
    if (fresh) { g.x = b[2] - 34; g.y = b[3] - 34; }   // parked in the corner
    else if (s.grab < 0) { g.x = clamp(g.x, b[0] + 34, b[2] - 34); g.y = clamp(g.y, b[1] + 34, b[3] - 34); }   // carrying, it is on the rope
    if (this.ptr) this.ptr = [clamp(this.ptr[0], b[0], b[2]), clamp(this.ptr[1], b[1], b[3])];
    if (!fresh) this.emit('action', { type: 'table', box: b.slice() });
    this.wake();
  };

  // The hint tries candidate moves on copies of the rope, a few per frame so
  // the page stays responsive, and shows the best one as an arrow.
  Game.prototype.showHint = function () {
    var self = this, s = this.sim;
    if (this.phase !== 'idle' || !this.nCross) return;
    var moves = s.candidateMoves(), now = s.score(), best = null, i = 0, token = {};
    this.planning = token;
    this.hinted = null;
    this.status('Hint: trying out moves…');
    (function chunk() {
      if (self.planning !== token || self.sim !== s) return;   // the rope changed meanwhile
      for (var end = Math.min(moves.length, i + 3); i < end; i++) {
        var c = s.clone();
        c.tryMove(moves[i].k, moves[i].x, moves[i].y);
        var sc = c.score() + Math.hypot(moves[i].x - s.x[moves[i].k], moves[i].y - s.y[moves[i].k]) / 1000;
        if (!best || sc < best.score) best = { k: moves[i].k, x: moves[i].x, y: moves[i].y, score: sc, crossings: c.crossingCount() };
      }
      if (i < moves.length) { setTimeout(chunk, 0); return; }
      self.planning = null;
      self.hinted = best;
      self.hints++;
      self.emit('action', { type: 'hint', node: best.k, x: best.x, y: best.y });
      var gain = self.nCross - best.crossings;
      self.status(gain > 0
        ? 'Hint: grab the rope at the dashed ring and drag it to the orange ring. That should remove ' + gain + (gain === 1 ? ' crossing.' : ' crossings.')
        : best.score < now
          ? 'Hint: grab the rope at the dashed ring and drag it to the orange ring. That loosens the knot without adding crossings.'
          : 'Hint: no single move helps much here. Try the dashed ring to the orange ring, or spread the loops apart.');
      if (self.tut) self.tutHinted();
      self.wake();
    })();
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
    this.hinted = null;
    this.planning = null;
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

  Game.prototype.release = function () {
    this.emit('action', { type: 'release', node: this.sim.grab, x: this.g.x, y: this.g.y });
    this.sim.grab = -1;
    this.g.open = true;
    this.phase = 'lift';
  };

  Game.prototype.finish = function (ok) {
    this.running = false;
    this.finished = true;
    this.hinted = null;
    if (this.sim.grab >= 0) this.release();
    this.emit('finish', { ok: ok, t: this.t, grasps: this.grasps, misses: this.misses, hints: this.hints, round: this.round, timeLimit: this.timeLimit });
    if (ok && this.tut) {
      this.tut.step = 'done';
      this.guide = null;
      this.coachSay(COACH.done + (this.lab ? ' Using it halves the points for that knot.' : '') +
        (this.locked ? ' Sign in above the game to play for the leaderboard.' : ''), 'Start playing');
      this.status('Practice knot untied in ' + this.t.toFixed(1) + ' s.');
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
      this.status(this.lab ? 'Time is up. Press Next knot for another one.' : 'Time is up. Press New knot to try again, or use the hint.');
    }
    this.flashText(ok ? 'Untied!' : 'Time is up', ok ? '#2f7d4f' : '#b4451f');
  };

  // ---------- tutorial ----------

  // A practice round on an easy overhand knot. The panel above the table says
  // what to do, and the hint planner's moves are shown one at a time (press
  // and hold on the dashed ring, drag to the orange ring, let go) until the
  // knot is untied. Practice rounds carry round.tutorial, so the Knot Lab
  // does not record them, and they can be played before signing in.
  var SEEN = 'dloTutorialSeen';
  var COACH = {
    goal: 'Untie the knot: it counts as untied when the rope no longer crosses itself. Crossings, above the table, shows how many are left.',
    finding: 'Finding a good move for you…',
    grasp: 'Press and hold on the rope inside the dashed ring. The gripper moves there, comes down and closes on the rope.',
    miss: 'Missed. Press right on the rope inside the dashed ring, and keep holding.',
    drag: 'Keep holding and drag to the orange ring, then let go to put the rope down.',
    again: 'Well done! Next move: press and hold on the dashed ring, drag to the orange ring, let go.',
    done: 'Untied! That is the game. Every round deals a new knot with its own thickness and stiffness. Stuck? Hint shows a move.'
  };

  Game.prototype.tutorialSeen = function () {
    try { return !!window.localStorage.getItem(SEEN); } catch (e) { return true; }   // no storage: do not insist
  };

  Game.prototype.coachSay = function (text, next) {
    var c = this.coach, b = c.querySelector('[data-coach-next]');
    c.hidden = false;
    this.root.classList.add('is-tutorial');
    c.querySelector('.game-coach-text').textContent = text;
    b.hidden = !next;
    if (next) b.textContent = next;
    this.coachSpace();
  };

  // In full screen the panel is a bar along the top and the table starts below it.
  Game.prototype.coachSpace = function () {
    var h = this.full && !this.coach.hidden ? this.coach.offsetHeight : 0;
    this.root.style.setProperty('--coach-h', h + 'px');
  };

  Game.prototype.startTutorial = function () {
    if (this.tut) this.closeTutorial();
    var saved = { knot: this.knot, rad: this.rad, stiffness: this.stiffness };
    this.knot = '3_1'; this.rad = 8; this.stiffness = 0.3;
    this.newKnot({ tutorial: true });
    this.tut = { step: 'goal', moves: 0, saved: saved };
    this.coachSay(COACH.goal, 'Next');
    this.status('Tutorial: a practice knot.');
  };

  // Leave the tutorial; the caller deals the next round.
  Game.prototype.closeTutorial = function () {
    var saved = this.tut.saved;
    this.tut = null;
    this.guide = null;
    this.coach.hidden = true;
    this.root.classList.remove('is-tutorial');
    this.coachSpace();
    this.knot = saved.knot; this.rad = saved.rad; this.stiffness = saved.stiffness;
    try { window.localStorage.setItem(SEEN, '1'); } catch (e) { /* private mode */ }
  };

  Game.prototype.endTutorial = function () { if (this.tut) this.newKnot(); };

  Game.prototype.tutNext = function () {
    if (!this.tut) return;
    if (this.tut.step === 'done') this.endTutorial(); else this.tutMove();
  };

  // Plan the next move once the gripper is up and the rope has settled.
  Game.prototype.tutMove = function () {
    var self = this, t = this.tut, waited = 0;
    t.step = 'finding';
    this.guide = null;
    this.coachSay(COACH.finding);
    (function wait() {
      if (self.tut !== t || self.finished) return;
      if (self.phase !== 'idle' || (self.sim.moved > 0.05 && waited < 25)) { waited++; setTimeout(wait, 120); return; }
      self.showHint();
    })();
  };

  Game.prototype.tutHinted = function () {
    var t = this.tut;
    this.guide = this.hinted;
    t.step = 'grasp';
    this.coachSay(t.moves ? COACH.again : COACH.grasp);
  };

  Game.prototype.tutAction = function (d) {
    var t = this.tut;
    if (d.type === 'miss' && t.step === 'grasp') this.coachSay(COACH.miss);
    else if (d.type === 'grasp' && (t.step === 'grasp' || t.step === 'finding')) { t.step = 'drag'; this.coachSay(COACH.drag); }
    else if (d.type === 'release' && t.step === 'drag') { t.moves++; this.tutMove(); }
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
      var busy = self.running || self.phase !== 'idle' || self.flashT > 0 || self.hinted || self.planning || self.sim.moved > 0.002;
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
    this.funnels.setAttribute('d', 'M' + pts.map(function (q) { return f1(q[0]) + ' ' + f1(q[1]); }).join('L'));
    this.funnels.setAttribute('opacity', this.assist ? 1 : 0);
    var guided = this.target && this.target.node >= 0 && (this.phase === 'travel' || this.phase === 'descend');
    if (guided) {
      var T = this.targetPos();
      this.intentRing.setAttribute('cx', f1(T[0])); this.intentRing.setAttribute('cy', f1(T[1]));
    }
    this.intentRing.setAttribute('opacity', guided ? 1 : 0);

    // Hint: grab at the dashed ring, drop at the solid ring. In the tutorial
    // the move stays on screen while it is carried out.
    var h = this.hinted || this.guide;
    if (h) {
      var gx = s.x[h.k], gy = s.y[h.k], dx = h.x - gx, dy = h.y - gy, L = Math.hypot(dx, dy) || 1;
      this.hintEnd.setAttribute('cx', f1(gx)); this.hintEnd.setAttribute('cy', f1(gy));
      this.hintRing.setAttribute('cx', f1(h.x)); this.hintRing.setAttribute('cy', f1(h.y));
      this.hintRing.setAttribute('r', f1(14 + 2.5 * Math.sin(performance.now() / 160)));
      this.hintArrow.setAttribute('x1', f1(gx + dx / L * 14)); this.hintArrow.setAttribute('y1', f1(gy + dy / L * 14));
      this.hintArrow.setAttribute('x2', f1(h.x - dx / L * 18)); this.hintArrow.setAttribute('y2', f1(h.y - dy / L * 18));
    }
    this.hintG.setAttribute('opacity', h ? 1 : 0);

    var p = this.hover ? this.ptr : null;
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
