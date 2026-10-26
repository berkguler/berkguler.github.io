/* Knot-untangling game for the AssistDLO project page.
 *
 * A loose overhand knot lies on a table, seen from above. Click a strand to
 * grasp it, drag to carry it, and release to put it down. The knot counts as
 * untied once lifting both ends and pulling them apart would leave a straight
 * rope (see knotDeterminant); the paper's study asked for no crossings at
 * all, which the game reports alongside. The rope is a small 3D simulation
 * (gravity, table friction and self-collision), so strands cannot pass
 * through each other.
 *
 * The gripper lands a few pixels away from where you click (a slowly
 * drifting offset that stands in for misjudging depth through a camera).
 * The four modes follow the study conditions in simplified form:
 *   pt   pure teleoperation: no help
 *   va   visual assistance: height-coded rope estimate and the true landing point
 *   lb   linear blending: the gripper heads straight for the rope point nearest
 *        to where it would land, descending on the way
 *   cbf  barrier funnels: the gripper stays above the rope and descends only
 *        onto a rope point, with the jaws turned across the rope
 * Rendering uses rope.js (window.RopeDraw).
 */
(function () {
  'use strict';

  var W = 640, H = 400;
  var RAD = 7, SEG = 7;                  // rope radius and node spacing (px)
  var GRAV = 1800, SUB = 3, ITER = 6;    // px/s², substeps per frame, solver iterations
  var BEND = 1.45 * SEG;                 // nodes i and i+2 stay at least this far apart
  var W_TABLE = 0.2;                     // rope resting on the table is harder to move (friction)
  var CELL = 16, COLS = Math.ceil(W / CELL), ROWS = Math.ceil(H / CELL);
  var FINGER = 4;                        // finger pad radius (px)
  var GAP_OPEN = 11.5, GAP_SHUT = 7;     // finger centres from the jaw centre
  var Z_HOVER = 80, Z_CARRY = 34;        // gripper heights (px above the table)
  var V_XY = 330, V_Z = 240;             // gripper speed limits (px/s)
  var OFFSET = 10;                       // landing error (px)
  var LB_RADIUS = 40, CBF_RADIUS = 28;   // how far the assistance looks for a rope point
  var TIME_LIMIT = 120;

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

  function Sim(pts, overFirst) {
    var n = this.n = pts.length;
    var A = function () { return new Float64Array(n); };
    this.x = A(); this.y = A(); this.z = A(); this.px = A(); this.py = A(); this.pz = A();
    for (var i = 0; i < n; i++) { this.x[i] = pts[i][0]; this.y[i] = pts[i][1]; }
    // Alternating over/under along the rope: lift the upper strand at each crossing.
    var cr = crossings(this.x, this.y, n), events = [];
    cr.forEach(function (c, k) { events.push({ at: c[0], k: k }, { at: c[1], k: k }); });
    events.sort(function (a, b) { return a.at - b.at; });
    var over = {};
    events.forEach(function (e, idx) {
      if (over[e.k] !== undefined) return;
      var first = (idx % 2 === 0) === overFirst;
      over[e.k] = first ? e.at : (e.at === cr[e.k][0] ? cr[e.k][1] : cr[e.k][0]);
    });
    for (var k in over) {
      for (var d = -4; d <= 5; d++) {
        var m = over[k] + d;
        if (m >= 0 && m < n) this.z[m] = Math.max(this.z[m], 2 * RAD * Math.exp(-(d - 0.5) * (d - 0.5) / 5));
      }
    }
    this.px.set(this.x); this.py.set(this.y); this.pz.set(this.z);
    this.grab = -1;
    this.head = new Int32Array(COLS * ROWS);
    this.next = new Int32Array(n);
    for (var s = 0; s < 40; s++) this.step(1 / 60, null);
  }

  Sim.prototype.buildGrid = function () {
    var head = this.head, next = this.next;
    head.fill(-1);
    for (var i = 0; i < this.n; i++) {
      var c = clamp(Math.floor(this.x[i] / CELL), 0, COLS - 1) + COLS * clamp(Math.floor(this.y[i] / CELL), 0, ROWS - 1);
      next[i] = head[c];
      head[c] = i;
    }
  };

  // One frame. grip: {x, y, z, open} or null. Returns how far the gripper
  // body pushed rope that it was not holding.
  Sim.prototype.step = function (dt, grip) {
    var x = this.x, y = this.y, z = this.z, px = this.px, py = this.py, pz = this.pz, n = this.n;
    var h = dt / SUB, g = GRAV * h * h, pushed = 0;
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
        if (grip) pushed += this.solveGripper(grip);
        this.solveBounds();
      }
    }
    return pushed;
  };

  // Inverse mass for the solver: zero when held, small when resting on the table.
  Sim.prototype.weight = function (i) {
    return i === this.grab ? 0 : this.z[i] < 0.6 ? W_TABLE : 1;
  };

  Sim.prototype.solveLinks = function () {
    var x = this.x, y = this.y, z = this.z, n = this.n;
    for (var pass = 0; pass < 2; pass++) {
      var gap = pass === 0 ? 1 : 2;
      for (var i = 0; i + gap < n; i++) {
        var j = i + gap;
        var dx = x[j] - x[i], dy = y[j] - y[i], dz = z[j] - z[i];
        var L = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-6;
        var rest = gap === 1 ? SEG : BEND;
        if (gap === 2 && L >= rest) continue;
        var wi = this.weight(i), wj = this.weight(j);
        if (!wi && !wj) continue;
        var k = (L - rest) / L / (wi + wj);
        x[i] += dx * k * wi; y[i] += dy * k * wi; z[i] += dz * k * wi;
        x[j] -= dx * k * wj; y[j] -= dy * k * wj; z[j] -= dz * k * wj;
      }
    }
  };

  // Rope against rope: spheres of radius RAD on every node; neighbours along
  // the rope are skipped. Nodes resting on the table push each other sideways.
  Sim.prototype.solveContacts = function () {
    var x = this.x, y = this.y, z = this.z, head = this.head, next = this.next;
    var D = 2 * RAD, D2 = D * D;
    for (var i = 0; i < this.n; i++) {
      var cx = clamp(Math.floor(x[i] / CELL), 0, COLS - 1), cy = clamp(Math.floor(y[i] / CELL), 0, ROWS - 1);
      for (var oy = -1; oy <= 1; oy++) {
        var ry = cy + oy;
        if (ry < 0 || ry >= ROWS) continue;
        for (var ox = -1; ox <= 1; ox++) {
          var rx = cx + ox;
          if (rx < 0 || rx >= COLS) continue;
          for (var j = head[rx + COLS * ry]; j >= 0; j = next[j]) {
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
  // The strand around grip.hold (held or just let go) is left alone, and
  // pushes near grip.keep (the strand being grasped) are not counted as
  // disturbance.
  Sim.prototype.solveGripper = function (grip) {
    var x = this.x, y = this.y, z = this.z, pushed = 0, D = FINGER + RAD;
    var a = grip.yaw * Math.PI / 180, ax = -Math.sin(a), ay = Math.cos(a);
    var gap = grip.open ? GAP_OPEN : GAP_SHUT;
    for (var f = -1; f <= 1; f += 2) {
      var fx = grip.x + f * gap * ax, fy = grip.y + f * gap * ay;
      for (var i = 0; i < this.n; i++) {
        if (grip.hold >= 0 && Math.abs(i - grip.hold) <= 8) continue;   // the strand hanging from the jaws
        if (z[i] + RAD <= grip.z - 6) continue;
        var dx = x[i] - fx, dy = y[i] - fy, d = Math.sqrt(dx * dx + dy * dy);
        if (d >= D) continue;
        if (d < 1e-6) { dx = -f * ax; dy = -f * ay; d = 1; }
        var k = (D - d) / d;
        x[i] += dx * k; y[i] += dy * k;
        if (grip.keep < 0 || Math.abs(i - grip.keep) > 4) pushed += D - d;
      }
    }
    return pushed;
  };

  Sim.prototype.solveBounds = function () {
    for (var i = 0; i < this.n; i++) {
      if (this.z[i] < 0) this.z[i] = 0;
      this.x[i] = clamp(this.x[i], RAD, W - RAD);
      this.y[i] = clamp(this.y[i], RAD, H - RAD);
    }
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

  // Loose overhand knot, sampled at the node spacing, with the tails cut to
  // TAIL nodes beyond the outermost crossings.
  var TAIL = 12;
  function knotPoints(R, mirror) {
    var dense = R.spline(R.overhand(320, 232, 42, 0, 0), 10), L = 0;
    for (var i = 1; i < dense.length; i++) L += Math.hypot(dense[i][0] - dense[i - 1][0], dense[i][1] - dense[i - 1][1]);
    var pts = R.resample(dense, Math.round(L / SEG) + 1);
    var xs = pts.map(function (p) { return p[0]; }), ys = pts.map(function (p) { return p[1]; });
    var cr = crossings(xs, ys, pts.length), lo = Infinity, hi = -Infinity;
    cr.forEach(function (c) { lo = Math.min(lo, c[0]); hi = Math.max(hi, c[1] + 1); });
    pts = pts.slice(Math.max(0, lo - TAIL), Math.min(pts.length, hi + TAIL + 1));
    return mirror ? pts.map(function (p) { return [W - p[0], p[1]]; }) : pts;
  }

  // ---------- page widget ----------

  var NS = 'http://www.w3.org/2000/svg';
  var C = { orange: '#FE7743', navy: '#273F4F', blue: '#447D9B', gray: '#D7D7D7' };
  var MODES = { pt: 'Pure teleoperation', va: 'Visual assistance', lb: 'SA-LB', cbf: 'SA-CBF' };

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
    this.mode = 'pt';
    this.results = {};
    this.visible = false;
    this.build();
    this.bind();
    this.newKnot();
  }

  Game.prototype.q = function (s) { return this.root.querySelector(s); };

  Game.prototype.build = function () {
    var svg = this.svg, R = window.RopeDraw;
    var text = function (x, y, anchor, size, weight, parent) {
      return el('text', { x: x, y: y, 'text-anchor': anchor, 'font-family': 'Roboto, sans-serif', 'font-size': size, 'font-weight': weight, fill: C.navy }, parent || svg);
    };
    this.halo = el('path', { fill: 'none', stroke: C.blue, 'stroke-opacity': 0.12, 'stroke-width': 2 * CBF_RADIUS, 'stroke-linecap': 'round', 'stroke-linejoin': 'round' }, svg);
    this.rope = new R.Rope(svg, { width: 2 * RAD });
    this.dots = el('g', {}, svg);
    this.lbLine = el('line', { stroke: C.orange, 'stroke-width': 2, 'stroke-dasharray': '5 4' }, svg);
    this.intentRing = el('circle', { r: 11, fill: 'none', stroke: C.orange, 'stroke-width': 2.5 }, svg);
    this.ghost = el('g', { 'pointer-events': 'none' }, svg);
    el('circle', { r: GAP_OPEN - FINGER, fill: 'none', stroke: C.navy, 'stroke-width': 1.5 }, this.ghost);
    el('path', { d: 'M-4 0H4M0 -4V4', stroke: C.navy, 'stroke-width': 1.5 }, this.ghost);
    this.cursor = el('circle', { r: 10, fill: 'none', stroke: C.blue, 'stroke-width': 2, 'stroke-dasharray': '3 3' }, svg);
    this.shadow = el('ellipse', { fill: C.navy }, svg);
    var g = this.grip = el('g', {}, svg);
    this.bar = el('rect', { x: -3, width: 6, rx: 3, fill: C.navy }, g);
    this.f1 = el('rect', { x: -5, width: 10, height: 7, rx: 2, fill: C.navy }, g);
    this.f2 = el('rect', { x: -5, width: 10, height: 7, rx: 2, fill: C.navy }, g);
    el('circle', { r: 3.5, fill: C.orange, stroke: '#fff', 'stroke-width': 1.2 }, g);
    var gauge = this.gauge = el('g', { transform: 'translate(612 92)' }, svg);
    el('rect', { x: 0, y: 0, width: 10, height: 120, rx: 5, fill: C.gray }, gauge);
    this.gaugeFill = el('rect', { x: 0, width: 10, rx: 5, fill: C.blue }, gauge);
    text(5, 136, 'middle', 10, 400, gauge).textContent = 'height';
    this.flash = text(320, 384, 'middle', 15, 700);
    var root = this.root;
    this.out = {};
    ['time', 'knot', 'cross', 'grasps', 'misses', 'bumps'].forEach(function (k) { this.out[k] = root.querySelector('[data-out="' + k + '"]'); }, this);
  };

  Game.prototype.bind = function () {
    var self = this, stage = this.stage;
    function local(ev) {
      var pt = self.svg.createSVGPoint();
      pt.x = ev.clientX; pt.y = ev.clientY;
      var p = pt.matrixTransform(self.svg.getScreenCTM().inverse());
      return [clamp(p.x, 0, W), clamp(p.y, 0, H)];
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
    ['pointerup', 'pointercancel'].forEach(function (t) {
      stage.addEventListener(t, function () { self.held = false; self.wake(); });
    });
    // Keyboard: arrows move the hand, Space or Enter holds the grasp.
    stage.addEventListener('keydown', function (ev) {
      var k = ev.key, d = ev.shiftKey ? 24 : 8;
      var step = { ArrowLeft: [-d, 0], ArrowRight: [d, 0], ArrowUp: [0, -d], ArrowDown: [0, d] }[k];
      if (step) {
        var p = self.ptr || [W / 2, H / 2];
        self.ptr = [clamp(p[0] + step[0], 0, W), clamp(p[1] + step[1], 0, H)];
        self.hover = true;
        self.wake();
        ev.preventDefault();
      } else if (k === ' ' || k === 'Enter') {
        if (!ev.repeat) { self.ptr = self.ptr || [W / 2, H / 2]; self.hover = true; self.press(); }
        ev.preventDefault();
      }
    });
    stage.addEventListener('keyup', function (ev) {
      if (ev.key === ' ' || ev.key === 'Enter') { self.held = false; self.wake(); }
    });

    this.root.addEventListener('click', function (ev) {
      var b = ev.target.closest('button');
      if (!b) return;
      if (b.hasAttribute('data-mode')) {
        self.mode = b.getAttribute('data-mode');
        self.root.querySelectorAll('[data-mode]').forEach(function (o) { o.setAttribute('aria-pressed', String(o === b)); });
        self.q('.game-mode-help').textContent = b.getAttribute('data-help');
        self.newKnot();
      }
      if (b.hasAttribute('data-new')) self.newKnot();
    });

    if ('IntersectionObserver' in window) {
      new IntersectionObserver(function (entries) {
        self.visible = entries[0].isIntersecting;
        self.wake();
      }).observe(this.stage);
    } else {
      this.visible = true;
    }
  };

  Game.prototype.newKnot = function () {
    var R = window.RopeDraw;
    this.sim = new Sim(knotPoints(R, Math.random() < 0.5), Math.random() < 0.5);
    this.g = { x: W / 2, y: H - 30, z: Z_HOVER, open: true, yaw: 0, keep: -1, hold: -1 };
    this.phase = 'idle';
    this.target = null;
    this.held = false;
    this.running = false;
    this.finished = false;
    this.t = 0;
    this.theta = Math.random() * 2 * Math.PI;
    this.grasps = 0; this.misses = 0; this.bumps = 0;
    this.lastBump = -1;
    this.flashT = 0;
    this.solvedFor = 0;
    this.nCross = this.sim.crossingCount();
    this.tied = true;
    this.status('Untie the knot: click a strand to grasp it, drag to move it, release to put it down.');
    this.render();
    this.wake();
  };

  Game.prototype.status = function (text) { this.q('.game-status').textContent = text; };

  Game.prototype.flashText = function (text, color) {
    // Same on-screen size on a phone as on a desktop.
    var w = this.svg.getBoundingClientRect().width || W;
    var size = 15 * clamp(W / w, 1, 2.4);
    this.flash.setAttribute('font-size', f1(size));
    this.flash.setAttribute('y', f1(H - 8 - size * 0.3));
    this.flash.textContent = text;
    this.flash.setAttribute('fill', color);
    this.flashT = 1.4;
  };

  // Where the gripper actually goes for a hand position p.
  Game.prototype.landing = function (p) {
    return [clamp(p[0] + OFFSET * Math.cos(this.theta), 0, W), clamp(p[1] + OFFSET * Math.sin(this.theta), 0, H)];
  };

  // Hand pressed: start an approach toward the rope.
  Game.prototype.press = function () {
    this.held = true;
    this.wake();
    if (this.finished || (this.phase !== 'idle' && this.phase !== 'lift')) return;
    if (!this.running) this.running = true;
    var L = this.landing(this.ptr), node = -1;
    if (this.mode === 'lb') node = this.sim.pick(L[0], L[1], LB_RADIUS);
    if (this.mode === 'cbf') {
      node = this.sim.pick(L[0], L[1], CBF_RADIUS);
      if (node < 0) {
        this.flashText('Barrier: no rope point here to descend to', C.navy);
        return;
      }
    }
    this.target = { node: node, x: L[0], y: L[1] };
    this.start = { x: this.g.x, y: this.g.y, z: this.g.z };
    // Like an operator would, turn the jaws across the strand below the landing point.
    var below = node >= 0 ? node : this.sim.pick(L[0], L[1], 30);
    if (below >= 0) this.g.yaw = this.tangentAngle(below);
    this.phase = 'travel';
  };

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
    var s = this.sim, top = 0;
    for (var i = 0; i < s.n; i++) {
      if (Math.hypot(s.x[i] - this.g.x, s.y[i] - this.g.y) < GAP_OPEN - FINGER) top = Math.max(top, s.z[i]);
    }
    return top;
  };

  Game.prototype.update = function (dt) {
    var g = this.g, s = this.sim;
    if (this.running) this.t += dt;
    this.theta += dt * 0.35;
    if (this.flashT) this.flashT = Math.max(0, this.flashT - dt);

    if (this.phase === 'travel') {
      var T = this.targetPos();
      if (this.mode === 'lb') {
        // Linear blending: straight at the rope point, descending on the way.
        var total = Math.hypot(T[0] - this.start.x, T[1] - this.start.y) || 1;
        var rem = this.moveXY(T[0], T[1], dt);
        g.z = Math.max(T[2], T[2] + (this.start.z - T[2]) * rem / total);
        if (rem === 0) { this.moveZ(T[2], dt); if (Math.abs(g.z - T[2]) < 0.5) this.phase = 'close'; }
      } else {
        this.moveZ(Z_HOVER, dt);
        if (this.moveXY(T[0], T[1], dt) === 0 && g.z === Z_HOVER) this.phase = 'descend';
      }
      if (this.mode === 'cbf') this.alignYaw();
    } else if (this.phase === 'descend') {
      var floor = this.floorUnder();
      if (this.mode === 'cbf') {
        // Down the funnel: keep tracking the rope point, jaws across the rope.
        var P = this.targetPos();
        g.x = P[0]; g.y = P[1]; floor = P[2];
        this.alignYaw();
      }
      if (this.moveZ(floor, dt) < 0.5) this.phase = 'close';
    } else if (this.phase === 'close') {
      this.close();
    } else if (this.phase === 'carry') {
      // Lift clear of the strands below first, then follow the hand.
      var c = [clamp(this.ptr[0] + this.carryOff[0], RAD, W - RAD), clamp(this.ptr[1] + this.carryOff[1], RAD, H - RAD)];
      this.moveZ(Z_CARRY, dt);
      var left = g.z < 3 * RAD ? 1 : this.moveXY(c[0], c[1], dt);
      if (!this.held && left === 0) this.release();
    } else if (this.phase === 'lift') {
      g.open = true;
      if (this.moveZ(Z_HOVER, dt) === 0) this.phase = 'idle';
    }

    g.keep = this.phase === 'descend' || this.phase === 'close' ? s.pick(g.x, g.y, GAP_OPEN + RAD) : -1;
    if (this.phase === 'carry') g.hold = s.grab;
    else if (this.phase !== 'lift') g.hold = -1;
    var pushed = s.step(dt, g);
    if (pushed > 1.5 && this.running && this.t - this.lastBump > 0.6) {
      this.bumps++;
      this.lastBump = this.t;
      this.flashText('Bumped the rope', '#b4451f');
    }

    this.nCross = s.crossingCount();
    this.tied = s.knotted();
    if (this.running) {
      if (!this.tied && s.grab < 0) this.solvedFor += dt; else this.solvedFor = 0;
      if (this.solvedFor > 0.5) this.finish(true);
      else if (this.t > TIME_LIMIT) this.finish(false);
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

  Game.prototype.alignYaw = function () { this.g.yaw = this.tangentAngle(this.target.node); };

  // Close the jaws: grasp the rope point between them, if any.
  Game.prototype.close = function () {
    var s = this.sim, g = this.g, best = -1, bd = Infinity;
    for (var i = 0; i < s.n; i++) {
      var d = Math.hypot(s.x[i] - g.x, s.y[i] - g.y);
      if (d >= GAP_OPEN - FINGER || Math.abs(s.z[i] - g.z) > RAD + 3) continue;
      if (i === this.target.node) { best = i; break; }
      if (d < bd) { bd = d; best = i; }
    }
    if (best < 0) {
      this.misses++;
      this.flashText('Missed: the gripper closed on air', '#b4451f');
      this.phase = 'lift';
      return;
    }
    this.grasps++;
    s.grab = best;
    g.open = false;
    g.x = s.x[best]; g.y = s.y[best];
    this.carryOff = [g.x - this.ptr[0], g.y - this.ptr[1]];
    this.phase = 'carry';
  };

  Game.prototype.release = function () {
    this.sim.grab = -1;
    this.g.open = true;
    this.phase = 'lift';
  };

  Game.prototype.finish = function (ok) {
    this.running = false;
    this.finished = true;
    if (this.sim.grab >= 0) this.release();
    var r = this.results[this.mode] || (this.results[this.mode] = { runs: 0, solved: 0, time: 0, grasps: 0, misses: 0, bumps: 0 });
    r.runs++;
    if (ok) { r.solved++; r.time += this.t; }
    r.grasps += this.grasps; r.misses += this.misses; r.bumps += this.bumps;
    var count = function (k, word) { return k + ' ' + word + (k === 1 ? '' : word.slice(-1) === 's' ? 'es' : 's'); };
    var left = this.nCross ? ' The ' + (this.nCross > 1 ? this.nCross + ' crossings left are loose loops' : 'crossing left is a loose loop') +
      ' that would slide out if you pulled the ends.' : '';
    this.status(ok
      ? 'Untied in ' + this.t.toFixed(1) + ' s with ' + count(this.grasps, 'grasp') + ', ' + count(this.misses, 'miss') +
        ' and ' + count(this.bumps, 'bump') + '.' + left + ' Try another mode, or press New knot.'
      : 'Time is up. Press New knot to try again.');
    this.flashText(ok ? 'Untied!' : 'Time is up', ok ? '#2f7d4f' : '#b4451f');
    this.renderResults();
  };

  Game.prototype.renderResults = function () {
    var body = this.q('.game-results tbody'), self = this;
    body.textContent = '';
    Object.keys(MODES).forEach(function (m) {
      var r = self.results[m];
      var tr = document.createElement('tr');
      if (m === self.mode) tr.className = 'is-current';
      var cells = r ? [
        MODES[m], r.solved + ' / ' + r.runs,
        r.solved ? (r.time / r.solved).toFixed(1) + ' s' : '–',
        (r.grasps / r.runs).toFixed(1), (r.misses / r.runs).toFixed(1), (r.bumps / r.runs).toFixed(1)
      ] : [MODES[m], '–', '–', '–', '–', '–'];
      cells.forEach(function (txt) { var td = document.createElement('td'); td.textContent = txt; tr.appendChild(td); });
      body.appendChild(tr);
    });
    this.q('.game-results').hidden = false;
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
      var busy = self.running || self.phase !== 'idle' || self.flashT > 0 || self.sim.moved > 0.002;
      self.raf = self.visible && busy ? requestAnimationFrame(frame) : null;
    }
    this.raf = requestAnimationFrame(frame);
  };

  Game.prototype.render = function () {
    var s = this.sim, g = this.g, n = s.n, mode = this.mode;
    var pts = new Array(n);
    for (var i = 0; i < n; i++) pts[i] = [s.x[i], s.y[i]];
    // Painter's order by height, so the upper strand covers the lower one.
    var order = [];
    for (var k = 0; k < n - 1; k++) order.push(k);
    order.sort(function (a, b) { return (s.z[a] + s.z[a + 1]) - (s.z[b] + s.z[b + 1]) || a - b; });
    this.rope.render(pts, order);

    var line = 'M' + pts.map(function (p) { return f1(p[0]) + ' ' + f1(p[1]); }).join('L');
    this.halo.setAttribute('d', line);
    this.halo.setAttribute('opacity', mode === 'cbf' ? 1 : 0);

    // Visual assistance: the rope estimate, darker where it is higher.
    var va = mode === 'va';
    if (va && !this.dots.childNodes.length) {
      for (var d = 0; d < n; d += 2) el('circle', { r: 2.4 }, this.dots);
    }
    if (va) {
      for (var m = 0, j = 0; m < n; m += 2, j++) {
        var dot = this.dots.childNodes[j];
        dot.setAttribute('cx', f1(s.x[m])); dot.setAttribute('cy', f1(s.y[m]));
        dot.setAttribute('fill', s.z[m] > RAD ? C.navy : '#fff');
      }
    }
    this.dots.setAttribute('opacity', va ? 0.9 : 0);

    var p = this.hover ? this.ptr : null;
    this.cursor.setAttribute('opacity', p ? 0.8 : 0);
    if (p) { this.cursor.setAttribute('cx', f1(p[0])); this.cursor.setAttribute('cy', f1(p[1])); }
    var hovering = p && (this.phase === 'idle' || this.phase === 'lift');
    if (hovering) {
      var L = this.landing(p);
      this.ghost.setAttribute('transform', 'translate(' + f1(L[0]) + ' ' + f1(L[1]) + ')');
    }
    this.ghost.setAttribute('opacity', va && hovering ? 0.85 : 0);

    var approaching = this.target && (this.phase === 'travel' || this.phase === 'descend') && this.target.node >= 0;
    if (approaching) {
      var T = this.targetPos();
      this.intentRing.setAttribute('cx', f1(T[0])); this.intentRing.setAttribute('cy', f1(T[1]));
      this.lbLine.setAttribute('x1', f1(g.x)); this.lbLine.setAttribute('y1', f1(g.y));
      this.lbLine.setAttribute('x2', f1(T[0])); this.lbLine.setAttribute('y2', f1(T[1]));
    }
    this.intentRing.setAttribute('opacity', approaching ? 1 : 0);
    this.lbLine.setAttribute('opacity', approaching && mode === 'lb' ? 0.8 : 0);

    // Gripper seen from above: larger when higher; shadow and gauge with visual assistance.
    var scale = 1 + g.z / 150, gap = (g.open ? GAP_OPEN : GAP_SHUT) - 3.5;
    this.bar.setAttribute('y', f1(-gap - 7)); this.bar.setAttribute('height', f1(2 * gap + 14));
    this.f1.setAttribute('y', f1(-gap - 7)); this.f2.setAttribute('y', f1(gap));
    this.grip.setAttribute('transform', 'translate(' + f1(g.x) + ' ' + f1(g.y) + ') rotate(' + f1(g.yaw) + ') scale(' + f1(scale * 100) / 100 + ')');
    this.shadow.setAttribute('cx', f1(g.x)); this.shadow.setAttribute('cy', f1(g.y));
    this.shadow.setAttribute('rx', 12); this.shadow.setAttribute('ry', 12);
    this.shadow.setAttribute('opacity', va ? f1(0.35 - g.z / 400) : 0);
    this.gauge.setAttribute('opacity', va ? 1 : 0);
    var frac = clamp(g.z / Z_HOVER, 0, 1);
    this.gaugeFill.setAttribute('y', f1(120 * (1 - frac)));
    this.gaugeFill.setAttribute('height', f1(120 * frac));

    var out = this.out;
    out.time.textContent = this.t.toFixed(1) + ' s';
    out.knot.textContent = this.tied ? 'tied' : 'untied';
    out.knot.className = this.tied ? '' : 'is-ok';
    out.cross.textContent = this.nCross;
    out.grasps.textContent = this.grasps;
    out.misses.textContent = this.misses;
    out.bumps.textContent = this.bumps;
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
  if (typeof module !== 'undefined') module.exports = { Sim: Sim, crossings: crossings, knotPoints: knotPoints, knotDeterminant: knotDeterminant };
})();
