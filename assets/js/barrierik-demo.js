/* Planar shared-autonomy demo for the BarrierIK project page.
 *
 * A three-link arm follows a target given by the operator (the pointer), or,
 * with shared autonomy, a linear blend of the operator's target and an
 * autonomous target (the block), weighted by a sigmoid of their distance as
 * in the paper. Each frame an inverse-kinematics step turns the blended
 * target into joint motion. The three solvers follow the paper's baselines:
 *   n  Baseline N: tracking only, obstacles are ignored
 *   p  Baseline P: an obstacle-proximity penalty in the objective (soft)
 *   b  BarrierIK: control-barrier-function constraints on the step (hard)
 * Every step is a small quadratic program, min ½Δθᵀ(JᵀJ + λI)Δθ − (Jᵀe)ᵀΔθ,
 * with joint-limit, step-size and (for BarrierIK) linearised CBF
 * constraints ∇h·Δθ + γh ≥ 0, solved with Hildreth's dual method. This is a
 * planar toy, not the paper's SLSQP solver.
 *
 * Usage: an element with [data-barrierik] containing the markup from
 * _includes/barrierik_demo.html.
 */
(function () {
  'use strict';

  // ---------- model (px, rad; y points up, origin at the robot base) ----------

  var LINKS = [125, 110, 85];
  var LINK_R = 8;                        // link capsule radius
  var EPS = 8;                           // safety margin of the barrier
  var GAMMA = 0.35;                      // CBF class-K gain (per step)
  var LAMBDA = 900;                      // damping of the least-squares step
  var STEP_MAX = 0.06;                   // joint step bound (rad)
  var E_MAX = 7;                         // Cartesian error per step (px)
  var W_COL = 3000, DELTA = 30;          // Baseline P proximity penalty
  var LIM = [[0.15, Math.PI - 0.15], [-2.7, 2.7], [-2.7, 2.7]];
  var MM_PER_PX = 2.5;

  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

  // Joint positions [base, elbow 1, elbow 2, tool].
  function fk(th) {
    var p = [[0, 0]], a = 0;
    for (var i = 0; i < 3; i++) {
      a += th[i];
      var q = p[i];
      p.push([q[0] + LINKS[i] * Math.cos(a), q[1] + LINKS[i] * Math.sin(a)]);
    }
    return p;
  }

  // Jacobian (2 × 3) of point pt on link k: joints up to k move it.
  function jacobianAt(P, k, pt) {
    var J = [[0, 0, 0], [0, 0, 0]];
    for (var i = 0; i <= k; i++) {
      J[0][i] = -(pt[1] - P[i][1]);
      J[1][i] = pt[0] - P[i][0];
    }
    return J;
  }

  // Signed distance between link k and a circular obstacle, with its gradient.
  function linkDistance(P, k, o) {
    var a = P[k], b = P[k + 1], dx = b[0] - a[0], dy = b[1] - a[1];
    var t = clamp(((o.x - a[0]) * dx + (o.y - a[1]) * dy) / (dx * dx + dy * dy), 0, 1);
    var pt = [a[0] + t * dx, a[1] + t * dy];
    var vx = pt[0] - o.x, vy = pt[1] - o.y, d = Math.hypot(vx, vy) || 1e-6;
    var J = jacobianAt(P, k, pt), n = [vx / d, vy / d];
    return {
      phi: d - o.r - LINK_R,
      grad: [0, 1, 2].map(function (i) { return n[0] * J[0][i] + n[1] * J[1][i]; }),
      pt: pt
    };
  }

  function clearance(th, obstacles) {
    var P = fk(th), best = Infinity;
    obstacles.forEach(function (o) {
      for (var k = 0; k < 3; k++) best = Math.min(best, linkDistance(P, k, o).phi);
    });
    return best;
  }

  function inv3(m) {
    var a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
    var A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
    return [
      [A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
      [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
      [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]
    ];
  }

  function mv(M, v) { return M.map(function (r) { return r[0] * v[0] + r[1] * v[1] + r[2] * v[2]; }); }

  // min ½xᵀHx + fᵀx  s.t.  A x ≤ b, by Hildreth's dual coordinate descent.
  function qp(H, f, A, b) {
    var Hi = inv3(H), x0 = mv(Hi, f).map(function (v) { return -v; });
    var violated = A.some(function (a, k) { return a[0] * x0[0] + a[1] * x0[1] + a[2] * x0[2] > b[k] + 1e-9; });
    if (!violated) return x0;
    var m = A.length, HiA = A.map(function (a) { return mv(Hi, a); });
    var Pm = A.map(function (a) { return HiA.map(function (h) { return a[0] * h[0] + a[1] * h[1] + a[2] * h[2]; }); });
    var d = A.map(function (a, k) { return b[k] - (a[0] * x0[0] + a[1] * x0[1] + a[2] * x0[2]); });
    var lam = new Array(m).fill(0);
    for (var it = 0; it < 80; it++) {
      var change = 0;
      for (var i = 0; i < m; i++) {
        var s = d[i];
        for (var j = 0; j < m; j++) if (j !== i) s += Pm[i][j] * lam[j];
        var nl = Math.max(0, -s / Pm[i][i]);
        change = Math.max(change, Math.abs(nl - lam[i]));
        lam[i] = nl;
      }
      if (change < 1e-10) break;
    }
    var x = x0.slice();
    for (var k = 0; k < m; k++) for (var r = 0; r < 3; r++) x[r] -= lam[k] * HiA[k][r];
    return x;
  }

  // One IK step toward the Cartesian target with the chosen solver.
  function ikStep(th, target, obstacles, solver) {
    var P = fk(th), tool = P[3];
    var ex = target[0] - tool[0], ey = target[1] - tool[1], en = Math.hypot(ex, ey);
    if (en > E_MAX) { ex *= E_MAX / en; ey *= E_MAX / en; }
    var J = jacobianAt(P, 2, tool);
    var H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], f = [0, 0, 0];
    for (var i = 0; i < 3; i++) {
      for (var j = 0; j < 3; j++) H[i][j] = J[0][i] * J[0][j] + J[1][i] * J[1][j] + (i === j ? LAMBDA : 0);
      f[i] = -(J[0][i] * ex + J[1][i] * ey);
    }
    var A = [], b = [];
    for (var q = 0; q < 3; q++) {
      var row = [0, 0, 0], neg = [0, 0, 0];
      row[q] = 1; neg[q] = -1;
      A.push(row); b.push(Math.min(STEP_MAX, LIM[q][1] - th[q]));
      A.push(neg); b.push(Math.min(STEP_MAX, th[q] - LIM[q][0]));
    }
    obstacles.forEach(function (o) {
      for (var k = 0; k < 3; k++) {
        var L = linkDistance(P, k, o);
        if (solver === 'p') {
          // Proximity penalty w/(φ² + δ): its gradient pushes away, but tracking can win.
          var phi = Math.max(L.phi, 0.5);
          var g = -2 * W_COL * phi / Math.pow(phi * phi + DELTA, 2);
          for (var r = 0; r < 3; r++) f[r] += g * L.grad[r];
        } else if (solver === 'b' && L.phi < 60) {
          // CBF h = φ − ε:  ∇h·Δθ + γh ≥ 0   ⇔   −∇h·Δθ ≤ γh.
          A.push(L.grad.map(function (v) { return -v; }));
          b.push(GAMMA * (L.phi - EPS));
        }
      }
    });
    var d = qp(H, f, A, b);
    return [th[0] + d[0], th[1] + d[1], th[2] + d[2]];
  }

  // Sigmoid arbitration over the operator–autonomy disagreement (paper, Sec. III).
  function arbitration(xh, xr) {
    var dist = Math.hypot(xh[0] - xr[0], xh[1] - xr[1]);
    return 0.85 / (1 + Math.exp(dist / 45 - 2.5));
  }

  // ---------- page widget ----------

  var NS = 'http://www.w3.org/2000/svg';
  var C = { orange: '#FE7743', navy: '#273F4F', blue: '#447D9B', gray: '#D7D7D7' };
  var W = 640, H = 400, BX = 320, BY = 384;   // robot base in the SVG
  var HOME = [2.594, -0.869, -1.002];
  var NAMES = { n: 'Baseline N', p: 'Baseline P', b: 'BarrierIK' };
  var SCENES = {
    pillar: { block: [220, 170], obstacles: [[90, 210, 26], [235, 255, 22], [-170, 140, 24], [205, 72, 22]] },
    gap: { block: [225, 215], obstacles: [[120, 252, 28], [120, 175, 28], [-170, 140, 24], [215, 95, 22]] }
  };

  function toSvg(p) { return [BX + p[0], BY - p[1]]; }
  function toModel(x, y) { return [x - BX, BY - y]; }
  function el(name, attrs, parent) {
    var n = document.createElementNS(NS, name);
    for (var k in attrs) n.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(n);
    return n;
  }
  function f1(x) { return Math.round(x * 10) / 10; }

  function Demo(root) {
    this.root = root;
    this.svg = root.querySelector('.bik-svg');
    this.assist = 'sa';
    this.solver = 'b';
    this.results = [];
    this.visible = false;
    this.loadScene('pillar');
    this.build();
    this.bind();
    this.reset();
  }

  Demo.prototype.q = function (s) { return this.root.querySelector(s); };

  Demo.prototype.loadScene = function (name) {
    var sc = SCENES[name], obs = this.obstacles;
    this.block = { x: sc.block[0], y: sc.block[1] };
    this.obstacles = sc.obstacles.map(function (o, i) {
      var cur = obs && obs[i] ? obs[i] : {};
      cur.x = o[0]; cur.y = o[1]; cur.r = o[2];
      return cur;
    });
  };

  Demo.prototype.build = function () {
    var svg = this.svg;
    el('rect', { x: BX - 46, y: BY, width: 92, height: 14, rx: 3, fill: C.navy }, svg);
    this.obsG = el('g', {}, svg);
    this.obsEls = this.obstacles.map(function () {
      var g = el('g', { class: 'bik-obstacle' }, this.obsG);
      g.margin = el('circle', { fill: 'none', stroke: C.blue, 'stroke-dasharray': '3 4', 'stroke-opacity': 0.55 }, g);
      g.body = el('circle', { fill: '#c9cfd3', stroke: C.navy, 'stroke-width': 1.5 }, g);
      el('path', { d: 'M-6 0H6M0 -6V6', stroke: C.navy, 'stroke-width': 1.5, 'stroke-opacity': 0.5 }, g);
      return g;
    }, this);
    this.blockEl = el('g', { class: 'bik-block' }, svg);
    el('rect', { x: -11, y: -11, width: 22, height: 22, rx: 3, fill: C.orange, stroke: C.navy, 'stroke-width': 1.5 }, this.blockEl);
    this.handEl = el('circle', { r: 10, fill: 'none', stroke: C.blue, 'stroke-width': 2, 'stroke-dasharray': '3 3' }, svg);
    this.blendLine = el('line', { stroke: '#b4451f', 'stroke-width': 1.5, 'stroke-dasharray': '4 4' }, svg);
    this.blendEl = el('circle', { r: 6, fill: 'none', stroke: '#b4451f', 'stroke-width': 2 }, svg);
    this.trail = el('polyline', { fill: 'none', stroke: C.navy, 'stroke-opacity': 0.25, 'stroke-width': 2 }, svg);
    this.linkEls = LINKS.map(function () {
      return el('line', { stroke: C.navy, 'stroke-width': 2 * LINK_R, 'stroke-linecap': 'round' }, svg);
    });
    this.jointEls = [0, 1, 2].map(function () { return el('circle', { r: 5.5, fill: '#fff', stroke: C.navy, 'stroke-width': 2 }, svg); });
    var g = this.gripEl = el('g', {}, svg);
    el('rect', { x: -3, y: -12, width: 6, height: 24, rx: 3, fill: C.navy }, g);
    el('rect', { x: 0, y: -12, width: 14, height: 5, rx: 2, fill: C.navy }, g);
    el('rect', { x: 0, y: 7, width: 14, height: 5, rx: 2, fill: C.navy }, g);
    this.closest = el('line', { stroke: C.orange, 'stroke-width': 2 }, svg);
  };

  Demo.prototype.bind = function () {
    var self = this, stage = this.q('.bik-stage');
    function local(ev) {
      var pt = self.svg.createSVGPoint();
      pt.x = ev.clientX; pt.y = ev.clientY;
      var p = pt.matrixTransform(self.svg.getScreenCTM().inverse());
      return toModel(clamp(p.x, 0, W), clamp(p.y, 0, H));
    }
    // Drag an obstacle or the block; anywhere else moves the operator's hand.
    stage.addEventListener('pointerdown', function (ev) {
      var p = local(ev);
      self.dragging = null;
      self.obstacles.forEach(function (o) { if (Math.hypot(p[0] - o.x, p[1] - o.y) < o.r + 6) self.dragging = o; });
      if (Math.hypot(p[0] - self.block.x, p[1] - self.block.y) < 18) self.dragging = self.block;
      if (!self.dragging) { self.script = null; self.hand = p; self.begin(); }
      stage.setPointerCapture(ev.pointerId);
      self.pressed = true;
      ev.preventDefault();
      self.wake();
    });
    stage.addEventListener('pointermove', function (ev) {
      var p = local(ev);
      if (self.dragging) {
        self.dragging.x = clamp(p[0], -300, 300);
        self.dragging.y = clamp(p[1], 20, 370);
      } else if ((self.pressed || ev.pointerType === 'mouse') && !self.script) {
        self.hand = p;
        self.begin();
      }
      self.wake();
    });
    ['pointerup', 'pointercancel'].forEach(function (t) {
      stage.addEventListener(t, function () { self.dragging = null; self.pressed = false; });
    });
    stage.addEventListener('keydown', function (ev) {
      var d = ev.shiftKey ? 20 : 6;
      var step = { ArrowLeft: [-d, 0], ArrowRight: [d, 0], ArrowUp: [0, d], ArrowDown: [0, -d] }[ev.key];
      if (!step) return;
      self.script = null;
      self.hand = [self.hand[0] + step[0], clamp(self.hand[1] + step[1], 0, H)];
      self.begin();
      self.wake();
      ev.preventDefault();
    });

    this.root.addEventListener('click', function (ev) {
      var b = ev.target.closest('button');
      if (!b) return;
      var group = b.closest('[data-group]');
      if (group) {
        group.querySelectorAll('button').forEach(function (o) { o.setAttribute('aria-pressed', String(o === b)); });
        var key = group.getAttribute('data-group');
        if (key === 'scene') self.loadScene(b.getAttribute('data-value'));
        else self[key] = b.getAttribute('data-value');
        self.reset();
      }
      if (b.hasAttribute('data-script')) { self.reset(); self.runScript(); }
      if (b.hasAttribute('data-reset')) self.reset();
    });

    if ('IntersectionObserver' in window) {
      new IntersectionObserver(function (entries) {
        self.visible = entries[0].isIntersecting;
        self.wake();
      }).observe(stage);
    } else {
      this.visible = true;
    }
  };

  Demo.prototype.reset = function () {
    this.th = HOME.slice();
    this.hand = fk(this.th)[3].slice();
    this.active = false;
    this.done = false;
    this.script = null;
    this.t = 0; this.tViol = 0; this.tMargin = 0; this.minClear = Infinity;
    this.lastTool = null; this.stalled = 0;
    this.near = 0;
    this.path = [];
    this.update(0);
    this.render();
    this.status(this.assist === 'sa'
      ? 'Move the pointer (or drag on touch screens) toward the orange block. The assistance pulls your target toward it.'
      : 'Move the pointer (or drag on touch screens) to steer the gripper to the orange block.');
  };

  Demo.prototype.begin = function () { if (!this.active && !this.done) this.active = true; };

  Demo.prototype.status = function (text, cls) {
    var e = this.q('.bik-status');
    e.textContent = text;
    e.className = 'comanip-status bik-status' + (cls ? ' is-' + cls : '');
  };

  // A simulated operator: a minimum-jerk reach straight at the block, which
  // passes close to the obstacles in the default scene.
  Demo.prototype.runScript = function () {
    var from = fk(this.th)[3].slice(), to = [this.block.x, this.block.y];
    this.script = { t: 0, T: 2.6, from: from, to: to };
    this.active = true;
    this.wake();
  };

  Demo.prototype.update = function (dt) {
    if (this.script) {
      var s = this.script;
      s.t += dt;
      var u = clamp(s.t / s.T, 0, 1), m = u * u * u * (10 - 15 * u + 6 * u * u);
      this.hand = [s.from[0] + (s.to[0] - s.from[0]) * m, s.from[1] + (s.to[1] - s.from[1]) * m];
    }
    var xr = [this.block.x, this.block.y];
    this.alpha = this.assist === 'sa' ? arbitration(this.hand, xr) : 0;
    this.target = [this.hand[0] + this.alpha * (xr[0] - this.hand[0]), this.hand[1] + this.alpha * (xr[1] - this.hand[1])];
    if (dt > 0) {
      for (var i = 0; i < 2; i++) this.th = ikStep(this.th, this.target, this.obstacles, this.solver);
    }
    this.clear = clearance(this.th, this.obstacles);
    var tool = fk(this.th)[3];
    if (this.active && !this.done && dt > 0) {
      this.lastTool = this.lastTool || tool;
      this.t += dt;
      if (this.clear < 0) this.tViol += dt;
      if (this.clear < EPS - 0.5) this.tMargin += dt;
      this.minClear = Math.min(this.minClear, this.clear);
      if (this.path.length === 0 || this.t - this.path[this.path.length - 1][2] > 0.03) this.path.push([tool[0], tool[1], this.t]);
      this.lastTool = tool;
      this.near = Math.hypot(tool[0] - xr[0], tool[1] - xr[1]) < 14 ? this.near + dt : 0;
      // A scripted reach that has come to rest short of the block is over.
      var speed = this.lastTool ? Math.hypot(tool[0] - this.lastTool[0], tool[1] - this.lastTool[1]) / dt : 0;
      this.stalled = this.script && this.script.t > this.script.T && speed < 2 ? (this.stalled || 0) + dt : 0;
      if (this.near > 0.3) this.finish(true);
      else if (this.t > 20 || this.stalled > 1) this.finish(false);
    }
  };

  Demo.prototype.finish = function (ok) {
    this.done = true;
    this.active = false;
    this.script = null;
    var T = Math.max(this.t, 1e-6);
    var row = {
      cond: (this.assist === 'sa' ? 'Shared autonomy' : 'Teleoperation') + ' · ' + NAMES[this.solver],
      ok: ok, t: this.t, minClear: this.minClear * MM_PER_PX,
      viol: this.tViol / T * 100, margin: this.tMargin / T * 100
    };
    this.results.unshift(row);
    this.results.length = Math.min(this.results.length, 8);
    var safety = ' · min clearance ' + row.minClear.toFixed(0) + ' mm · in collision ' + row.viol.toFixed(0) +
      '% and inside the margin ' + row.margin.toFixed(0) + '% of the time.';
    var why = ok || row.viol > 0 ? '' : row.margin < 0.5
      ? ' The arm stopped at the edge of the safety margin rather than enter it.'
      : ' The penalty held the arm back, but only after it had entered the margin.';
    this.status((ok ? 'Reached in ' + row.t.toFixed(1) + ' s' : 'Stopped short of the block') + safety + why +
      ' Try another solver.', row.viol > 0 ? 'bad' : ok ? 'ok' : '');
    this.renderResults();
  };

  Demo.prototype.renderResults = function () {
    var body = this.q('.bik-results tbody');
    body.textContent = '';
    this.results.forEach(function (r) {
      var tr = document.createElement('tr');
      [r.cond, r.ok ? r.t.toFixed(1) + ' s' : 'no', r.minClear.toFixed(0) + ' mm', r.viol.toFixed(1) + '%', r.margin.toFixed(1) + '%'].forEach(function (txt) {
        var td = document.createElement('td'); td.textContent = txt; tr.appendChild(td);
      });
      body.appendChild(tr);
    });
    this.q('.bik-results').hidden = !this.results.length;
  };

  Demo.prototype.wake = function () {
    if (this.raf || !this.visible) return;
    var self = this, last = performance.now(), still = 0;
    function frame(now) {
      var dt = Math.min((now - last) / 1000, 1 / 30);
      last = now;
      var before = self.th.slice();
      self.update(dt);
      self.render();
      var moved = Math.abs(self.th[0] - before[0]) + Math.abs(self.th[1] - before[1]) + Math.abs(self.th[2] - before[2]);
      still = moved < 1e-5 && !self.script && !self.dragging ? still + 1 : 0;
      self.raf = self.visible && still < 20 ? requestAnimationFrame(frame) : null;
    }
    this.raf = requestAnimationFrame(frame);
  };

  Demo.prototype.render = function () {
    var P = fk(this.th).map(toSvg), self = this;
    var hitLink = [false, false, false];
    var Pm = fk(this.th);
    this.obstacles.forEach(function (o, i) {
      var c = toSvg([o.x, o.y]), inside = false;
      for (var k = 0; k < 3; k++) if (linkDistance(Pm, k, o).phi < 0) { hitLink[k] = true; inside = true; }
      self.obsEls[i].setAttribute('transform', 'translate(' + f1(c[0]) + ' ' + f1(c[1]) + ')');
      self.obsEls[i].margin.setAttribute('r', o.r + EPS);
      self.obsEls[i].body.setAttribute('r', o.r);
      self.obsEls[i].classList.toggle('is-hit', inside);
    });
    this.linkEls.forEach(function (l, k) {
      l.setAttribute('x1', f1(P[k][0])); l.setAttribute('y1', f1(P[k][1]));
      l.setAttribute('x2', f1(P[k + 1][0])); l.setAttribute('y2', f1(P[k + 1][1]));
      l.setAttribute('stroke', hitLink[k] ? '#b4451f' : C.navy);
    });
    this.jointEls.forEach(function (j, k) { j.setAttribute('cx', f1(P[k][0])); j.setAttribute('cy', f1(P[k][1])); });
    var ang = -(this.th[0] + this.th[1] + this.th[2]) * 180 / Math.PI;
    this.gripEl.setAttribute('transform', 'translate(' + f1(P[3][0]) + ' ' + f1(P[3][1]) + ') rotate(' + f1(ang) + ')');
    var b = toSvg([this.block.x, this.block.y]);
    this.blockEl.setAttribute('transform', 'translate(' + f1(b[0]) + ' ' + f1(b[1]) + ')');
    var h = toSvg(this.hand), t = toSvg(this.target);
    this.handEl.setAttribute('cx', f1(h[0])); this.handEl.setAttribute('cy', f1(h[1]));
    this.blendEl.setAttribute('cx', f1(t[0])); this.blendEl.setAttribute('cy', f1(t[1]));
    this.blendLine.setAttribute('x1', f1(h[0])); this.blendLine.setAttribute('y1', f1(h[1]));
    this.blendLine.setAttribute('x2', f1(t[0])); this.blendLine.setAttribute('y2', f1(t[1]));
    var sa = this.assist === 'sa';
    this.blendEl.setAttribute('opacity', sa ? 1 : 0);
    this.blendLine.setAttribute('opacity', sa ? 0.8 : 0);
    this.trail.setAttribute('points', this.path.map(function (p) { var s = toSvg(p); return f1(s[0]) + ',' + f1(s[1]); }).join(' '));

    // The closest link–obstacle pair.
    var best = null;
    this.obstacles.forEach(function (o) {
      for (var k = 0; k < 3; k++) {
        var L = linkDistance(Pm, k, o);
        if (!best || L.phi < best.phi) best = { phi: L.phi, pt: L.pt, o: o };
      }
    });
    if (best && best.phi < 50) {
      var a = toSvg(best.pt), dx = best.pt[0] - best.o.x, dy = best.pt[1] - best.o.y, dn = Math.hypot(dx, dy) || 1;
      var e = toSvg([best.o.x + dx / dn * best.o.r, best.o.y + dy / dn * best.o.r]);
      this.closest.setAttribute('x1', f1(a[0])); this.closest.setAttribute('y1', f1(a[1]));
      this.closest.setAttribute('x2', f1(e[0])); this.closest.setAttribute('y2', f1(e[1]));
      this.closest.setAttribute('opacity', 0.9);
    } else {
      this.closest.setAttribute('opacity', 0);
    }

    var set = function (k, v) { self.q('[data-out="' + k + '"]').textContent = v; };
    set('alpha', sa ? this.alpha.toFixed(2) : '–');
    set('clear', (this.clear * MM_PER_PX).toFixed(0) + ' mm');
    set('min', isFinite(this.minClear) ? (this.minClear * MM_PER_PX).toFixed(0) + ' mm' : '–');
    set('viol', (this.t ? this.tViol / this.t * 100 : 0).toFixed(1) + '%');
    set('margin', (this.t ? this.tMargin / this.t * 100 : 0).toFixed(1) + '%');
    set('time', this.t.toFixed(1) + ' s');
    this.q('.ro--clear').classList.toggle('is-bad', this.clear < 0);
  };

  function init() {
    Array.prototype.forEach.call(document.querySelectorAll('[data-barrierik]'), function (root) { root.demo = new Demo(root); });
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }
  if (typeof module !== 'undefined') {
    module.exports = { fk: fk, ikStep: ikStep, clearance: clearance, arbitration: arbitration, qp: qp, HOME: HOME };
  }
})();
