/* Animated SVG ropes for the deformable-linear-object projects.
 *
 * The rope look follows Stanko Tadić's "Draw SVG rope using JavaScript"
 * (https://muffinman.io/blog/draw-svg-rope-using-javascript/ and
 * https://codepen.io/stanko/pen/vYaEMKX): walk the centre line in equal
 * arc-length steps and draw every step as a slanted segment with rounded
 * corners, so the outlines read as twisted strands. Crossings get an
 * alternating over/under assignment so that knots look like knots.
 *
 * Usage: <svg data-rope="scene-name" viewBox="0 0 400 250"></svg>
 * Scenes: shape-matching, assist, loop, particles.
 */
(function () {
  'use strict';

  var NS = 'http://www.w3.org/2000/svg';
  var C = { orange: '#FE7743', navy: '#273F4F', blue: '#447D9B', gray: '#D7D7D7' };

  // ---------- small helpers ----------

  function el(name, attrs, parent) {
    var node = document.createElementNS(NS, name);
    for (var k in attrs) node.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(node);
    return node;
  }

  function lerp(a, b, t) { return a + (b - a) * t; }
  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
  function ease(t) { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); }
  function dist(p, q) { return Math.hypot(q[0] - p[0], q[1] - p[1]); }
  function f1(x) { return Math.round(x * 10) / 10; }

  // Catmull-Rom spline through control points, returned as a dense polyline.
  function spline(ctrl, steps) {
    var out = [];
    var n = ctrl.length;
    for (var i = 0; i < n - 1; i++) {
      var p0 = ctrl[Math.max(i - 1, 0)], p1 = ctrl[i], p2 = ctrl[i + 1], p3 = ctrl[Math.min(i + 2, n - 1)];
      for (var s = 0; s < steps; s++) {
        var t = s / steps, t2 = t * t, t3 = t2 * t;
        out.push([0, 1].map(function (d) {
          return 0.5 * (2 * p1[d] + (p2[d] - p0[d]) * t +
            (2 * p0[d] - 5 * p1[d] + 4 * p2[d] - p3[d]) * t2 +
            (3 * p1[d] - p0[d] - 3 * p2[d] + p3[d]) * t3);
        }));
      }
    }
    out.push(ctrl[n - 1].slice());
    return out;
  }

  // `count` points spaced equally along a polyline.
  function resample(poly, count) {
    var cum = [0];
    for (var i = 1; i < poly.length; i++) cum.push(cum[i - 1] + dist(poly[i - 1], poly[i]));
    var total = cum[cum.length - 1];
    var out = [];
    var j = 1;
    for (var k = 0; k < count; k++) {
      var d = total * k / (count - 1);
      while (j < poly.length - 1 && cum[j] < d) j++;
      var span = (cum[j] - cum[j - 1]) || 1;
      var t = clamp((d - cum[j - 1]) / span, 0, 1);
      out.push([lerp(poly[j - 1][0], poly[j][0], t), lerp(poly[j - 1][1], poly[j][1], t)]);
    }
    return out;
  }

  function polyLength(poly) {
    var L = 0;
    for (var i = 1; i < poly.length; i++) L += dist(poly[i - 1], poly[i]);
    return L;
  }

  function lerpPoly(a, b, t) {
    return a.map(function (p, i) { return [lerp(p[0], b[i][0], t), lerp(p[1], b[i][1], t)]; });
  }

  function arc(cx, cy, r, a0, a1, n) {
    var out = [];
    for (var i = 0; i <= n; i++) {
      var a = (a0 + (a1 - a0) * i / n) * Math.PI / 180;
      out.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
    }
    return out;
  }

  function line(p, q, n) {
    var out = [];
    for (var i = 0; i <= n; i++) out.push([lerp(p[0], q[0], i / n), lerp(p[1], q[1], i / n)]);
    return out;
  }

  function join() {
    var out = [];
    for (var i = 0; i < arguments.length; i++) {
      var part = arguments[i];
      out = out.concat(out.length ? part.slice(1) : part);
    }
    return out;
  }

  function pathD(pts) {
    return 'M' + pts.map(function (p) { return f1(p[0]) + ' ' + f1(p[1]); }).join('L');
  }

  // Tangent (unit) at every sample.
  function tangents(pts) {
    var n = pts.length;
    return pts.map(function (p, i) {
      var a = pts[Math.max(i - 1, 0)], b = pts[Math.min(i + 1, n - 1)];
      var dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1;
      return [dx / L, dy / L];
    });
  }

  // ---------- crossings: alternating over/under ----------

  function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }

  function intersects(p1, p2, p3, p4) {
    var d1 = cross(p3, p4, p1), d2 = cross(p3, p4, p2), d3 = cross(p1, p2, p3), d4 = cross(p1, p2, p4);
    return ((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0));
  }

  // Painter's order for the segments: segments passing over a crossing are
  // drawn last. Walking along the rope, crossings alternate over/under.
  function paintOrder(pts, overFirst) {
    var nSeg = pts.length - 1;
    var found = [];
    for (var i = 0; i < nSeg; i++) {
      for (var j = i + 3; j < nSeg; j++) {
        if (!intersects(pts[i], pts[i + 1], pts[j], pts[j + 1])) continue;
        var dup = found.some(function (c) { return Math.abs(c[0] - i) <= 2 && Math.abs(c[1] - j) <= 2; });
        if (!dup) found.push([i, j]);
      }
    }
    var events = [];
    found.forEach(function (c, k) { events.push({ at: c[0], c: k }, { at: c[1], c: k }); });
    events.sort(function (a, b) { return a.at - b.at; });
    var overSeg = {};
    events.forEach(function (e, idx) {
      if (overSeg[e.c] !== undefined) return;
      var first = (idx % 2 === 0) === overFirst;
      var pair = found[e.c];
      overSeg[e.c] = first ? e.at : (e.at === pair[0] ? pair[1] : pair[0]);
    });
    var layer = new Array(nSeg).fill(0);
    Object.keys(overSeg).forEach(function (k) {
      for (var d = -2; d <= 2; d++) {
        var s = overSeg[k] + d;
        if (s >= 0 && s < nSeg) layer[s] = 1;
      }
    });
    var order = [];
    for (var s0 = 0; s0 < nSeg; s0++) if (!layer[s0]) order.push(s0);
    for (var s1 = 0; s1 < nSeg; s1++) if (layer[s1]) order.push(s1);
    return order;
  }

  // ---------- the rope ----------

  // One slanted, corner-rounded quad from sample k to k+1.
  function segmentD(pts, tan, k, half, skew, round) {
    var n = pts.length - 1;
    var corners = [];
    [[k, 1], [k + 1, 1], [k + 1, -1], [k, -1]].forEach(function (c) {
      var i = c[0], side = c[1];
      var s = (i === 0 || i === n) ? 0 : skew;   // square ends
      var t = tan[i], nx = -t[1], ny = t[0];
      corners.push([
        pts[i][0] + side * (nx * half + t[0] * s),
        pts[i][1] + side * (ny * half + t[1] * s)
      ]);
    });
    var d = '';
    for (var c = 0; c < 4; c++) {
      var P = corners[c], prev = corners[(c + 3) % 4], next = corners[(c + 1) % 4];
      var r1 = Math.min(round / (dist(P, prev) || 1), 0.5);
      var r2 = Math.min(round / (dist(P, next) || 1), 0.5);
      var a = [lerp(P[0], prev[0], r1), lerp(P[1], prev[1], r1)];
      var b = [lerp(P[0], next[0], r2), lerp(P[1], next[1], r2)];
      d += (c === 0 ? 'M' : 'L') + f1(a[0]) + ' ' + f1(a[1]) +
        'Q' + f1(P[0]) + ' ' + f1(P[1]) + ' ' + f1(b[0]) + ' ' + f1(b[1]);
    }
    return d + 'Z';
  }

  // opts.segments defaults to one segment per ~0.75 rope widths of length,
  // measured on the first draw.
  function Rope(parent, opts) {
    this.opts = opts;
    this.segments = opts.segments || 0;
    this.width = opts.width || 12;
    this.fill = opts.fill || C.orange;
    this.endFill = opts.endFill || C.navy;
    this.layered = opts.layered !== false;
    this.overFirst = opts.overFirst !== false;
    this.g = el('g', { 'class': 'rope', opacity: opts.opacity == null ? 1 : opts.opacity }, parent);
    if (opts.shadow !== false) {
      this.shadow = el('path', {
        fill: 'none', stroke: C.navy, 'stroke-opacity': 0.13, 'stroke-width': this.width + 5,
        'stroke-linecap': 'round', 'stroke-linejoin': 'round', transform: 'translate(2.5 3.5)'
      }, this.g);
    }
    this.paths = [];
  }

  Rope.prototype.build = function (count) {
    this.segments = count;
    for (var i = 0; i < count; i++) {
      this.paths.push(el('path', {
        stroke: this.opts.stroke || C.navy, 'stroke-opacity': this.opts.strokeOpacity || 0.55,
        'stroke-width': 1.1, 'stroke-linejoin': 'round'
      }, this.g));
    }
  };

  // Draw the rope along control points; returns the resampled centre line.
  Rope.prototype.draw = function (ctrl) {
    var dense = spline(ctrl, 10);
    if (!this.paths.length) {
      this.build(this.segments || clamp(Math.round(polyLength(dense) / (this.width * 0.75)), 12, 140));
    }
    var pts = resample(dense, this.segments + 1);
    var tan = tangents(pts);
    var half = this.width / 2;
    var order = this.layered ? paintOrder(pts, this.overFirst) : pts.slice(1).map(function (_, i) { return i; });
    if (this.shadow) this.shadow.setAttribute('d', pathD(pts));
    for (var m = 0; m < order.length; m++) {
      var k = order[m];
      var path = this.paths[m];
      path.setAttribute('d', segmentD(pts, tan, k, half, half * 0.55, half * 0.7));
      path.setAttribute('fill', (k === 0 || k === order.length - 1) ? this.endFill : this.fill);
    }
    this.points = pts;
    this.tangents = tan;
    return pts;
  };

  Rope.prototype.at = function (u) {
    var i = Math.round(clamp(u, 0, 1) * (this.points.length - 1));
    return { p: this.points[i], t: this.tangents[i] };
  };

  // ---------- shared props ----------

  // Parallel-jaw gripper seen from above; local x runs along the rope.
  function Gripper(parent) {
    this.g = el('g', { 'class': 'gripper' }, parent);
    this.bar = el('rect', { x: -3, y: -16, width: 6, height: 32, rx: 3, fill: C.navy }, this.g);
    this.f1 = el('rect', { x: -7, width: 14, height: 6, rx: 2, fill: C.navy }, this.g);
    this.f2 = el('rect', { x: -7, width: 14, height: 6, rx: 2, fill: C.navy }, this.g);
    el('circle', { r: 3.2, fill: C.orange, stroke: '#fff', 'stroke-width': 1.2 }, this.g);
  }

  Gripper.prototype.set = function (x, y, angleDeg, open, opacity) {
    var gap = lerp(7.5, 15, open);
    this.f1.setAttribute('y', f1(-gap - 6));
    this.f2.setAttribute('y', f1(gap));
    this.bar.setAttribute('y', f1(-gap - 6));
    this.bar.setAttribute('height', f1(2 * gap + 12));
    this.g.setAttribute('transform', 'translate(' + f1(x) + ' ' + f1(y) + ') rotate(' + f1(angleDeg) + ')');
    this.g.setAttribute('opacity', opacity == null ? 1 : f1(opacity * 100) / 100);
  };

  function label(parent, x, y, text, opts) {
    opts = opts || {};
    var t = el('text', {
      x: x, y: y, fill: opts.color || C.navy, 'font-family': 'Roboto, sans-serif',
      'font-size': opts.size || 13, 'font-weight': opts.weight || 500, 'letter-spacing': '.04em'
    }, parent);
    t.textContent = text;
    return t;
  }

  function swatch(parent, x, y, color, text, opacity) {
    el('rect', { x: x, y: y - 8, width: 16, height: 8, rx: 4, fill: color, opacity: opacity || 1 }, parent);
    return label(parent, x + 22, y, text, { size: 12 });
  }

  // ---------- rope shapes ----------

  // Loose overhand knot: an open trefoil whose ends leave over the top lobe.
  // `pull` drags the right tail outwards (0..1).
  function overhand(cx, cy, s, phase, pull) {
    var d = 0.35;
    var knot = [];
    for (var i = 0; i <= 64; i++) {
      var t = 3 * Math.PI - d - (2 * Math.PI - 2 * d) * i / 64;
      var w = 1 + 0.025 * Math.sin(phase + t * 1.5);
      knot.push([cx + s * w * (Math.sin(t) + 2 * Math.sin(2 * t)), cy + s * w * (Math.cos(t) - 2 * Math.cos(2 * t))]);
    }
    var ty = cy - 3.75 * s;
    var wave = function (k) { return 3 * Math.sin(phase * 1.3 + k); };
    var px = (pull || 0) * 20, py = (pull || 0) * -10;
    var left = [[cx - 160, ty + 12 + wave(0)], [cx - 120, ty + 2 + wave(1)], [cx - 80, ty + 3 + wave(2)], [cx - 1.6 * s, ty + 0.45 * s]];
    var right = [[cx + 1.6 * s + px * 0.3, ty + 0.45 * s + py * 0.3], [cx + 80 + px * 0.7, ty + 3 + wave(3) + py * 0.7],
      [cx + 120 + px, ty + 2 + wave(4) + py], [cx + 160 + px, ty + 12 + wave(5) + py]];
    return left.concat(resample(knot, 20)).concat(right);
  }

  // A rope with one loop (prolate cycloid), tails flattening out to the sides.
  function looped(cx, cy, k, b, tailLift) {
    var out = [];
    var T = 1.5 * Math.PI;
    for (var i = 0; i <= 40; i++) {
      var t = -T + 2 * T * i / 40;
      var lift = t > 0 ? tailLift * Math.pow(t / T, 3) : 0;
      out.push([cx + k * (t - b * Math.sin(t)), cy - k * b * Math.cos(t) + lift]);
    }
    return out;
  }

  // Letter-shaped targets for shape matching, all of roughly equal length.
  function letterShapes() {
    var S = join(arc(200, 82, 40, -20, -270, 30), arc(200, 162, 40, -90, 160, 30));
    var U = join(line([140, 60], [140, 140], 12), arc(200, 140, 60, 180, 0, 30), line([260, 140], [260, 60], 12));
    var J = join(line([187, 30], [240, 30], 8), line([240, 30], [240, 150], 16), arc(195, 150, 45, 0, 180, 26), line([150, 150], [150, 115], 6));
    return [['S', S], ['U', U], ['J', J]].map(function (e) { return { name: e[0], pts: resample(e[1], 48) }; });
  }

  // ---------- scenes ----------

  var SCENES = {};

  // ForwardDLO: bimanual shape matching, morphing between target letters.
  SCENES['shape-matching'] = function (svg) {
    var shapes = letterShapes();
    var target = el('path', {
      fill: 'none', stroke: C.blue, 'stroke-width': 2, 'stroke-dasharray': '5 6',
      'stroke-linecap': 'round', opacity: 0.85
    }, svg);
    var rope = new Rope(svg, { width: 12, layered: false });
    var gl = new Gripper(svg), gr = new Gripper(svg);
    var caption = label(svg, 14, 22, '');
    var HOLD = 1.3, MOVE = 2.2, CYCLE = HOLD + MOVE;
    return function (time) {
      var idx = Math.floor(time / CYCLE) % shapes.length;
      var local = time % CYCLE;
      var from = shapes[idx], to = shapes[(idx + 1) % shapes.length];
      var moving = local > HOLD;
      var u = moving ? ease((local - HOLD) / MOVE) : 0;
      var goal = moving ? to : from;
      target.setAttribute('d', pathD(goal.pts));
      caption.textContent = 'TARGET SHAPE: ' + goal.name;
      rope.draw(lerpPoly(from.pts, to.pts, u));
      var show = moving ? Math.sin(Math.PI * clamp((local - HOLD) / MOVE, 0, 1)) : 0;
      [[gl, 0.22], [gr, 0.74]].forEach(function (g) {
        var a = rope.at(g[1]);
        g[0].set(a.p[0], a.p[1], Math.atan2(a.t[1], a.t[0]) * 180 / Math.PI, 0, clamp(show * 1.6, 0, 1));
      });
    };
  };

  // AssistDLO: the operator's command (dashed ring) is pulled into a
  // Gaussian funnel around a rope point while the gripper aligns its yaw.
  SCENES.assist = function (svg) {
    var halo = el('path', {
      fill: 'none', stroke: C.blue, 'stroke-opacity': 0.08, 'stroke-width': 28,
      'stroke-linecap': 'round', 'stroke-linejoin': 'round'
    }, svg);
    var funnel = el('g', {}, svg);
    [30, 20, 11].forEach(function (r, i) {
      el('circle', { r: r, fill: C.blue, 'fill-opacity': 0.1 + i * 0.07 }, funnel);
    });
    var rope = new Rope(svg, { width: 11 });
    var cmd = el('circle', { r: 9, fill: 'none', stroke: C.blue, 'stroke-width': 2, 'stroke-dasharray': '3 3' }, svg);
    var trail = el('path', { fill: 'none', stroke: C.navy, 'stroke-width': 1.5, 'stroke-dasharray': '2 4', opacity: 0.6 }, svg);
    var grip = new Gripper(svg);
    label(svg, 14, 22, 'SA-CBF GRASP ASSISTANCE');
    swatch(svg, 14, 238, C.blue, 'operator command', 0.9);
    swatch(svg, 160, 238, C.navy, 'assisted gripper');
    var CYCLE = 5.2;
    return function (time) {
      var pts = rope.draw(overhand(200, 160, 26, time * 1.2, 0));
      halo.setAttribute('d', pathD(pts));
      var tgt = rope.at(0.58);
      var yaw = Math.atan2(tgt.t[1], tgt.t[0]) * 180 / Math.PI;
      funnel.setAttribute('transform', 'translate(' + f1(tgt.p[0]) + ' ' + f1(tgt.p[1]) + ')');
      var local = time % CYCLE;
      var u = ease(local / 2.6);
      var start = [362, 150];
      var aim = [tgt.p[0] + 16, tgt.p[1] - 12];
      var c = [lerp(start[0], aim[0], u), lerp(start[1], aim[1], u)];
      // The assistance blends in as the command enters the funnel.
      var pullIn = ease((u - 0.45) / 0.55);
      var gx = lerp(c[0], tgt.p[0], pullIn), gy = lerp(c[1], tgt.p[1], pullIn);
      var close = ease((local - 2.7) / 0.6);
      var fade = 1 - ease((local - 4.5) / 0.6);
      cmd.setAttribute('cx', f1(c[0]));
      cmd.setAttribute('cy', f1(c[1]));
      cmd.setAttribute('opacity', f1(fade * 100) / 100);
      trail.setAttribute('d', pathD([start, [lerp(start[0], gx, 0.5), lerp(start[1], gy, 0.5) - 6], [gx, gy]]));
      trail.setAttribute('opacity', f1(0.6 * fade * 100) / 100);
      grip.set(gx, gy, lerp(-30, yaw, pullIn), 1 - close, fade);
    };
  };

  // RopeDreamer: observed rope (orange) and a learned model's rollout (blue).
  SCENES.loop = function (svg) {
    var ghost = new Rope(svg, { width: 11, fill: C.blue, endFill: C.blue, opacity: 0.45, shadow: false });
    var rope = new Rope(svg, { width: 11 });
    var grip = new Gripper(svg);
    label(svg, 14, 22, 'LONG-HORIZON PREDICTION');
    swatch(svg, 14, 238, C.orange, 'observed');
    swatch(svg, 112, 238, C.blue, 'predicted rollout', 0.6);
    function shape(t) {
      var b = 2.45 + 0.3 * Math.sin(t * 0.9);
      return looped(196, 118, 21, b, 18 * Math.sin(t * 0.9 + 1));
    }
    return function (time) {
      ghost.draw(shape(time - 0.45).map(function (p) { return [p[0] + 3, p[1] + 4]; }));
      rope.draw(shape(time));
      var end = rope.at(1);
      grip.set(end.p[0] - end.t[0] * 6, end.p[1] - end.t[1] * 6, Math.atan2(end.t[1], end.t[0]) * 180 / Math.PI, 0, 1);
    };
  };

  // Sim-grounded policies: one snapshot is turned into a particle state,
  // then the policy's grasp-and-pull macro action loosens the knot.
  SCENES.particles = function (svg) {
    var rope = new Rope(svg, { width: 11 });
    var dots = el('g', {}, svg);
    var particles = [];
    for (var i = 0; i < 24; i++) {
      particles.push(el('circle', { r: 2.6, fill: C.navy, stroke: '#fff', 'stroke-width': 1 }, dots));
    }
    var grip = new Gripper(svg);
    var flash = el('rect', { x: 0, y: 0, width: 400, height: 250, fill: '#fff', opacity: 0 }, svg);
    var caption = label(svg, 14, 22, '');
    var CYCLE = 6.4;
    return function (time) {
      var local = time % CYCLE;
      var pull = ease((local - 1.6) / 3.2);
      var pts = rope.draw(overhand(200, 160, 26, 0.6, pull));
      var show = ease((local - 0.5) / 0.6) * (1 - ease((local - 5.6) / 0.6));
      dots.setAttribute('opacity', f1(show * 100) / 100);
      particles.forEach(function (c, k) {
        var p = pts[Math.round(k * (pts.length - 1) / (particles.length - 1))];
        c.setAttribute('cx', f1(p[0]));
        c.setAttribute('cy', f1(p[1]));
      });
      flash.setAttribute('opacity', f1(0.75 * (1 - ease(local / 0.5)) * 100) / 100);
      caption.textContent = local < 1.6 ? 'SNAPSHOT → PARTICLE STATE' : 'MACRO ACTION: GRASP & PULL';
      var end = rope.at(0.97);
      grip.set(end.p[0], end.p[1], Math.atan2(end.t[1], end.t[0]) * 180 / Math.PI, 1 - ease((local - 1.2) / 0.4), ease((local - 1.0) / 0.4) * (1 - ease((local - 5.6) / 0.6)));
    };
  };

  // ---------- run ----------

  var reduceMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var active = [];

  function start() {
    var nodes = document.querySelectorAll('svg[data-rope]');
    if (!nodes.length) return;
    var instances = [];
    Array.prototype.forEach.call(nodes, function (svg) {
      var make = SCENES[svg.getAttribute('data-rope')];
      if (!make) return;
      var inst = { update: make(svg), visible: false, offset: Math.random() * 3 };
      inst.update(1.2 + inst.offset);
      instances.push(inst);
      if (!reduceMotion && 'IntersectionObserver' in window) {
        new IntersectionObserver(function (entries) {
          inst.visible = entries[0].isIntersecting;
          var i = active.indexOf(inst);
          if (inst.visible && i < 0) active.push(inst);
          if (!inst.visible && i >= 0) active.splice(i, 1);
        }).observe(svg);
      }
    });
    if (reduceMotion) return;
    var t0 = performance.now();
    (function frame(now) {
      var t = (now - t0) / 1000;
      for (var i = 0; i < active.length; i++) active[i].update(t + 1.2 + active[i].offset);
      requestAnimationFrame(frame);
    })(t0);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();

  // Exposed for tests and for reuse.
  window.RopeDraw = { Rope: Rope, scenes: SCENES, paintOrder: paintOrder, overhand: overhand, looped: looped, spline: spline, resample: resample };
})();
