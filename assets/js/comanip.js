/* Push-and-park demo for the M.Sc. thesis page.
 *
 * An illustrative re-implementation of the thesis simulation's admittance
 * model (KocUniversity/pHRI-simulation, simulation_new.py): a 1-DoF virtual
 * mass-damper driven by the scaled human force, followed by the identified
 * velocity loop of the real robot. Two parts are stand-ins and labelled as
 * such on the page: intent comes from a force heuristic instead of the
 * thesis's neural network, and the "simulated human" is a minimum-jerk
 * approximation instead of the study's learned human model.
 *
 * Usage: an element with [data-comanip] containing the markup from
 * _includes/comanip_demo.html.
 */
(function () {
  'use strict';

  // ---------- model (units: m, m/s, N) ----------

  var M = 40, B = 100, DT = 1 / 125;            // virtual mass, damping, 125 Hz
  var C1 = (1 / B) * (1 - Math.exp(-(B / M) * DT));
  var C2 = Math.exp(-(B / M) * DT);
  var G_NOM = 1, G_MIN = 0.33, G_MAX = 2.68;     // force-scaling gain range
  var ALPHA = 3;                                 // gain integration constant
  var LP = 1 - Math.exp(-2 * Math.PI * 3 * DT);  // 3 Hz low-pass on the intent signal
  var D = 0.40;                                  // reach distance
  var WIDTHS = { wide: 0.40 / 15, narrow: 0.40 / 63 };   // W = D / (2^ID - 1), ID = 4, 6

  function Model() { this.reset(); }

  Model.prototype.reset = function () {
    this.vRef = [0, 0, 0, 0, 0];   // v_ref[k], v_ref[k-1], ...
    this.vAct = [0, 0, 0, 0];      // v_act[k], v_act[k-1], ...
    this.fIntPrev = 0;
    this.x = 0;
    this.k = 0;
    this.kiiInt = 0;
    this.kii = 0;
    this.gain = G_NOM;
  };

  // One 125 Hz step. fH: human force; gainMode: number (fixed) or 'adaptive'.
  Model.prototype.step = function (fH, gainMode, rate) {
    if (gainMode === 'adaptive') {
      // Stand-in for the intent network's p_accel - p_decel: the commanded
      // acceleration m*dv/dt = G_H*F_h - b*v, normalised and low-passed.
      var kii = Math.tanh((this.gain * fH - B * this.vRef[0]) / 8);
      this.kii += LP * (kii - this.kii);
      this.kiiInt += rate * this.kii * ALPHA * DT;
      var s = -1 + 2 / (1 + Math.exp(-this.kiiInt));
      this.gain = G_NOM + (s > 0 ? G_MAX - G_NOM : G_NOM - G_MIN) * s;
    } else {
      this.gain = gainMode;
    }
    // Admittance (exact ZOH), driven by the previous step's force.
    var vRef = C1 * this.fIntPrev + C2 * this.vRef[0];
    this.fIntPrev = this.gain * fH;
    this.vRef.unshift(vRef); this.vRef.length = 5;
    // Identified velocity loop of the robot.
    var r = this.vRef, a = this.vAct;
    var vAct = 0.3443 * r[3] + 0.2995 * r[4] - 0.1151 * a[0] + 0.4569 * a[1] + 0.01417 * a[2];
    this.vAct.unshift(vAct); this.vAct.length = 4;
    this.x += vAct * DT;
    this.k++;
    return vAct;
  };

  // Minimum-jerk reference from 0 to D over T seconds.
  function minJerk(t, T) {
    var u = Math.min(Math.max(t / T, 0), 1);
    var u3 = u * u * u;
    return {
      x: D * (10 * u3 - 15 * u3 * u + 6 * u3 * u * u),
      v: D / T * (30 * u * u - 60 * u3 + 30 * u3 * u),
      a: D / (T * T) * (60 * u - 180 * u * u + 120 * u3)
    };
  }

  // ---------- page widget ----------

  var NS = 'http://www.w3.org/2000/svg';
  var PX0 = 70, PX_PER_M = 1000;                 // rail mapping in the SVG
  function px(x) { return PX0 + x * PX_PER_M; }

  function Demo(root) {
    this.root = root;
    this.svg = root.querySelector('.comanip-svg');
    this.model = new Model();
    this.human = 'you';
    this.gainMode = G_NOM;
    this.rate = 1;
    this.width = 'wide';
    this.keyForce = 0;
    this.pointerX = null;
    this.running = false;
    this.visible = false;
    this.bind();
    this.resetTrial();
    this.render();
  }

  Demo.prototype.q = function (sel) { return this.root.querySelector(sel); };

  Demo.prototype.bind = function () {
    var self = this;
    this.root.addEventListener('click', function (e) {
      var b = e.target.closest('button');
      if (!b) return;
      var group = b.closest('[data-group]');
      if (group) {
        group.querySelectorAll('button').forEach(function (o) { o.setAttribute('aria-pressed', String(o === b)); });
        var key = group.getAttribute('data-group'), val = b.getAttribute('data-value');
        if (key === 'human') self.human = val;
        if (key === 'gain') self.gainMode = val === 'adaptive' ? 'adaptive' : parseFloat(val);
        if (key === 'rate') self.rate = parseFloat(val);
        if (key === 'width') self.width = val;
        self.root.classList.toggle('is-adaptive', self.gainMode === 'adaptive');
        self.resetTrial();
      }
      if (b.hasAttribute('data-reset')) self.resetTrial();
    });

    var stage = this.q('.comanip-stage');
    function toModelX(ev) {
      var pt = self.svg.createSVGPoint();
      pt.x = ev.clientX; pt.y = ev.clientY;
      var loc = pt.matrixTransform(self.svg.getScreenCTM().inverse());
      return (loc.x - PX0) / PX_PER_M;
    }
    stage.addEventListener('pointerdown', function (ev) {
      if (self.human !== 'you') return;
      stage.setPointerCapture(ev.pointerId);
      self.pointerX = toModelX(ev);
      self.start();
    });
    stage.addEventListener('pointermove', function (ev) {
      if (self.pointerX !== null) self.pointerX = toModelX(ev);
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(function (t) {
      stage.addEventListener(t, function () { self.pointerX = null; });
    });
    stage.addEventListener('keydown', function (ev) {
      if (self.human !== 'you') return;
      if (ev.key === 'ArrowRight' || ev.key === 'ArrowLeft') {
        self.keyForce = ev.key === 'ArrowRight' ? 15 : -15;
        self.start();
        ev.preventDefault();
      }
    });
    stage.addEventListener('keyup', function (ev) {
      if (ev.key === 'ArrowRight' || ev.key === 'ArrowLeft') self.keyForce = 0;
    });

    if ('IntersectionObserver' in window) {
      new IntersectionObserver(function (entries) {
        self.visible = entries[0].isIntersecting;
        if (self.visible) self.loop();
      }).observe(this.root);
    } else {
      this.visible = true;
      this.loop();
    }
  };

  Demo.prototype.resetTrial = function () {
    this.model.reset();
    this.t = 0;
    this.started = false;
    this.parkedFor = 0;
    this.done = false;
    this.effort = 0;
    this.jerk = 0;
    this.prevAcc = 0;
    this.fH = 0;
    this.trace = [];
    this.status('Ready. ' + (this.human === 'you'
      ? 'Drag on the rail (or hold ← →) to push the object into the target.'
      : 'Press "Run" to watch the simulated human.'), '');
    this.q('[data-run]').hidden = this.human === 'you';
    this.drawTarget();
    this.render();
  };

  Demo.prototype.start = function () {
    if (!this.started) this.started = true;
    this.loop();
  };

  Demo.prototype.status = function (text, cls) {
    var el = this.q('.comanip-status');
    el.textContent = text;
    el.className = 'comanip-status' + (cls ? ' is-' + cls : '');
  };

  Demo.prototype.humanForce = function () {
    var m = this.model;
    if (this.human === 'sim') {
      // Minimum-jerk plan, turned into force through the nominal (G = 1)
      // admittance, plus a PD correction. The simulated human does not know
      // the current gain, just like a person who expects the nominal robot.
      var r = minJerk(this.t, 1.8);
      var f = M * r.a + B * r.v + 400 * (r.x - m.x) + 60 * (r.v - m.vAct[0]);
      return Math.max(-60, Math.min(60, f));
    }
    if (this.pointerX !== null) {
      return Math.max(-40, Math.min(40, 400 * (this.pointerX - m.x)));
    }
    return this.keyForce;
  };

  Demo.prototype.loop = function () {
    if (this.raf) return;
    var self = this, last = performance.now(), acc = 0;
    function frame(now) {
      acc += Math.min(now - last, 100) / 1000;
      last = now;
      while (acc >= DT) { self.tick(); acc -= DT; }
      self.render();
      var active = self.started && !self.done;
      if (self.visible && (active || self.pointerX !== null || self.keyForce)) {
        self.raf = requestAnimationFrame(frame);
      } else {
        self.raf = null;
      }
    }
    this.raf = requestAnimationFrame(frame);
  };

  Demo.prototype.tick = function () {
    if (!this.started || this.done) return;
    var m = this.model;
    var f = this.humanForce();
    var vPrev = m.vAct[0];
    var v = m.step(f, this.gainMode, this.rate);
    var acc = (v - vPrev) / DT;
    if (m.k > 5) this.jerk += Math.abs(acc - this.prevAcc);   // ∫|da/dt| dt
    this.prevAcc = acc;
    this.effort += Math.abs(f * v) * DT;
    this.fH = f;
    this.t += DT;
    if (m.k % 4 === 0) this.trace.push([this.t, m.x, m.gain]);

    var half = WIDTHS[this.width] / 2;
    var inside = Math.abs(m.x - D) <= half;
    this.parkedFor = inside && Math.abs(v) < 0.005 ? this.parkedFor + DT : 0;
    if (this.parkedFor >= 0.4) {
      this.done = true;
      this.fH = 0;
      this.status('Parked in ' + this.t.toFixed(2) + ' s · effort ' + this.effort.toFixed(2) + ' J', 'ok');
    } else if (m.x > D + 0.08) {
      this.done = true;
      this.status('Overshot the target. Try a lower gain, or brake earlier.', 'bad');
    } else if (this.t > 12) {
      this.done = true;
      this.status('Time is up (12 s). Press Reset to try again.', 'bad');
    } else if (this.human === 'you') {
      this.status(inside ? 'Inside the target, now hold it still…' : 'Pushing…', '');
    }
  };

  Demo.prototype.drawTarget = function () {
    var w = WIDTHS[this.width];
    var zone = this.q('.cm-zone');
    zone.setAttribute('x', px(D - w / 2).toFixed(1));
    zone.setAttribute('width', Math.max(w * PX_PER_M, 1.5).toFixed(1));
    this.q('.cm-zone-label').textContent = 'target ±' + (w * 500).toFixed(1) + ' mm';
  };

  Demo.prototype.render = function () {
    var m = this.model;
    var bx = px(m.x);
    this.q('.cm-object').setAttribute('transform', 'translate(' + bx.toFixed(1) + ' 0)');
    // Force arrow on the handle.
    var f = this.fH;
    var arrow = this.q('.cm-force');
    var len = Math.max(-90, Math.min(90, f * 2.2));
    arrow.setAttribute('d', 'M' + (bx - 52).toFixed(1) + ' 92h' + len.toFixed(1));
    arrow.setAttribute('opacity', Math.abs(f) > 0.5 ? 1 : 0);
    var spring = this.q('.cm-spring');
    if (this.pointerX !== null) {
      spring.setAttribute('d', 'M' + (bx - 46).toFixed(1) + ' 92L' + px(this.pointerX).toFixed(1) + ' 92');
      spring.setAttribute('opacity', 1);
    } else {
      spring.setAttribute('opacity', 0);
    }
    // Readout.
    var set = function (sel, txt) { this.q(sel).textContent = txt; }.bind(this);
    set('[data-out="gain"]', m.gain.toFixed(2));
    set('[data-out="force"]', f.toFixed(1) + ' N');
    set('[data-out="vel"]', (m.vAct[0] * 100).toFixed(1) + ' cm/s');
    set('[data-out="err"]', ((m.x - D) * 1000).toFixed(1) + ' mm');
    set('[data-out="effort"]', this.effort.toFixed(2) + ' J');
    set('[data-out="time"]', this.t.toFixed(2) + ' s');
    this.q('.cm-gain-fill').style.width = ((m.gain - G_MIN) / (G_MAX - G_MIN) * 100).toFixed(1) + '%';
    this.plot();
  };

  // Position and gain over the trial, in the small chart under the rail.
  Demo.prototype.plot = function () {
    var tr = this.trace;
    var T = Math.max(4, this.t);
    var xs = tr.map(function (p) { return (p[0] / T * 600).toFixed(1) + ',' + (60 - p[1] / 0.5 * 56).toFixed(1); });
    var gs = tr.map(function (p) { return (p[0] / T * 600).toFixed(1) + ',' + (60 - (p[2] - G_MIN) / (G_MAX - G_MIN) * 56).toFixed(1); });
    this.q('.cm-plot-x').setAttribute('points', xs.join(' '));
    this.q('.cm-plot-g').setAttribute('points', gs.join(' '));
    var w = WIDTHS[this.width];
    var band = this.q('.cm-plot-band');
    band.setAttribute('y', (60 - (D + w / 2) / 0.5 * 56).toFixed(1));
    band.setAttribute('height', Math.max(w / 0.5 * 56, 1).toFixed(1));
  };

  function init() {
    Array.prototype.forEach.call(document.querySelectorAll('[data-comanip]'), function (root) {
      var demo = new Demo(root);
      var run = root.querySelector('[data-run]');
      if (run) run.addEventListener('click', function () { demo.resetTrial(); demo.start(); });
    });
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }
  if (typeof window !== 'undefined') window.CoManip = { Model: Model, minJerk: minJerk, constants: { M: M, B: B, DT: DT, C1: C1, C2: C2 } };
  if (typeof module !== 'undefined') module.exports = { Model: Model, minJerk: minJerk, C1: C1, C2: C2 };
})();
