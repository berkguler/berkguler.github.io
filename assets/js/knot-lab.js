/* Knot Lab: records the knot game for research and keeps a leaderboard.
 *
 * Works with the game in assistdlo-game.js (a [data-dlo-game][data-lab]
 * element), which deals a shuffled, verified knot each round and reports
 * what happens through knot:new / knot:action / knot:frame / knot:finish
 * events. Players sign in with a username and a 4-digit PIN (no email).
 * Successful runs are encoded (see tools/knot-lab/README.md) and saved
 * through the Supabase functions in tools/knot-lab/schema.sql; failed or
 * skipped runs are thrown away. Without a configured backend the game is
 * still playable but nothing is saved.
 */
(function () {
  'use strict';

  var RATE = 20;          // samples per second
  var RES = 20;           // rope positions are stored in 1/RES px
  var MAX_B64 = 1800000;  // above this, rope frames are thinned out
  var PHASES = { idle: 0, travel: 1, descend: 2, close: 3, carry: 4, lift: 5 };
  var STORE = 'knotLab.';

  function store(kind) {
    try { return window[kind]; } catch (e) { return null; }
  }
  function get(kind, key) {
    try { var s = store(kind); return s ? s.getItem(STORE + key) : null; } catch (e) { return null; }
  }
  function put(kind, key, value) {
    try { var s = store(kind); if (s) { if (value === null) s.removeItem(STORE + key); else s.setItem(STORE + key, value); } } catch (e) { /* storage full or blocked */ }
  }

  // ---------- recording ----------

  function Recorder(game) {
    this.game = game;
    this.reset();
  }

  Recorder.prototype.reset = function () {
    this.t = []; this.grip = []; this.hand = []; this.rope = []; this.events = []; this.table = null;
    this.next = 0;
    this.device = 'mouse';
  };

  // One sample of the whole scene at game time t.
  Recorder.prototype.sample = function (t) {
    var g = this.game, s = g.sim, gr = g.g, n = s.n, r = new Int16Array(3 * n);
    if (!this.table) this.table = s.p.box.slice();   // the table when recording starts
    for (var i = 0; i < n; i++) {
      r[3 * i] = Math.round(s.x[i] * RES);
      r[3 * i + 1] = Math.round(s.y[i] * RES);
      r[3 * i + 2] = Math.round(s.z[i] * RES);
    }
    this.t.push(t);
    this.grip.push(gr.x, gr.y, gr.z, gr.yaw, gr.open ? 1 : 0, s.grab, PHASES[g.phase] || 0);
    var p = g.ptr || [gr.x, gr.y];
    this.hand.push(p[0], p[1], g.held ? 1 : 0, g.hover ? 1 : 0);
    this.rope.push(r);
  };

  // Sample at the first frame on or after each 1/RATE tick; after a stall the
  // missed ticks are skipped rather than bunched up. `t` keeps the exact times.
  Recorder.prototype.frame = function (t) {
    if (t + 1e-9 < this.next) return;
    this.sample(t);
    this.next = Math.floor(t * RATE + 1e-6) / RATE + 1 / RATE;
  };

  Recorder.prototype.action = function (d) {
    var g = this.game;
    if (d.type === 'table' && !this.t.length) return;   // before the first press: the header's table has it
    if (d.type === 'press' && !this.t.length) { this.sample(0); this.next = 1 / RATE; }
    var e = { t: +g.t.toFixed(3), type: d.type };
    ['x', 'y'].forEach(function (k) { if (d[k] != null) e[k] = +d[k].toFixed(2); });
    if (d.node != null) e.node = d.node;
    if (d.box) e.box = d.box;   // 'table': the table changed size (screen turned, full screen)
    this.events.push(e);
  };

  // The recording as one binary file (format in tools/knot-lab/README.md),
  // keeping every `every`-th rope frame.
  Recorder.prototype.pack = function (result, every) {
    var g = this.game, T = this.t.length, n = g.sim.n, keep = [];
    for (var k = 0; k < T; k += every) keep.push(k);
    if (keep[keep.length - 1] !== T - 1) keep.push(T - 1);
    var header = {
      format: 'knot-lab/2', rate: RATE, res: RES, table: this.table, n: n, samples: T, ropeFrames: keep,
      round: g.round, params: g.sim.p, events: this.events, result: result, device: this.device
    };
    var json = new TextEncoder().encode(JSON.stringify(header));
    var pad = (4 - ((8 + json.length) % 4)) % 4;
    var size = 8 + json.length + pad + 4 * T + 4 * 7 * T + 4 * 4 * T + 2 * 3 * n * keep.length;
    var buf = new ArrayBuffer(size), bytes = new Uint8Array(buf), view = new DataView(buf);
    bytes.set([75, 78, 84, 49]);   // "KNT1"
    view.setUint32(4, json.length, true);
    bytes.set(json, 8);
    var off = 8 + json.length + pad;
    var put32 = function (arr) { new Float32Array(buf, off, arr.length).set(arr); off += 4 * arr.length; };
    put32(this.t); put32(this.grip); put32(this.hand);
    // Rope: first kept frame as is, the others as differences to the frame before.
    var out = new Int16Array(buf, off, 3 * n * keep.length), prev = null;
    keep.forEach(function (k, j) {
      var fr = this.rope[k];
      for (var i = 0; i < fr.length; i++) out[j * 3 * n + i] = prev ? fr[i] - prev[i] : fr[i];
      prev = fr;
    }, this);
    return bytes;
  };

  function gzip(bytes) {
    if (typeof CompressionStream === 'undefined') return Promise.resolve(bytes);
    var stream = new Blob([bytes]).stream().pipeThrough(new CompressionStream('gzip'));
    return new Response(stream).arrayBuffer().then(function (b) { return new Uint8Array(b); });
  }

  function base64(bytes) {
    var s = '';
    for (var i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
    return btoa(s);
  }

  // Encode, thinning out rope frames until the upload is small enough.
  Recorder.prototype.encode = function (result) {
    var self = this;
    var attempt = function (every) {
      return gzip(self.pack(result, every)).then(function (z) {
        var b64 = base64(z);
        return b64.length > MAX_B64 && every < 8 ? attempt(every * 2) : { data: b64, every: every };
      });
    };
    return attempt(1);
  };

  // ---------- backend (Supabase RPC) ----------

  function Backend(url, key) {
    this.url = (url || '').replace(/\/+$/, '');
    this.key = key || '';
  }
  Backend.prototype.ready = function () { return !!(this.url && this.key); };
  Backend.prototype.call = function (fn, args) {
    var headers = { apikey: this.key, 'Content-Type': 'application/json' };
    // A legacy anon key (a JWT) also goes in Authorization; a publishable key (sb_publishable_...) must not.
    if (/^eyJ/.test(this.key)) headers.Authorization = 'Bearer ' + this.key;
    return fetch(this.url + '/rest/v1/rpc/' + fn, {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(args)
    }).then(function (res) {
      if (res.ok) return res.json();
      // Keep the server's answer (PostgREST sends {code, message}) to say what went wrong.
      return res.json().catch(function () { return {}; }).then(function (body) {
        var err = new Error('HTTP ' + res.status + (body.message ? ': ' + body.message : ''));
        err.status = res.status;
        err.code = body.code;
        throw err;
      });
    });
  };

  // What to tell the player when a call failed; the details go to the console.
  function problem(e) {
    if (window.console) console.warn('Knot Lab server:', e);
    if (!e || !e.status) return 'Could not reach the server. Check your connection and try again.';
    if (e.code === 'PGRST202' || e.status === 404) return 'The game server is not set up yet (its database functions are missing).';
    if (e.status === 401 || e.status === 403) return 'The game server did not accept this page\'s key.';
    return 'The game server had a problem (' + e.message + ').';
  }

  var ERRORS = {
    bad_username: 'Usernames are 3 to 20 letters, digits, - or _.',
    bad_pin: 'The PIN is exactly 4 digits.',
    wrong_pin: 'This username is taken, and the PIN does not match.',
    locked: 'Too many wrong PINs for this name. Try again in 15 minutes.',
    bad_login: 'Please sign in again.',
    too_fast: 'Saving too fast, retrying in a moment.',
    bad_data: 'This run could not be saved.',
    bad_meta: 'This run could not be saved.'
  };

  // ---------- music ----------

  // Background music from a YouTube video, in a small player under the game
  // (YouTube's embedding rules ask for a visible player). Browsers only allow
  // sound after the visitor has clicked or typed, so it starts then; the
  // speaker button under the game turns it off and on, and the choice is kept.
  // Nothing is loaded from YouTube while the music is off.
  function Music(root, gameRoot) {
    var self = this;
    this.id = root.getAttribute('data-music-video');
    this.card = root.querySelector('[data-lab-music]');
    this.mount = root.querySelector('[data-lab-music-player]');
    this.button = gameRoot.querySelector('[data-music]');
    this.on = get('localStorage', 'music') !== 'off';
    this.button.hidden = false;
    this.show();
    this.button.addEventListener('click', function () {
      self.on = !self.on;
      put('localStorage', 'music', self.on ? null : 'off');
      self.show();
      if (self.on) self.play(); else self.call('pauseVideo');
    });
    var first = function (ev) {
      if (self.button.contains(ev.target)) return;   // the button's own click decides
      document.removeEventListener('pointerdown', first, true);
      document.removeEventListener('keydown', first, true);
      if (self.on) self.play();
    };
    document.addEventListener('pointerdown', first, true);
    document.addEventListener('keydown', first, true);
  }

  Music.prototype.show = function () {
    var b = this.button;
    b.setAttribute('aria-pressed', String(this.on));
    b.setAttribute('aria-label', 'Music');
    b.title = this.on ? 'Mute music' : 'Play music';
    b.querySelector('i').className = 'fa-solid ' + (this.on ? 'fa-volume-high' : 'fa-volume-xmark');
  };

  Music.prototype.call = function (fn) {
    try { if (this.player && typeof this.player[fn] === 'function') this.player[fn](); } catch (e) { /* not ready yet */ }
  };

  Music.prototype.fail = function () {
    this.failed = true;
    this.card.hidden = true;
    this.button.hidden = true;
  };

  Music.prototype.play = function () {
    var self = this;
    if (this.failed) return;
    if (this.player) { this.call('playVideo'); return; }   // until it is ready, onReady starts it
    this.card.hidden = false;
    var make = function () {
      self.player = new window.YT.Player(self.mount, {
        host: 'https://www.youtube-nocookie.com',
        videoId: self.id, width: 356, height: 200,
        playerVars: { loop: 1, playlist: self.id, playsinline: 1, rel: 0 },
        events: {
          onReady: function (e) { e.target.setVolume(35); if (self.on) e.target.playVideo(); },
          onStateChange: function (e) { if (e.data === 0 && self.on) { e.target.seekTo(0); e.target.playVideo(); } },
          onError: function () { self.fail(); }   // e.g. the video does not allow embedding
        }
      });
    };
    if (window.YT && window.YT.Player) { make(); return; }
    if (this.loading) return;
    this.loading = true;
    var before = window.onYouTubeIframeAPIReady;
    window.onYouTubeIframeAPIReady = function () { if (before) before(); make(); };
    var script = document.createElement('script');
    script.src = 'https://www.youtube.com/iframe_api';
    script.onerror = function () { self.fail(); };   // blocked or offline
    document.head.appendChild(script);
  };

  // ---------- page ----------

  function Lab(root) {
    var gameRoot = root.querySelector('[data-dlo-game]');
    this.root = root;
    this.game = gameRoot.game;
    this.backend = new Backend(root.getAttribute('data-supabase-url'), root.getAttribute('data-supabase-key'));
    this.rec = new Recorder(this.game);
    this.user = get('localStorage', 'user') || '';
    this.pin = get('sessionStorage', 'pin') || '';
    this.session = { runs: 0, points: 0 };
    this.bind(gameRoot);
    var cbf = gameRoot.querySelector('[data-assist="cbf"]');
    if (cbf && get('localStorage', 'assist') === 'cbf') cbf.click();
    if (root.getAttribute('data-music-video') && gameRoot.querySelector('[data-music]')) this.music = new Music(root, gameRoot);
    this.showRound(this.game.round);
    if (this.backend.ready()) {
      this.q('[data-lab-user]').value = this.user;
      if (this.user && this.pin) this.login(this.user, this.pin, true); else this.signedOut();
      this.loadBoard();
    } else {
      this.q('[data-lab-login]').hidden = true;
      this.q('[data-lab-offline]').hidden = false;
      this.q('[data-lab-board-wrap]').hidden = true;
    }
    // First visit: start with the tutorial (it can be played before signing in).
    if (!this.game.tutorialSeen()) this.game.startTutorial();
  }

  Lab.prototype.q = function (s) { return this.root.querySelector(s); };

  Lab.prototype.bind = function (gameRoot) {
    var self = this;
    var next = gameRoot.querySelector('[data-new]');
    var label = function (text) { next.lastChild.textContent = text; };
    // The tutorial's practice knot is not recorded.
    var practice = function () { return self.game.round && self.game.round.tutorial; };
    gameRoot.addEventListener('knot:new', function (ev) { self.rec.reset(); self.showRound(ev.detail.round); label('Skip knot'); });
    gameRoot.addEventListener('knot:frame', function (ev) { if (!practice()) self.rec.frame(ev.detail.t); });
    gameRoot.addEventListener('knot:action', function (ev) { if (!practice()) self.rec.action(ev.detail); });
    gameRoot.addEventListener('knot:finish', function (ev) { if (!practice()) { label('Next knot'); self.finished(ev.detail); } });
    gameRoot.addEventListener('pointerdown', function (ev) { self.rec.device = ev.pointerType || 'mouse'; }, true);
    // Remember the assistance mode (off or SA-CBF) for the next visit.
    gameRoot.addEventListener('click', function (ev) {
      var b = ev.target.closest && ev.target.closest('[data-assist]');
      if (b) put('localStorage', 'assist', b.getAttribute('data-assist'));
    });
    // Signed out and playing in full screen: back to the page to sign in.
    gameRoot.addEventListener('knot:locked', function () {
      if (!self.game.full) return;
      var input = self.q('[data-lab-user]'), done = false;
      var go = function () {
        if (done) return;
        done = true;
        input.focus({ preventScroll: true });
        input.scrollIntoView({ block: 'center' });
      };
      self.game.toggleFull(false);
      // Focus once the browser has left its full screen (it takes a moment).
      if (document.fullscreenElement || document.webkitFullscreenElement) {
        document.addEventListener('fullscreenchange', go, { once: true });
        document.addEventListener('webkitfullscreenchange', go, { once: true });
        setTimeout(go, 600);
      } else {
        setTimeout(go, 0);
      }
    });
    // No text selection anywhere on the page outside the form fields (the
    // game root already blocks it, along with its context menu).
    document.addEventListener('selectstart', function (ev) {
      var el = ev.target.nodeType === 3 ? ev.target.parentNode : ev.target;
      if (!(el && el.closest && el.closest('input, textarea'))) ev.preventDefault();
    });
    this.q('[data-lab-login]').addEventListener('submit', function (ev) {
      ev.preventDefault();
      self.login(self.q('[data-lab-user]').value.trim(), self.q('[data-lab-pin]').value.trim(), false);
    });
    this.q('[data-lab-logout]').addEventListener('click', function () {
      self.user = ''; self.pin = '';
      put('sessionStorage', 'pin', null);
      self.signedOut();
    });
  };

  Lab.prototype.showRound = function (r) {
    if (!r) return;
    if (r.tutorial) { this.q('[data-lab-round]').textContent = 'practice knot (tutorial, not recorded)'; return; }
    var stiff = r.stiffness < 0.05 ? 'limp' : r.stiffness < 0.35 ? 'soft' : r.stiffness < 0.7 ? 'medium' : 'stiff';
    this.q('[data-lab-round]').textContent = r.name + ' · ' + r.crossings + ' crossings · ' + stiff + ' · ' + (2 * r.rad) + ' mm' +
      (r.assist === 'sa-cbf' ? ' · SA-CBF' : '');
  };

  Lab.prototype.say = function (text, cls) {
    var el = this.q('[data-lab-msg]');
    el.textContent = text;
    el.className = 'lab-msg' + (cls ? ' is-' + cls : '');
    // In full screen the page is hidden, so show it on the table.
    if (this.game.full && text) this.game.flashText(text, cls === 'bad' ? '#b4451f' : cls === 'ok' ? '#2f7d4f' : '#273F4F');
  };

  Lab.prototype.signedOut = function () {
    this.q('[data-lab-login]').hidden = false;
    this.q('[data-lab-who]').hidden = true;
    this.game.locked = 'Sign in above to play';
    this.say('');
  };

  Lab.prototype.login = function (user, pin, quiet) {
    var self = this, btn = this.q('[data-lab-login] button');
    btn.disabled = true;
    if (!quiet) this.say('Signing in…');
    this.backend.call('knot_login', { p_username: user, p_pin: pin }).then(function (r) {
      btn.disabled = false;
      if (!r.ok) { self.say(ERRORS[r.error] || 'Could not sign in.', 'bad'); if (quiet) self.signedOut(); return; }
      self.user = r.username; self.pin = pin;
      self.pendingList = null;   // the queue belongs to this player
      put('localStorage', 'user', r.username);
      put('sessionStorage', 'pin', pin);
      self.q('[data-lab-login]').hidden = true;
      self.q('[data-lab-who]').hidden = false;
      self.q('[data-lab-name]').textContent = r.username;
      self.q('[data-lab-pin]').value = '';
      self.game.locked = null;
      self.say(r.new ? 'Welcome, ' + r.username + '! Your name is saved with this PIN.' : 'Welcome back, ' + r.username + '.', 'ok');
      self.flush();
      self.highlight();
    }).catch(function (e) {
      btn.disabled = false;
      self.say(problem(e), 'bad');
      if (quiet) self.signedOut();
    });
  };

  Lab.prototype.finished = function (d) {
    var self = this, rec = this.rec;
    if (!d.ok) return;            // only successful runs are kept
    if (!rec.t.length || d.t > rec.t[rec.t.length - 1] + 1e-6) rec.sample(d.t);
    if (!this.backend.ready() || !this.user) return;
    var result = { ok: true, seconds: +d.t.toFixed(2), grasps: d.grasps, misses: d.misses, timeLimit: d.timeLimit };
    this.say('Saving your run…');
    rec.encode(result).then(function (enc) {
      var r = d.round;
      var meta = {
        knot: r.knot, crossings: r.crossings, seconds: result.seconds, grasps: d.grasps, misses: d.misses,
        hints: 0,   // the game has no hints any more; the server still expects the field
        format: 'knot-lab/2', rate: RATE, ropeEvery: enc.every, n: self.game.sim.n, samples: rec.t.length,
        rad: r.rad, stiffness: +r.stiffness.toFixed(3), assist: r.assist || 'off', device: rec.device, bytes: enc.data.length
      };
      self.queue({ meta: meta, data: enc.data });
      self.flush();
    });
  };

  // Runs wait in a small queue (also kept in this browser) until the server has them.
  Lab.prototype.queue = function (item) {
    var list = this.pending();
    list.push(item);
    this.pendingList = list;
    put('localStorage', 'pending.' + this.user, JSON.stringify(list.slice(-5)));
  };
  Lab.prototype.pending = function () {
    if (this.pendingList) return this.pendingList;
    try { this.pendingList = JSON.parse(get('localStorage', 'pending.' + this.user) || '[]'); } catch (e) { this.pendingList = []; }
    return this.pendingList;
  };

  Lab.prototype.flush = function () {
    var self = this, list = this.pending();
    if (this.sending || !list.length || !this.user || !this.pin) return;
    this.sending = true;
    var item = list[0];
    this.backend.call('knot_submit', { p_username: this.user, p_pin: this.pin, p_meta: item.meta, p_data: item.data }).then(function (r) {
      self.sending = false;
      if (r.ok || r.error === 'bad_data' || r.error === 'bad_meta') {
        list.shift();
        put('localStorage', 'pending.' + self.user, JSON.stringify(list));
      }
      if (r.ok) {
        self.session.runs++; self.session.points += r.points;
        self.say('Saved! +' + r.points + ' points. This session: ' + self.session.runs + (self.session.runs === 1 ? ' knot, ' : ' knots, ') + self.session.points + ' points.', 'ok');
        self.loadBoard();
        self.flush();
      } else if (r.error === 'too_fast') {
        setTimeout(function () { self.flush(); }, 6000);
      } else {
        self.say(ERRORS[r.error] || 'This run could not be saved.', 'bad');
        if (r.error === 'wrong_pin' || r.error === 'bad_login' || r.error === 'locked') self.signedOut();
      }
    }).catch(function (e) {
      self.sending = false;
      self.say(problem(e) + ' Your run is kept and will be sent again.', 'bad');
      setTimeout(function () { self.flush(); }, 15000);
    });
  };

  Lab.prototype.loadBoard = function () {
    var self = this;
    this.backend.call('knot_leaderboard', { p_limit: 20 }).then(function (b) {
      self.board = b;
      self.q('[data-lab-count]').textContent = b.runs + (b.runs === 1 ? ' knot untied by ' : ' knots untied by ') + b.players + (b.players === 1 ? ' player' : ' players') + ' so far';
      var body = self.q('[data-lab-board] tbody');
      body.textContent = '';
      b.top.forEach(function (row, i) {
        var tr = document.createElement('tr');
        tr.setAttribute('data-name', row.username.toLowerCase());
        [i + 1, row.username, row.points, row.untied, row.hardest, row.best_seconds + ' s'].forEach(function (v) {
          var td = document.createElement('td'); td.textContent = v; tr.appendChild(td);
        });
        body.appendChild(tr);
      });
      if (!b.top.length) {
        var tr = document.createElement('tr'), td = document.createElement('td');
        td.colSpan = 6; td.textContent = 'No runs yet. Be the first!';
        tr.appendChild(td); body.appendChild(tr);
      }
      self.highlight();
    }).catch(function (e) {
      self.q('[data-lab-count]').textContent = 'The leaderboard could not be loaded. ' + problem(e);
    });
  };

  Lab.prototype.highlight = function () {
    var me = (this.user || '').toLowerCase();
    this.root.querySelectorAll('[data-lab-board] tr[data-name]').forEach(function (tr) {
      tr.classList.toggle('is-me', tr.getAttribute('data-name') === me);
    });
  };

  function init() {
    Array.prototype.forEach.call(document.querySelectorAll('[data-knot-lab]'), function (root) {
      if (root.querySelector('[data-dlo-game]').game) root.lab = new Lab(root);
    });
  }
  // After the game has set itself up (both scripts are deferred, the game first).
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
