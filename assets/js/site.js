/* Small page behaviours: mobile nav, section highlighting, BibTeX toggles,
   copy-to-clipboard, and the "older news" toggle. No dependencies. */
(function () {
  'use strict';

  // Mobile navigation
  var toggle = document.querySelector('.nav-toggle');
  var links = document.getElementById('nav-links');
  if (toggle && links) {
    var setOpen = function (open) {
      links.classList.toggle('is-open', open);
      toggle.setAttribute('aria-expanded', String(open));
      toggle.querySelector('i').className = open ? 'fa-solid fa-xmark' : 'fa-solid fa-bars';
    };
    toggle.addEventListener('click', function () {
      setOpen(!links.classList.contains('is-open'));
    });
    links.addEventListener('click', function (e) {
      if (e.target.closest('a')) setOpen(false);
    });
  }

  // Highlight the nav entry of the section in view (home page only)
  var navAnchors = Array.prototype.slice.call(document.querySelectorAll('.nav-links a[href^="#"]'));
  if (navAnchors.length && 'IntersectionObserver' in window) {
    var byId = {};
    navAnchors.forEach(function (a) { byId[a.getAttribute('href').slice(1)] = a; });
    var visible = {};
    var spy = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) { visible[entry.target.id] = entry.isIntersecting; });
      var current = null;
      navAnchors.forEach(function (a) {
        var id = a.getAttribute('href').slice(1);
        if (!current && visible[id]) current = id;
      });
      navAnchors.forEach(function (a) {
        a.classList.toggle('is-active', a.getAttribute('href') === '#' + current);
      });
    }, { rootMargin: '-45% 0px -50% 0px' });
    Object.keys(byId).forEach(function (id) {
      var el = document.getElementById(id);
      if (el) spy.observe(el);
    });
  }

  // BibTeX toggles and copy buttons
  document.addEventListener('click', function (e) {
    var bibBtn = e.target.closest('[data-bibtex-toggle]');
    if (bibBtn) {
      var block = document.getElementById(bibBtn.getAttribute('aria-controls'));
      if (block) {
        var open = block.hasAttribute('hidden');
        block.toggleAttribute('hidden', !open);
        bibBtn.setAttribute('aria-expanded', String(open));
      }
      return;
    }

    var copyBtn = e.target.closest('[data-copy]');
    if (copyBtn) {
      var src = document.getElementById(copyBtn.getAttribute('data-copy'));
      if (!src || !navigator.clipboard) return;
      navigator.clipboard.writeText(src.textContent).then(function () {
        var label = copyBtn.querySelector('span');
        var icon = copyBtn.querySelector('i');
        label.textContent = 'Copied';
        icon.className = 'fa-solid fa-check';
        setTimeout(function () {
          label.textContent = 'Copy';
          icon.className = 'fa-regular fa-copy';
        }, 1600);
      });
    }
  });

  // Older news
  var newsBtn = document.querySelector('[data-news-toggle]');
  if (newsBtn) {
    var list = document.getElementById(newsBtn.getAttribute('aria-controls'));
    newsBtn.addEventListener('click', function () {
      var expanded = list.classList.toggle('is-expanded');
      newsBtn.setAttribute('aria-expanded', String(expanded));
      newsBtn.querySelector('span').textContent = expanded ? 'Show less' : 'Show older news';
      newsBtn.querySelector('i').className = expanded ? 'fa-solid fa-angle-up' : 'fa-solid fa-angle-down';
    });
  }

  // Research theme tabs. Links like /?theme=dlo#research open a given tab.
  var tabs = Array.prototype.slice.call(document.querySelectorAll('.tab[role="tab"]'));
  if (tabs.length) {
    var selectTab = function (id, focus) {
      if (!tabs.some(function (t) { return t.getAttribute('data-theme') === id; })) id = tabs[0].getAttribute('data-theme');
      tabs.forEach(function (t) {
        var on = t.getAttribute('data-theme') === id;
        t.setAttribute('aria-selected', String(on));
        t.tabIndex = on ? 0 : -1;
        document.getElementById(t.getAttribute('aria-controls')).hidden = !on;
        if (on && focus) t.focus();
      });
      return id;
    };
    var params = new URLSearchParams(location.search);
    selectTab(params.get('theme') || (location.hash.indexOf('#theme-') === 0 ? location.hash.slice(7) : ''));
    tabs.forEach(function (t, i) {
      t.addEventListener('click', function () {
        var id = selectTab(t.getAttribute('data-theme'));
        if (history.replaceState) history.replaceState(null, '', '?theme=' + id + '#research');
      });
      t.addEventListener('keydown', function (e) {
        var step = { ArrowRight: 1, ArrowLeft: -1 }[e.key];
        if (!step) return;
        var next = tabs[(i + step + tabs.length) % tabs.length];
        selectTab(next.getAttribute('data-theme'), true);
        e.preventDefault();
      });
    });
  }

  // Abstract / AI summary toggles and "Read more" on clamped abstracts.
  document.addEventListener('click', function (e) {
    var tab = e.target.closest('.summary-tab');
    if (tab) {
      var box = tab.closest('[data-summary]');
      var view = tab.getAttribute('data-view');
      box.querySelectorAll('.summary-tab').forEach(function (b) { b.setAttribute('aria-pressed', String(b === tab)); });
      box.querySelectorAll('[data-panel]').forEach(function (p) { p.hidden = p.getAttribute('data-panel') !== view; });
      return;
    }
    var more = e.target.closest('[data-more]');
    if (more) {
      var text = more.parentNode.querySelector('.summary-text');
      var open = text.classList.toggle('is-clamped') === false;
      more.setAttribute('aria-expanded', String(open));
      more.textContent = open ? 'Show less' : 'Read more';
    }
  });

  // Publication filters: type chips, first-author toggle, and search.
  var pubs = Array.prototype.slice.call(document.querySelectorAll('.pub[data-kind]'));
  if (pubs.length) {
    var chips = Array.prototype.slice.call(document.querySelectorAll('.chip[data-filter]'));
    var firstBtn = document.querySelector('[data-first-author]');
    var search = document.querySelector('[data-pub-search]');
    var empty = document.querySelector('.pub-empty');
    var kind = 'all', firstOnly = false;
    var apply = function () {
      var terms = search.value.toLowerCase().split(/\s+/).filter(Boolean);
      var shown = 0;
      pubs.forEach(function (li) {
        var ok = (kind === 'all' || li.getAttribute('data-kind') === kind) &&
          (!firstOnly || li.getAttribute('data-first') === 'true') &&
          terms.every(function (term) { return li.getAttribute('data-search').indexOf(term) >= 0; });
        li.hidden = !ok;
        if (ok) shown++;
      });
      document.querySelectorAll('.pub-group').forEach(function (g) {
        g.hidden = !g.querySelector('.pub:not([hidden])');
      });
      empty.hidden = shown > 0;
    };
    chips.forEach(function (c) {
      c.addEventListener('click', function () {
        kind = c.getAttribute('data-filter');
        chips.forEach(function (o) { o.setAttribute('aria-pressed', String(o === c)); });
        apply();
      });
    });
    firstBtn.addEventListener('click', function () {
      firstOnly = !firstOnly;
      firstBtn.setAttribute('aria-pressed', String(firstOnly));
      apply();
    });
    search.addEventListener('input', apply);
  }

  // Fade cards and timeline entries in as they scroll into view.
  var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var revealItems = document.querySelectorAll('.reveal');
  if (revealItems.length && !reduce && 'IntersectionObserver' in window) {
    document.documentElement.classList.add('reveal-on');
    var revealer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (!entry.isIntersecting) return;
        entry.target.classList.add('is-visible');
        revealer.unobserve(entry.target);
      });
    }, { rootMargin: '0px 0px -6% 0px' });
    revealItems.forEach(function (el) { revealer.observe(el); });
  }

  // Hide the rope hint after the first grab.
  document.addEventListener('ropegrab', function () {
    var hint = document.querySelector('.rope-hint');
    if (hint) hint.classList.add('is-gone');
  });

  // Back-to-top button.
  var toTop = document.createElement('button');
  toTop.type = 'button';
  toTop.className = 'to-top';
  toTop.setAttribute('aria-label', 'Back to top');
  toTop.innerHTML = '<i class="fa-solid fa-arrow-up" aria-hidden="true"></i>';
  toTop.addEventListener('click', function () { window.scrollTo({ top: 0, behavior: reduce ? 'auto' : 'smooth' }); });
  document.body.appendChild(toTop);
  var onScroll = function () { toTop.classList.toggle('is-shown', window.scrollY > 700); };
  window.addEventListener('scroll', onScroll, { passive: true });
  onScroll();
})();
