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
})();
