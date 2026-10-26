# berkguler.github.io

Personal academic website of Berk Güler, built with Jekyll and served by GitHub Pages.

## Editing content

Most content lives in data files, so updates rarely touch HTML:

| What | Where |
| --- | --- |
| Publications (with links and BibTeX) | `_data/publications.yml` |
| News items | `_data/news.yml` |
| Education timeline | `_data/education.yml` |
| Teaching | `_data/teaching.yml` |
| Reviewing | `_data/service.yml` |
| Research themes | `_data/themes.yml` |
| Name spellings highlighted in author lists | `_data/me.yml` |
| Bio and research interests | `index.html` |
| Profile links, CV link, analytics | `_config.yml` |

### Adding a project

Create `_projects/<slug>.html`. The front matter links it to a paper and a theme:

```yaml
---
title: "Short project title"
pub: <id from _data/publications.yml>
theme: <id from _data/themes.yml>
topic: DLO · Dynamics prediction   # small tag on the card
order: 1            # position within its theme
summary: >-
  One or two sentences: the card's AI summary.
ai_summary: >-
  <p>Longer AI-written summary (HTML) for the project page.</p>
related: [<publication id>]        # optional cross-links
interactive: try-it                # optional: id of a demo on the page; adds an "Interactive" badge
play_title: Untie the knot         # optional, with play_blurb, play_visual, play_order:
                                   # a tile in the home page's Interactives section
teaser:
  type: image       # image | rope | svg | icon
  src: /assets/images/projects/example.jpg
  alt: Description of the image
---
<h2>Overview</h2>
...
```

The page is published at `/projects/<slug>/` and appears as a card under its theme on the home page.

Cards and project pages show the paper's own `abstract` (from `_data/publications.yml`) by default,
with an **AI summary** toggle for `summary` / `ai_summary`. Projects without an abstract can set
`abstract:` and `abstract_label:` in their front matter.

Teaser types:

- `image`: `src`, `alt`, optional `fit: contain` and `position` (CSS object-position)
- `rope`: animated rope from `assets/js/rope.js`; `scene` is one of `shape-matching`, `assist`, `loop`, `particles` (the hero's draggable rope is `playground`)
- `svg`: inline SVG from `_includes/teasers/<file>`
- `icon`: Font Awesome `icon` with a `label`

## Design

- Font: Roboto (Google Fonts); icons: Font Awesome 6
- Palette: `#FE7743` orange, `#273F4F` navy, `#447D9B` blue, `#D7D7D7` gray (CSS variables in `assets/css/main.css`)
- The rope illustrations follow Stanko Tadić's [Draw SVG rope using JavaScript](https://muffinman.io/blog/draw-svg-rope-using-javascript/) ([CodePen](https://codepen.io/stanko/pen/vYaEMKX)).

## Interactive demos

Each demo is an include plus a script, placed on a project page under a heading with
`id="try-it"` (the target of the project's `interactive:` badge). Projects with `play_title`
also get a tile in the home page's **Interactives** section; `play_visual` is `knot` (the animated
`untie` rope scene) or the name of an SVG in `_includes/plays/`.

- **Push and park** (`_includes/comanip_demo.html`, `assets/js/comanip.js`, M.Sc. thesis page):
  re-implements the thesis simulation's admittance model, checked step by step against
  `simulation_new.py` in KocUniversity/pHRI-simulation. Reports human effort ∫|F·v| dt and
  integrated absolute jerk ∫|v̈| dt, and plots the visitor's parked trials against each other.
  The intent signal and the simulated human are labelled simplifications.
- **Knot untangling** (`_includes/assistdlo_game.html`, `assets/js/assistdlo-game.js`, AssistDLO
  page): a 3D position-based rope simulation (gravity, table friction, bending stiffness,
  self-collision) drawn with `RopeDraw.Rope.render`, teleoperated directly or with a simplified
  SA-CBF (barrier with funnels around rope points). Knots: the overhand knot and prime knots from
  the Rolfsen table (`KNOTS`), drawn as closed braids and cut open on the outer strand; their knot
  determinants match the table. Rope thickness and bending stiffness are adjustable
  (`ropeParams`). The knot counts as untied when the rope no longer crosses itself. The hint
  simulates candidate moves on copies of the rope (`candidateMoves`, `tryMove`, `score`) and draws
  the best one as an arrow.
- **Shared autonomy with a barrier** (`_includes/barrierik_demo.html`, `assets/js/barrierik-demo.js`,
  BarrierIK page): a planar three-link arm with linear blending and three IK solvers (no
  obstacles, proximity penalty, CBF constraints), each step a small QP solved with Hildreth's
  method.

The physics in each script is exported for Node (`module.exports`), so it can be tested headlessly.

## Local preview

```sh
bundle install
bundle exec jekyll serve
```
