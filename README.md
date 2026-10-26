# berkguler.github.io

Personal academic website of Berk Güler, built with Jekyll and served by GitHub Pages.

## Editing content

Most content lives in data files, so updates rarely touch HTML:

| What | Where |
| --- | --- |
| Publications (with links and BibTeX) | `_data/publications.yml` |
| News items | `_data/news.yml` |
| Education timeline | `_data/education.yml` |
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
order: 1            # position within its theme
summary: >-
  One or two sentences for the research card.
teaser:
  type: image       # image | rope | svg | icon
  src: /assets/images/projects/example.jpg
  alt: Description of the image
---
<h2>Overview</h2>
...
```

The page is published at `/projects/<slug>/` and appears as a card under its theme on the home page.

Teaser types:

- `image`: `src`, `alt`, optional `fit: contain` and `position` (CSS object-position)
- `rope`: animated rope from `assets/js/rope.js`; `scene` is one of `shape-matching`, `assist`, `loop`, `particles`
- `svg`: inline SVG from `_includes/teasers/<file>`
- `icon`: Font Awesome `icon` with a `label`

## Design

- Font: Roboto (Google Fonts); icons: Font Awesome 6
- Palette: `#FE7743` orange, `#273F4F` navy, `#447D9B` blue, `#D7D7D7` gray (CSS variables in `assets/css/main.css`)
- The rope illustrations follow Stanko Tadić's [Draw SVG rope using JavaScript](https://muffinman.io/blog/draw-svg-rope-using-javascript/) ([CodePen](https://codepen.io/stanko/pen/vYaEMKX)).

## Local preview

```sh
bundle install
bundle exec jekyll serve
```
