'use strict';
// motion-deck engine: theme palette, text metrics + fit warnings, primitives that register native animations,
// and the recurring components of the white+sky deck. Slides load it with require('motion-deck').
const fs = require('fs');
const os = require('os');
const path = require('path');
const pptxgen = require('pptxgenjs');
const React = require('react');
const RDS = require('react-dom/server');
const sharp = require('sharp');
const lu = require('react-icons/lu');

const DECK = path.resolve(process.env.DECK_DIR || process.cwd());
const CFG = (() => {
  const f = path.join(DECK, 'deck.config.js');
  return fs.existsSync(f) ? require(f) : {};
})();
const expandHome = (p) => p.replace(/^(~|\$HOME)(?=$|\/)/, os.homedir());
const ASSETS = CFG.assetsDir ? path.resolve(DECK, expandHome(CFG.assetsDir)) : path.join(DECK, 'assets');
const A = (p) => (path.isAbsolute(p) ? p : path.join(ASSETS, p));
const MET = JSON.parse(fs.readFileSync(path.join(__dirname, 'metrics.json'), 'utf8'));
// STATIC=1 builds the print/PDF variant: no slideshow-only covers over late videos
const STATIC = !!process.env.STATIC;

// ---------- theme ----------
// main = dominant colour, sub = secondary accent, acc = "different from the usual" highlight (keep it rare)
const THEMES = {
  sky: {
    ink: '1F3A52', ink2: '2A4A63', ink3: '35556E', inkLine: '4A6A84',
    main: '1690D6', mainD: '0B78BF', mainM: '6FC3F0', mainT: 'E8F6FE', mainT2: 'CDEBFB', mainT25: 'B9E2F9', mainT3: 'A6DAF6',
    sub: '3BB3EA', subD: '0E86C9', subT: 'E6F4FB',
    text: '17304A', muted: '557085', faint: '7A90A3', line: 'DCEBF5', panel: 'F3F9FD',
    dText: 'D2DEEE', dMuted: 'A6BBD6',
    orbL1: 'EEF8FE', orbL2: 'E3F4FD', orbD1: '102B52', orbD2: '163A68', chipD: '9EC3EC', miniOff: 'D3E3EE',
  },
  teal: {
    ink: '0B2A33', ink2: '123B46', ink3: '1B4A56', inkLine: '2A5663',
    main: '0E7C86', mainD: '0A5F67', mainM: '6CB9C0', mainT: 'E3F2F3', mainT2: 'C9E7E9', mainT25: 'B5DDE1', mainT3: '9ED3D8',
    sub: '3BB3EA', subD: '1B7FB5', subT: 'E6F4FB',
    text: '16242B', muted: '56656D', faint: '86939A', line: 'DCE4E8', panel: 'F3F6F8',
    dText: 'D2DFE2', dMuted: 'A9C2C8',
    orbL1: 'EEF7F8', orbL2: 'E6F3FB', orbD1: '0F3A44', orbD2: '134A57', chipD: '9FC9D0', miniOff: 'D5E2E4',
  },
  blue: {
    ink: '0A1E3F', ink2: '12305C', ink3: '1B3F70', inkLine: '2A4D80',
    main: '1769D2', mainD: '0E4C9E', mainM: '72A9E8', mainT: 'E8F1FC', mainT2: 'CFE1F8', mainT25: 'BAD5F5', mainT3: 'A6C8F1',
    sub: '3BB3EA', subD: '1585C5', subT: 'E6F4FB',
    text: '14213A', muted: '536379', faint: '8492A6', line: 'DAE3EF', panel: 'F2F6FC',
    dText: 'D2DEEE', dMuted: 'A6BBD6',
    orbL1: 'EDF4FD', orbL2: 'E6F3FB', orbD1: '102B52', orbD2: '163A68', chipD: '9EC3EC', miniOff: 'D3DEEE',
  },
};
const C = Object.assign(
  { acc: 'DD6A12', accD: 'B4560A', accT: 'FDEEDF', white: 'FFFFFF' },
  THEMES[CFG.theme || 'sky'],
  CFG.palette || {},
);
const FR = 'Pretendard';
const FSB = 'Pretendard SemiBold';
// 16:9 wide canvas; ML = left margin, CR = content right edge, CW = content width
const W = 13.333, H = 7.5, ML = 0.6, CR = 12.733, CW = 12.133;
const BRAND = CFG.brand || {};

const pres = new pptxgen();
pres.layout = 'LAYOUT_WIDE';
pres.author = CFG.author || '';
pres.company = CFG.company || '';
pres.title = CFG.title || '';
pres.theme = { headFontFace: FR, bodyFontFace: FR };

// ---------- text metrics (Pretendard advance widths, em) ----------
function tw(s, pt, wt = 'r') {
  const m = MET[wt] || MET.r;
  let e = 0;
  for (const ch of String(s)) {
    if (m[ch] !== undefined) e += m[ch];
    else if (/[ᄀ-ᇿ㄰-㆏가-힣]/.test(ch)) e += m.HAN;
    else e += 0.62;
  }
  return (e * pt) / 72;
}
function nLines(s, pt, width, wt = 'r') {
  let n = 0;
  for (const para of String(s).split('\n')) {
    const toks = para.split(/(\s+)/).filter((t) => t.length);
    let cur = 0, l = 1;
    for (const t of toks) {
      const w = tw(t, pt, wt);
      if (/^\s+$/.test(t)) { cur += w; continue; }
      if (cur + w > width && cur > 0) { l++; cur = 0; }
      if (w > width) { l += Math.floor(w / width); cur = w % width; } else cur += w;
    }
    n += l;
  }
  return n;
}
function plain(text) {
  if (typeof text === 'string') return { s: text, pt: null, paras: text.split('\n').length };
  let s = '', pt = 0, paras = 1;
  text.forEach((r, i) => {
    s += r.text;
    const o = r.options || {};
    if (o.fontSize) pt = Math.max(pt, o.fontSize);
    if (o.breakLine && i < text.length - 1) { s += '\n'; paras++; }
  });
  return { s, pt: pt || null, paras };
}

// ---------- slide context ----------
const WARN = [];
const SLIDES = {};
let S = null;
let CUR = 0;
// build.js sets the slide's position before calling it, so page numbers stay right in ONLY=… subset builds
function setCurrent(n) { CUR = n; }
// newSlide() or newSlide({ dark, transition: 'morph' | 'fade' }); a leading number is accepted and ignored
function newSlide(a, b) {
  const opts = a && typeof a === 'object' ? a : b || {};
  const n = CUR || Object.keys(SLIDES).length + 1;
  const slide = pres.addSlide();
  slide.background = { color: opts.dark ? C.ink : C.white };
  S = { n, slide, k: 0, dark: !!opts.dark };
  SLIDES[n] = { items: [], video: null, transition: opts.transition || 'morph', dark: !!opts.dark };
  return slide;
}
function nm(p) { S.k += 1; return `s${S.n}_${p}${S.k}`; }
// a = { e, d (ms from slide start), t (s), ...extra }; an array gives one object several effects
function anim(name, a) {
  if (!a) return;
  (Array.isArray(a) ? a : [a]).forEach((x) => SLIDES[S.n].items.push({ name, ...x }));
}
function check(text, o, w, h) {
  if (o.check === false) return;
  const p = plain(text);
  const pt = p.pt || o.fontSize || 14;
  const wt = o.bold ? 'b' : o.fontFace === FSB ? 'sb' : 'r';
  const m = (o.margin || 0) / 72;
  const innerW = w - 2 * m - 0.02, innerH = h - 2 * m;
  const L = nLines(p.s, pt, innerW, wt);
  const ls = o.lineSpacingMultiple || 1.2;
  const est = (L * pt * ls) / 72 + ((p.paras - 1) * (o.paraSpaceAfter || 0)) / 72;
  if (est > innerH * 1.04) WARN.push(`s${S.n} H-overflow est=${est.toFixed(2)} box=${innerH.toFixed(2)} L=${L} | ${p.s.replace(/\n/g, '⏎').slice(0, 50)}`);
}

// ---------- primitives ----------
function shadow(kind) {
  if (kind === 'dark') return { type: 'outer', color: '000000', opacity: 0.35, blur: 14, offset: 4, angle: 90 };
  return { type: 'outer', color: C.ink, opacity: 0.1, blur: 16, offset: 4, angle: 90 };
}
function txt(text, x, y, w, h, o = {}, a) {
  const name = o.name || nm('t');
  const opt = {
    x, y, w, h, fontFace: o.font || FR, fontSize: o.size || 14, color: o.color || (S.dark ? C.white : C.text),
    bold: !!o.bold, italic: !!o.italic, align: o.align || 'left', valign: o.valign || 'top',
    margin: o.margin ?? 0, isTextBox: true, objectName: name,
  };
  if (o.ls) opt.lineSpacingMultiple = o.ls;
  if (o.psa) opt.paraSpaceAfter = o.psa;
  if (o.cs) opt.charSpacing = o.cs;
  if (o.fill) opt.fill = { color: o.fill, transparency: o.ft || 0 };
  if (o.shape) opt.shape = o.shape;
  if (o.r !== undefined) opt.rectRadius = o.r;
  if (o.line) opt.line = { color: o.line, width: o.lw || 1, dashType: o.dash || 'solid' };
  if (o.shadow) opt.shadow = shadow(o.shadow);
  if (o.hyperlink) opt.hyperlink = o.hyperlink;
  if (o.rot) opt.rotate = o.rot;
  check(text, { ...opt, check: o.check }, w, h);
  S.slide.addText(text, opt);
  anim(name, a);
  return name;
}
function box(x, y, w, h, o = {}, a) {
  const name = o.name || nm('r');
  const opt = { x, y, w, h, objectName: name };
  opt.fill = o.fill ? { color: o.fill, transparency: o.ft || 0 } : { type: 'none' };
  opt.line = o.line ? { color: o.line, width: o.lw || 1, dashType: o.dash || 'solid', beginArrowType: o.ba, endArrowType: o.ea } : { type: 'none' };
  if (o.r !== undefined) opt.rectRadius = o.r;
  if (o.shadow) opt.shadow = shadow(o.shadow);
  if (o.rot) opt.rotate = o.rot;
  if (o.flipH) opt.flipH = true;
  if (o.flipV) opt.flipV = true;
  S.slide.addShape(o.shape || (o.r !== undefined ? pres.shapes.ROUNDED_RECTANGLE : pres.shapes.RECTANGLE), opt);
  anim(name, a);
  return name;
}
function circle(cx, cy, d, o = {}, a) { return box(cx - d / 2, cy - d / 2, d, d, { ...o, shape: pres.shapes.OVAL }, a); }
function line(x1, y1, x2, y2, o = {}, a) {
  const name = o.name || nm('l');
  const x = Math.min(x1, x2), y = Math.min(y1, y2);
  const opt = {
    x, y, w: Math.max(Math.abs(x2 - x1), 0.0001), h: Math.max(Math.abs(y2 - y1), 0.0001), objectName: name,
    line: { color: o.color || C.line, width: o.w || 1.5, dashType: o.dash || 'solid', beginArrowType: o.ba, endArrowType: o.ea },
  };
  if ((x2 < x1) !== (y2 < y1) && x1 !== x2 && y1 !== y2) opt.flipV = true;
  if (x2 < x1 && y1 === y2) opt.flipH = true;
  if (y2 < y1 && x1 === x2) opt.flipV = true;
  S.slide.addShape(pres.shapes.LINE, opt);
  anim(name, a);
  return name;
}
function img(src, x, y, w, h, o = {}, a) {
  const name = o.name || nm('i');
  const opt = { x, y, w, h, objectName: name, altText: o.alt || '' };
  if (String(src).startsWith('image/')) opt.data = src; else opt.path = src;
  if (o.rounding) opt.rounding = true;
  if (o.transparency) opt.transparency = o.transparency;
  S.slide.addImage(opt);
  anim(name, a);
  return name;
}

// ---------- assets ----------
const ICON = {};
// Lucide icon (react-icons/lu name, e.g. 'LuBot') rendered to PNG in one colour
async function icon(nameI, color, px = 256) {
  const key = `${nameI}|${color}`;
  if (ICON[key]) return ICON[key];
  if (!lu[nameI]) throw new Error('missing icon ' + nameI);
  const svg = RDS.renderToStaticMarkup(React.createElement(lu[nameI], { color: '#' + color, size: px }));
  const buf = await sharp(Buffer.from(svg)).png().toBuffer();
  ICON[key] = 'image/png;base64,' + buf.toString('base64');
  return ICON[key];
}
async function iconImg(nameI, color, x, y, s, a, o = {}) { return img(await icon(nameI, color), x, y, s, s, o, a); }
// icon centred in a filled circle
async function iconDot(nameI, cx, cy, d, fill, color, a, o = {}) {
  const n1 = circle(cx, cy, d, { fill, line: o.ring, lw: o.rw }, a);
  const s = d * (o.scale || 0.56);
  const n2 = await iconImg(nameI, color, cx - s / 2, cy - s / 2, s, a);
  return [n1, n2];
}
async function cover(file, w, h, pos = 'centre', fmt = 'jpeg') {
  const buf = await sharp(A(file)).resize({ width: Math.round(w * 220), height: Math.round(h * 220), fit: 'cover', position: pos })[fmt]({ quality: 88 }).toBuffer();
  return `image/${fmt === 'jpeg' ? 'jpeg' : 'png'};base64,` + buf.toString('base64');
}
// cover-crop to the box and round the corners with real transparency (same in PowerPoint and LibreOffice).
// Output is one PNG frame: never pass an animated GIF here, it would freeze on its first frame.
async function rounded(file, w, h, r = 0.14, pos = 'centre') {
  const pw = Math.round(w * 220), ph = Math.round(h * 220), pr = Math.round(r * 220);
  const mask = Buffer.from(`<svg width="${pw}" height="${ph}"><rect x="0" y="0" width="${pw}" height="${ph}" rx="${pr}" ry="${pr}" fill="#fff"/></svg>`);
  const buf = await sharp(A(file)).resize({ width: pw, height: ph, fit: 'cover', position: pos }).ensureAlpha()
    .composite([{ input: mask, blend: 'dest-in' }]).png({ compressionLevel: 9 }).toBuffer();
  return 'image/png;base64,' + buf.toString('base64');
}
// largest box with the image's aspect ratio that fits inside (x, y, w, h), centred
async function contain(file, x, y, w, h) {
  const m = await sharp(A(file)).metadata();
  const r = m.width / m.height;
  let ww = w, hh = w / r;
  if (hh > h) { hh = h; ww = h * r; }
  return { x: x + (w - ww) / 2, y: y + (h - hh) / 2, w: ww, h: hh };
}
async function dataUri(file) {
  const ext = path.extname(file).slice(1).toLowerCase();
  const mime = ext === 'jpg' ? 'jpeg' : ext;
  return `data:image/${mime};base64,` + fs.readFileSync(A(file)).toString('base64');
}

// ---------- recurring components ----------
// two background orbs per slide; same names on every slide so Morph glides them between positions
const ORB = {
  a: [[9.6, -3.2, 6.4], [-0.35, 6.55, 1.25]],
  b: [[10.4, 4.5, 5.8], [-0.55, -0.7, 1.5]],
  c: [[-3.3, 4.3, 6.2], [12.75, 6.6, 1.1]],
  d: [[4.4, 6.2, 5.2], [12.6, 5.6, 1.5]],
  e: [[-2.6, -3.4, 6.0], [-0.55, 6.55, 1.3]],
  t: [[6.3, -1.75, 8.8], [5.55, 6.65, 1.2]],
};
function orbs(key, dark, c1, c2) {
  const [o1, o2] = ORB[key || 'a'];
  box(o1[0], o1[1], o1[2], o1[2], { shape: pres.shapes.OVAL, fill: c1 || (dark ? C.orbD1 : C.orbL1), name: '!!orb1' });
  box(o2[0], o2[1], o2[2], o2[2], { shape: pres.shapes.OVAL, fill: c2 || (dark ? C.orbD2 : C.orbL2), name: '!!orb2' });
}
function fitTitle(t, maxW, maxPt = 30, minPt = 20) {
  let pt = maxPt;
  maxW *= 0.93;
  while (pt > minPt && tw(t, pt, 'b') > maxW) pt -= 1;
  if (tw(t, pt, 'b') > maxW) WARN.push(`s${S.n} title too long at ${pt}pt: ${t}`);
  return pt;
}
function brandMark() {
  if (!BRAND.mark) return;
  const w = BRAND.markW || 0.44;
  img(A(BRAND.mark), 12.73 - w, 0.42, w, BRAND.markH || 0.418, { name: '!!logo', alt: BRAND.alt || '' });
}
// standard content-slide header: section chip, label, fitted title, brand mark, page number.
// mini = active quadrant (1..4) of the small team map; sub = one-line subtitle under the title
function header({ num, label, title, orb, mini, sub }) {
  const dark = S.dark;
  orbs(orb, dark);
  const numW = num.length > 2 ? 0.62 : 0.46;
  txt(num, ML, 0.47, numW, 0.3, { name: '!!chipnum', shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.15, fill: dark ? C.sub : C.main, color: dark ? C.ink : C.white, bold: true, size: 11, align: 'center', valign: 'middle', check: false });
  txt(label, ML + numW + 0.14, 0.47, 8.6, 0.3, { name: '!!chiplabel', font: FSB, size: 12.5, color: dark ? C.chipD : C.main, valign: 'middle', check: false });
  const tW = mini ? 10.9 : 11.45;
  const pt = fitTitle(title, tW - 0.05);
  txt(title, ML, 0.88, tW, 0.72, { name: '!!title', size: pt, bold: true, color: dark ? C.white : C.ink, valign: 'middle', check: false });
  brandMark();
  pageNum();
  if (mini) miniMap(mini);
  if (sub) txt(sub, ML, 1.58, 11.4, 0.34, { size: 14.5, color: dark ? C.dText : C.muted });
}
function pageNum() {
  txt(String(S.n).padStart(2, '0'), 12.03, 7.0, 0.7, 0.26, { name: '!!pg', size: 10, color: S.dark ? C.dMuted : C.faint, align: 'right', check: false });
}
function miniMap(active) {
  const x0 = 11.62, y0 = 0.45, s = 0.17, g = 0.05;
  const pos = { 1: [0, 0], 2: [1, 0], 3: [0, 1], 4: [1, 1] };
  for (const q of [1, 2, 3, 4]) {
    const [cx, cy] = pos[q];
    box(x0 + cx * (s + g), y0 + cy * (s + g), s, s, { name: `!!q${q}`, r: 0.03, fill: q === active ? C.main : C.miniOff });
  }
}
// label pill + sentence on a full-width tinted bar (the deck's "so what" line)
function callout(label, text, y, o = {}, a) {
  const dark = S.dark;
  const h = o.h || 0.6, x = o.x ?? ML, w = o.w ?? CW;
  const lw = o.lw || Math.max(0.95, tw(label, 12.5, 'b') + 0.42);
  box(x, y, w, h, { r: 0.1, fill: dark ? C.ink2 : C.panel }, a);
  txt(label, x + 0.14, y + (h - 0.36) / 2, lw, 0.36, { shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.18, fill: o.color || (dark ? C.sub : C.ink), color: dark && !o.color ? C.ink : C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a);
  txt(text, x + 0.14 + lw + 0.2, y + 0.06, w - lw - 0.62, h - 0.12, { size: o.size || 14.5, font: FSB, color: dark ? C.white : C.text, valign: 'middle', ls: 1.15 }, a);
}
// small rounded tag; returns its width so tags can be chained
function tagPill(t, x, y, d, o = {}) {
  const w = tw(t, 12, 'sb') + 0.4;
  txt(t, x, y, w, 0.34, { shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.17, fill: o.fill || C.mainT, color: o.color || C.mainD, font: FSB, size: 12, align: 'center', valign: 'middle', check: false }, { e: 'fade', d, t: 0.4 });
  return w;
}
// register a video for auto-play: looping in PowerPoint, plays once in LibreOffice; click toggles pause.
// Muted by default; opts.mute = false keeps the clip's sound (narrated demos).
function setVideo(name, durMs, delay, opts = {}) { SLIDES[S.n].video = { name, durMs, delay, mute: opts.mute !== false }; }
// LibreOffice ignores entrance effects on media, so a video meant to appear late is hidden by a
// background-coloured box that fades out at `delay` (skipped in the STATIC/PDF build)
function coverUntil(x, y, w, h, delay, o = {}) {
  if (STATIC) return null;
  return box(x, y, w, h, { fill: o.fill || (S.dark ? C.ink : C.white), name: o.name }, { e: 'out', d: delay, t: o.t || 0.5 });
}
// shadowed card with an auto-playing clip. o.appear = when the clip fades in (default 0);
// o.cover hides it in LibreOffice until then (LibreOffice ignores entrance effects on media); o.sound plays the audio
async function videoCard(name, file, poster, x, y, w, h, durMs, playAt, o = {}) {
  box(x, y, w, h, { r: 0.14, fill: C.white, shadow: 'soft' }, { e: 'fade', d: 0, t: 0.5 });
  S.slide.addMedia({ type: 'video', path: A(file), cover: await dataUri(poster), x, y, w, h, objectName: name });
  anim(name, { e: 'fade', d: o.appear || 0, t: 0.6 });
  if (o.cover) coverUntil(x - 0.02, y - 0.02, w + 0.04, h + 0.04, o.appear ?? playAt, { name: name + '_cover' });
  setVideo(name, durMs, playAt, { mute: !o.sound });
}
// "영상 순서" pill + numbered step sentence under a video
function seqLine(steps, x, y, w, d, label = '영상 순서') {
  const runs = [];
  steps.forEach((s, i) => {
    runs.push({ text: `${i + 1} `, options: { bold: true, color: C.mainD } });
    runs.push({ text: s, options: { color: C.text } });
    if (i < steps.length - 1) runs.push({ text: ' → ', options: { color: C.faint } });
  });
  txt(label, x, y, 0.95, 0.34, { shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.17, fill: C.mainT, color: C.mainD, size: 11, font: FSB, align: 'center', valign: 'middle', check: false }, { e: 'fade', d, t: 0.4 });
  txt(runs, x + 1.1, y, w - 1.1, 0.34, { size: 11.5, font: FSB, valign: 'middle', check: false }, { e: 'wipeR', d: d + 100, t: 0.9 });
  if (tw(steps.join(' → ') + ' 1 2 3 4 5 6', 11.5, 'sb') * 1.12 > w - 1.1) WARN.push(`s${S.n} seqLine too long`);
}
// pentagon + chevrons process strip, drawn left to right
function chevrons(items, x, y, w, h, d0, o = {}) {
  const n = items.length, ov = 0.16, cw = (w + ov * (n - 1)) / n;
  items.forEach((it, i) => {
    const last = i === n - 1;
    const fill = o.fills ? o.fills[i] : last ? C.main : [C.mainT, C.mainT2, C.mainT25, C.mainT3, C.mainM][i % 5];
    const dark = fill === C.main || fill === C.ink || fill === C.mainD;
    txt(it, x + i * (cw - ov), y, cw, h, { shape: i === 0 ? pres.shapes.PENTAGON : pres.shapes.CHEVRON, fill, color: dark ? C.white : C.ink, size: o.size || 12.5, bold: true, align: 'center', valign: 'middle', ls: 1.05, margin: 0, check: false }, { e: 'wipeR', d: d0 + i * 150, t: 0.35 });
  });
}
// person/part panel: tinted card with a big name and a role pill
function profile(x, y, w, h, name, role, a) {
  box(x, y, w, h, { r: 0.14, fill: C.panel }, a);
  txt(name, x + 0.3, y + 0.25, 2.2, 0.55, { size: 26, bold: true, color: C.ink, valign: 'middle' }, a);
  txt(role, x + w - 1.85, y + 0.33, 1.55, 0.38, { shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.19, fill: C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a);
}
// small coloured label + body paragraph
function sec(label, body, x, y, w, h, a, o = {}) {
  txt(label, x, y, w, 0.28, { size: 12, bold: true, color: C.main }, a);
  txt(body, x, y + 0.3, w, h - 0.3, { size: o.size || 13, color: C.text, ls: o.ls || 1.15, psa: o.psa }, a);
}
// big number + unit as one text run pair, e.g. bigNum('68.8', '%', 64)
function bigNum(num, unit, pt = 46, color = C.main) {
  return [{ text: num, options: { fontSize: pt, bold: true, color } }, { text: unit, options: { fontSize: Math.round(pt * 0.5), bold: true, color } }];
}
// plain bullet runs for txt()
function bullets(arr, o = {}) {
  return arr.map((t, j) => ({ text: t, options: { bullet: { indent: o.indent || 10 }, breakLine: j < arr.length - 1 } }));
}

module.exports = {
  pres, C, THEMES, FR, FSB, W, H, ML, CR, CW, A, DECK, CFG, STATIC, tw, nLines, WARN, SLIDES, setCurrent, newSlide, nm, anim,
  txt, box, circle, line, img, icon, iconImg, iconDot, cover, rounded, contain, dataUri,
  orbs, fitTitle, header, pageNum, brandMark, miniMap, callout, tagPill, setVideo, coverUntil, videoCard, seqLine, chevrons,
  profile, sec, bigNum, bullets,
  get S() { return S; },
};
