'use strict';
// Build <deck>/out/deck_raw.pptx and its animation spec (.anim.json) from <deck>/slides/*.js.
// Slide files load in file-name order; each exports one async slide function or an array of them.
// ONLY=3,7 builds just those slide positions (page numbers keep their full-deck values).
const fs = require('fs');
const path = require('path');
const L = require('./motion-deck');

const dir = path.join(L.DECK, 'slides');
const slides = fs.readdirSync(dir).filter((f) => f.endsWith('.js')).sort()
  .flatMap((f) => [].concat(require(path.join(dir, f))));

(async () => {
  const out = path.resolve(process.argv[2] || path.join(L.DECK, 'out', 'deck_raw.pptx'));
  fs.mkdirSync(path.dirname(out), { recursive: true });
  const only = (process.env.ONLY || '').split(',').filter(Boolean).map(Number);
  for (const [i, fn] of slides.entries()) {
    if (only.length && !only.includes(i + 1)) continue;
    L.setCurrent(i + 1);
    await fn();
  }
  await L.pres.writeFile({ fileName: out });
  // spec keys follow the slide parts in the file (1..k), whatever subset was built
  const spec = {};
  Object.keys(L.SLIDES).map(Number).sort((a, b) => a - b).forEach((k, i) => { spec[i + 1] = L.SLIDES[k]; });
  fs.writeFileSync(out.replace(/\.pptx$/, '.anim.json'), JSON.stringify(spec, null, 1));
  console.log('wrote', out, 'slides', Object.keys(L.SLIDES).length, L.STATIC ? '(static)' : '');
  console.log(L.WARN.length ? 'FIT WARNINGS:\n' + L.WARN.join('\n') : 'no fit warnings');
})().catch((e) => { console.error(e); process.exit(1); });
