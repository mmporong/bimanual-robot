'use strict';
// Title: light orbs, white plate + ring framing a transparent product render, staged text
const L = require('motion-deck');
const { C, FSB, A, CFG, txt, circle, img, contain, newSlide } = L;

module.exports = async function title() {
  newSlide({ transition: 'fade' });
  L.orbs('t', false, C.orbL2, C.mainT2);
  circle(10.15, 3.72, 5.5, { fill: C.white, shadow: 'soft' }, { e: 'zoom', d: 0, t: 0.7 });
  circle(10.15, 3.72, 6.05, { line: C.mainM, lw: 1.5 }, { e: 'zoom', d: 250, t: 0.8 });
  const r = await contain('robot_w.png', 10.15 - 2.15, 3.72 - 1.62, 4.3, 3.24);
  img(A('robot_w.png'), r.x, r.y, r.w, r.h, { alt: '완성품 렌더' }, { e: 'up', d: 350, t: 0.8 });
  txt('과정 이름 · 최종 프로젝트 발표', 0.75, 1.5, 6.3, 0.36, { size: 14, font: FSB, color: C.mainD }, { e: 'fade', d: 300, t: 0.5 });
  txt('완성품 이름을\n두 줄로', 0.75, 1.95, 6.4, 2.25, { size: 58, bold: true, color: C.ink, ls: 1.08 }, { e: 'wipeR', d: 450, t: 0.7 });
  txt('무엇을, 누구에게 해 주는지 한 문장', 0.75, 4.35, 6.4, 0.5, { size: 22, font: FSB, color: C.main }, { e: 'fade', d: 1000, t: 0.6 });
  txt('2026.00.00', 0.75, 5.0, 4, 0.4, { size: 16, color: C.muted }, { e: 'fade', d: 1200, t: 0.5 });
  const b = CFG.brand || {};
  if (b.wordmark) img(A(b.wordmark), 0.75, 6.12, b.wordW, b.wordH, { name: '!!logo', alt: b.alt || '' }, { e: 'fade', d: 1350, t: 0.5 });
};
