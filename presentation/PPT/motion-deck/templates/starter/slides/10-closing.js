'use strict';
// Closing: title-slide backdrop, two question cards, tagline, thanks, link, wordmark.
// Builds its own header (no default orbs) but keeps the !!names so Morph still pairs them.
const L = require('motion-deck');
const { pres, C, FSB, ML, CR, CW, A, CFG, txt, box, img, newSlide } = L;

module.exports = async function closing() {
  newSlide();
  L.orbs('t', false, C.orbL2, C.mainT2);
  txt('08', ML, 0.47, 0.46, 0.3, { name: '!!chipnum', shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.15, fill: C.main, color: C.white, bold: true, size: 11, align: 'center', valign: 'middle', check: false });
  txt('마무리', ML + 0.6, 0.47, 8.6, 0.3, { name: '!!chiplabel', font: FSB, size: 12.5, color: C.mainD, valign: 'middle', check: false });
  txt('질문 받고 싶은 두 가지', ML, 0.88, 11.45, 0.72, { name: '!!title', size: 30, bold: true, color: C.ink, valign: 'middle', check: false });
  L.brandMark();
  L.pageNum();
  const qs = [['Q1', '질문 1', '피드백 받고 싶은 주제', '왜 이 부분이 궁금한지, 어떤 조언이 도움이 되는지 두 줄 정도로.'], ['Q2', '질문 2', '피드백 받고 싶은 주제', '왜 이 부분이 궁금한지, 어떤 조언이 도움이 되는지 두 줄 정도로.']];
  const w = 5.97, gx = CW - 2 * w, y = 1.95, h = 2.75;
  qs.forEach((q, i) => {
    const x = ML + i * (w + gx), a = { e: 'up', d: i * 220, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    txt(q[0], x + 0.32, y + 0.25, 1.2, 0.72, { size: 40, bold: true, color: C.main, valign: 'middle', check: false }, a);
    txt(q[1], x + 1.45, y + 0.44, 2.5, 0.32, { size: 12.5, color: C.muted, valign: 'middle' }, a);
    txt(q[2], x + 0.32, y + 1.05, w - 0.6, 0.45, { size: 20, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(q[3], x + 0.32, y + 1.58, w - 0.6, 1.0, { size: 13.5, color: C.muted, ls: 1.2 }, a);
  });
  txt('발표를 한 문장으로 닫는 태그라인', ML, 5.08, CW, 0.45, { size: 19, font: FSB, color: C.mainD }, { e: 'wipeR', d: 700, t: 0.9 });
  txt('감사합니다', ML, 5.68, 5, 0.75, { size: 36, bold: true, color: C.ink, valign: 'middle' }, { e: 'zoom', d: 1500, t: 0.5 });
  txt('github.com/<저장소>', ML, 6.42, 6, 0.34, { size: 14.5, color: C.mainD }, { e: 'fade', d: 1700, t: 0.4 });
  const b = CFG.brand || {};
  if (b.wordmark) img(A(b.wordmark), CR - b.wordW, 5.9, b.wordW, b.wordH, { alt: b.alt || '' }, { e: 'fade', d: 1700, t: 0.5 });
};
