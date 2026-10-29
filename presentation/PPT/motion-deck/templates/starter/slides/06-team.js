'use strict';
// Team: four part cards spring out of a shared hub; the hub pulses twice at the end
const L = require('motion-deck');
const { pres, C, FSB, ML, CR, txt, box, circle, iconDot, header, newSlide } = L;

module.exports = async function team() {
  newSlide();
  header({ num: '05', label: '팀 — 역할과 기여', title: '네 파트가 하나의 인터페이스에서 만난다', orb: 'e' });
  const cw = 4.95, ch = 2.2, xs = [ML, CR - cw], ys = [1.95, 4.5];
  const parts = [
    ['LuRoute', '파트 1', '이름', '맡은 일과 결과를 한 줄로 · 핵심 기술 · 검증 방법', '넘겨준 결과물'],
    ['LuHand', '파트 2', '이름', '맡은 일과 결과를 한 줄로', '넘겨준 결과물'],
    ['LuWrench', '파트 3', '이름', '맡은 일과 결과를 한 줄로', '넘겨준 결과물'],
    ['LuScanEye', '파트 4', '이름', '맡은 일과 결과를 한 줄로', '넘겨준 결과물'],
  ];
  const hx = 6.667, hy = 4.35;
  circle(hx, hy, 2.55, { line: C.mainT3, lw: 1.5, dash: 'dash' }, [{ e: 'zoom', d: 0, t: 0.5 }, { e: 'pulse', d: 1600, t: 0.6, size: 108, repeat: 2 }]);
  txt('공통\n인터페이스', hx - 1.0, hy - 1.0, 2.0, 2.0, { shape: pres.shapes.OVAL, fill: C.ink, color: C.white, bold: true, size: 17, align: 'center', valign: 'middle', ls: 1.1, check: false }, { e: 'grow', d: 100, t: 0.45 });
  for (let i = 0; i < 4; i++) {
    const [ic, part, who, desc, hand] = parts[i];
    const x = xs[i % 2], y = ys[Math.floor(i / 2)], a = { e: 'zoom', d: 450 + i * 170, t: 0.45 };
    // !!q1..!!q4 pair with the mini team map of later slides, so Morph shrinks each card into its quadrant
    box(x, y, cw, ch, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft', name: `!!q${i + 1}` }, a);
    await iconDot(ic, x + 0.5, y + 0.48, 0.56, C.mainT, C.main, a);
    txt(part, x + 0.9, y + 0.3, 2.6, 0.36, { size: 15, bold: true, color: C.main, valign: 'middle' }, a);
    txt(who, x + cw - 1.6, y + 0.27, 1.35, 0.42, { size: 21, bold: true, color: C.ink, align: 'right', valign: 'middle' }, a);
    txt(desc, x + 0.3, y + 0.86, cw - 0.6, 0.6, { size: 13, color: C.muted, ls: 1.1 }, a);
    txt([{ text: '넘겨주는 것  ', options: { bold: true, color: C.main } }, { text: hand, options: { color: C.ink } }], x + 0.2, y + ch - 0.6, cw - 0.4, 0.42, { fill: C.mainT, size: 12.5, font: FSB, valign: 'middle', margin: 5 }, a);
  }
};
