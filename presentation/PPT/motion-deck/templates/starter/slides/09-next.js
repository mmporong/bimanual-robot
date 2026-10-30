'use strict';
// Problem -> response rows: domain chip, problem wipes in, arrow, response wipes in
const L = require('motion-deck');
const { C, FSB, ML, CR, txt, box, iconImg, header, newSlide } = L;

module.exports = async function next() {
  newSlide();
  header({ num: '07', label: '한계와 다음 단계', title: '남은 문제와 다음에 할 일', orb: 'd' });
  const xD = ML, wD = 1.5, xP = 2.25, wP = 4.85, xA = 7.17, xS = 7.62, wS = CR - 7.62;
  const ha = { e: 'fade', d: 0, t: 0.4 };
  txt('분야', xD, 1.95, wD, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('남은 문제', xP, 1.95, wP, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('다음 단계', xS, 1.95, wS, 0.28, { size: 12, bold: true, color: C.main }, ha);
  const rows = [['LuNavigation', '이동', '관찰한 한계 한 줄', '대응 계획 한 줄'], ['LuHand', '조작', '관찰한 한계 한 줄', '대응 계획 한 줄'], ['LuScanEye', '인지', '관찰한 한계 한 줄', '대응 계획 한 줄'], ['LuMerge', '통합', '관찰한 한계 한 줄', '대응 계획 한 줄']];
  for (let i = 0; i < rows.length; i++) {
    const [ic, dom, p, s] = rows[i], y = 2.32 + i * 0.84, h = 0.7, d = 200 + i * 200;
    const a = { e: 'fade', d, t: 0.35 };
    box(xD, y, wD, h, { r: 0.1, fill: C.mainD }, a);
    await iconImg(ic, C.white, xD + 0.18, y + (h - 0.26) / 2, 0.26, a);
    txt(dom, xD + 0.52, y, wD - 0.6, h, { size: 14, bold: true, color: C.white, valign: 'middle' }, a);
    txt(p, xP, y, wP, h, { fill: C.panel, size: 13.5, color: C.text, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 80, t: 0.4 });
    await iconImg('LuArrowRight', C.main, xA, y + (h - 0.28) / 2, 0.28, { e: 'fade', d: d + 200, t: 0.25 });
    txt(s, xS, y, wS, h, { fill: C.mainT, size: 13.5, font: FSB, color: C.ink, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 260, t: 0.4 });
  }
};
