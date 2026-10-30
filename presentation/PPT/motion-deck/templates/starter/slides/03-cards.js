'use strict';
// Four cards with connectors drawn after every card (so the next card never hides them) + callout
const L = require('motion-deck');
const { C, ML, CW, txt, box, circle, iconImg, iconDot, header, callout, newSlide } = L;

module.exports = async function cards() {
  newSlide();
  header({ num: '02', label: '해결 방법', title: '문제를 네 단계로 나눠 한 시스템이 모두 수행한다', orb: 'd' });
  const cards = [['01', '준비', '첫 단계에서\n하는 일', 'LuGlassWater', true], ['02', '이동', '두 번째 단계에서\n하는 일', 'LuRoute', false], ['03', '전달', '세 번째 단계에서\n하는 일', 'LuHandPlatter', true], ['04', '복귀', '마지막 단계에서\n하는 일', 'LuBatteryCharging', false]];
  const w = 2.86, g = (CW - 4 * w) / 3, y = 2.1, h = 2.98;
  for (let i = 0; i < 4; i++) {
    const [no, t, desc, ic, acc] = cards[i];
    const x = ML + i * (w + g), a = { e: 'up', d: i * 220, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, lw: 1, shadow: 'soft' }, a);
    await iconDot(ic, x + 0.75, y + 0.78, 0.92, acc ? C.accT : C.mainT, acc ? C.acc : C.main, a);
    txt(no, x + w - 1.05, y + 0.38, 0.8, 0.6, { size: 30, bold: true, color: acc ? 'F6C69E' : C.mainT2, align: 'right', check: false }, a);
    txt(t, x + 0.3, y + 1.5, w - 0.6, 0.62, { size: 28, bold: true, color: acc ? C.accD : C.ink }, a);
    txt(desc, x + 0.3, y + 2.15, w - 0.5, 0.85, { size: 15, color: C.muted, ls: 1.15 }, a);
  }
  for (let i = 0; i < 3; i++) {
    const cx = ML + i * (w + g) + w + g / 2, ac = { e: 'grow', d: i * 220 + 260, t: 0.3 };
    circle(cx, y + h / 2, 0.42, { fill: C.white, line: C.line, lw: 1 }, ac);
    await iconImg('LuChevronRight', C.main, cx - 0.13, y + h / 2 - 0.13, 0.26, ac);
  }
  circle(ML + 0.08, 5.6, 0.16, { fill: C.acc }, { e: 'fade', d: 1000, t: 0.4 });
  txt('주황 = 기존 방식에 없는 단계', ML + 0.26, 5.47, 5, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 1000, t: 0.4 });
  callout('핵심', '이 슬라이드에서 기억할 한 문장', 6.12, {}, { e: 'up', d: 1150, t: 0.5 });
};
