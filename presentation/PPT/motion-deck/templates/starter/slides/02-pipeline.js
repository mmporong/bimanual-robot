'use strict';
// Pipeline: a badge rides a motion path along the track while each step lights up as it passes
const L = require('motion-deck');
const { pres, C, FSB, ML, txt, box, circle, line, iconImg, iconDot, header, newSlide } = L;

module.exports = async function pipeline() {
  newSlide();
  header({ num: '01', label: '개요 — 전체 흐름', title: '요청 한 번이 결과로 이어지는 전체 과정', orb: 'b' });
  const steps = ['요청 접수', '이동', '정렬', '핵심 작업', '확인', '전달', '복귀'];
  const hi = [3, 5]; // steps that other systems do not do (accent colour)
  const x0 = 1.25, x1 = 12.08, cy = 3.6, dx = (x1 - x0) / (steps.length - 1), run = 2400;
  line(x0, cy, x1, cy, { color: C.line, w: 3 }, { e: 'wipeR', d: 0, t: 0.8 });
  line(x0, cy, x1, cy, { color: C.main, w: 3 }, { e: 'wipeR', d: 300, t: run / 1000 });
  // motion path dx/dy are fractions of the slide size (13.333 x 7.5 in)
  const ride = [{ e: 'fade', d: 0, t: 0.3 }, { e: 'path', d: 300, t: run / 1000, dx: (x1 - x0) / 13.333, dy: 0 }];
  circle(x0, 2.62, 0.66, { fill: C.white, line: C.main, lw: 2, shadow: 'soft', name: 'p_bot' }, ride);
  await iconImg('LuBot', C.main, x0 - 0.19, 2.62 - 0.19, 0.38, ride, { name: 'p_boticon' });
  steps.forEach((s, i) => {
    const cx = x0 + i * dx, acc = hi.includes(i), d = 300 + Math.round((i / (steps.length - 1)) * run);
    txt(String(i + 1), cx - 0.42, cy - 0.42, 0.84, 0.84, { shape: pres.shapes.OVAL, fill: acc ? C.acc : C.main, color: C.white, bold: true, size: 20, align: 'center', valign: 'middle', line: C.white, lw: 3, check: false }, { e: 'grow', d, t: 0.35 });
    txt(s, cx - 0.85, 4.18, 1.7, 0.75, { size: 15, font: FSB, color: acc ? C.accD : C.ink, align: 'center', ls: 1.1 }, { e: 'fade', d: d + 100, t: 0.4 });
  });
  circle(ML + 0.08, 5.32, 0.16, { fill: C.acc }, { e: 'fade', d: 2900, t: 0.4 });
  txt('주황 = 기존 방식에 없는 단계', ML + 0.26, 5.19, 5, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 2900, t: 0.4 });
  const rules = [['LuShieldCheck', '안전 규칙이나 운영 원칙 한 줄'], ['LuGauge', '성능 조건이나 제약 한 줄']];
  for (let i = 0; i < 2; i++) {
    const x = ML + i * (5.95 + 0.233), y = 5.9, a = { e: 'up', d: 3100 + i * 150, t: 0.5 };
    box(x, y, 5.95, 0.82, { r: 0.12, fill: C.panel }, a);
    await iconDot(rules[i][0], x + 0.52, y + 0.41, 0.5, C.mainT, C.main, a);
    txt(rules[i][1], x + 0.98, y + 0.12, 4.8, 0.58, { size: 16.5, font: FSB, color: C.ink, valign: 'middle' }, a);
  }
};
