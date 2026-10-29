'use strict';
// Demo: auto-playing muted clip in a card, step line under it, headline number + facts on the right.
// The clip appears with the build at 1.2 s; cover:true hides it in LibreOffice until then.
const L = require('motion-deck');
const { C, ML, CR, txt, line, header, newSlide, tagPill, videoCard, seqLine, bigNum } = L;

module.exports = async function demo() {
  newSlide();
  header({ num: '03', label: '데모 — 완성품 시연', title: '그래서, 실제로 되나요?', orb: 'a' });
  const vw = 7.75, vh = vw * 9 / 16;
  // (name, file, poster, x, y, w, h, clip length ms, play at ms, { cover, appear })
  await videoCard('demo_video', 'sample_clip.mp4', 'sample_poster.jpg', ML, 1.9, vw, vh, 6000, 1200, { cover: true, appear: 1200 });
  seqLine(['요청', '이동', '핵심 작업', '확인', '복귀'], ML, 1.9 + vh + 0.16, vw, 400);
  const x = 8.75, w = CR - x;
  const t1 = tagPill('실물 · 연속 0회', x, 1.9, 300);
  tagPill('영상 0배속', x + t1 + 0.12, 1.9, 300, { fill: C.panel, color: C.muted });
  txt(bigNum('000.0', '초', 60), x, 2.42, w, 1.05, { valign: 'bottom', check: false }, { e: 'zoom', d: 450, t: 0.6 });
  txt('대표 지표 한 줄 설명', x, 3.5, w, 0.32, { size: 13.5, color: C.muted }, { e: 'fade', d: 550, t: 0.4 });
  const rows = [['성공 단계', [{ text: '0/0', options: { fontSize: 24, bold: true, color: C.main } }]], ['사람 개입', [{ text: '0회', options: { fontSize: 24, bold: true, color: C.main } }]], ['조건', [{ text: '시연 조건 한 줄', options: { fontSize: 16, bold: true, color: C.ink } }]]];
  rows.forEach((r, i) => {
    const y = 4.05 + i * 0.66, a = { e: 'up', d: 700 + i * 150, t: 0.4 };
    line(x, y, CR, y, { color: C.line, w: 1 }, a);
    txt(r[0], x, y + 0.08, 1.55, 0.5, { size: 12.5, color: C.muted, valign: 'middle' }, a);
    txt(r[1], x + 1.55, y + 0.06, w - 1.55, 0.54, { valign: 'middle', check: false }, a);
  });
  txt('한계나 조건을 정직하게 한 줄로 (예: 시뮬 1회 결과, 실물 재현은 다음 단계)', x, 6.05, w, 0.66, { size: 11.5, color: C.muted, ls: 1.15 }, { e: 'fade', d: 1200, t: 0.4 });
};
