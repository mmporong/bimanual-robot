'use strict';
// Part detail: profile panel + sections on the left, rounded screenshot + process chevrons on the right.
// mini: 1 lights quadrant 1 of the team map (Morph from the team slide shrinks card 1 into it)
const L = require('motion-deck');
const { C, ML, CR, txt, img, rounded, header, newSlide, profile, sec, chevrons } = L;

module.exports = async function part() {
  newSlide();
  header({ num: '05', label: '팀 — 파트 1 상세', title: '파트가 해낸 일을 결과 중심으로 한 문장에', orb: 'a', mini: 1 });
  const px = ML, pw = 4.3, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 4.9, '이름', '파트 1', a0);
  sec('맡은 범위', '핵심 기술 · 도구\n담당 하위 시스템', px + 0.3, 2.95, pw - 0.5, 0.85, { e: 'up', d: 150, t: 0.4 });
  sec('수행 내용', '구현한 기능 1\n구현한 기능 2\n검증한 방법과 결과\n해결한 문제 하나', px + 0.3, 3.95, pw - 0.5, 1.6, { e: 'up', d: 300, t: 0.4 }, { size: 12.5, ls: 1.12 });
  sec('배운 점 · 다음 단계', '남은 과제나 개선 방향 한 줄', px + 0.3, 5.75, pw - 0.5, 0.8, { e: 'up', d: 450, t: 0.4 }, { size: 12.5 });
  const ix = 5.2, iw = CR - ix, ih = 3.2;
  // rounded() flattens to one PNG frame: use it for photos/screenshots, never for animated GIFs
  img(await rounded('sample_photo.jpg', iw, ih, 0.14), ix, 1.95, iw, ih, { alt: '결과 화면' }, { e: 'zoom', d: 300, t: 0.6 });
  txt([{ text: '화면이 보여 주는 것', options: { bold: true, color: C.ink, fontSize: 15 } }, { text: '   보조 설명 한 줄', options: { color: C.muted, fontSize: 12.5 } }], ix, 1.95 + ih + 0.12, iw, 0.36, { valign: 'middle', check: false }, { e: 'fade', d: 700, t: 0.4 });
  chevrons(['01\n입력', '02\n처리 1', '03\n처리 2', '04\n판단', '05\n결과'], ix, 5.72, iw, 0.9, 850, { size: 11 });
};
