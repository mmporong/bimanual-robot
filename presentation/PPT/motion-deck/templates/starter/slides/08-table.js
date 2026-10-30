'use strict';
// Comparison table (editable, wipes down) with the chosen row tinted + callout
const L = require('motion-deck');
const { C, FR, ML, CW, txt, header, callout, newSlide } = L;

module.exports = async function table() {
  newSlide();
  header({ num: '06', label: '비교 — 시도와 선택', title: '여러 방법을 같은 조건에서 비교하고 하나를 골랐다', orb: 'c' });
  const hdr = ['방법', '조건', '결과', '판단'].map((t) => ({ text: t, options: { bold: true, color: C.white, fill: { color: C.ink }, fontSize: 13 } }));
  const data = [['방법 A', '같은 데이터 · 같은 평가', '지표 값', '기준선'], ['방법 B', '같은 데이터 · 같은 평가', '지표 값', '보류'], ['방법 C', '같은 데이터 · 같은 평가', '지표 값', '최종 채택'], ['방법 D', '하드웨어 제약', '–', '제외']];
  const rows = [hdr].concat(data.map((r) => r.map((c, j) => {
    const win = r[3] === '최종 채택';
    return { text: c, options: { bold: win || j === 0, color: win && j === 3 ? C.main : j === 0 ? C.ink : C.text, fill: { color: win ? C.mainT : C.white }, fontSize: 13 } };
  })));
  // margins are in inches here (a [0,8,0,8] "points" guess breaks the table)
  L.S.slide.addTable(rows, { x: ML, y: 2.05, w: CW, colW: [2.6, 4.2, 2.9, CW - 9.7], rowH: [0.46, 0.6, 0.6, 0.6, 0.6], fontFace: FR, valign: 'middle', margin: [0.04, 0.14, 0.04, 0.14], border: { type: 'solid', pt: 0.75, color: C.line }, objectName: 't_table' });
  L.anim('t_table', { e: 'wipeD', d: 200, t: 0.8 });
  txt('공통 조건: 평가 횟수 · 환경 · 기준을 한 줄로', ML, 5.2, CW, 0.3, { size: 12, color: C.muted }, { e: 'fade', d: 900, t: 0.4 });
  callout('선택 이유', '왜 이 방법을 골랐는지 근거 한 문장', 6.15, {}, { e: 'up', d: 1100, t: 0.45 });
};
